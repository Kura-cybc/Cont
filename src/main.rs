#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// ===========================================================================
// Rig configuration.  Every value marked CALIBRATE must be measured on the
// actual teaching rig before the first enable; do not invent numbers.
//
// Calibration procedure:
//   1. Flash with ZERO_CALIBRATED = false and keep the gimbal unpowered/off.
//   2. Hold yaw straight ahead and pitch horizontal.  Read the stable
//      `yaw.motor.angle_raw` / `pitch.motor.angle_raw` values from RTT.
//   3. Put those counts into YAW_ZERO_RAW / PITCH_ZERO_RAW, set
//      ZERO_CALIBRATED = true, and flash again.
//   4. Check the feedback sign with a small hand movement, then check the
//      command direction at a low output limit.
// ===========================================================================

/// GM6020 voltage group command.  The 0x1FE current group does not move this rig.
const MOTOR_COMMAND_ID: u16 = 0x1FF;

/// Motor IDs from the rig's DIP switches.  Feedback is 0x205 + ID - 1 and the
/// command slot is (ID - 1) * 2 inside the 0x1FF frame.
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;
const YAW_FEEDBACK_ID: u16 = 0x205 + (YAW_MOTOR_ID as u16) - 1;
const PITCH_FEEDBACK_ID: u16 = 0x205 + (PITCH_MOTOR_ID as u16) - 1;

/// CALIBRATE: raw single-turn counts (0..=8191) with the rig held at the
/// reference pose (yaw straight ahead, pitch horizontal).
const YAW_ZERO_RAW: u16 = 3379;
const PITCH_ZERO_RAW: u16 = 1520;

/// Set to true only after YAW_ZERO_RAW and PITCH_ZERO_RAW were measured.
/// While false, enabling the gimbal is refused and Error is latched instead of
/// recentering toward an arbitrary count.
const ZERO_CALIBRATED: bool = true;

/// CALIBRATE: +1 if a positive joint angle is the mechanical direction you
/// want, -1 to invert the feedback sign.  For pitch, hand-move the arm up and
/// watch `joint`: if it decreases, use -1.0 here.
const YAW_MOTOR_DIR: f32 = 1.0;
const PITCH_MOTOR_DIR: f32 = 1.0;

/// CALIBRATE: +1 if "stick right/up" must increase the target angle, else -1.
const YAW_STICK_DIR: f32 = 1.0;
const PITCH_STICK_DIR: f32 = 1.0;

/// CALIBRATE: joint limits relative to the calibrated zero.  Yaw is either a
/// full turn (`-PI..PI`, continuous, shortest path) or a cable/stop-limited
/// range of at most half a turn.  Pitch does not wrap and must not be driven
/// into its stops.
///
/// Measured pitch counts: lower stop 1078, horizontal ~1520 (the zero),
/// upper stop ~1775.  Relative to horizontal the stops are about -0.34 rad
/// and +0.20 rad; the range below keeps ~0.02 rad of margin before each stop.
/// Re-measure and adjust these numbers after any mechanical change.
const YAW_MIN_RAD: f32 = -PI;
const YAW_MAX_RAD: f32 = PI;
const PITCH_MIN_RAD: f32 = -0.32;
const PITCH_MAX_RAD: f32 = 0.17;
const YAW_SHORTEST_PATH: bool = true;
const PITCH_SHORTEST_PATH: bool = false;
// Configuration sanity checks; a bad value stops the build instead of moving
// the gimbal in an unexpected direction.
const _: () = assert!(YAW_MOTOR_DIR == 1.0 || YAW_MOTOR_DIR == -1.0);
const _: () = assert!(PITCH_MOTOR_DIR == 1.0 || PITCH_MOTOR_DIR == -1.0);
const _: () = assert!(YAW_STICK_DIR == 1.0 || YAW_STICK_DIR == -1.0);
const _: () = assert!(PITCH_STICK_DIR == 1.0 || PITCH_STICK_DIR == -1.0);
const _: () = assert!(YAW_MOTOR_ID != PITCH_MOTOR_ID);
const _: () = assert!(YAW_MOTOR_ID >= 1 && YAW_MOTOR_ID <= 4);
const _: () = assert!(PITCH_MOTOR_ID >= 1 && PITCH_MOTOR_ID <= 4);
const _: () = assert!(STICK_DEADZONE >= 0.0 && STICK_DEADZONE < 1.0);
const _: () = assert!(YAW_MIN_RAD < YAW_MAX_RAD);
const _: () = assert!((YAW_MAX_RAD - YAW_MIN_RAD) <= TAU);
// Yaw is either a full-turn (continuous, shortest path) axis or a range of at
// most half a turn (shortest path stays inside the allowed range).  A range
// between half a turn and a full turn would need extra path/limit logic.
const _: () =
    assert!((YAW_MAX_RAD - YAW_MIN_RAD <= PI) || (YAW_MIN_RAD <= -PI && YAW_MAX_RAD >= PI));
// The wrapped error used below is the unique reachable pitch path only while
// the allowed range is no longer than half a turn.  Fail the build otherwise.
const _: () = assert!(PITCH_MIN_RAD < PITCH_MAX_RAD);
const _: () = assert!((PITCH_MAX_RAD - PITCH_MIN_RAD) <= PI);
const _: () = assert!(YAW_OUTPUT_LIMIT_RAW > 0.0 && YAW_OUTPUT_LIMIT_RAW <= PROTOCOL_LIMIT_RAW);
const _: () = assert!(PITCH_OUTPUT_LIMIT_RAW > 0.0 && PITCH_OUTPUT_LIMIT_RAW <= PROTOCOL_LIMIT_RAW);

/// Maximum rate at which the stick moves a target angle, rad/s.
const YAW_MAX_TARGET_RATE_RAD_S: f32 = 1.0;
const PITCH_MAX_TARGET_RATE_RAD_S: f32 = 0.5;
/// Normalized stick input inside this band is treated as centered.
const STICK_DEADZONE: f32 = 0.05;

/// Bench voltage limits in protocol counts (full protocol range is +/-25000).
/// Yaw has no gravity load; the pitch arm is heavier, so allow it more torque.
/// Raise these only after the direction and mechanical stops are known.
const YAW_OUTPUT_LIMIT_RAW: f32 = 5_000.0;
const PITCH_OUTPUT_LIMIT_RAW: f32 = 12_000.0;
const PROTOCOL_LIMIT_RAW: f32 = 25_000.0;
/// Position-loop output limit, rpm.  Set this PID's `maxout` to 0 to tune the
/// speed loop alone.  When both position loops are disabled this way, Prepare
/// hands over to Running at the current pose instead of timing out (the
/// position loop cannot recenter with zero output).
const POSITION_MAX_RPM: f32 = 30.0;

// PID gains.  These are safe starting points, not a finished tuning.
const YAW_POSITION_KP: f32 = 70.0;
const YAW_POSITION_KI: f32 = 0.0;
const YAW_POSITION_KD: f32 = 0.0;
const PITCH_POSITION_KP: f32 = 150.0;
const PITCH_POSITION_KI: f32 = 0.0;
const PITCH_POSITION_KD: f32 = 0.0;
const YAW_SPEED_KP: f32 = 200.0;
const PITCH_SPEED_KP: f32 = 200.0;
/// Integral gain in counts per (rpm * s).  The yaw log showed the axis stopping
/// about 0.41 rad short because P-only speed control could not overcome static
/// friction.  Pitch needs a larger integral to lift the gravity-loaded arm.
/// For the first speed-loop tuning pass both are zero; add them after Kp.
const YAW_SPEED_KI: f32 = 200.0;
const PITCH_SPEED_KI: f32 = 200.0;
const SPEED_KD: f32 = 0.0;

/// Gravity feedforward for the pitch arm, in voltage counts at horizontal.
/// It supplies the steady torque that the PID would otherwise have to build
/// through a large integral, which is what causes stick-slip/jerky motion.
/// Tune this up if the arm still sags or cannot start, down if it jumps up.
const PITCH_GRAVITY_FF_RAW: f32 = 7400.0;
/// The feedforward is applied the whole time the axis is enabled.  PITCH_MIN_RAD
/// lies above the lower stop, so the motor always carries the arm in Prepare and
/// Running; gating the feedforward near the limit would drop the arm onto the
/// stop and leave the integral to wind up against gravity.
/// Gravity torque ~ cos(angle from horizontal), approximated as 1 - K*angle^2.
const PITCH_GRAVITY_FF_COS_K: f32 = 0.5;

/// Recentering (Prepare) limits and completion condition.
const PREPARE_MAX_RPM: f32 = 30.0;
const YAW_PREPARE_OUTPUT_LIMIT_RAW: f32 = 1_500.0;
const PITCH_PREPARE_OUTPUT_LIMIT_RAW: f32 = 5_000.0;
const PREPARE_ANGLE_TOL_RAD: f32 = 0.1;
const PREPARE_SPEED_TOL_RPM: f32 = 10.0;
/// Both axes must stay inside the tolerance for 300 ms.
const PREPARE_SETTLE_TICKS: u16 = 300;
/// With both position loops at maxout=0, enter Running after 200 ms so the
/// speed loop can be tuned without waiting for an impossible recenter.
const SPEED_TUNING_BYPASS_TICKS: u16 = 200;
/// 6 s without settling is a fault.
const PREPARE_TIMEOUT_TICKS: u16 = 6_000;
/// A reference pose for both axes; the calibrated zero is the reference.
const YAW_REFERENCE_RAD: f32 = 0.0;
const PITCH_REFERENCE_RAD: f32 = 0.0;
/// Set false only when the rig cannot recenter (for example while tuning or
/// when the reference pose is not reachable); enabling then starts Running at
/// the current pose instead of entering Prepare.
const RECENTER_ON_ENABLE: bool = true;

/// Feedback older than 20 ms is stale.
const FEEDBACK_TIMEOUT_TICKS: u16 = 20;

/// Stall: target speed >= 20 rpm, output >= 500 counts in the same direction,
/// but measured speed stays below 5 rpm for 1200 ms.  Target 0 never counts.
/// The longer window gives a gravity-loaded pitch axis time to ramp its output.
const STALL_TARGET_RPM: f32 = 20.0;
const STALL_SPEED_RPM: f32 = 5.0;
const STALL_OUTPUT_RAW: f32 = 500.0;
const STALL_TICKS: u16 = 1_200;

/// Oscillation: speed crosses +/-20 rpm repeatedly, each complete cycle is
/// faster than 25 Hz, and that condition lasts for 500 ms.
const OSC_AMPLITUDE_RPM: f32 = 20.0;
const OSC_MIN_FREQ_HZ: f32 = 25.0;
const OSC_CONFIRM_TICKS: u16 = 500;
/// A half cycle longer than 200 ms is not the oscillation we are looking for.
const OSC_TIMEOUT_TICKS: u16 = 200;

// Operator gestures used by this program:
//   enable  : right switch down (2), then middle (3); both sticks centered and
//             both feedbacks fresh.
//   disable : right switch to down (2) or up (1); up is never an enable.
//   recover : in Error, keep the right switch in the middle (3), move the left
//             switch down (2) then up (1), with both sticks centered.  This is
//             a one-shot gesture; holding a switch cannot recover by itself.

// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);
const DT_S: f32 = 0.001;
const RAW_TO_RAD: f32 = TAU / 8192.0;

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Raw single-turn mechanical count, 0..=8191.
    pub angle_raw: u16,
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Motor temperature in degrees Celsius, from byte 6 of the feedback frame.
    pub temperature_c: i8,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    /// Decode one 8-byte GM6020 feedback frame (big-endian fields):
    /// [0..2] mechanical angle 0..8191, [2..4] speed rpm, [4..6] current raw,
    /// [6] temperature, [7] reserved.
    pub fn read(&mut self, data: &[u8; 8]) {
        let angle_raw = u16::from_be_bytes([data[0], data[1]]) & 0x1FFF;
        self.angle_raw = angle_raw;
        self.angle_rad = angle_raw as f32 * RAW_TO_RAD;
        self.speed_rpm = i16::from_be_bytes([data[2], data[3]]) as f32;
        self.current_raw = i16::from_be_bytes([data[4], data[5]]);
        self.temperature_c = data[6] as i8;
    }
}

/// PID gains, plus the integral and history kept between ticks.
#[derive(Clone, Copy)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Symmetric output limit: the returned value is always in [-maxout, maxout].
    pub maxout: f32,
    /// Symmetric limit of the integral term, in output units. 0 disables this
    /// clamp (the integral can then grow up to the output saturation logic).
    pub integral_limit: f32,
    pub integral: f32,
    /// Previous error, used for the derivative term.  Differentiating the
    /// wrapped error keeps the D term finite when the raw count crosses 8191/0.
    pub prev_error: f32,
    pub initialized: bool,
    pub p_term: f32,
    pub i_term: f32,
    pub d_term: f32,
    pub last_output: f32,
}

impl Default for Pid {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0)
    }
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, maxout: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            maxout,
            integral_limit: maxout,
            integral: 0.0,
            prev_error: 0.0,
            initialized: false,
            p_term: 0.0,
            i_term: 0.0,
            d_term: 0.0,
            last_output: 0.0,
        }
    }

    /// Clear the integral and history. The next calculation initializes the
    /// derivative history from its measurement and therefore returns no D kick.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.initialized = false;
        self.p_term = 0.0;
        self.i_term = 0.0;
        self.d_term = 0.0;
        self.last_output = 0.0;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        self.calculate_error(target - measured, measured, dt_s)
    }

    /// Same as `calculate_error`, but the caller supplies the limit that the
    /// current state applies after the PID (for example Prepare's 1500-count
    /// output limit).  The PID uses that tighter limit for output clamping and
    /// integral anti-windup, then restores its configured limit.
    pub fn calculate_error_limited(
        &mut self,
        error: f32,
        measured: f32,
        dt_s: f32,
        output_limit: f32,
    ) -> f32 {
        let saved_maxout = self.maxout;
        let saved_integral_limit = self.integral_limit;
        let limit = if output_limit.is_finite() {
            output_limit.max(0.0)
        } else {
            0.0
        };
        self.maxout = self.maxout.min(limit);
        self.integral_limit = if self.integral_limit > 0.0 {
            self.integral_limit.min(self.maxout)
        } else {
            0.0
        };
        let out = self.calculate_error(error, measured, dt_s);
        self.maxout = saved_maxout;
        self.integral_limit = saved_integral_limit;
        out
    }

    /// Same as `calculate`, but the caller supplies the already-unwrapped error.
    /// Yaw uses this so an angle crossing 8191/0 does not become a huge error.
    pub fn calculate_error(&mut self, error: f32, _measured: f32, dt_s: f32) -> f32 {
        let error = if error.is_finite() { error } else { 0.0 };
        let dt_s = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s
        } else {
            0.0
        };
        let maxout = if self.maxout.is_finite() {
            self.maxout.max(0.0)
        } else {
            0.0
        };
        let first = !self.initialized;

        self.p_term = self.kp * error;

        if first {
            self.prev_error = error;
            self.initialized = true;
        }

        // With Ki = 0 the integral must not contribute (and must stay zero).
        if self.ki != 0.0 && dt_s > 0.0 {
            self.integral += error * dt_s;
            let limit = if self.integral_limit.is_finite() && self.integral_limit > 0.0 {
                self.integral_limit / self.ki.abs()
            } else {
                f32::MAX
            };
            self.integral = self.integral.clamp(-limit, limit);
        }
        self.i_term = self.ki * self.integral;

        // Derivative on the (already wrapped) error.  Do not differentiate the
        // raw single-turn measurement: it jumps by 2*pi at the 8191/0 boundary.
        self.d_term = if dt_s > 0.0 && !first {
            self.kd * (error - self.prev_error) / dt_s
        } else {
            0.0
        };

        let mut out = self.p_term + self.i_term + self.d_term;
        let clamped = if maxout > 0.0 {
            out.clamp(-maxout, maxout)
        } else {
            0.0
        };

        // Conditional integration: undo this step if it only pushed an already
        // saturated output further into the same limit.
        if self.ki != 0.0
            && dt_s > 0.0
            && (out - clamped).abs() > 1e-9
            && error * (out - clamped) > 0.0
        {
            self.integral -= error * dt_s;
            self.i_term = self.ki * self.integral;
            out = self.p_term + self.i_term + self.d_term;
        }

        self.prev_error = error;
        self.last_output = if maxout > 0.0 {
            out.clamp(-maxout, maxout)
        } else {
            0.0
        };
        self.last_output
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// Latched fault reason. `None` means no fault since the last Prepare/recovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaultCode {
    None,
    NotCalibrated,
    RecenterTimeout,
    YawStall,
    PitchStall,
    YawOscillation,
    PitchOscillation,
}

#[derive(Clone, Copy)]
enum FaultSide {
    Yaw,
    Pitch,
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,

    /// Calibrated zero in raw counts and radians.
    pub zero_raw: u16,
    pub zero_rad: f32,
    /// Feedback sign for this axis: +1 or -1.
    pub direction: f32,
    /// Allowed joint range relative to the calibrated zero.
    pub min_rad: f32,
    pub max_rad: f32,
    /// Yaw wraps its target across a full turn; pitch clamps inside its limits.
    pub shortest_path: bool,
    /// Gravity feedforward in voltage counts; 0 for an axis without gravity load.
    pub gravity_ff_raw: f32,
    /// Joint angle in rad relative to the calibrated zero, updated with each frame.
    /// This is the value to compare against `target_angle_rad`.
    pub joint_angle_rad: f32,

    /// Control ticks since the last valid feedback frame.
    pub feedback_age: u16,
    /// Consecutive ticks that satisfy the stall condition.
    pub stall_ticks: u16,
    /// Oscillation detector state: +1 above +AMP, -1 below -AMP, 0 idle.
    pub osc_level: i8,
    pub osc_half_ticks: u16,
    pub osc_over_ticks: u16,
    pub osc_freq_hz: f32,
}

impl Axis {
    fn new(
        zero_raw: u16,
        direction: f32,
        min_rad: f32,
        max_rad: f32,
        shortest_path: bool,
        gravity_ff_raw: f32,
        position_pid: Pid,
        speed_pid: Pid,
    ) -> Self {
        Self {
            motor: Motor::default(),
            target_angle_rad: 0.0,
            target_rpm: 0.0,
            position_pid,
            speed_pid,
            zero_raw,
            zero_rad: zero_raw as f32 * RAW_TO_RAD,
            direction,
            min_rad,
            max_rad,
            shortest_path,
            gravity_ff_raw,
            joint_angle_rad: 0.0,
            feedback_age: u16::MAX,
            stall_ticks: 0,
            osc_level: 0,
            osc_half_ticks: 0,
            osc_over_ticks: 0,
            osc_freq_hz: 0.0,
        }
    }

    /// Joint angle in rad relative to the calibrated zero.
    fn measured_angle_rad(&self) -> f32 {
        wrap_pi((self.motor.angle_rad - self.zero_rad) * self.direction)
    }

    /// Position error in joint coordinates. Both axes wrap the difference so a
    /// raw-count crossing at 8191/0 stays a small error. Pitch targets are
    /// clamped inside a range no longer than half a turn (checked above), so the
    /// wrapped value is the only reachable path to the clamped target.
    fn position_error(&self, target: f32) -> f32 {
        wrap_pi(target - self.measured_angle_rad())
    }

    /// Keep the target inside the calibrated range.  A full-turn range wraps so
    /// continuous yaw is possible; a narrower (cable-limited) range clamps and
    /// never jumps to the other side.
    fn clamp_target(&mut self) {
        if self.shortest_path && self.min_rad <= -PI && self.max_rad >= PI {
            self.target_angle_rad = wrap_pi(self.target_angle_rad);
        }
        self.target_angle_rad = self.target_angle_rad.clamp(self.min_rad, self.max_rad);
    }

    fn reset_control(&mut self) {
        self.position_pid.reset();
        self.speed_pid.reset();
        self.target_rpm = 0.0;
        self.motor.disable();
    }

    fn reset_detection(&mut self) {
        self.stall_ticks = 0;
        self.osc_level = 0;
        self.osc_half_ticks = 0;
        self.osc_over_ticks = 0;
        self.osc_freq_hz = 0.0;
    }

    /// Run the position and speed cascade for one tick.
    fn track(&mut self, target_angle: f32, dt_s: f32, rpm_limit: f32, output_limit: f32) {
        // Work in motor coordinates. Pick the 2*pi-equivalent target nearest the
        // current motor angle; the joint target was already clamped to the
        // allowed range, so this is the reachable path and not a blind wrap.
        let target_motor = self.motor.angle_rad
            + wrap_pi(self.zero_rad + target_angle * self.direction - self.motor.angle_rad);
        let error = target_motor - self.motor.angle_rad;
        let measured = self.motor.angle_rad;
        let rpm = self
            .position_pid
            .calculate_error_limited(error, measured, dt_s, rpm_limit);
        self.target_rpm = clamp_command(rpm, rpm_limit);

        let out = self.speed_pid.calculate_error_limited(
            self.target_rpm - self.motor.speed_rpm,
            self.motor.speed_rpm,
            dt_s,
            output_limit,
        );
        // Gravity feedforward (zero = horizontal).  It must not switch with the
        // error sign or with the target position; either makes the output jump.
        let mut ff = 0.0;
        if self.gravity_ff_raw > 0.0 {
            // Always on while enabled: the allowed range ends above the lower
            // stop, so there is no pose where the arm may rest unsupported.
            // Scale by cos(angle from horizontal), approximated to second order.
            let joint = self.measured_angle_rad();
            let cos = (1.0 - PITCH_GRAVITY_FF_COS_K * joint * joint).clamp(0.0, 1.0);
            ff = self.gravity_ff_raw * cos;
            // (Waiting and Error never reach this function; their output is zero.)
        }
        // The state limit bounds the PID part; ff rides on top, capped by the axis limit.
        self.motor.command_voltage_raw = clamp_command(out + ff, (output_limit + ff).min(self.speed_pid.maxout));
    }
}

/// State shared by every control tick.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,

    /// True after the right switch was seen in the down (2) position. A middle
    /// (3) position only starts a new enable when this latch is set.
    pub enable_armed: bool,
    /// True after the left switch was seen down (2) while in Error. Recovery
    /// needs a following up (1) edge, so holding the switch cannot recover.
    pub recovery_armed: bool,
    pub prepare_ticks: u16,
    pub prepare_stable_ticks: u16,
    pub fault: FaultCode,
    /// Rate-limited logging counter for the command frame.
    pub tx_log_ticks: u16,
}

/// Start in `Waiting` with no remote input.
impl Default for Control {
    fn default() -> Self {
        let yaw = Axis::new(
            YAW_ZERO_RAW,
            YAW_MOTOR_DIR,
            YAW_MIN_RAD,
            YAW_MAX_RAD,
            YAW_SHORTEST_PATH,
            0.0,
            Pid::new(
                YAW_POSITION_KP,
                YAW_POSITION_KI,
                YAW_POSITION_KD,
                POSITION_MAX_RPM,
            ),
            Pid::new(YAW_SPEED_KP, YAW_SPEED_KI, SPEED_KD, YAW_OUTPUT_LIMIT_RAW),
        );
        let pitch = Axis::new(
            PITCH_ZERO_RAW,
            PITCH_MOTOR_DIR,
            PITCH_MIN_RAD,
            PITCH_MAX_RAD,
            PITCH_SHORTEST_PATH,
            PITCH_GRAVITY_FF_RAW,
            Pid::new(
                PITCH_POSITION_KP,
                PITCH_POSITION_KI,
                PITCH_POSITION_KD,
                POSITION_MAX_RPM,
            ),
            Pid::new(
                PITCH_SPEED_KP,
                PITCH_SPEED_KI,
                SPEED_KD,
                PITCH_OUTPUT_LIMIT_RAW,
            ),
        );
        Self {
            state: State::Waiting,
            yaw,
            pitch,
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            enable_armed: false,
            recovery_armed: false,
            prepare_ticks: 0,
            prepare_stable_ticks: 0,
            fault: FaultCode::None,
            tx_log_ticks: 0,
        }
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            // Losing the link must not leave an old enable/recovery gesture armed.
            self.enable_armed = false;
            self.recovery_armed = false;
            if self.state != State::Error {
                // This also clears the PIDs and detection history and returns
                // to Waiting, so a reconnect cannot silently continue a run.
                self.enter_waiting();
            } else {
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        self.yaw.feedback_age = self.yaw.feedback_age.saturating_add(1);
        self.pitch.feedback_age = self.pitch.feedback_age.saturating_add(1);

        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    if frame.header().rtr() || frame.data().len() != 8 {
                        continue;
                    }
                    let id = match frame.id() {
                        embedded_can::Id::Standard(id) => id.as_raw(),
                        embedded_can::Id::Extended(_) => continue,
                    };
                    let mut data = [0u8; 8];
                    data.copy_from_slice(frame.data());
                    // A GM6020 mechanical angle is 0..=8191.  Reject a frame
                    // outside that range instead of accepting a corrupted angle.
                    if u16::from_be_bytes([data[0], data[1]]) > 8191 {
                        continue;
                    }

                    if id == YAW_FEEDBACK_ID {
                        self.yaw.motor.read(&data);
                        self.yaw.joint_angle_rad = self.yaw.measured_angle_rad();
                        self.yaw.feedback_age = 0;
                    } else if id == PITCH_FEEDBACK_ID {
                        self.pitch.motor.read(&data);
                        self.pitch.joint_angle_rad = self.pitch.measured_angle_rad();
                        self.pitch.feedback_age = 0;
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    fn feedback_fresh(&self) -> bool {
        self.yaw.feedback_age < FEEDBACK_TIMEOUT_TICKS
            && self.pitch.feedback_age < FEEDBACK_TIMEOUT_TICKS
    }

    /// Both sticks inside the deadzone.  Used before enable and recovery so a
    /// deflected stick cannot move the target on the first Running tick.
    fn sticks_centered(&self) -> bool {
        let Some(remote) = self.remote else {
            return false;
        };
        deadzone(remote.channels[2] as f32 / 660.0) == 0.0
            && deadzone(remote.channels[3] as f32 / 660.0) == 0.0
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        let Some(remote) = self.remote else {
            // Link loss already forced Waiting in read_remote; Error stays latched.
            self.enable_armed = false;
            self.recovery_armed = false;
            return;
        };

        match self.state {
            State::Waiting => {
                // Up (1) is not an enable. A fresh enable is down (2)->middle (3).
                match remote.right_switch {
                    2 => self.enable_armed = true,
                    3 => {
                        if self.enable_armed {
                            if !self.feedback_fresh() {
                                // Keep the latch: once both feedbacks arrive while the
                                // switch is still in the middle, this same explicit
                                // down->middle gesture may start the gimbal.
                                return;
                            }
                            if !self.sticks_centered() {
                                return;
                            }
                            if !ZERO_CALIBRATED {
                                rtt_target::rprintln!(
                                    "Enable refused: YAW_ZERO_RAW/PITCH_ZERO_RAW not calibrated"
                                );
                                self.enter_error(FaultCode::NotCalibrated);
                                return;
                            }
                            if RECENTER_ON_ENABLE {
                                self.enter_prepare();
                            } else {
                                self.enter_running_at_current_pose();
                            }
                        }
                    }
                    _ => self.enable_armed = false,
                }
            }
            State::Prepare => {
                // Any position other than middle stops output. Down also arms a
                // later middle->enable; up requires down first.
                if remote.right_switch != 3 {
                    self.enter_waiting();
                    self.enable_armed = remote.right_switch == 2;
                    return;
                }
                if !self.feedback_fresh() {
                    self.enter_waiting();
                    return;
                }

                self.prepare_ticks = self.prepare_ticks.saturating_add(1);
                if self.prepare_ticks >= PREPARE_TIMEOUT_TICKS {
                    self.enter_error(FaultCode::RecenterTimeout);
                    return;
                }
                if let Some(fault) = self.detect_faults() {
                    self.enter_error(fault);
                    return;
                }
                if self.recenter_disabled() {
                    if self.prepare_ticks >= SPEED_TUNING_BYPASS_TICKS {
                        self.enter_running_for_speed_tuning();
                    }
                    return;
                }
                if self.prepare_settled() {
                    self.prepare_stable_ticks = self.prepare_stable_ticks.saturating_add(1);
                    if self.prepare_stable_ticks >= PREPARE_SETTLE_TICKS {
                        self.enter_running_from_prepare();
                    }
                } else {
                    self.prepare_stable_ticks = 0;
                }
            }
            State::Running => {
                if remote.right_switch != 3 {
                    self.enter_waiting();
                    self.enable_armed = remote.right_switch == 2;
                    return;
                }
                if !self.feedback_fresh() {
                    self.enter_waiting();
                    return;
                }
                if let Some(fault) = self.detect_faults() {
                    self.enter_error(fault);
                }
            }
            State::Error => {
                // Outputs are held at zero until an explicit left down->up gesture
                // while the right switch stays in the middle. Link loss and time
                // never recover.
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
                self.enable_armed = false;

                if remote.right_switch != 3 {
                    self.recovery_armed = false;
                    return;
                }
                if !ZERO_CALIBRATED {
                    self.recovery_armed = false;
                    return;
                }
                if remote.left_switch == 2 {
                    self.recovery_armed = true;
                } else if remote.left_switch == 1 && self.recovery_armed {
                    if self.feedback_fresh() && self.sticks_centered() {
                        self.recover_to_running();
                    } else {
                        // Require a fresh down->up gesture once the sticks are
                        // centered and feedback is fresh.
                        self.recovery_armed = false;
                    }
                }
            }
        }
    }

    /// A position loop that cannot produce an rpm target: its output limit or
    /// its proportional gain was deliberately zeroed for speed-loop tuning.
    fn position_loop_unusable(pid: &Pid) -> bool {
        pid.maxout <= 0.0 || pid.kp == 0.0
    }

    /// Either position loop is disabled, so a normal two-axis recenter cannot
    /// complete.  In that configuration Prepare hands over to Running at the
    /// current pose after a short delay instead of waiting for the 6 s timeout.
    fn recenter_disabled(&self) -> bool {
        Self::position_loop_unusable(&self.yaw.position_pid)
            || Self::position_loop_unusable(&self.pitch.position_pid)
    }

    /// Enter Running at the current pose.  Used when the build deliberately
    /// skips recentering (RECENTER_ON_ENABLE=false); the position loops remain
    /// active, so the stick can still move the targets.
    fn enter_running_at_current_pose(&mut self) {
        self.yaw.target_angle_rad = self.yaw.measured_angle_rad();
        self.pitch.target_angle_rad = self.pitch.measured_angle_rad();
        self.yaw.reset_control();
        self.pitch.reset_control();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.fault = FaultCode::None;
        self.enable_armed = false;
        self.recovery_armed = false;
        self.prepare_ticks = 0;
        self.prepare_stable_ticks = 0;
        self.state = State::Running;
        rtt_target::rprintln!(
            "state Waiting -> Running (RECENTER_ON_ENABLE=false): yaw={:.3} pitch={:.3} rad, targets set to current; PIDs reset",
            self.yaw.target_angle_rad,
            self.pitch.target_angle_rad
        );
    }

    /// Speed-tuning handover: the position loop cannot recenter, so keep the
    /// current pose, clear history and enter Running with target_rpm = 0.
    fn enter_running_for_speed_tuning(&mut self) {
        let yaw_disabled = Self::position_loop_unusable(&self.yaw.position_pid);
        let pitch_disabled = Self::position_loop_unusable(&self.pitch.position_pid);
        self.yaw.target_angle_rad = self.yaw.measured_angle_rad();
        self.pitch.target_angle_rad = self.pitch.measured_angle_rad();
        self.yaw.reset_control();
        self.pitch.reset_control();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.prepare_ticks = 0;
        self.prepare_stable_ticks = 0;
        self.state = State::Running;
        rtt_target::rprintln!(
            "state Prepare -> Running (speed tuning: yaw_disabled={} pitch_disabled={}, recenter skipped) yaw={:.3} pitch={:.3} rad; target_rpm=0, PIDs reset",
            yaw_disabled,
            pitch_disabled,
            self.yaw.target_angle_rad,
            self.pitch.target_angle_rad
        );
    }

    /// True when both axes have been inside the recentering tolerance long enough
    /// to let `update_mode` count a stable tick.
    fn prepare_settled(&self) -> bool {
        let yaw_angle_ok = self.yaw.position_error(YAW_REFERENCE_RAD).abs() < PREPARE_ANGLE_TOL_RAD;
        let pitch_angle_ok =
            self.pitch.position_error(PITCH_REFERENCE_RAD).abs() < PREPARE_ANGLE_TOL_RAD;
        yaw_angle_ok
            && pitch_angle_ok
            && self.yaw.motor.speed_rpm.abs() < PREPARE_SPEED_TOL_RPM
            && self.pitch.motor.speed_rpm.abs() < PREPARE_SPEED_TOL_RPM
    }

    /// Update both axes' stall and oscillation detectors. Returns the first fault.
    fn detect_faults(&mut self) -> Option<FaultCode> {
        if let Some(fault) = update_axis_fault(&mut self.yaw, FaultSide::Yaw) {
            return Some(fault);
        }
        update_axis_fault(&mut self.pitch, FaultSide::Pitch)
    }

    fn enter_waiting(&mut self) {
        if self.state != State::Waiting {
            rtt_target::rprintln!(
                "state {:?} -> Waiting: outputs zero, new enable needs down(2)->middle(3)",
                self.state
            );
        }
        self.state = State::Waiting;
        self.disable_all();
        self.yaw.reset_control();
        self.pitch.reset_control();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.enable_armed = false;
        self.recovery_armed = false;
        self.prepare_ticks = 0;
        self.prepare_stable_ticks = 0;
        self.fault = FaultCode::None;
    }

    fn enter_prepare(&mut self) {
        self.state = State::Prepare;
        self.fault = FaultCode::None;
        self.yaw.target_angle_rad = YAW_REFERENCE_RAD;
        self.pitch.target_angle_rad = PITCH_REFERENCE_RAD;
        self.yaw.reset_control();
        self.pitch.reset_control();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.prepare_ticks = 0;
        self.prepare_stable_ticks = 0;
        self.enable_armed = false;
        self.recovery_armed = false;
        rtt_target::rprintln!(
            "state Waiting -> Prepare: yaw {:.3}->{:.3} rad, pitch {:.3}->{:.3} rad",
            self.yaw.measured_angle_rad(),
            YAW_REFERENCE_RAD,
            self.pitch.measured_angle_rad(),
            PITCH_REFERENCE_RAD
        );
        if Self::position_loop_unusable(&self.yaw.position_pid) {
            rtt_target::rprintln!(
                "WARNING: yaw position loop unusable (kp={:.1}, maxout={:.1}); recenter cannot move yaw",
                self.yaw.position_pid.kp,
                self.yaw.position_pid.maxout
            );
        }
        if Self::position_loop_unusable(&self.pitch.position_pid) {
            rtt_target::rprintln!(
                "WARNING: pitch position loop unusable (kp={:.1}, maxout={:.1}); recenter cannot move pitch",
                self.pitch.position_pid.kp,
                self.pitch.position_pid.maxout
            );
        }
    }

    fn enter_running_from_prepare(&mut self) {
        self.state = State::Running;
        self.prepare_stable_ticks = 0;
        self.prepare_ticks = 0;
        rtt_target::rprintln!(
            "state Prepare -> Running after {} ms stable",
            PREPARE_SETTLE_TICKS
        );
    }

    fn enter_error(&mut self, fault: FaultCode) {
        self.state = State::Error;
        self.fault = fault;
        // Log the values that caused the fault before the outputs are cleared.
        rtt_target::rprintln!(
            "FAULT {:?} -> Error | yaw rpm={:.1} tgt_rpm={:.1} out={:.0} cur={} temp={} stall={}ms osc={:.1}Hz/{}ms | pitch rpm={:.1} tgt_rpm={:.1} out={:.0} cur={} temp={} stall={}ms osc={:.1}Hz/{}ms",
            fault,
            self.yaw.motor.speed_rpm,
            self.yaw.target_rpm,
            self.yaw.motor.command_voltage_raw,
            self.yaw.motor.current_raw,
            self.yaw.motor.temperature_c,
            self.yaw.stall_ticks,
            self.yaw.osc_freq_hz,
            self.yaw.osc_over_ticks,
            self.pitch.motor.speed_rpm,
            self.pitch.target_rpm,
            self.pitch.motor.command_voltage_raw,
            self.pitch.motor.current_raw,
            self.pitch.motor.temperature_c,
            self.pitch.stall_ticks,
            self.pitch.osc_freq_hz,
            self.pitch.osc_over_ticks
        );
        self.disable_all();
        self.yaw.reset_control();
        self.pitch.reset_control();
        // Keep the detector counters latched so Tuning Studio can show the
        // evidence; they are cleared on recovery or on the next Prepare.
        self.enable_armed = false;
        self.recovery_armed = false;
    }

    /// Manual recovery: keep the current physical pose, clear all history and
    /// continue in Running. The old target angle is deliberately discarded.
    fn recover_to_running(&mut self) {
        let yaw_angle = self.yaw.measured_angle_rad();
        let pitch_angle = self.pitch.measured_angle_rad();

        self.yaw.target_angle_rad = yaw_angle;
        self.pitch.target_angle_rad = pitch_angle;
        self.yaw.reset_control();
        self.pitch.reset_control();
        self.yaw.reset_detection();
        self.pitch.reset_detection();
        self.fault = FaultCode::None;
        self.recovery_armed = false;
        self.enable_armed = false;
        self.prepare_ticks = 0;
        self.prepare_stable_ticks = 0;
        self.state = State::Running;
        rtt_target::rprintln!(
            "RECOVER Error -> Running: yaw current={:.3} new_target={:.3}, pitch current={:.3} new_target={:.3} rad; PIDs reset",
            yaw_angle,
            self.yaw.target_angle_rad,
            pitch_angle,
            self.pitch.target_angle_rad
        );
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting | State::Error => {
                self.disable_all();
                self.yaw.target_rpm = 0.0;
                self.pitch.target_rpm = 0.0;
            }
            State::Prepare => {
                // The stick must not change the target while recentering.
                self.yaw.track(
                    YAW_REFERENCE_RAD,
                    DT_S,
                    PREPARE_MAX_RPM,
                    YAW_PREPARE_OUTPUT_LIMIT_RAW,
                );
                self.pitch.track(
                    PITCH_REFERENCE_RAD,
                    DT_S,
                    PREPARE_MAX_RPM,
                    PITCH_PREPARE_OUTPUT_LIMIT_RAW,
                );
            }
            State::Running => {
                self.apply_stick_targets();

                let yaw_target = self.yaw.target_angle_rad;
                self.yaw
                    .track(yaw_target, DT_S, POSITION_MAX_RPM, YAW_OUTPUT_LIMIT_RAW);
                let pitch_target = self.pitch.target_angle_rad;
                self.pitch
                    .track(pitch_target, DT_S, POSITION_MAX_RPM, PITCH_OUTPUT_LIMIT_RAW);
            }
        }
    }

    /// Left stick: horizontal moves yaw, vertical moves pitch. The target angle
    /// moves at a rate proportional to the stick deflection and stays where it
    /// is when the stick returns to the deadzone.
    fn apply_stick_targets(&mut self) {
        let Some(remote) = self.remote else {
            return;
        };
        let yaw_input = deadzone(remote.channels[2] as f32 / 660.0) * YAW_STICK_DIR;
        let pitch_input = deadzone(remote.channels[3] as f32 / 660.0) * PITCH_STICK_DIR;

        self.yaw.target_angle_rad += yaw_input * YAW_MAX_TARGET_RATE_RAD_S * DT_S;
        self.pitch.target_angle_rad += pitch_input * PITCH_MAX_TARGET_RATE_RAD_S * DT_S;
        self.yaw.clamp_target();
        self.pitch.clamp_target();
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];
        self.yaw.motor.command_voltage_raw =
            clamp_command(self.yaw.motor.command_voltage_raw, YAW_OUTPUT_LIMIT_RAW);
        self.pitch.motor.command_voltage_raw =
            clamp_command(self.pitch.motor.command_voltage_raw, PITCH_OUTPUT_LIMIT_RAW);

        // Signed 16-bit big-endian voltage counts in the slot for each motor ID.
        encode_voltage(
            &mut data,
            YAW_MOTOR_ID,
            self.yaw.motor.command_voltage_raw,
            YAW_OUTPUT_LIMIT_RAW,
        );
        encode_voltage(
            &mut data,
            PITCH_MOTOR_ID,
            self.pitch.motor.command_voltage_raw,
            PITCH_OUTPUT_LIMIT_RAW,
        );

        // Verify the encoding without flooding the 1 kHz loop.
        self.tx_log_ticks = self.tx_log_ticks.wrapping_add(1);
        if self.tx_log_ticks >= 1_000 {
            self.tx_log_ticks = 0;
            rtt_target::rprintln!(
                "CAN TX id=0x{:03X} data={:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                MOTOR_COMMAND_ID,
                data[0],
                data[1],
                data[2],
                data[3],
                data[4],
                data[5],
                data[6],
                data[7]
            );
        }

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// One tick of the stall and oscillation detectors for one axis.
fn update_axis_fault(axis: &mut Axis, side: FaultSide) -> Option<FaultCode> {
    // ---- Suspected stall -------------------------------------------------
    // A zero target (normal holding, or position maxout=0 while tuning the
    // speed loop) can never be a stall.
    let drive_in_same_direction = axis.target_rpm * axis.motor.command_voltage_raw > 0.0;
    let stalled = axis.target_rpm.abs() >= STALL_TARGET_RPM
        && drive_in_same_direction
        && axis.motor.command_voltage_raw.abs() >= STALL_OUTPUT_RAW
        && axis.motor.speed_rpm.abs() < STALL_SPEED_RPM;
    if stalled {
        axis.stall_ticks = axis.stall_ticks.saturating_add(1);
    } else {
        axis.stall_ticks = 0;
    }
    if axis.stall_ticks >= STALL_TICKS {
        return Some(match side {
            FaultSide::Yaw => FaultCode::YawStall,
            FaultSide::Pitch => FaultCode::PitchStall,
        });
    }

    // ---- Sustained high-frequency oscillation ----------------------------
    // Count complete half cycles from speed crossings of +/-OSC_AMPLITUDE_RPM.
    // A single overshoot only produces one crossing and times out below.
    let level = if axis.motor.speed_rpm > OSC_AMPLITUDE_RPM {
        1
    } else if axis.motor.speed_rpm < -OSC_AMPLITUDE_RPM {
        -1
    } else {
        axis.osc_level
    };

    if level != axis.osc_level {
        if axis.osc_level != 0 && axis.osc_half_ticks > 0 {
            let half_ticks = axis.osc_half_ticks.max(1) as f32;
            let freq_hz = 0.5 / (half_ticks * DT_S);
            axis.osc_freq_hz = freq_hz;
            if freq_hz >= OSC_MIN_FREQ_HZ {
                // This transition closes one half cycle; its duration is the
                // elapsed high-frequency time.
                axis.osc_over_ticks = axis.osc_over_ticks.saturating_add(axis.osc_half_ticks);
            } else {
                axis.osc_over_ticks = 0;
            }
        }
        axis.osc_level = level;
        axis.osc_half_ticks = 0;
    } else {
        axis.osc_half_ticks = axis.osc_half_ticks.saturating_add(1);
        // No new amplitude crossing for 200 ms: not a sustained oscillation.
        if axis.osc_half_ticks > OSC_TIMEOUT_TICKS {
            axis.osc_level = 0;
            axis.osc_half_ticks = 0;
            axis.osc_over_ticks = 0;
        }
    }

    if axis.osc_over_ticks >= OSC_CONFIRM_TICKS {
        return Some(match side {
            FaultSide::Yaw => FaultCode::YawOscillation,
            FaultSide::Pitch => FaultCode::PitchOscillation,
        });
    }

    None
}

/// Fold an angle into [-pi, pi]. This makes a raw-count crossing at 8191->0 a
/// small error instead of almost a full turn.
fn wrap_pi(angle: f32) -> f32 {
    if !angle.is_finite() {
        return 0.0;
    }
    let mut wrapped = angle % TAU;
    if wrapped > PI {
        wrapped -= TAU;
    } else if wrapped < -PI {
        wrapped += TAU;
    }
    wrapped
}

/// Normalize a stick channel to [-1, 1], remove the center deadzone, and rescale
/// so the response is continuous from 0 at the deadzone edge to 1 at full throw.
fn deadzone(value: f32) -> f32 {
    let value = if value.is_finite() {
        value.clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let magnitude = value.abs();
    if magnitude <= STICK_DEADZONE {
        0.0
    } else {
        let scaled = ((magnitude - STICK_DEADZONE) / (1.0 - STICK_DEADZONE)).min(1.0);
        scaled.copysign(value)
    }
}

/// Clamp a command to a finite symmetric range.
fn clamp_command(value: f32, limit: f32) -> f32 {
    let limit = limit.abs();
    if value.is_finite() {
        value.clamp(-limit, limit)
    } else {
        0.0
    }
}

/// Encode one signed voltage count into the 0x1FF group frame slot.
fn encode_voltage(data: &mut [u8; 8], motor_id: u8, value: f32, output_limit: f32) {
    if !(1..=4).contains(&motor_id) {
        return;
    }
    let slot = (motor_id as usize - 1) * 2;
    let limit = output_limit.min(PROTOCOL_LIMIT_RAW);
    // `f32::round` lives in std, so round to nearest manually in no_std.
    let counts = clamp_command(value, limit);
    let counts = if counts >= 0.0 {
        counts + 0.5
    } else {
        counts - 0.5
    };
    let counts = counts.clamp(-PROTOCOL_LIMIT_RAW, PROTOCOL_LIMIT_RAW) as i16;
    data[slot..slot + 2].copy_from_slice(&counts.to_be_bytes());
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!(
        "Gimbal control started: yaw id={} fb=0x{:03X} zero={} | pitch id={} fb=0x{:03X} zero={} | calibrated={}",
        YAW_MOTOR_ID,
        YAW_FEEDBACK_ID,
        YAW_ZERO_RAW,
        PITCH_MOTOR_ID,
        PITCH_FEEDBACK_ID,
        PITCH_ZERO_RAW,
        ZERO_CALIBRATED
    );
    if !ZERO_CALIBRATED {
        rtt_target::rprintln!(
            "CALIBRATION REQUIRED: hold the rig at the reference pose, read yaw/pitch motor.angle_raw below, set YAW_ZERO_RAW/PITCH_ZERO_RAW, then set ZERO_CALIBRATED=true and reflash"
        );
    }
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);
        // Keep `control` in memory and visible to the debugger/Tuning Studio.
        // Without this barrier the compiler may keep parts of it only in
        // registers in an optimized build, so the Live Watch tree can lose it.
        core::hint::black_box(&mut control);
        watchdog.pet();

        // About 5 Hz: enough to watch the state machine without flooding RTT.
        log_ticks += 1;
        if log_ticks >= 200 {
            log_ticks = 0;
            let (right_switch, left_switch, yaw_channel, pitch_channel) = match control.remote {
                Some(remote) => (
                    remote.right_switch,
                    remote.left_switch,
                    remote.channels[2],
                    remote.channels[3],
                ),
                None => (0, 0, 0, 0),
            };
            rtt_target::rprintln!(
                "state={:?} fault={:?} rs={} ls={} ch2={} ch3={} yaw: raw={} age={} motor_angle={:.3} joint={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0} cur={} temp={} stall={}ms osc={:.1}Hz/{}ms",
                control.state,
                control.fault,
                right_switch,
                left_switch,
                yaw_channel,
                pitch_channel,
                control.yaw.motor.angle_raw,
                control.yaw.feedback_age,
                control.yaw.motor.angle_rad,
                control.yaw.joint_angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.yaw.motor.command_voltage_raw,
                control.yaw.motor.current_raw,
                control.yaw.motor.temperature_c,
                control.yaw.stall_ticks,
                control.yaw.osc_freq_hz,
                control.yaw.osc_over_ticks,
            );
            rtt_target::rprintln!(
                "                        pitch: raw={} age={} motor_angle={:.3} joint={:.3} target={:.3} rpm={:.1} target_rpm={:.1} out={:.0} cur={} temp={} stall={}ms osc={:.1}Hz/{}ms prep={}ms/{}ms",
                control.pitch.motor.angle_raw,
                control.pitch.feedback_age,
                control.pitch.motor.angle_rad,
                control.pitch.joint_angle_rad,
                control.pitch.target_angle_rad,
                control.pitch.motor.speed_rpm,
                control.pitch.target_rpm,
                control.pitch.motor.command_voltage_raw,
                control.pitch.motor.current_raw,
                control.pitch.motor.temperature_c,
                control.pitch.stall_ticks,
                control.pitch.osc_freq_hz,
                control.pitch.osc_over_ticks,
                control.prepare_stable_ticks,
                control.prepare_ticks,
            );
        }
    }
}
