//! STM32 flight-controller board: binds the HAL to the
//! [`tpt_abstractions`] sensor/actuator/OS traits, and carries the runtime
//! actuator/sensor state used by the superloop.
//!
//! On the host build the IMU/GNSS/altimeter values are injectable so the
//! closed-loop supervisor can be exercised in unit tests; on real hardware
//! these would be populated by the external chip drivers (SPI/I2C).

use crate::hal::{GpioBank, Regs, RegisterInterface};
use core::convert::Infallible;
use tpt_abstractions::{
    FixType, GeoPosition, Gnss, Imu, Motor, RadarAltimeter,
    os::Scheduler,
};
use tpt_math::Vector3;
use tpt_protocols::antispoof::RaimMonitor;

/// Number of motors driven by the board (quad-X).
pub const MOTOR_COUNT: usize = 4;

/// RAIM position-consensus threshold (m) for flagging a jammed/spoofed GNSS
/// solution (§19.1).
const RAIM_THRESHOLD_M: f64 = 5.0;

/// A flight-controller board built on the [`Regs`] peripheral interface.
pub struct Stm32Board {
    /// Peripheral register interface (RAM mirror on host, MMIO on target).
    pub regs: Regs,
    /// Latest IMU accelerometer sample (body frame, m/s^2).
    pub imu_accel: Vector3<f64>,
    /// Latest IMU gyroscope sample (body frame, rad/s).
    pub imu_gyro: Vector3<f64>,
    /// Latest IMU magnetometer sample (body frame, uT).
    pub imu_mag: Vector3<f64>,
    /// Latest GNSS position solution.
    pub gnss_pos: GeoPosition,
    /// Latest GNSS velocity (local tangent plane, m/s).
    pub gnss_vel: Vector3<f64>,
    /// Current GNSS fix quality.
    pub fix: FixType,
    /// Radar-altimeter height above ground level (m).
    pub alt_agl: f64,
    /// Whether GNSS is jammed/spoofed (surfaced by the integrity monitor).
    pub gnss_compromised: bool,
    /// RAIM consistency monitor fed by the independent GNSS/constellation
    /// solutions; its alarm drives [`Gnss::is_jammed_or_spoofed`].
    raim: RaimMonitor,
    /// Most recent actuator commands written by the control loop.
    pub motors: [f64; MOTOR_COUNT],
}

impl Stm32Board {
    /// Create a board with sane defaults (telemetry TX-ready, level IMU).
    pub fn new() -> Self {
        Self {
            regs: Regs::default(),
            imu_accel: Vector3::new(0.0, 0.0, 9.81),
            imu_gyro: Vector3::zeros(),
            imu_mag: Vector3::new(25.0, 0.0, 40.0),
            gnss_pos: GeoPosition::new(0.0, 0.0, 0.0),
            gnss_vel: Vector3::zeros(),
            fix: FixType::Fix3D,
            alt_agl: 0.0,
            gnss_compromised: false,
            raim: RaimMonitor::new(RAIM_THRESHOLD_M),
            motors: [0.0; MOTOR_COUNT],
        }
    }

    /// Inject a stationary, level IMU sample (1 g along body z, no rotation).
    pub fn set_stationary_imu(&mut self) {
        self.imu_accel = Vector3::new(0.0, 0.0, 9.81);
        self.imu_gyro = Vector3::zeros();
    }

    /// Feed the RAIM monitor with independent GNSS/constellation position
    /// solutions (NED, m) and latch the jamming/spoofing alarm. The result is
    /// surfaced through [`Gnss::is_jammed_or_spoofed`]. `solutions` should hold
    /// at least two independent estimates (e.g. GPS, GLONASS, Galileo).
    pub fn update_integrity(&mut self, solutions: &[Vector3<f64>]) {
        self.raim.check(solutions);
        self.gnss_compromised = self.raim.is_alarmed();
    }

    /// Current RAIM alarm state (true if a solution was rejected as outlier).
    pub const fn raim_alarmed(&self) -> bool {
        self.raim.is_alarmed()
    }

    /// Apply the four normalized motor commands to the actuators.
    ///
    /// On real hardware this would write TIMx CCR registers for PWM; here it
    /// updates the board state and toggles the status GPIO as a heartbeat.
    pub fn apply_motors(&mut self, motors: &[f64; MOTOR_COUNT]) {
        self.motors = *motors;
        // Heartbeat: light PB0 when any rotor is spinning.
        let spinning = motors.iter().any(|&m| m > 0.01);
        if spinning {
            self.regs.gpio_set(GpioBank::B, 0x0001);
        } else {
            self.regs.gpio_clear(GpioBank::B, 0x0001);
        }
    }
}

impl Default for Stm32Board {
    fn default() -> Self {
        Self::new()
    }
}

impl Imu for Stm32Board {
    type Error = Infallible;
    fn read_accelerometer(&mut self) -> Result<Vector3<f64>, Self::Error> {
        Ok(self.imu_accel)
    }
    fn read_gyroscope(&mut self) -> Result<Vector3<f64>, Self::Error> {
        Ok(self.imu_gyro)
    }
    fn read_magnetometer(&mut self) -> Result<Vector3<f64>, Self::Error> {
        Ok(self.imu_mag)
    }
}

impl Gnss for Stm32Board {
    type Error = Infallible;
    fn read_position(&mut self) -> Result<GeoPosition, Self::Error> {
        Ok(self.gnss_pos)
    }
    fn read_velocity(&mut self) -> Result<Vector3<f64>, Self::Error> {
        Ok(self.gnss_vel)
    }
    fn get_fix_type(&self) -> FixType {
        self.fix
    }
    fn is_jammed_or_spoofed(&self) -> bool {
        // Driven by the RAIM consistency monitor (§19.1): a position solution
        // that is an outlier from the multi-constellation consensus, or any
        // externally-flagged compromise, is reported as jammed/spoofed.
        self.raim.is_alarmed() || self.gnss_compromised
    }
}

impl RadarAltimeter for Stm32Board {
    type Error = Infallible;
    fn read_altitude_agl(&mut self) -> Result<f64, Self::Error> {
        Ok(self.alt_agl)
    }
}

impl Scheduler for Stm32Board {
    type Error = Infallible;
    fn monotonic_micros(&self) -> Result<u64, Self::Error> {
        Ok(self.regs.monotonic_micros())
    }
}

/// A single PWM motor channel bound to a board slot. Allows the per-motor
/// [`Motor`] trait to be driven independently by the mixer/actuator layer.
pub struct MotorChannel<'a> {
    board: &'a mut Stm32Board,
    index: usize,
}

impl<'a> MotorChannel<'a> {
    pub fn new(board: &'a mut Stm32Board, index: usize) -> Self {
        debug_assert!(index < MOTOR_COUNT);
        Self { board, index }
    }
}

impl Motor for MotorChannel<'_> {
    type Error = Infallible;
    fn set_thrust(&mut self, command: f64) -> Result<(), Self::Error> {
        self.board.motors[self.index] = command.clamp(0.0, 1.0);
        Ok(())
    }
    fn current_thrust(&self) -> f64 {
        self.board.motors[self.index]
    }
    fn is_healthy(&self) -> bool {
        self.board.motors[self.index].is_finite()
    }
}

/// Convenience: enable every peripheral the supervisor depends on.
pub fn bring_up(board: &mut Stm32Board) {
    crate::hal::init_clocks_and_io(&mut board.regs);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_trait_contract() {
        let mut b = Stm32Board::new();
        b.set_stationary_imu();
        // Stationary, level: 1 g along body z, no rotation, quiet mag.
        assert!((b.read_accelerometer().unwrap().z - 9.81).abs() < 1e-9);
        assert_eq!(b.read_gyroscope().unwrap(), Vector3::zeros());
        assert_eq!(b.read_magnetometer().unwrap(), Vector3::new(25.0, 0.0, 40.0));
    }

    #[test]
    fn gnss_trait_contract() {
        let mut b = Stm32Board::new();
        b.gnss_pos = GeoPosition::new(37.0, -122.0, 10.0);
        assert_eq!(b.read_position().unwrap(), GeoPosition::new(37.0, -122.0, 10.0));
        assert_eq!(b.get_fix_type(), FixType::Fix3D);
        // No integrity input yet -> not flagged.
        assert!(!b.is_jammed_or_spoofed());
    }

    #[test]
    fn gnss_jamming_flagged_by_raim() {
        let mut b = Stm32Board::new();
        // Two consistent solutions (GPS + GLONASS) -> no alarm.
        let good = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.1, -0.1, 0.0),
        ];
        b.update_integrity(&good);
        assert!(!b.is_jammed_or_spoofed());

        // A spoofed outlier solution is rejected and latched.
        let spoofed = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.1, -0.1, 0.0),
            Vector3::new(50.0, 0.0, 0.0),
        ];
        b.update_integrity(&spoofed);
        assert!(b.is_jammed_or_spoofed());
        assert!(b.raim_alarmed());
    }

    #[test]
    fn motor_channel_trait_contract() {
        let mut b = Stm32Board::new();
        let mut m0 = MotorChannel::new(&mut b, 0);
        // Over-range command is clamped to [0, 1].
        m0.set_thrust(1.5).unwrap();
        assert!((m0.current_thrust() - 1.0).abs() < 1e-9);
        assert!(m0.is_healthy());
        m0.set_thrust(-0.5).unwrap();
        assert!((m0.current_thrust() - 0.0).abs() < 1e-9);
    }

    #[test]
    fn scheduler_trait_contract() {
        let b = Stm32Board::new();
        let t0 = b.monotonic_micros().unwrap();
        // Time is monotonic and non-negative.
        assert!(t0 < b.monotonic_micros().unwrap() || t0 == 0);
    }
}
