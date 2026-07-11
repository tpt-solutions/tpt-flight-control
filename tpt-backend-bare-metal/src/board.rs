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

/// Number of motors driven by the board (quad-X).
pub const MOTOR_COUNT: usize = 4;

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
            motors: [0.0; MOTOR_COUNT],
        }
    }

    /// Inject a stationary, level IMU sample (1 g along body z, no rotation).
    pub fn set_stationary_imu(&mut self) {
        self.imu_accel = Vector3::new(0.0, 0.0, 9.81);
        self.imu_gyro = Vector3::zeros();
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
        self.gnss_compromised
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
