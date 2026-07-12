//! # tpt-mixer
//!
//! Actuator mixing and propulsion allocation (`spec.txt` §9). Translates
//! desired forces and moments into individual actuator commands.
//!
//! - [`quad_x`] — quadcopter **X** mixer (Phase 0).
//! - [`dep`] — distributed electric propulsion fault-tolerant mixer (Phase 3).
//! - [`tiltrotor`] — tilt-rotor hover-to-cruise transition logic (Phase 3).

#![no_std]
#![forbid(unsafe_code)]

#[cfg(feature = "mixer-dep")]
pub mod dep;
#[cfg(feature = "mixer-quad")]
pub mod quad_x;
#[cfg(feature = "mixer-dep")]
pub use dep::{DepMixer, Rotor};
#[cfg(feature = "mixer-tilt")]
pub mod tiltrotor;
#[cfg(feature = "mixer-tilt")]
pub use tiltrotor::{TiltPhase, TiltRotor};

#[cfg(feature = "mixer-quad")]
pub use quad_x::QuadXMixer;

/// A motor-mixing allocator: maps a force/moment command to per-motor outputs.
pub trait MotorMixer {
    /// Number of motors this mixer allocates.
    fn motor_count(&self) -> usize;
    /// Write per-motor commands (normalized `[0, 1]`) into `out`.
    fn mix(&self, cmd: &ControlCommand, out: &mut [f64]);
}

/// Force / moment command produced by the control laws (body frame).
///
/// - `thrust` — collective normalized thrust `[0, 1]`.
/// - `roll` / `pitch` / `yaw` — body moment commands (sign per mixer frame).
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlCommand {
    pub thrust: f64,
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}
