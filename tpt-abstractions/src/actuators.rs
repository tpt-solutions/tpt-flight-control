//! Actuator abstraction traits (`spec.txt` §5.3).

/// A motor / rotor actuator. `command` is a normalized thrust in `[0, 1]`.
pub trait Motor {
    type Error;
    /// Set the normalized thrust command.
    fn set_thrust(&mut self, command: f64) -> Result<(), Self::Error>;
    /// Current measured/estimated thrust in `[0, 1]`, if available.
    fn current_thrust(&self) -> f64;
    /// Whether the actuator is healthy (e.g. not failed / stalled).
    fn is_healthy(&self) -> bool;
}

/// A control surface (elevator, aileron, rudder, flap, ...).
pub trait ControlSurface {
    type Error;
    /// Set the surface deflection in radians (signed).
    fn set_deflection(&mut self, radians: f64) -> Result<(), Self::Error>;
    /// Current deflection in radians.
    fn deflection(&self) -> f64;
}
