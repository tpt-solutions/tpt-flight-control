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

    // --- Prognostics extension (§16.3, `tpt-core::prognostics`) ---
    // Defaulted so existing implementors (`tpt-backend-bare-metal`) keep
    // compiling; backends override when they can measure the quantities.

    /// Measured rotational speed (rpm) (default 0 = unknown).
    fn rpm(&self) -> f64 {
        0.0
    }
    /// Winding / electronics temperature (°C) (default 0 = cold/unknown).
    fn temperature_c(&self) -> f64 {
        0.0
    }
    /// Normalized mechanical load in `[0, 1]` (default 0 = unknown).
    fn load(&self) -> f64 {
        0.0
    }
}

/// A control surface (elevator, aileron, rudder, flap, ...).
pub trait ControlSurface {
    type Error;
    /// Set the surface deflection in radians (signed).
    fn set_deflection(&mut self, radians: f64) -> Result<(), Self::Error>;
    /// Current deflection in radians.
    fn deflection(&self) -> f64;
}
