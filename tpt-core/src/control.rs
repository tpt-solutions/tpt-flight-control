//! Cascaded attitude controller (`spec.txt` §6.1).
//!
//! Implements the attitude portion of the cascaded architecture: an outer
//! angle loop produces rate commands that feed an inner rate loop. The output
//! is a body-frame moment command `(roll, pitch, yaw)` that the mixer converts
//! into actuator commands.

use crate::pid::{Pid, PidConfig};
use crate::state::{AttitudeSetpoint, VehicleState};

/// Body-frame moment command produced by the attitude controller.
#[derive(Debug, Clone, Copy, Default)]
pub struct MomentCommand {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Cascaded (angle → rate) attitude controller.
#[derive(Debug, Clone)]
pub struct AttitudeController {
    roll_angle: Pid,
    pitch_angle: Pid,
    yaw_rate: Pid,
    roll_rate: Pid,
    pitch_rate: Pid,
}

impl AttitudeController {
    /// Build with sensible default gains for a small quadcopter.
    pub fn new() -> Self {
        let angle = PidConfig::new(6.0, 2.0, 0.2);
        let rate = PidConfig::new(0.15, 0.05, 0.005);
        Self {
            roll_angle: Pid::new(angle),
            pitch_angle: Pid::new(angle),
            yaw_rate: Pid::new(PidConfig::new(3.0, 1.0, 0.1)),
            roll_rate: Pid::new(rate),
            pitch_rate: Pid::new(rate),
        }
    }

    /// Reset all PID state (e.g. on arming).
    pub fn reset(&mut self) {
        self.roll_angle.reset();
        self.pitch_angle.reset();
        self.yaw_rate.reset();
        self.roll_rate.reset();
        self.pitch_rate.reset();
    }

    /// Update the controller for one inner-loop step.
    ///
    /// `dt` is the inner-loop period (typically 1 ms). Body rates are taken
    /// from `state.body_rates` in `(x=roll, y=pitch, z=yaw)` order.
    pub fn update(
        &mut self,
        sp: &AttitudeSetpoint,
        state: &VehicleState,
        dt: f64,
    ) -> MomentCommand {
        let (roll, pitch, _) = state.attitude;
        let rates = state.body_rates;

        // Outer angle loops -> rate setpoints.
        let roll_rate_cmd = self.roll_angle.update(sp.roll - roll, roll, dt);
        let pitch_rate_cmd = self.pitch_angle.update(sp.pitch - pitch, pitch, dt);

        // Inner rate loops -> moments.
        let roll_moment = self.roll_rate.update(roll_rate_cmd - rates.x, rates.x, dt);
        let pitch_moment = self.pitch_rate.update(pitch_rate_cmd - rates.y, rates.y, dt);
        let yaw_moment = self.yaw_rate.update(sp.yaw_rate - rates.z, rates.z, dt);

        MomentCommand {
            roll: roll_moment,
            pitch: pitch_moment,
            yaw: yaw_moment,
        }
    }
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::VehicleState;

    #[test]
    fn zero_error_yields_zero_moment() {
        let mut c = AttitudeController::new();
        let sp = AttitudeSetpoint {
            roll: 0.0,
            pitch: 0.0,
            yaw_rate: 0.0,
            thrust: 0.5,
        };
        let state = VehicleState::default();
        let m = c.update(&sp, &state, 0.001);
        assert!(m.roll.abs() < 1e-9);
        assert!(m.pitch.abs() < 1e-9);
        assert!(m.yaw.abs() < 1e-9);
    }

    #[test]
    fn responds_to_attitude_error() {
        let mut c = AttitudeController::new();
        let sp = AttitudeSetpoint {
            roll: 0.2,
            pitch: 0.0,
            yaw_rate: 0.0,
            thrust: 0.5,
        };
        let mut state = VehicleState::default();
        // Vehicle is level; expect a positive roll moment command.
        let m = c.update(&sp, &state, 0.001);
        assert!(m.roll > 0.0, "roll moment = {}", m.roll);
        // Apply the moment to the (mock) body rate so the loop can react.
        state.body_rates.x += m.roll * 0.001;
        let m2 = c.update(&sp, &state, 0.001);
        // Rate feedback should reduce the demanded moment over time.
        assert!(m2.roll <= m.roll + 1e-9, "{} vs {}", m2.roll, m.roll);
    }
}
