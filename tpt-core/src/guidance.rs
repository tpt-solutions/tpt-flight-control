//! Position / guidance controller (`spec.txt` §6.1, §18 Phase 1).
//!
//! Translates a [`PositionTarget`] into an [`AttitudeSetpoint`] for the
//! cascaded attitude controller. This is the navigation/guidance loop:
//! waypoint following, altitude hold, and heading control.
//!
//! Key detail for stability: when the vehicle tilts to accelerate
//! horizontally, the *vertical* component of thrust drops. The controller
//! therefore scales the collective thrust by `1/cos(tilt)` (tilt compensation)
//! so the vertical lift stays at the hover point and the altitude loop does
//! not destabilize. This is what makes position-hold tractable without a
//! dedicated vertical loop.

use crate::state::{AttitudeSetpoint, PositionTarget, VehicleState};
use libm::{asin, cos, sin, sqrt};
use tpt_math::clamp;

/// Gains and limits for the position controller.
#[derive(Debug, Clone, Copy)]
pub struct PositionGains {
    /// Horizontal position proportional gain (1/s^2).
    pub kp_xy: f64,
    /// Horizontal position derivative gain (1/s).
    pub kd_xy: f64,
    /// Vertical position proportional gain (1/s^2).
    pub kp_z: f64,
    /// Vertical position derivative gain (1/s).
    pub kd_z: f64,
    /// Heading hold gain (rad/s per rad of yaw error).
    pub kp_yaw: f64,
    /// Maximum commanded tilt (rad). Caps horizontal acceleration.
    pub max_tilt: f64,
    /// Collective thrust at hover (fraction, ~0.5).
    pub hover_thrust: f64,
}

impl Default for PositionGains {
    fn default() -> Self {
        Self {
            kp_xy: 0.5,
            kd_xy: 0.9,
            kp_z: 0.5,
            kd_z: 0.9,
            kp_yaw: 0.6,
            max_tilt: 0.35, // ~20 deg
            hover_thrust: 0.5,
        }
    }
}

/// Position / guidance controller.
#[derive(Debug, Clone, Copy)]
pub struct PositionController {
    gains: PositionGains,
}

impl PositionController {
    pub const fn new(gains: PositionGains) -> Self {
        Self { gains }
    }

    /// Compute the attitude setpoint that drives `state` toward `target`.
    ///
    /// `gravity` is the gravitational acceleration (m/s^2) used for the
    /// tilt/acceleration mapping and thrust compensation.
    pub fn update(&self, target: &PositionTarget, state: &VehicleState, gravity: f64) -> AttitudeSetpoint {
        // Horizontal desired acceleration (world NED, yaw ~ 0).
        let ax = self.gains.kp_xy * (target.x - state.position.x)
            + self.gains.kd_xy * (target.vx - state.velocity.x);
        let ay = self.gains.kp_xy * (target.y - state.position.y)
            + self.gains.kd_xy * (target.vy - state.velocity.y);

        // Clamp horizontal acceleration to the tilt limit.
        let a_max = gravity * sin(self.gains.max_tilt);
        let ax = clamp(ax, -a_max, a_max);
        let ay = clamp(ay, -a_max, a_max);

        // a_x = -g·sin(pitch), a_y = +g·sin(roll)  (see tpt-sim plant model).
        let pitch = asin(clamp(-ax / gravity, -self.gains.max_tilt, self.gains.max_tilt));
        let roll = asin(clamp(ay / gravity, -self.gains.max_tilt, self.gains.max_tilt));
        let tilt = sqrt(pitch * pitch + roll * roll);
        let cos_tilt = cos(tilt);

        // Vertical desired acceleration (NED `z` is down-positive, so an
        // upward correction has NEGATIVE a_z). Tilt-compensated thrust keeps
        // the vertical lift at the hover point: u = hover·(1 − a_z/g)/cos.
        let ez = target.z - state.position.z;
        let az = self.gains.kp_z * ez + self.gains.kd_z * (target.vz - state.velocity.z);
        let thrust = clamp(self.gains.hover_thrust * (1.0 - az / gravity) / cos_tilt, 0.0, 1.0);

        let yaw_rate = self.gains.kp_yaw * (target.yaw - state.attitude.2);

        AttitudeSetpoint {
            roll,
            pitch,
            yaw_rate,
            thrust,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::VehicleState;

    fn level_state() -> VehicleState {
        VehicleState::default()
    }

    #[test]
    fn hold_at_origin_is_level_and_hovering() {
        let c = PositionController::new(PositionGains::default());
        let sp = c.update(&PositionTarget::origin(), &level_state(), 9.81);
        assert!(sp.roll.abs() < 1e-9);
        assert!(sp.pitch.abs() < 1e-9);
        assert!((sp.thrust - 0.5).abs() < 1e-9);
    }

    #[test]
    fn north_target_commands_forward_pitch() {
        // Target is +x (north). Expect negative pitch (thrust vector tilts
        // north => -g·sin(pitch) > 0 => +x acceleration).
        let c = PositionController::new(PositionGains::default());
        let mut tgt = PositionTarget::origin();
        tgt.x = 5.0;
        let sp = c.update(&tgt, &level_state(), 9.81);
        assert!(sp.pitch < 0.0, "pitch={}", sp.pitch);
        assert!(sp.thrust > 0.5, "thrust={}", sp.thrust);
    }

    #[test]
    fn thrust_compensates_for_tilt() {
        // Large east target -> large roll -> thrust must rise above hover.
        let c = PositionController::new(PositionGains::default());
        let mut tgt = PositionTarget::origin();
        tgt.y = 50.0; // far east, clamped by max_tilt
        let sp = c.update(&tgt, &level_state(), 9.81);
        assert!(sp.roll > 0.2);
        assert!(sp.thrust > 0.5);
    }
}
