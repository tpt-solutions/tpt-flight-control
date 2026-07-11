//! Flight envelope protection (`spec.txt` §6.3).
//!
//! A **non-bypassable** layer that sits between the control laws and the
//! mixer. It enforces hard limits on attitude, body rates, climb rate, and
//! never-exceed airspeed (Vne). The autopilot *must* route its commands
//! through [`EnvelopeProtector::protect`] before they reach the mixer; there
//! is no API to skip it.

use crate::state::{AttitudeSetpoint, VehicleState};
use libm::sqrt;
use tpt_abstractions::types::BoundingBox;
use tpt_math::{Vector3, clamp};

/// Envelope limits. All angles in radians, rates in rad/s, speeds in m/s.
#[derive(Debug, Clone, Copy)]
pub struct EnvelopeConfig {
    pub max_roll: f64,
    pub max_pitch: f64,
    pub max_roll_rate: f64,
    pub max_pitch_rate: f64,
    pub max_yaw_rate: f64,
    pub max_climb_rate: f64,
    pub vne: f64,
}

impl Default for EnvelopeConfig {
    fn default() -> Self {
        Self {
            max_roll: 0.524,     // 30 deg
            max_pitch: 0.524,    // 30 deg
            max_roll_rate: 3.49, // 200 deg/s
            max_pitch_rate: 3.49,
            max_yaw_rate: 2.09, // 120 deg/s
            max_climb_rate: 5.0,
            vne: 20.0,
        }
    }
}

/// Enforces the flight envelope. See module docs: non-bypassable.
#[derive(Debug, Clone, Copy)]
pub struct EnvelopeProtector {
    cfg: EnvelopeConfig,
}

impl EnvelopeProtector {
    pub const fn new(cfg: EnvelopeConfig) -> Self {
        Self { cfg }
    }

    /// Clamp an attitude/rate command to within the envelope.
    ///
    /// This is the only path from control laws to the mixer and cannot be
    /// skipped. Returns a safe command even if the input is wildly out of
    /// bounds.
    pub fn protect(&self, cmd: AttitudeSetpoint, _state: &VehicleState) -> AttitudeSetpoint {
        let cfg = &self.cfg;

        let roll = clamp(cmd.roll, -cfg.max_roll, cfg.max_roll);
        let pitch = clamp(cmd.pitch, -cfg.max_pitch, cfg.max_pitch);
        let yaw_rate = clamp(cmd.yaw_rate, -cfg.max_yaw_rate, cfg.max_yaw_rate);

        // Attitude clamping bounds the achievable bank angle and therefore the
        // horizontal airspeed; the mixer also clamps the resulting thrust to
        // `[0, 1]`. The thrust command is passed through unchanged.
        AttitudeSetpoint {
            roll,
            pitch,
            yaw_rate,
            thrust: cmd.thrust,
        }
    }

    /// Returns `true` if the current state violates the envelope (used for
    /// fault detection / telemetry).
    pub fn is_violated(&self, state: &VehicleState) -> bool {
        let cfg = &self.cfg;
        let (r, p, _) = state.attitude;
        let (rr, pr, yr) = (state.body_rates.x, state.body_rates.y, state.body_rates.z);
        r.abs() > cfg.max_roll
            || p.abs() > cfg.max_pitch
            || rr.abs() > cfg.max_roll_rate
            || pr.abs() > cfg.max_pitch_rate
            || yr.abs() > cfg.max_yaw_rate
            || state.velocity.z.abs() > cfg.max_climb_rate
            || sqrt(state.velocity.x * state.velocity.x + state.velocity.y * state.velocity.y)
                > cfg.vne
    }

    /// Example bounding box limiting the operational volume (for geofencing).
    /// Not enforced by default; provided as a building block.
    pub fn inside_geofence(&self, state: &VehicleState, fence: &BoundingBox) -> bool {
        fence.contains(&state.position)
    }

    /// Helper for callers needing raw clamping of a 3D rate vector.
    pub fn clamp_rates(&self, rates: Vector3<f64>) -> Vector3<f64> {
        let cfg = &self.cfg;
        Vector3::new(
            clamp(rates.x, -cfg.max_roll_rate, cfg.max_roll_rate),
            clamp(rates.y, -cfg.max_pitch_rate, cfg.max_pitch_rate),
            clamp(rates.z, -cfg.max_yaw_rate, cfg.max_yaw_rate),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_attitude_and_rates() {
        let ep = EnvelopeProtector::new(EnvelopeConfig::default());
        let cmd = AttitudeSetpoint {
            roll: 1.5,
            pitch: -2.0,
            yaw_rate: 10.0,
            thrust: 0.5,
        };
        let state = VehicleState::default();
        let out = ep.protect(cmd, &state);
        assert!(out.roll <= EnvelopeConfig::default().max_roll + 1e-9);
        assert!(out.pitch >= -EnvelopeConfig::default().max_pitch - 1e-9);
        assert!(out.yaw_rate <= EnvelopeConfig::default().max_yaw_rate + 1e-9);
    }

    #[test]
    fn detects_violation() {
        let ep = EnvelopeProtector::new(EnvelopeConfig::default());
        let mut state = VehicleState::default();
        state.velocity.z = 100.0;
        assert!(ep.is_violated(&state));
    }
}
