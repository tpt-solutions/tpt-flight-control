//! Quadcopter **X** configuration mixer (`spec.txt` §9.2).
//!
//! Coordinate frame: body `x` = forward, `y` = right, `z` = up. Motor layout:
//!
//! ```text
//!      front
//!        M1(x+,y+)   M3(x+,y-)
//!           \         /
//!            \       /
//!             +-----+   (x forward)
//!            /       \
//!           /         \
//!        M4(x-,y+)   M2(x-,y-)
//!      rear
//! ```
//!
//! The allocation is an orthonormal basis on the four actuators, so `roll`,
//! `pitch`, `yaw`, and collective `thrust` are fully decoupled. The same
//! `ROLL`/`PITCH`/`YAW` vectors are used by the `tpt-sim` plant (see
//! `plant.rs`), which is what makes the closed loop self-consistent.
//!
//! Spin assignment (reaction torque about `z`): `M1` and `M2` spin CW, `M3`
//! and `M4` spin CCW.

use crate::{ControlCommand, MotorMixer};
use tpt_math::clamp;

/// Scale applied to the moment commands before allocation. The attitude
/// controller issues large moment demands; dividing them here keeps the
/// resulting per-motor deflection within the collective-thrust headroom so
/// the `[0, 1]` clamp never *distorts* the summed thrust (which would make
/// a low-thrust command sum to >1 and launch the vehicle). The orthonormal
/// basis still keeps the per-axis torques independent.
const MOMENT_SCALE: f64 = 7.0;

/// Roll allocation vector (right motors `+y` vs left `-y`).
pub const ROLL: [f64; 4] = [1.0, -1.0, -1.0, 1.0];
/// Pitch allocation vector (front motors `+x` vs rear `-x`).
pub const PITCH: [f64; 4] = [1.0, -1.0, 1.0, -1.0];
/// Yaw allocation vector (diagonal CW pair `M1,M2` vs CCW pair `M3,M4`).
pub const YAW: [f64; 4] = [1.0, 1.0, -1.0, -1.0];

/// Quadcopter X mixer (4 motors).
#[derive(Debug, Clone, Copy, Default)]
pub struct QuadXMixer;

impl QuadXMixer {
    pub const MOTOR_COUNT: usize = 4;

    /// Mix without requiring a `MotorMixer` trait object.
    ///
    /// `m[i] = (thrust + (ROLL[i]*roll + PITCH[i]*pitch + YAW[i]*yaw) / MOMENT_SCALE) / 4`.
    pub fn mix_into(cmd: &ControlCommand, out: &mut [f64]) {
        debug_assert!(out.len() >= Self::MOTOR_COUNT);
        let t = cmd.thrust;
        let r = cmd.roll / MOMENT_SCALE;
        let p = cmd.pitch / MOMENT_SCALE;
        let y = cmd.yaw / MOMENT_SCALE;
        out[0] = (t + ROLL[0] * r + PITCH[0] * p + YAW[0] * y) * 0.25; // M1 front-right
        out[1] = (t + ROLL[1] * r + PITCH[1] * p + YAW[1] * y) * 0.25; // M2 rear-left
        out[2] = (t + ROLL[2] * r + PITCH[2] * p + YAW[2] * y) * 0.25; // M3 front-left
        out[3] = (t + ROLL[3] * r + PITCH[3] * p + YAW[3] * y) * 0.25; // M4 rear-right
        for v in out.iter_mut() {
            *v = clamp(*v, 0.0, 1.0);
        }
    }
}

impl MotorMixer for QuadXMixer {
    fn motor_count(&self) -> usize {
        Self::MOTOR_COUNT
    }

    fn mix(&self, cmd: &ControlCommand, out: &mut [f64]) {
        Self::mix_into(cmd, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mix(cmd: ControlCommand) -> [f64; 4] {
        let mut out = [0.0; 4];
        QuadXMixer::mix_into(&cmd, &mut out);
        out
    }

    #[test]
    fn pure_thrust_balances_all_motors() {
        let m = mix(ControlCommand {
            thrust: 0.8,
            ..Default::default()
        });
        for v in m {
            assert!((v - 0.2).abs() < 1e-12, "v={v}");
        }
    }

    #[test]
    fn roll_differentiates_left_right() {
        // Positive roll -> right motors (M1, M4) increase, left (M2, M3) drop.
        let m = mix(ControlCommand {
            thrust: 0.5,
            roll: 0.2,
            ..Default::default()
        });
        assert!(m[0] > 0.125 && m[3] > 0.125);
        assert!(m[1] < 0.125 && m[2] < 0.125);
        assert!((m[0] - m[3]).abs() < 1e-12);
        assert!((m[1] - m[2]).abs() < 1e-12);
    }

    #[test]
    fn yaw_differentiates_spin_directions() {
        let m = mix(ControlCommand {
            thrust: 0.5,
            yaw: 0.2,
            ..Default::default()
        });
        // CW pair (M1, M2) increase, CCW pair (M3, M4) decrease.
        assert!(m[0] > 0.125 && m[1] > 0.125);
        assert!(m[2] < 0.125 && m[3] < 0.125);
    }

    #[test]
    fn outputs_clamped() {
        let m = mix(ControlCommand {
            thrust: 2.0,
            ..Default::default()
        });
        for v in m {
            assert!((0.0..=1.0).contains(&v));
        }
    }
}
