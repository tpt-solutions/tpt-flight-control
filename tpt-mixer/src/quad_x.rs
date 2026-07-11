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
//! `M1` and `M4` spin CW (negative yaw reaction), `M3` and `M2` spin CCW.

use crate::{ControlCommand, MotorMixer};
use tpt_math::clamp;

/// Quadcopter X mixer (4 motors).
#[derive(Debug, Clone, Copy, Default)]
pub struct QuadXMixer;

impl QuadXMixer {
    pub const MOTOR_COUNT: usize = 4;

    /// Mix without requiring a `MotorMixer` trait object.
    pub fn mix_into(cmd: &ControlCommand, out: &mut [f64]) {
        debug_assert!(out.len() >= Self::MOTOR_COUNT);
        let t = cmd.thrust;
        let r = cmd.roll;
        let p = cmd.pitch;
        let y = cmd.yaw;
        out[0] = (t + r - p - y) * 0.25; // M1 front-right (CW)
        out[1] = (t - r + p + y) * 0.25; // M2 rear-left  (CCW)
        out[2] = (t - r - p + y) * 0.25; // M3 front-left (CCW)
        out[3] = (t + r + p - y) * 0.25; // M4 rear-right (CW)
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
        let m = mix(ControlCommand { thrust: 0.8, ..Default::default() });
        for v in m {
            assert!((v - 0.2).abs() < 1e-12, "v={v}");
        }
    }

    #[test]
    fn roll_differentiates_left_right() {
        // Positive roll -> right motors (M1, M4) increase, left (M2, M3) drop.
        let m = mix(ControlCommand { thrust: 0.5, roll: 0.2, ..Default::default() });
        assert!(m[0] > 0.125 && m[3] > 0.125);
        assert!(m[1] < 0.125 && m[2] < 0.125);
        assert!((m[0] - m[3]).abs() < 1e-12);
        assert!((m[1] - m[2]).abs() < 1e-12);
    }

    #[test]
    fn yaw_differentiates_spin_directions() {
        let m = mix(ControlCommand { thrust: 0.5, yaw: 0.2, ..Default::default() });
        // CW motors (M1, M4) decrease, CCW (M2, M3) increase.
        assert!(m[1] > 0.125 && m[2] > 0.125);
        assert!(m[0] < 0.125 && m[3] < 0.125);
    }

    #[test]
    fn outputs_clamped() {
        let m = mix(ControlCommand { thrust: 2.0, ..Default::default() });
        for v in m {
            assert!((0.0..=1.0).contains(&v));
        }
    }
}
