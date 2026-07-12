//! # tpt-math
//!
//! Verified-friendly, `#![no_std]` math library for TPT Flight Control
//! (`spec.txt` §15.3). Provides the linear-algebra and estimation primitives
//! used by every other crate.
//!
//! The crate is a thin, opinionated wrapper over [`nalgebra`] configured for
//! `no_std` (the `libm` feature supplies transcendental functions). Keeping
//! the math in one small, dependency-light crate is what makes the core
//! amenable to formal verification with Kani / Creusot (§3.6, §16).
//!
//! ## Modules
//! - [`vector`] / [`quaternion`] — re-exported, stack-allocated types.
//! - [`kalman`] — discrete Kalman filter primitives (scalar + N-dimensional).
//! - [`angles`] — angle wrapping / limiting helpers used across the stack.

#![no_std]
#![forbid(unsafe_code)]

pub use nalgebra::{Matrix3, Quaternion, SMatrix, SVector, UnitQuaternion, Vector2, Vector3};

pub mod angles;
pub mod kalman;

pub mod prelude {
    //! Convenient re-exports.
    pub use crate::{Matrix3, Quaternion, SMatrix, SVector, UnitQuaternion, Vector3};
    pub use crate::{angles, kalman};
}

/// Clamp `x` to the inclusive range `[lo, hi]`.
#[inline]
pub const fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// Symmetric dead-zone: returns `0.0` when `|x| <= width`, otherwise
/// `x - sign(x)*width`. Useful for stick/centering hysteresis.
#[inline]
pub fn deadzone(x: f64, width: f64) -> f64 {
    if x.abs() <= width {
        0.0
    } else {
        x - x.signum() * width
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_bounds() {
        assert_eq!(clamp(5.0, 0.0, 1.0), 1.0);
        assert_eq!(clamp(-5.0, 0.0, 1.0), 0.0);
        assert_eq!(clamp(0.5, 0.0, 1.0), 0.5);
    }

    #[test]
    fn deadzone_center() {
        assert_eq!(deadzone(0.01, 0.05), 0.0);
        assert!((deadzone(0.2, 0.05) - 0.15).abs() < 1e-9);
    }
}
