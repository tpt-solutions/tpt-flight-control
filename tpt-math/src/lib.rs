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

/// Kani proof harnesses (`cargo kani -p tpt-math`, §16 / REQ-M-7).
///
/// The `kani` crate is injected automatically by the Kani compiler when this
/// crate is built with `cargo kani`; it does not appear in `Cargo.toml`.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// `clamp` always returns a value inside `[lo, hi]` when `lo <= hi`.
    #[kani::proof]
    fn clamp_bounded() {
        let x: f64 = kani::any();
        let lo: f64 = kani::any();
        let hi: f64 = kani::any();
        kani::assume(x.is_finite() && lo.is_finite() && hi.is_finite());
        kani::assume(lo <= hi);
        let y = clamp(x, lo, hi);
        assert!(y >= lo && y <= hi);
    }

    /// `clamp` is the identity for inputs already inside `[lo, hi]`.
    #[kani::proof]
    fn clamp_identity_inside_range() {
        let x: f64 = kani::any();
        let lo: f64 = kani::any();
        let hi: f64 = kani::any();
        kani::assume(x.is_finite() && lo.is_finite() && hi.is_finite());
        kani::assume(lo <= x && x <= hi);
        assert_eq!(clamp(x, lo, hi), x);
    }

    /// `deadzone` never increases the magnitude of its input, for any
    /// non-negative band width.
    #[kani::proof]
    fn deadzone_shrinks_magnitude() {
        let x: f64 = kani::any();
        let width: f64 = kani::any();
        kani::assume(x.is_finite());
        kani::assume(width.is_finite() && width >= 0.0);
        let y = deadzone(x, width);
        assert!(y.abs() <= x.abs());
    }

    /// `deadzone` collapses to exactly zero for any input inside the band.
    #[kani::proof]
    fn deadzone_zero_inside_band() {
        let x: f64 = kani::any();
        let width: f64 = kani::any();
        kani::assume(x.is_finite());
        kani::assume(width.is_finite() && width >= 0.0);
        kani::assume(x.abs() <= width);
        assert_eq!(deadzone(x, width), 0.0);
    }
}
