//! Discrete Kalman filter primitives.
//!
//! These are intentionally small, allocation-free, and written so they can be
//! formally verified (§3.6, §16). Two flavors are provided:
//!
//! - [`ScalarKalman`] — a single-scalar filter for simple signals.
//! - [`KalmanFilter`] — a generic `S`-state / `M`-measurement discrete Kalman
//!   filter built on stack-allocated [`SMatrix`]/[`SVector`].

use crate::{SMatrix, SVector};

/// Single-scalar discrete Kalman filter.
///
/// State model: `x = x + w`, measurement: `z = x + v`, with scalar process
/// and measurement noise variances `q` and `r`.
#[derive(Debug, Clone, Copy)]
pub struct ScalarKalman {
    x: f64,
    p: f64,
    q: f64,
    r: f64,
}

impl ScalarKalman {
    /// Create a filter with the given initial estimate, covariance, and noises.
    pub fn new(x0: f64, p0: f64, q: f64, r: f64) -> Self {
        Self { x: x0, p: p0, q, r }
    }

    /// Advance the time update (no control input, identity transition).
    pub fn predict(&mut self) {
        self.p += self.q;
    }

    /// Incorporate a measurement `z`, returning the corrected estimate.
    pub fn update(&mut self, z: f64) -> f64 {
        let k = self.p / (self.p + self.r);
        self.x += k * (z - self.x);
        self.p *= 1.0 - k;
        self.x
    }

    /// Current state estimate.
    pub fn estimate(&self) -> f64 {
        self.x
    }
}

/// Generic discrete Kalman filter over `S` states and `M` measurements.
///
/// All storage is stack-resident; no heap allocation occurs.
#[derive(Debug, Clone, Copy)]
pub struct KalmanFilter<const S: usize, const M: usize> {
    x: SVector<f64, S>,
    p: SMatrix<f64, S, S>,
    q: SMatrix<f64, S, S>,
    r: SMatrix<f64, M, M>,
}

impl<const S: usize, const M: usize> KalmanFilter<S, M> {
    /// Create a filter from initial state, covariance, process noise `Q`, and
    /// measurement noise `R`.
    pub fn new(
        x0: SVector<f64, S>,
        p0: SMatrix<f64, S, S>,
        q: SMatrix<f64, S, S>,
        r: SMatrix<f64, M, M>,
    ) -> Self {
        Self { x: x0, p: p0, q, r }
    }

    /// Time update with state-transition matrix `f` (`x <- f x`).
    pub fn predict(&mut self, f: &SMatrix<f64, S, S>) {
        self.x = f * self.x;
        self.p = f * self.p * f.transpose() + self.q;
    }

    /// Measurement update with observation matrix `h` and measurement `z`.
    ///
    /// Returns `None` if the innovation covariance is singular.
    pub fn update(&mut self, h: &SMatrix<f64, M, S>, z: &SVector<f64, M>) -> Option<()> {
        // S = H P H^T + R
        let s = h * self.p * h.transpose() + self.r;
        let s_inv = s.try_inverse()?;
        // K = P H^T S^-1
        let k = self.p * h.transpose() * s_inv;
        // x = x + K (z - H x)
        let y = z - h * self.x;
        self.x += k * y;
        // P = (I - K H) P
        let i = SMatrix::<f64, S, S>::identity();
        let new_p = (i - k * *h) * self.p;
        self.p = new_p;
        Some(())
    }

    /// Current state estimate.
    pub fn state(&self) -> &SVector<f64, S> {
        &self.x
    }

    /// Current estimate covariance.
    pub fn covariance(&self) -> &SMatrix<f64, S, S> {
        &self.p
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_converges() {
        let mut k = ScalarKalman::new(0.0, 1.0, 1e-3, 0.1);
        for _ in 0..50 {
            k.predict();
            k.update(1.0);
        }
        assert!((k.estimate() - 1.0).abs() < 0.05);
    }

    #[test]
    fn nd_position_velocity_filters_noise() {
        // State: [position, velocity], measurement: position only.
        let x0 = SVector::<f64, 2>::zeros();
        let p0 = SMatrix::<f64, 2, 2>::identity();
        let q = SMatrix::<f64, 2, 2>::from_row_slice(&[1e-4, 0.0, 0.0, 1e-4]);
        let r = SMatrix::<f64, 1, 1>::from_row_slice(&[0.5]);
        let mut kf = KalmanFilter::<2, 1>::new(x0, p0, q, r);

        let dt = 0.01;
        let f = SMatrix::<f64, 2, 2>::from_row_slice(&[1.0, dt, 0.0, 1.0]);
        let h = SMatrix::<f64, 1, 2>::from_row_slice(&[1.0, 0.0]);

        let mut truth = 0.0f64;
        for step in 0..1000 {
            truth += dt; // velocity = 1.0
            kf.predict(&f);
            // Deterministic, slowly varying observation error (kept small).
            let z = SVector::<f64, 1>::new(truth + (step as f64 * 0.013).sin() * 0.1);
            let _ = kf.update(&h, &z);
        }
        assert!(
            (kf.state()[0] - truth).abs() < 0.05,
            "pos err {}",
            kf.state()[0] - truth
        );
        assert!(
            (kf.state()[1] - 1.0).abs() < 0.05,
            "vel err {}",
            kf.state()[1] - 1.0
        );
    }
}

/// Kani proof harnesses (`cargo kani -p tpt-math`, §16 / REQ-M-7).
///
/// These proofs live inside `kalman.rs` (rather than a top-level
/// `kani_proofs` module) so they can reach the private `x`/`p` fields and
/// check the estimator's internal invariants directly, not just its public
/// `estimate()`/`state()` outputs.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// The scalar filter's covariance never goes negative across a
    /// predict/update cycle, and an update never *increases* it, for any
    /// admissible (non-negative process noise, positive measurement noise)
    /// parameters. A negative covariance is not just wrong but a warning
    /// sign of numerical blow-up, so this is a basic sanity envelope for
    /// every downstream consumer of `estimate()`.
    #[kani::proof]
    fn scalar_kalman_covariance_stays_nonnegative() {
        let x0: f64 = kani::any();
        let p0: f64 = kani::any();
        let q: f64 = kani::any();
        let r: f64 = kani::any();
        let z: f64 = kani::any();
        kani::assume(x0.is_finite() && x0.abs() <= 1e6);
        kani::assume(p0.is_finite() && p0 >= 0.0 && p0 <= 1e6);
        kani::assume(q.is_finite() && q >= 0.0 && q <= 1e6);
        kani::assume(r.is_finite() && r > 1e-9 && r <= 1e6);
        kani::assume(z.is_finite() && z.abs() <= 1e6);

        let mut kf = ScalarKalman::new(x0, p0, q, r);
        kf.predict();
        assert!(kf.p >= 0.0);
        let p_before_update = kf.p;
        let _ = kf.update(z);
        assert!(kf.p >= 0.0);
        assert!(kf.p <= p_before_update + 1e-9);
    }

    /// A measurement update never overshoots: the corrected estimate is no
    /// farther from the measurement than the prior estimate was (the
    /// Kalman gain is always in `[0, 1]` for non-negative `p`/positive `r`).
    #[kani::proof]
    fn scalar_kalman_update_is_a_contraction() {
        let x0: f64 = kani::any();
        let p0: f64 = kani::any();
        let r: f64 = kani::any();
        let z: f64 = kani::any();
        kani::assume(x0.is_finite() && x0.abs() <= 1e6);
        kani::assume(p0.is_finite() && p0 >= 0.0 && p0 <= 1e6);
        kani::assume(r.is_finite() && r > 1e-9 && r <= 1e6);
        kani::assume(z.is_finite() && z.abs() <= 1e6);

        let mut kf = ScalarKalman::new(x0, p0, 0.0, r);
        let prior_err = (x0 - z).abs();
        let x_new = kf.update(z);
        let post_err = (x_new - z).abs();
        assert!(post_err <= prior_err + 1e-9);
    }

    /// Best-effort proof for the generic `S`-state filter at a small,
    /// concrete dimension (2 states / 1 measurement — the same shape as the
    /// `nd_position_velocity_filters_noise` test). This exercises real
    /// `nalgebra` matrix-multiply/inverse code rather than hand-derived
    /// scalar algebra, so it is more solver-intensive than the scalar
    /// proofs above; if it does not terminate in CI, narrow the assumed
    /// ranges further before raising the unwind bound.
    #[kani::proof]
    #[kani::unwind(2)]
    fn kalman_filter_2x1_diagonal_stays_nonnegative() {
        let p00: f64 = kani::any();
        let p11: f64 = kani::any();
        let r00: f64 = kani::any();
        let dt: f64 = kani::any();
        let z: f64 = kani::any();

        kani::assume(p00.is_finite() && p00 >= 0.0 && p00 <= 1e3);
        kani::assume(p11.is_finite() && p11 >= 0.0 && p11 <= 1e3);
        kani::assume(r00.is_finite() && r00 > 1e-6 && r00 <= 1e3);
        kani::assume(dt.is_finite() && dt.abs() <= 10.0);
        kani::assume(z.is_finite() && z.abs() <= 1e3);

        let x0 = SVector::<f64, 2>::zeros();
        let p0 = SMatrix::<f64, 2, 2>::from_row_slice(&[p00, 0.0, 0.0, p11]);
        let q = SMatrix::<f64, 2, 2>::zeros();
        let r = SMatrix::<f64, 1, 1>::from_row_slice(&[r00]);
        let mut kf = KalmanFilter::<2, 1>::new(x0, p0, q, r);

        let f = SMatrix::<f64, 2, 2>::from_row_slice(&[1.0, dt, 0.0, 1.0]);
        kf.predict(&f);
        assert!(kf.p[(0, 0)] >= -1e-6);
        assert!(kf.p[(1, 1)] >= -1e-6);

        let h = SMatrix::<f64, 1, 2>::from_row_slice(&[1.0, 0.0]);
        let meas = SVector::<f64, 1>::new(z);
        if kf.update(&h, &meas).is_some() {
            assert!(kf.p[(0, 0)] >= -1e-6);
            assert!(kf.p[(1, 1)] >= -1e-6);
        }
    }
}
