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
        self.x = self.x + k * y;
        // P = (I - K H) P
        let i = SMatrix::<f64, S, S>::identity();
        self.p = (i - k * *h) * self.p;
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
        assert!((kf.state()[0] - truth).abs() < 0.05, "pos err {}", kf.state()[0] - truth);
        assert!((kf.state()[1] - 1.0).abs() < 0.05, "vel err {}", kf.state()[1] - 1.0);
    }
}
