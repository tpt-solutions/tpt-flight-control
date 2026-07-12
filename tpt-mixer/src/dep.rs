//! Distributed Electric Propulsion (DEP) fault-tolerant mixer (`spec.txt` §9.1,
//! Phase 3).
//!
//! Allocates a body-frame wrench (collective thrust, roll/pitch/yaw moments) to
//! `N` independently-thrusting rotors. The allocation matrix `A` (4×N) maps
//! per-rotor thrust to `[thrust, roll, pitch, yaw]`, and the commanded thrust
//! vector is the Moore-Penrose pseudo-inverse `A⁺ = Aᵀ (A Aᵀ)⁻¹` times the
//! wrench. When a rotor fails, its column is dropped and `A⁺` is recomputed, so
//! the remaining rotors absorb the lost authority — the basis of fault-tolerant
//! reallocation.

use tpt_math::{SMatrix, SVector, Vector2};
use crate::ControlCommand;

/// A rotor's geometric placement and spin sense.
#[derive(Debug, Clone, Copy)]
pub struct Rotor {
    /// Body-frame position (x forward, y right) in meters.
    pub pos: Vector2<f64>,
    /// Spin sense: `+1` CCW (viewed from above), `-1` CW. Drives yaw moment.
    pub spin: f64,
    /// Yaw moment coefficient per unit thrust (sign carried by `spin`).
    pub yaw_coeff: f64,
    /// Whether the rotor is currently healthy / producing thrust.
    pub healthy: bool,
}

impl Rotor {
    /// Create a healthy rotor at `pos` with the given spin sense.
    pub const fn new(x: f64, y: f64, spin: f64) -> Self {
        Self {
            pos: Vector2::new(x, y),
            spin,
            yaw_coeff: 0.1,
            healthy: true,
        }
    }
}

/// Fault-tolerant DEP mixer for `N` rotors.
#[derive(Debug, Clone)]
pub struct DepMixer<const N: usize> {
    rotors: [Rotor; N],
}

impl<const N: usize> DepMixer<N> {
    /// Build a mixer from a rotor set.
    pub const fn new(rotors: [Rotor; N]) -> Self {
        Self { rotors }
    }

    /// Mark rotor `i` as failed (excluded from allocation).
    pub fn fail(&mut self, i: usize) {
        if let Some(r) = self.rotors.get_mut(i) {
            r.healthy = false;
        }
    }

    /// Restore rotor `i`.
    pub fn restore(&mut self, i: usize) {
        if let Some(r) = self.rotors.get_mut(i) {
            r.healthy = true;
        }
    }

    /// Number of healthy rotors.
    pub fn healthy_count(&self) -> usize {
        self.rotors.iter().filter(|r| r.healthy).count()
    }

    /// Build the 4×`H` allocation matrix over healthy rotors. Returns the matrix,
    /// the list of healthy indices, and `H` (column count).
    fn build_a(&self) -> (SMatrix<f64, 4, N>, [usize; N], usize) {
        // We materialize a full 4×N but zero out failed columns.
        let mut a = SMatrix::<f64, 4, N>::zeros();
        let mut idx = [0usize; N];
        let mut h = 0usize;
        for (col, r) in self.rotors.iter().enumerate() {
            if !r.healthy {
                continue;
            }
            // Row 0: collective thrust.
            a[(0, col)] = 1.0;
            // Row 1: roll moment about body x from thrust at +y.
            a[(1, col)] = r.pos.y;
            // Row 2: pitch moment about body y from thrust at +x (front lifts nose).
            a[(2, col)] = -r.pos.x;
            // Row 3: yaw moment from spin sense.
            a[(3, col)] = r.spin * r.yaw_coeff;
            idx[h] = col;
            h += 1;
        }
        (a, idx, h)
    }

    /// Allocate `cmd` to per-rotor normalized thrusts in `out` (len `N`).
    ///
    /// Returns `false` if fewer than 4 healthy rotors remain (under-actuated;
    /// `out` is filled with an equal-split collective thrust only).
    pub fn allocate(&self, cmd: &ControlCommand, out: &mut [f64]) -> bool {
        debug_assert!(out.len() >= N);
        let (a, idx, h) = self.build_a();
        if h < 4 {
            // Under-actuated: split collective thrust equally over healthy
            // rotors, drop moments, and force failed rotors to zero.
            let alive = self.healthy_count().max(1);
            let t = cmd.thrust / alive as f64;
            for (i, r) in self.rotors.iter().enumerate() {
                out[i] = if r.healthy { tpt_math::clamp(t, 0.0, 1.0) } else { 0.0 };
            }
            return false;
        }

        // Wrench vector w = [thrust, roll, pitch, yaw].
        let w = SVector::<f64, 4>::new(cmd.thrust, cmd.roll, cmd.pitch, cmd.yaw);

        // M = A Aᵀ (4×4 over all columns; failed columns are zero so they do not
        // contribute to the product).
        let m = a * a.transpose();
        let m_inv = match m.try_inverse() {
            Some(inv) => inv,
            None => {
                let alive = self.healthy_count().max(1);
                let t = cmd.thrust / alive as f64;
                for (i, r) in self.rotors.iter().enumerate() {
                    out[i] = if r.healthy { tpt_math::clamp(t, 0.0, 1.0) } else { 0.0 };
                }
                return false;
            }
        };
        // v = M⁻¹ w.
        let v = m_inv * w;

        // thrust_i = A_row_i · v  (column i dotted with v).
        for col in 0..N {
            let dot = a[(0, col)] * v[0] + a[(1, col)] * v[1] + a[(2, col)] * v[2] + a[(3, col)] * v[3];
            out[col] = tpt_math::clamp(dot, 0.0, 1.0);
        }
        let _ = idx;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn octo() -> DepMixer<8> {
        // Symmetric X8: four CW, four CCW, at +/-x, +/-y.
        DepMixer::new([
            Rotor::new(1.0, 1.0, 1.0),
            Rotor::new(-1.0, -1.0, 1.0),
            Rotor::new(1.0, -1.0, -1.0),
            Rotor::new(-1.0, 1.0, -1.0),
            Rotor::new(1.0, 1.0, 1.0),
            Rotor::new(-1.0, -1.0, 1.0),
            Rotor::new(1.0, -1.0, -1.0),
            Rotor::new(-1.0, 1.0, -1.0),
        ])
    }

    #[test]
    fn pure_thrust_splits_equally() {
        let m = octo();
        let mut out = [0.0; 8];
        m.allocate(&ControlCommand { thrust: 0.8, ..Default::default() }, &mut out);
        for v in out {
            assert!((v - 0.1).abs() < 1e-9, "v={v}");
        }
    }

    #[test]
    fn roll_differentiates_sides() {
        let m = octo();
        let mut out = [0.0; 8];
        m.allocate(&ControlCommand { thrust: 0.5, roll: 0.2, ..Default::default() }, &mut out);
        // Right rotors (y>0): indices 0,3,4,7 should exceed left (1,2,5,6).
        let right: f64 = out[0] + out[3] + out[4] + out[7];
        let left: f64 = out[1] + out[2] + out[5] + out[6];
        assert!(right > left, "right={right} left={left}");
    }

    #[test]
    fn reallocates_after_single_failure() {
        let mut m = octo();
        let mut out = [0.0; 8];
        m.allocate(&ControlCommand { thrust: 0.6, yaw: 0.1, ..Default::default() }, &mut out);
        let before_ok = m.healthy_count() == 8;
        m.fail(0);
        assert_eq!(m.healthy_count(), 7);
        let ok = m.allocate(&ControlCommand { thrust: 0.6, yaw: 0.1, ..Default::default() }, &mut out);
        assert!(before_ok);
        assert!(ok, "7 healthy rotors should still be fully allocatable");
        // Failed rotor must produce no thrust.
        assert_eq!(out[0], 0.0);
        // All commands remain feasible (normalized 0..1).
        for v in out.iter() {
            assert!(*v >= 0.0 && *v <= 1.0, "thrust out of range: {v}");
        }
        // The min-norm solution may require some (infeasible) negative thrusts;
        // clamping them to zero keeps total thrust near the commanded value.
        let sum: f64 = out.iter().sum();
        assert!(sum > 0.5 && sum < 1.5, "sum={sum}");
    }

    #[test]
    fn under_actuated_falls_back() {
        let mut m = octo();
        for i in 0..5 {
            m.fail(i);
        }
        let mut out = [0.0; 8];
        let ok = m.allocate(&ControlCommand { thrust: 0.5, roll: 0.3, ..Default::default() }, &mut out);
        assert!(!ok);
        // Failed rotors zero, others get equal collective split.
        assert_eq!(out[0], 0.0);
        let alive: f64 = out[5] + out[6] + out[7];
        assert!((alive - 0.5).abs() < 1e-9, "alive={alive}");
    }
}
