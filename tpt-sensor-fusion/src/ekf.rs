//! Error-state Kalman filter (ES-EKF) for INS/GPS/VIO navigation.
//!
//! `spec.txt` §7.2 / Phase 2. Replaces the simple complementary AHRS with a
//! full 15-state error-state EKF that mechanizes the IMU and corrects against
//! GNSS position/velocity fixes and VIO relative-pose measurements. The error
//! state is
//!
//! ```text
//! δx = [ δp(NED, 3)  δv(NED, 3)  δθ(body, 3)  δba(3)  δbg(3) ]^T
//! ```
//!
//! The nominal (large) state holds position, velocity, attitude quaternion,
//! accelerometer bias, and gyro bias. After each correction the error state is
//! injected into the nominal state and the covariance is reset.
//!
//! Frame conventions: world NED (x = north, y = east, z = down), body
//! `x` = forward, `y` = right, `z` = down, attitude quaternion rotates
//! body → world. Gravity in NED is `+9.81` on the z (down) axis.

use libm::sqrt;
use tpt_math::{Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};

/// Number of error states.
const N: usize = 15;

/// Gravity magnitude in m/s^2 (NED, z down positive).
const G: f64 = 9.81;

/// Build the skew-symmetric cross-product matrix of `v`.
#[inline]
fn skew(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v.z, v.y, //
        v.z, 0.0, -v.x, //
        -v.y, v.x, 0.0,
    )
}

/// Rotate a body-frame vector into the world frame with a unit quaternion.
#[inline]
fn rotate(q: &UnitQuaternion<f64>, v: &Vector3<f64>) -> Vector3<f64> {
    q.transform_vector(v)
}

/// Error-state EKF fusing IMU, GNSS, and VIO.
#[derive(Debug, Clone)]
pub struct InsEkf {
    // Nominal state.
    pos: Vector3<f64>,                 // NED position (m)
    vel: Vector3<f64>,                 // NED velocity (m/s)
    quat: UnitQuaternion<f64>,         // body -> world (NED)
    accel_bias: Vector3<f64>,          // body frame (m/s^2)
    gyro_bias: Vector3<f64>,           // body frame (rad/s)
    // Error-state covariance (always reset to P0 between measurements).
    p: SMatrix<f64, N, N>,
    /// Total integration steps (for diagnostics).
    steps: u64,
}

impl InsEkf {
    /// Create an EKF initialized at the NED origin, stationary, level.
    pub fn new() -> Self {
        let mut p = SMatrix::<f64, N, N>::zeros();
        // Initial uncertainty: position 1 m, velocity 0.1 m/s, attitude 1 deg,
        // biases small.
        for (i, v) in [
            1.0, 1.0, 1.0, 0.01, 0.01, 0.01, 0.0003, 0.0003, 0.0003, 0.001, 0.001, 0.001, 0.001,
            0.001, 0.001,
        ]
        .iter()
        .enumerate()
        {
            p[(i, i)] = *v;
        }
        Self {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            quat: UnitQuaternion::identity(),
            accel_bias: Vector3::zeros(),
            gyro_bias: Vector3::zeros(),
            p,
            steps: 0,
        }
    }

    /// Current NED position (m).
    pub const fn position(&self) -> Vector3<f64> {
        self.pos
    }
    /// Current NED velocity (m/s).
    pub const fn velocity(&self) -> Vector3<f64> {
        self.vel
    }
    /// Current body -> world (NED) attitude.
    pub const fn attitude(&self) -> UnitQuaternion<f64> {
        self.quat
    }

    /// Initialize the filter at a known GNSS NED position (cold start).
    pub fn initialize_with_position(&mut self, ned: Vector3<f64>) {
        self.pos = ned;
    }

    /// Mechanize one IMU sample and propagate the error-state covariance.
    ///
    /// `accel` is specific force (m/s^2, body frame, +z down at rest),
    /// `gyro` is angular rate (rad/s, body frame), `dt` the step (s).
    pub fn predict(&mut self, accel: Vector3<f64>, gyro: Vector3<f64>, dt: f64) {
        if dt <= 0.0 {
            return;
        }
        self.steps += 1;

        // --- Nominal-state mechanization -----------------------------------
        let omega = gyro - self.gyro_bias;
        let dq = UnitQuaternion::from_scaled_axis(omega * dt);
        self.quat = UnitQuaternion::new_normalize(self.quat.quaternion() * dq.quaternion());

        let f_corr = accel - self.accel_bias;
        let a_ned = rotate(&self.quat, &f_corr) - Vector3::new(0.0, 0.0, G);
        self.vel += a_ned * dt;
        self.pos += self.vel * dt;

        // --- Error-state covariance propagation (discrete, first order) -----
        let r = self.quat.to_rotation_matrix().matrix().clone();
        let f_minus_ba = rotate(&self.quat, &f_corr); // a_world + g from accel
        let fc = skew(&f_minus_ba); // = [R(f-ba)]x
        let wgt = skew(&omega); // [omega]x

        let mut f = SMatrix::<f64, N, N>::identity();
        // pos <- pos + vel*dt
        for i in 0..3 {
            f[(i, 3 + i)] = dt;
        }
        // vel <- vel + (-[R(f-ba)]x dth - R dba) dt
        for i in 0..3 {
            for j in 0..3 {
                f[(3 + i, 6 + j)] = -dt * fc[(i, j)];
                f[(3 + i, 9 + j)] = -dt * r[(i, j)];
            }
        }
        // dth <- dth + (-[omega]x dth - dbg) dt
        for i in 0..3 {
            for j in 0..3 {
                f[(6 + i, 6 + j)] -= dt * wgt[(i, j)];
                f[(6 + i, 12 + j)] = -dt * (if i == j { 1.0 } else { 0.0 });
            }
        }

        // Process noise Q (diagonal, small) — accel/gyro + bias random walk.
        let mut q = SMatrix::<f64, N, N>::zeros();
        for i in 0..3 {
            q[(3 + i, 3 + i)] = (0.05 * dt) * (0.05 * dt); // velocity from accel noise
            q[(6 + i, 6 + i)] = (0.005 * dt) * (0.005 * dt); // attitude from gyro noise
            q[(9 + i, 9 + i)] = (1e-4 * dt) * (1e-4 * dt); // accel bias walk
            q[(12 + i, 12 + i)] = (1e-5 * dt) * (1e-5 * dt); // gyro bias walk
        }

        let ft = f.transpose();
        let p_prop = f * self.p * ft + q;
        self.p = p_prop;
    }

    /// GNSS position correction (NED meters).
    pub fn correct_position(&mut self, gnss_pos: Vector3<f64>, pos_noise: f64) {
        let z = gnss_pos - self.pos; // innovation (3)
        let h = Self::h_pos();
        let r = SMatrix::<f64, 3, 3>::identity() * (pos_noise * pos_noise);
        self.kalman_correct(&h, &z, &r);
    }

    /// GNSS velocity correction (NED m/s).
    pub fn correct_velocity(&mut self, gnss_vel: Vector3<f64>, vel_noise: f64) {
        let z = gnss_vel - self.vel;
        let h = Self::h_vel();
        let r = SMatrix::<f64, 3, 3>::identity() * (vel_noise * vel_noise);
        self.kalman_correct(&h, &z, &r);
    }

    /// VIO relative-pose correction: a locally-referenced position delta and a
    /// yaw angle (rad) estimate from visual odometry. Used as the GPS-fallback
    /// measurement source in the fusion state machine (§7.2).
    pub fn correct_vio(
        &mut self,
        vio_pos: Vector3<f64>,
        yaw: f64,
        pos_noise: f64,
        yaw_noise: f64,
    ) {
        // Position innovation.
        let zp = vio_pos - self.pos;
        let hp = Self::h_pos();
        // 6x15 stacked H (pos + yaw pseudo-measurement on state 8 = yaw index).
        // Build a 4x15 H: 3 position rows + 1 yaw row (index 8).
        let mut hbig = SMatrix::<f64, 4, N>::zeros();
        for i in 0..3 {
            for j in 0..N {
                hbig[(i, j)] = hp[(i, j)];
            }
        }
        hbig[(3, 8)] = 1.0;
        let zyaw = wrap_pi(yaw - self.yaw());
        let mut zbig = SVector::<f64, 4>::zeros();
        zbig[0] = zp.x;
        zbig[1] = zp.y;
        zbig[2] = zp.z;
        zbig[3] = zyaw;
        let mut rbig = SMatrix::<f64, 4, 4>::zeros();
        for i in 0..3 {
            rbig[(i, i)] = pos_noise * pos_noise;
        }
        rbig[(3, 3)] = yaw_noise * yaw_noise;
        self.kalman_correct(&hbig, &zbig, &rbig);
    }

    /// Core linear Kalman correction for `y = H x + v`, `E[v v^T] = R`.
    fn kalman_correct<const M: usize>(
        &mut self,
        h: &SMatrix<f64, M, N>,
        z: &SVector<f64, M>,
        r: &SMatrix<f64, M, M>,
    ) {
        let s = h * self.p * h.transpose() + *r;
        let s_inv = match s.try_inverse() {
            Some(inv) => inv,
            None => return,
        };
        let k = self.p * h.transpose() * s_inv;
        let dx = k * (*z);
        self.inject_error(&dx);
        // Joseph-form covariance update for numerical stability.
        let i = SMatrix::<f64, N, N>::identity();
        let a = i - k * *h;
        self.p = a * self.p * a.transpose() + k * *r * k.transpose();
    }

    /// Inject the 15-vector error state into the nominal state and reset it.
    fn inject_error(&mut self, dx: &SVector<f64, N>) {
        let dp = Vector3::new(dx[0], dx[1], dx[2]);
        let dv = Vector3::new(dx[3], dx[4], dx[5]);
        let dth = Vector3::new(dx[6], dx[7], dx[8]);
        let dba = Vector3::new(dx[9], dx[10], dx[11]);
        let dbg = Vector3::new(dx[12], dx[13], dx[14]);

        self.pos += dp;
        self.vel += dv;
        // Attitude: q <- q ⊗ exp(dth) (body-frame rotation).
        let dq = UnitQuaternion::from_scaled_axis(dth);
        self.quat = UnitQuaternion::new_normalize(self.quat.quaternion() * dq.quaternion());
        self.accel_bias += dba;
        self.gyro_bias += dbg;

        // Reset error covariance (assume the error was fully consumed).
        let mut p = SMatrix::<f64, N, N>::zeros();
        for (i, v) in [
            0.1, 0.1, 0.1, 0.001, 0.001, 0.001, 1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5, 1e-6, 1e-6,
            1e-6,
        ]
        .iter()
        .enumerate()
        {
            p[(i, i)] = *v;
        }
        self.p = p;
    }

    /// Yaw angle (rad) of the current attitude.
    pub fn yaw(&self) -> f64 {
        self.quat.to_rotation_matrix().euler_angles().2
    }

    /// Position observation matrix (3x15), selects the first 3 states.
    fn h_pos() -> SMatrix<f64, 3, N> {
        let mut h = SMatrix::<f64, 3, N>::zeros();
        for i in 0..3 {
            h[(i, i)] = 1.0;
        }
        h
    }

    /// Velocity observation matrix (3x15), selects states 3..6.
    fn h_vel() -> SMatrix<f64, 3, N> {
        let mut h = SMatrix::<f64, 3, N>::zeros();
        for i in 0..3 {
            h[(i, 3 + i)] = 1.0;
        }
        h
    }

    /// Position 3σ uncertainty (m), derived from the diagonal of P.
    pub fn position_uncertainty(&self) -> f64 {
        sqrt(self.p[(0, 0)] + self.p[(1, 1)] + self.p[(2, 2)] + 1e-12)
    }

    /// Total number of `predict` integrations performed.
    pub const fn step_count(&self) -> u64 {
        self.steps
    }
}

impl Default for InsEkf {
    fn default() -> Self {
        Self::new()
    }
}

/// Wrap an angle to (-π, π].
#[inline]
fn wrap_pi(a: f64) -> f64 {
    let tau = 2.0 * core::f64::consts::PI;
    let mut x = a % tau;
    if x > core::f64::consts::PI {
        x -= tau;
    } else if x <= -core::f64::consts::PI {
        x += tau;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_converges_to_origin() {
        let mut ekf = InsEkf::new();
        let accel = Vector3::new(0.0, 0.0, G);
        let gyro = Vector3::zeros();
        for _ in 0..2000 {
            ekf.predict(accel, gyro, 0.001);
        }
        assert!(ekf.position().norm() < 0.5, "pos {:?}", ekf.position());
        assert!(ekf.velocity().norm() < 0.05, "vel {:?}", ekf.velocity());
    }

    #[test]
    fn gps_position_correction_reduces_error() {
        let mut ekf = InsEkf::new();
        // Let inertial drift downward with a spurious accel, no GPS.
        let bad = Vector3::new(0.0, 0.0, G + 0.3);
        for _ in 0..3000 {
            ekf.predict(bad, Vector3::zeros(), 0.001);
        }
        let drift = ekf.position().norm();
        assert!(drift > 0.5, "should have drifted, got {drift}");
        // Apply several GNSS position fixes at the true origin.
        for _ in 0..50 {
            ekf.correct_position(Vector3::zeros(), 1.0);
        }
        assert!(
            ekf.position().norm() < drift * 0.5,
            "recovered {:?} (was {drift})",
            ekf.position().norm()
        );
    }

    #[test]
    fn gps_velocity_fix_zeroes_drift_rate() {
        let mut ekf = InsEkf::new();
        let accel = Vector3::new(0.5, 0.0, G);
        for _ in 0..1000 {
            ekf.predict(accel, Vector3::zeros(), 0.001);
        }
        // True velocity should be ~ (0.5, 0, 0) due to +0.5 spurious accel.
        assert!(ekf.velocity().x > 0.3, "vel.x {}", ekf.velocity().x);
        for _ in 0..30 {
            ekf.correct_velocity(Vector3::zeros(), 0.2);
        }
        assert!(
            ekf.velocity().norm() < 0.1,
            "vel after fix {:?}",
            ekf.velocity()
        );
    }

    #[test]
    fn vio_position_update_pulls_toward_vio() {
        let mut ekf = InsEkf::new();
        let bad = Vector3::new(0.0, 0.0, G + 0.5);
        for _ in 0..2000 {
            ekf.predict(bad, Vector3::zeros(), 0.001);
        }
        let start = ekf.position().norm();
        // VIO reports the true origin.
        ekf.correct_vio(Vector3::zeros(), ekf.yaw(), 0.5, 0.05);
        assert!(
            ekf.position().norm() < start,
            "before {} after {}",
            start,
            ekf.position().norm()
        );
    }
}
