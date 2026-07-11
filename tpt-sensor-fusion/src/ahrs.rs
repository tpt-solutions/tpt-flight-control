//! Complementary-filter AHRS (`spec.txt` §7.1).
//!
//! A lightweight, `no_std` attitude estimator suitable for `tpt-micro`. It
//! fuses gyroscope integration with an accelerometer-derived gravity
//! observation using a complementary (high-pass on gyro, low-pass on accel)
//! correction. The estimate is kept as a unit quaternion to avoid gimbal-lock
//! issues inherent in direct angle fusion.

use libm::{cos, sin};
use tpt_math::{Quaternion, UnitQuaternion, Vector3, clamp};

/// Build a unit quaternion from a rotation vector `axis * angle` (body frame).
#[inline]
fn from_rotation_vector(v: Vector3<f64>) -> Quaternion<f64> {
    let angle = v.norm();
    if angle < 1e-12 {
        return Quaternion::new(1.0, 0.0, 0.0, 0.0);
    }
    let half = 0.5 * angle;
    let s = sin(half);
    let inv = s / angle;
    Quaternion::new(cos(half), v.x * inv, v.y * inv, v.z * inv)
}

/// Complementary-filter AHRS.
#[derive(Debug, Clone)]
pub struct ComplementaryAhrs {
    q: UnitQuaternion<f64>,
    /// Accelerometer correction gain `[0, 1]`. Higher = trust accel more.
    gain: f64,
    initialized: bool,
}

impl ComplementaryAhrs {
    /// Create with the given accelerometer gain (typical `0.01 .. 0.1`).
    pub fn new(gain: f64) -> Self {
        Self {
            q: UnitQuaternion::identity(),
            gain: clamp(gain, 0.0, 1.0),
            initialized: false,
        }
    }

    /// Reset to level, identity orientation.
    pub fn reset(&mut self) {
        self.q = UnitQuaternion::identity();
        self.initialized = false;
    }

    /// Current attitude estimate as a unit quaternion (body → world, NED).
    pub fn quaternion(&self) -> UnitQuaternion<f64> {
        self.q
    }

    /// Current attitude as `(roll, pitch, yaw)` in radians (NED convention).
    ///
    /// Uses nalgebra's Euler extraction, which is `no_std`-safe (its internal
    /// trigonometry is provided by the `libm` feature).
    pub fn attitude(&self) -> (f64, f64, f64) {
        self.q.to_rotation_matrix().euler_angles()
    }

    /// Feed one IMU sample.
    ///
    /// `accel` is specific force in m/s^2 (body frame, z down at rest),
    /// `gyro` is angular rate in rad/s (body frame).
    pub fn update(&mut self, accel: Vector3<f64>, gyro: Vector3<f64>, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // 1) Gyro integration (body-frame rotation vector).
        let dq_gyro = from_rotation_vector(gyro * dt);
        let q_pred = UnitQuaternion::new_normalize(self.q.quaternion() * dq_gyro);

        // 2) Accelerometer correction: compare measured gravity direction with
        //    the gravity predicted by the current orientation.
        if let Some(a) = accel.try_normalize(1e-4) {
            // Gravity in world NED is (0,0,1); rotate it into the body frame.
            let g_world = Vector3::new(0.0, 0.0, 1.0);
            let g_body = q_pred.conjugate().transform_vector(&g_world);
            // Error rotation vector (body frame) between measured and predicted.
            let error = a.cross(&g_body);
            let dq_corr = from_rotation_vector(error * self.gain * dt);
            self.q = UnitQuaternion::new_normalize(q_pred.quaternion() * dq_corr);
        } else {
            self.q = q_pred;
        }
        self.initialized = true;
    }

    /// Whether the filter has processed at least one sample.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converges_to_level_when_stationary() {
        // Stationary vehicle: accel = +1g along body z (down), no rotation.
        let accel = Vector3::new(0.0, 0.0, 9.81);
        let gyro = Vector3::new(0.0, 0.0, 0.0);
        let mut ahrs = ComplementaryAhrs::new(0.05);
        for _ in 0..2000 {
            ahrs.update(accel, gyro, 0.001);
        }
        let (roll, pitch, yaw) = ahrs.attitude();
        assert!(roll.abs() < 0.02, "roll={roll}");
        assert!(pitch.abs() < 0.02, "pitch={pitch}");
        assert!(yaw.abs() < 0.05, "yaw={yaw}");
    }

    #[test]
    fn tracks_a_roll_rate() {
        // Pure roll rate of +1 rad/s; attitude should accumulate roll.
        let accel = Vector3::new(0.0, 0.0, 9.81);
        let gyro = Vector3::new(1.0, 0.0, 0.0);
        let mut ahrs = ComplementaryAhrs::new(0.02);
        for _ in 0..100 {
            ahrs.update(accel, gyro, 0.001);
        }
        let (roll, _, _) = ahrs.attitude();
        // ~0.1 rad expected (100 ms @ 1 rad/s), allow correction slack.
        assert!(roll > 0.05 && roll < 0.2, "roll={roll}");
    }
}
