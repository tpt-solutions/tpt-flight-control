//! GPS/INS navigation (`spec.txt` §7.2, Phase 1).
//!
//! A loosely-coupled strapdown navigator. It mechanizes the IMU (integrates
//! specific force through the current attitude to obtain NED acceleration,
//! then integrates velocity and position) and blends in GNSS position/velocity
//! fixes to bound the inertial drift.
//!
//! Frame conventions match the rest of the stack: world NED (x = north,
//! y = east, z = down), body `x` = forward, `y` = right, `z` = down, and the
//! attitude quaternion rotates body → world.

use tpt_abstractions::GeoPosition;
use tpt_math::{UnitQuaternion, Vector3, clamp};
use libm::cos;

/// Earth parameters for the local NED frame.
const EARTH_RADIUS_M: f64 = 6_378_137.0;
const DEG2RAD: f64 = core::f64::consts::PI / 180.0;

/// Loosely-coupled GPS/INS navigator.
#[derive(Debug, Clone)]
pub struct GpsInsNavigator {
    /// Position in the local NED frame (meters).
    pos: Vector3<f64>,
    /// Velocity in the local NED frame (m/s).
    vel: Vector3<f64>,
    /// Correction gain on GNSS position (0 = ignore GPS, 1 = trust GPS).
    gps_gain_pos: f64,
    /// Correction gain on GNSS velocity.
    gps_gain_vel: f64,
}

impl GpsInsNavigator {
    /// Create a navigator initialized at the local-frame origin, stationary.
    pub fn new() -> Self {
        Self {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            gps_gain_pos: 0.05,
            gps_gain_vel: 0.1,
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

    /// Set the GNSS correction gains (clamped to `[0, 1]`).
    pub fn set_gains(&mut self, pos_gain: f64, vel_gain: f64) {
        self.gps_gain_pos = clamp(pos_gain, 0.0, 1.0);
        self.gps_gain_vel = clamp(vel_gain, 0.0, 1.0);
    }

    /// Mechanize one IMU sample.
    ///
    /// `accel` is the specific force in body frame (m/s^2, z down positive at
    /// rest, per the stack convention), `quat` the current body→world NED
    /// attitude, `dt` the step (s).
    pub fn propagate(
        &mut self,
        accel: Vector3<f64>,
        quat: &UnitQuaternion<f64>,
        dt: f64,
    ) {
        if dt <= 0.0 {
            return;
        }
        // NED acceleration = R * f_body - g (z down positive).
        let a_ned = quat.transform_vector(&accel) - Vector3::new(0.0, 0.0, 9.81);
        self.vel += a_ned * dt;
        self.pos += self.vel * dt;
    }

    /// Blend the INS position toward a GNSS NED position fix.
    pub fn correct_position(&mut self, gnss_pos: Vector3<f64>) {
        self.pos += (gnss_pos - self.pos) * self.gps_gain_pos;
    }

    /// Blend the INS velocity toward a GNSS NED velocity fix.
    pub fn correct_velocity(&mut self, gnss_vel: Vector3<f64>) {
        self.vel += (gnss_vel - self.vel) * self.gps_gain_vel;
    }

    /// Convert a [`GeoPosition`] to the local NED frame relative to `origin`
    /// (equirectangular approximation, valid for local-area navigation).
    pub fn geo_to_ned(geo: GeoPosition, origin: GeoPosition) -> Vector3<f64> {
        let lat0 = origin.lat_deg * DEG2RAD;
        let d_lat = (geo.lat_deg - origin.lat_deg) * DEG2RAD;
        let d_lon = (geo.lon_deg - origin.lon_deg) * DEG2RAD;
        let x = d_lat * EARTH_RADIUS_M;
        let y = d_lon * EARTH_RADIUS_M * cos(lat0);
        let z = origin.alt_m - geo.alt_m; // NED z is down-positive
        Vector3::new(x, y, z)
    }

    /// Correct the INS state using a GNSS [`GeoPosition`] fix (position only).
    pub fn correct_with_geo(&mut self, geo: GeoPosition, origin: GeoPosition) {
        let ned = Self::geo_to_ned(geo, origin);
        self.correct_position(ned);
    }
}

impl Default for GpsInsNavigator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_math::UnitQuaternion;

    #[test]
    fn stationary_stays_put() {
        let mut nav = GpsInsNavigator::new();
        let q = UnitQuaternion::identity();
        // Level, at-rest specific force: +1g along body z (down).
        let accel = Vector3::new(0.0, 0.0, 9.81);
        for _ in 0..1000 {
            nav.propagate(accel, &q, 0.001);
        }
        assert!(nav.position().norm() < 1e-6, "pos {:?}", nav.position());
        assert!(nav.velocity().norm() < 1e-6, "vel {:?}", nav.velocity());
    }

    #[test]
    fn integrates_forward_acceleration() {
        let mut nav = GpsInsNavigator::new();
        let q = UnitQuaternion::identity();
        // Accelerate +x (north) at 1 m/s^2: specific force = (1, 0, g).
        let accel = Vector3::new(1.0, 0.0, 9.81);
        let dt = 0.001;
        for _ in 0..1000 {
            nav.propagate(accel, &q, dt);
        }
        // After 1 s: vel ~ (1,0,0), pos ~ (0.5,0,0) (semi-implicit Euler).
        assert!((nav.velocity().x - 1.0).abs() < 1e-6, "vel {:?}", nav.velocity());
        assert!((nav.position().x - 0.5).abs() < 0.01, "pos {:?}", nav.position());
    }

    #[test]
    fn gnss_pulls_drift_back() {
        let mut nav = GpsInsNavigator::new();
        let q = UnitQuaternion::identity();
        // Let the INS drift with a bogus constant acceleration for 5 s.
        let accel = Vector3::new(0.0, 0.0, 9.81 + 0.5); // 0.5 m/s^2 spurious (down)
        for _ in 0..5000 {
            nav.propagate(accel, &q, 0.001);
        }
        assert!(nav.position().z > 0.5, "drifted {}", nav.position().z);
        // A correct GNSS fix at the true origin pulls it back over time.
        for _ in 0..2000 {
            nav.correct_position(Vector3::zeros());
            nav.propagate(Vector3::new(0.0, 0.0, 9.81 + 0.5), &q, 0.001);
        }
        assert!(nav.position().norm() < 0.3, "recovered {:?}", nav.position());
    }

    #[test]
    fn geo_to_ned_basic() {
        let origin = GeoPosition::new(37.0, -122.0, 0.0);
        let north = GeoPosition::new(37.0009, -122.0, 10.0);
        let ned = GpsInsNavigator::geo_to_ned(north, origin);
        assert!(ned.x > 90.0 && ned.x < 110.0, "x {}", ned.x); // ~100 m north
        assert!(ned.z.abs() < 1e-9 + 10.0 - 0.0); // 10 m up => z = -10
        assert!((ned.z + 10.0).abs() < 1e-6, "z {}", ned.z);
    }
}
