//! Rigid-body quadcopter plant for simulation (6-DOF, simplified).
//!
//! World frame: **NED** (x = north, y = east, z = down). Body frame:
//! x = forward, y = right, z = down. Quaternion `q` rotates body → world.
//!
//! The plant converts four normalized motor commands into forces and torques
//! using the same geometry the [`tpt_mixer`] quad-X mixer assumes, so the
//! closed loop is self-consistent.

use tpt_math::{Quaternion, UnitQuaternion, Vector3, clamp};

// Motor geometry (quad-X, body x forward / y right / z down).
// Order: [M1 front-right, M2 rear-left, M3 front-left, M4 rear-right].
//
// The three torque axes form an orthonormal basis on the four actuators so that
// roll, pitch, yaw, and thrust are fully decoupled. The SAME vectors are used
// by the `tpt_mixer` quad-X mixer, which is what makes the closed loop
// self-consistent.
const ROLL: [f64; 4] = [1.0, -1.0, -1.0, 1.0];
const PITCH: [f64; 4] = [1.0, -1.0, 1.0, -1.0];
const YAW: [f64; 4] = [1.0, 1.0, -1.0, -1.0];

// Vehicle parameters (small 1 kg quad).
pub const MASS: f64 = 1.0;
pub const GRAVITY: f64 = 9.81;
/// Max thrust per motor (N). The quad-X mixer divides the collective
/// thrust command by 4, so total thrust = `thrust_cmd * T_MAX`. With
/// `T_MAX = 2 * m * g`, a hover command of `0.5` exactly balances gravity.
pub const T_MAX: f64 = 2.0 * MASS * GRAVITY;
pub const INERTIA: Vector3<f64> = Vector3::new(0.01, 0.01, 0.02);
/// Torque gain mapping a normalized moment command to N·m.
pub const TORQUE_GAIN: f64 = 0.04;

/// Dynamic state of the simulated vehicle.
#[derive(Debug, Clone)]
pub struct Plant {
    pub pos: Vector3<f64>,
    pub vel: Vector3<f64>,
    pub quat: UnitQuaternion<f64>,
    pub omega: Vector3<f64>,
}

impl Plant {
    pub fn new() -> Self {
        Self {
            pos: Vector3::zeros(),
            vel: Vector3::zeros(),
            quat: UnitQuaternion::identity(),
            omega: Vector3::zeros(),
        }
    }

    /// Simulate a perturbed initial attitude (e.g. to test stabilization).
    pub fn with_initial_attitude(roll: f64, pitch: f64, yaw: f64) -> Self {
        let q = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
        Self {
            quat: q,
            ..Self::new()
        }
    }

    /// Advance the dynamics by `dt` seconds given normalized motor commands.
    pub fn step(&mut self, dt: f64, motors: &[f64; 4]) {
        let sum: f64 = motors.iter().sum();
        let t_total = sum * T_MAX;

        // Body thrust force (dynamics): rotors push the vehicle up = -z body.
        let f_body = Vector3::new(0.0, 0.0, -t_total);
        // Body torques from motor distribution. The orthonormal basis makes
        // each body-axis torque depend only on the corresponding command.
        let roll_t = TORQUE_GAIN * dot(&ROLL, motors);
        let pitch_t = TORQUE_GAIN * dot(&PITCH, motors);
        let yaw_t = TORQUE_GAIN * dot(&YAW, motors);
        let tau = Vector3::new(roll_t, pitch_t, yaw_t);

        // World force = body thrust rotated to world + gravity (down = +z).
        let f_world = self.quat.transform_vector(&f_body) + Vector3::new(0.0, 0.0, MASS * GRAVITY);
        let accel = f_world / MASS;

        // Rigid-body angular dynamics with diagonal inertia.
        let iomega = Vector3::new(
            self.omega.x * INERTIA.x,
            self.omega.y * INERTIA.y,
            self.omega.z * INERTIA.z,
        );
        let gyro = self.omega.cross(&iomega);
        let domega = Vector3::new(
            (tau.x - gyro.x) / INERTIA.x,
            (tau.y - gyro.y) / INERTIA.y,
            (tau.z - gyro.z) / INERTIA.z,
        );

        // Integrate (semi-implicit Euler).
        self.vel += accel * dt;
        self.pos += self.vel * dt;
        self.omega += domega * dt;

        // Quaternion kinematics: q_dot = 0.5 * q * omega_body.
        let w = Quaternion::new(0.0, self.omega.x, self.omega.y, self.omega.z);
        let dq = self.quat.quaternion() * w * (0.5 * dt);
        self.quat = UnitQuaternion::new_normalize(self.quat.quaternion() + dq);
    }

    /// Simulated IMU reading: `(accelerometer, gyroscope)` in body frame.
    ///
    /// The accelerometer reports specific force; at hover it reads `+g` along
    /// body -z (down), matching the AHRS assumption.
    pub fn imu(&self, motors: &[f64; 4]) -> (Vector3<f64>, Vector3<f64>) {
        let sum: f64 = motors.iter().sum();
        let t_total = sum * T_MAX;
        let accel = Vector3::new(0.0, 0.0, t_total / MASS);
        (accel, self.omega)
    }
}

impl Default for Plant {
    fn default() -> Self {
        Self::new()
    }
}

fn dot(a: &[f64; 4], b: &[f64; 4]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// Clamp a motor command to the valid range (defensive; mixer already clamps).
pub fn sanitize_motors(m: &mut [f64; 4]) {
    for v in m.iter_mut() {
        *v = clamp(*v, 0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Collective command that makes `t_total == MASS * GRAVITY` (see the
    // `T_MAX` doc comment): all four motors equal, summing to 0.5.
    const HOVER_MOTORS: [f64; 4] = [0.125, 0.125, 0.125, 0.125];

    #[test]
    fn new_plant_is_at_rest_at_origin() {
        let p = Plant::new();
        assert_eq!(p.pos, Vector3::zeros());
        assert_eq!(p.vel, Vector3::zeros());
        assert_eq!(p.omega, Vector3::zeros());
        assert!((p.quat.quaternion().norm() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn default_matches_new() {
        let p = Plant::default();
        assert_eq!(p.pos, Plant::new().pos);
    }

    #[test]
    fn with_initial_attitude_sets_euler_angles() {
        let p = Plant::with_initial_attitude(0.1, -0.2, 0.3);
        let (r, pi, y) = p.quat.euler_angles();
        assert!((r - 0.1).abs() < 1e-9);
        assert!((pi - (-0.2)).abs() < 1e-9);
        assert!((y - 0.3).abs() < 1e-9);
        // Still at rest otherwise.
        assert_eq!(p.pos, Vector3::zeros());
        assert_eq!(p.omega, Vector3::zeros());
    }

    #[test]
    fn hover_thrust_holds_altitude() {
        let mut p = Plant::new();
        for _ in 0..1000 {
            p.step(0.001, &HOVER_MOTORS);
        }
        assert!(p.vel.z.abs() < 1e-6, "vz drifted: {}", p.vel.z);
        assert!(p.pos.z.abs() < 1e-6, "z drifted: {}", p.pos.z);
    }

    #[test]
    fn zero_thrust_falls_under_gravity() {
        let mut p = Plant::new();
        p.step(0.1, &[0.0, 0.0, 0.0, 0.0]);
        // NED: z is down-positive, so falling means vel.z and pos.z grow.
        assert!((p.vel.z - GRAVITY * 0.1).abs() < 1e-9);
        assert!(p.pos.z > 0.0);
        assert!((p.pos.x).abs() < 1e-12);
        assert!((p.pos.y).abs() < 1e-12);
    }

    #[test]
    fn roll_command_only_produces_roll_torque() {
        let mut p = Plant::new();
        // M1, M4 up / M2, M3 down relative to hover -> pure roll differential
        // (matches the ROLL basis vector [1, -1, -1, 1]).
        let motors = [0.15, 0.10, 0.10, 0.15];
        p.step(0.001, &motors);
        assert!(p.omega.x.abs() > 0.0, "expected nonzero roll rate");
        assert!(
            p.omega.y.abs() < 1e-9,
            "unexpected pitch rate: {}",
            p.omega.y
        );
        assert!(p.omega.z.abs() < 1e-9, "unexpected yaw rate: {}", p.omega.z);
    }

    #[test]
    fn pitch_command_only_produces_pitch_torque() {
        let mut p = Plant::new();
        // Matches the PITCH basis vector [1, -1, 1, -1].
        let motors = [0.15, 0.10, 0.15, 0.10];
        p.step(0.001, &motors);
        assert!(
            p.omega.x.abs() < 1e-9,
            "unexpected roll rate: {}",
            p.omega.x
        );
        assert!(p.omega.y.abs() > 0.0, "expected nonzero pitch rate");
        assert!(p.omega.z.abs() < 1e-9, "unexpected yaw rate: {}", p.omega.z);
    }

    #[test]
    fn yaw_command_only_produces_yaw_torque() {
        let mut p = Plant::new();
        // Matches the YAW basis vector [1, 1, -1, -1].
        let motors = [0.15, 0.15, 0.10, 0.10];
        p.step(0.001, &motors);
        assert!(
            p.omega.x.abs() < 1e-9,
            "unexpected roll rate: {}",
            p.omega.x
        );
        assert!(
            p.omega.y.abs() < 1e-9,
            "unexpected pitch rate: {}",
            p.omega.y
        );
        assert!(p.omega.z.abs() > 0.0, "expected nonzero yaw rate");
    }

    #[test]
    fn quaternion_stays_normalized_after_many_steps() {
        let mut p = Plant::new();
        let motors = [0.2, 0.1, 0.15, 0.1];
        for _ in 0..500 {
            p.step(0.001, &motors);
        }
        assert!((p.quat.quaternion().norm() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn imu_reports_hover_specific_force_and_body_rates() {
        let mut p = Plant::new();
        p.omega = Vector3::new(0.1, -0.2, 0.3);
        let (accel, gyro) = p.imu(&HOVER_MOTORS);
        assert!((accel.z - GRAVITY).abs() < 1e-9);
        assert!((accel.x).abs() < 1e-12);
        assert!((accel.y).abs() < 1e-12);
        assert_eq!(gyro, p.omega);
    }

    #[test]
    fn imu_reports_zero_specific_force_at_zero_thrust() {
        let p = Plant::new();
        let (accel, _gyro) = p.imu(&[0.0, 0.0, 0.0, 0.0]);
        assert_eq!(accel, Vector3::zeros());
    }

    #[test]
    fn sanitize_motors_clamps_to_unit_range() {
        let mut m = [-0.5, 0.3, 1.2, 2.0];
        sanitize_motors(&mut m);
        assert_eq!(m, [0.0, 0.3, 1.0, 1.0]);
    }
}
