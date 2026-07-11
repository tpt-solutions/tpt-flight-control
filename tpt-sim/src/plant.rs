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
const X: [f64; 4] = [1.0, -1.0, 1.0, -1.0];
const Y: [f64; 4] = [1.0, -1.0, -1.0, 1.0];
const SPIN: [f64; 4] = [-1.0, 1.0, 1.0, -1.0]; // CW = -1, CCW = +1

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
        // Body torques from motor distribution.
        let roll_t = TORQUE_GAIN * dot(&Y, motors);
        let pitch_t = -TORQUE_GAIN * dot(&X, motors);
        let yaw_t = TORQUE_GAIN * dot(&SPIN, motors);
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
