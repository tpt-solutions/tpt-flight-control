//! Shared vehicle state and setpoint types used by the control laws.

use tpt_abstractions::Pose6DOF;
use tpt_math::Vector3;

/// Snapshot of the vehicle state consumed by the control laws.
#[derive(Debug, Clone, Copy, Default)]
pub struct VehicleState {
    /// Local-frame position (meters).
    pub position: Vector3<f64>,
    /// Local-frame velocity (m/s).
    pub velocity: Vector3<f64>,
    /// Body attitude (roll, pitch, yaw in radians; NED-ish convention).
    pub attitude: (f64, f64, f64),
    /// Body angular rates (rad/s).
    pub body_rates: Vector3<f64>,
    /// 6-DOF pose estimate from the fusion engine (if available).
    pub pose: Option<Pose6DOF>,
    /// Remaining battery fraction `[0, 1]`.
    pub battery: f64,
}

/// Attitude / rate command produced by the guidance & navigation loops.
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeSetpoint {
    /// Desired roll (rad).
    pub roll: f64,
    /// Desired pitch (rad).
    pub pitch: f64,
    /// Desired yaw rate (rad/s).
    pub yaw_rate: f64,
    /// Collective thrust command `[0, 1]` (normalized total thrust).
    pub thrust: f64,
}

/// Velocity command for the outer (navigation) loop.
#[derive(Debug, Clone, Copy, Default)]
pub struct VelocitySetpoint {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
}
