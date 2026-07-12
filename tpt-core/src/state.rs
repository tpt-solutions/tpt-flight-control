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

/// Position/heading target for the guidance (navigation) loop.
///
/// Coordinates are in the local navigation frame (NED, meters). `z` is down
/// positive, so a target altitude of `0` is the origin and a *negative* `z`
/// means "above the origin". `yaw` is the desired heading in radians.
#[derive(Debug, Clone, Copy, Default)]
pub struct PositionTarget {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Desired velocity setpoints (m/s). Zero = position-hold at `x/y/z`.
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    /// Desired yaw (rad).
    pub yaw: f64,
}

impl PositionTarget {
    /// A position-hold target at the origin, zero heading.
    pub const fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            yaw: 0.0,
        }
    }
}
