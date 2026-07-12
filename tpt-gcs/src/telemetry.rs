//! Telemetry model shared between the vehicle, the link layer, and the UI.
//!
//! A [`Telemetry`] snapshot is the single object the GCS renders. It is built
//! either from a live link (`link`), from a simulator, or unit-test fixtures.
//! Keeping it free of any transport dependency means the same struct drives
//! the egui panel, the console dashboard, and the test harness.

use tpt_core::FlightMode;
use tpt_math::Vector3;
use tpt_sensor_fusion::FusionMode;

/// A point-in-time view of the vehicle, suitable for display and logging.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Telemetry {
    /// Roll angle (rad).
    pub roll: f64,
    /// Pitch angle (rad).
    pub pitch: f64,
    /// Yaw / heading angle (rad).
    pub yaw: f64,
    /// Local-frame position (NED, meters). `z` is down-positive.
    pub position: Vector3<f64>,
    /// Local-frame velocity (m/s).
    pub velocity: Vector3<f64>,
    /// Remaining battery fraction `[0, 1]`.
    pub battery: f64,
    /// Current flight mode.
    pub mode: FlightMode,
    /// Active navigation fusion mode (GPS / visual / terrain / coast).
    pub nav_mode: FusionMode,
}

impl Telemetry {
    /// Construct from attitude, position, velocity, battery and modes.
    pub const fn new(
        roll: f64,
        pitch: f64,
        yaw: f64,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        battery: f64,
        mode: FlightMode,
        nav_mode: FusionMode,
    ) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            position,
            velocity,
            battery,
            mode,
            nav_mode,
        }
    }

    /// Zeroed telemetry in `PositionHold` / GPS-aided state (used as a default
    /// before the first frame arrives).
    pub fn zeroed() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            battery: 1.0,
            mode: FlightMode::Disarmed,
            nav_mode: FusionMode::GpsAided,
        }
    }

    /// Horizontal speed (m/s) — handy for the HUD.
    pub fn ground_speed(&self) -> f64 {
        (self.velocity.x * self.velocity.x + self.velocity.y * self.velocity.y).sqrt()
    }
}

impl Default for Telemetry {
    fn default() -> Self {
        Self::zeroed()
    }
}
