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
    #[allow(clippy::too_many_arguments)]
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zeroed_is_disarmed_gps_aided_full_battery() {
        let t = Telemetry::zeroed();
        assert_eq!(t.roll, 0.0);
        assert_eq!(t.pitch, 0.0);
        assert_eq!(t.yaw, 0.0);
        assert_eq!(t.position, Vector3::zeros());
        assert_eq!(t.velocity, Vector3::zeros());
        assert_eq!(t.battery, 1.0);
        assert_eq!(t.mode, FlightMode::Disarmed);
        assert_eq!(t.nav_mode, FusionMode::GpsAided);
    }

    #[test]
    fn default_matches_zeroed() {
        assert_eq!(Telemetry::default(), Telemetry::zeroed());
    }

    #[test]
    fn new_populates_all_fields() {
        let pos = Vector3::new(1.0, 2.0, -3.0);
        let vel = Vector3::new(0.5, -0.5, 0.0);
        let t = Telemetry::new(
            0.1,
            0.2,
            0.3,
            pos,
            vel,
            0.75,
            FlightMode::PositionHold,
            FusionMode::VisualAided,
        );
        assert_eq!(t.roll, 0.1);
        assert_eq!(t.pitch, 0.2);
        assert_eq!(t.yaw, 0.3);
        assert_eq!(t.position, pos);
        assert_eq!(t.velocity, vel);
        assert_eq!(t.battery, 0.75);
        assert_eq!(t.mode, FlightMode::PositionHold);
        assert_eq!(t.nav_mode, FusionMode::VisualAided);
    }

    #[test]
    fn ground_speed_ignores_vertical_velocity() {
        let t = Telemetry {
            velocity: Vector3::new(3.0, 4.0, 100.0),
            ..Telemetry::zeroed()
        };
        assert!((t.ground_speed() - 5.0).abs() < 1e-12);
    }

    #[test]
    fn ground_speed_zero_when_stationary() {
        assert_eq!(Telemetry::zeroed().ground_speed(), 0.0);
    }
}
