//! Text dashboard rendering for the console GCS and as a fallback view-model
//! for the egui panel.
//!
//! [`render`] produces a compact, monospace-friendly status block from a
//! [`Telemetry`] snapshot. It is transport- and GUI-agnostic so the same
//! string drives both the `console` runner and logging.

use crate::telemetry::Telemetry;

/// Render a [`Telemetry`] snapshot as a multi-line status block.
pub fn render(t: &Telemetry) -> String {
    let deg = core::f64::consts::PI / 180.0;
    format!(
        "┌─ TPT Flight Control — telemetry ────────────────\n\
         │ Mode      : {:?}\n\
         │ Nav       : {:?}\n\
         │ Attitude  : roll {:6.1}°  pitch {:6.1}°  yaw {:6.1}°\n\
         │ Position  : N {:6.2}  E {:6.2}  alt {:6.2} m\n\
         │ Velocity  : {:6.2} m/s (ground)  vz {:6.2} m/s\n\
         │ Battery   : {:5.1}%\n\
         └──────────────────────────────────────────────────",
        t.mode,
        t.nav_mode,
        t.roll / deg,
        t.pitch / deg,
        t.yaw / deg,
        t.position.x,
        t.position.y,
        -t.position.z,
        t.ground_speed(),
        t.velocity.z,
        t.battery * 100.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_core::FlightMode;
    use tpt_math::Vector3;
    use tpt_sensor_fusion::FusionMode;

    #[test]
    fn render_zeroed_reports_expected_fields() {
        let out = render(&Telemetry::zeroed());
        assert!(out.contains("Disarmed"));
        assert!(out.contains("GpsAided"));
        assert!(out.contains("100.0%"));
        assert!(out.contains("roll    0.0"));
        assert!(out.contains("pitch    0.0"));
        assert!(out.contains("yaw    0.0"));
    }

    #[test]
    fn render_converts_radians_to_degrees() {
        let t = Telemetry {
            roll: core::f64::consts::FRAC_PI_2,
            mode: FlightMode::Armed,
            nav_mode: FusionMode::Coast,
            ..Telemetry::zeroed()
        };
        let out = render(&t);
        assert!(out.contains("90.0"), "expected 90 degrees in: {out}");
        assert!(out.contains("Armed"));
        assert!(out.contains("Coast"));
    }

    #[test]
    fn render_shows_altitude_as_negated_down_position() {
        let t = Telemetry {
            position: Vector3::new(0.0, 0.0, -5.0),
            ..Telemetry::zeroed()
        };
        let out = render(&t);
        // NED z is down-positive; altitude display negates it.
        assert!(out.contains("alt   5.00 m"), "unexpected altitude: {out}");
    }

    #[test]
    fn render_shows_ground_speed_and_vertical_speed_separately() {
        let t = Telemetry {
            velocity: Vector3::new(3.0, 4.0, 1.5),
            ..Telemetry::zeroed()
        };
        let out = render(&t);
        assert!(
            out.contains("5.00 m/s"),
            "expected ground speed 5.00: {out}"
        );
        assert!(out.contains("vz   1.50 m/s"), "expected vz 1.50: {out}");
    }
}
