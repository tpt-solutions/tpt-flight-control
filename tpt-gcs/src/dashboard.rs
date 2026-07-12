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
