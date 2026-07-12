//! # tpt-web
//!
//! Browser dashboard for TPT Flight Control (`spec.txt` §15.4). The crate is
//! split so the flight-relevant, host-buildable parts carry no wasm/UI
//! dependency, while the interactive `leptos` front-end lives behind the `web`
//! feature (`src/ui.rs`, enabled with `cargo add leptos --features web`).
//!
//! The core type is [`WebTelemetry`]: a compact, serializable view of the
//! vehicle that the GCS already renders ([`tpt_gcs::Telemetry`]) augmented with
//! the navigation-health assessment ([`tpt_sensor_fusion::NavHealth`]). It is
//! encoded as a small, dependency-free JSON object (`to_json` / `from_json`)
//! so a WebSocket bridge can stream it to the browser without pulling in
//! `serde`.

use tpt_core::FlightMode;
use tpt_gcs::Telemetry;
use tpt_math::Vector3;
use tpt_sensor_fusion::{FusionMode, NavHealth, SourceStatus};

/// A browser-friendly snapshot of the vehicle plus its navigation health.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WebTelemetry {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub pos: Vector3<f64>,
    pub vel: Vector3<f64>,
    pub battery: f64,
    pub mode: FlightMode,
    pub nav_mode: FusionMode,
    pub gps_healthy: bool,
    pub vio_healthy: bool,
    pub terrain_healthy: bool,
    pub horiz_uncert_m: f64,
}

impl WebTelemetry {
    /// Build from a GCS telemetry frame and a navigation-health snapshot.
    pub fn from_telemetry(t: &Telemetry, nav: &NavHealth) -> Self {
        Self {
            roll: t.roll,
            pitch: t.pitch,
            yaw: t.yaw,
            pos: t.position,
            vel: t.velocity,
            battery: t.battery,
            mode: t.mode,
            nav_mode: nav.mode,
            gps_healthy: nav.gps == SourceStatus::Healthy,
            vio_healthy: nav.vio == SourceStatus::Healthy,
            terrain_healthy: nav.terrain == SourceStatus::Healthy,
            horiz_uncert_m: nav.horiz_uncert_m,
        }
    }

    /// Horizontal ground speed (m/s) for the HUD.
    pub fn ground_speed(&self) -> f64 {
        (self.vel.x * self.vel.x + self.vel.y * self.vel.y).sqrt()
    }

    /// Encode as a compact, flat JSON object. The format is intentionally
    /// minimal (a fixed set of keys) so it round-trips through [`from_json`]
    /// without a `serde` dependency.
    pub fn to_json(&self) -> String {
        format!(
            "{{\"roll\":{r},\"pitch\":{p},\"yaw\":{y},\"pn\":{pn},\"pe\":{pe},\"pd\":{pd},\"vn\":{vn},\"ve\":{ve},\"vd\":{vd},\"batt\":{b},\"mode\":\"{m}\",\"nav\":\"{n}\",\"gps\":{g},\"vio\":{v},\"ter\":{t},\"unc\":{u}}}",
            r = self.roll,
            p = self.pitch,
            y = self.yaw,
            pn = self.pos.x,
            pe = self.pos.y,
            pd = self.pos.z,
            vn = self.vel.x,
            ve = self.vel.y,
            vd = self.vel.z,
            b = self.battery,
            m = flight_mode_str(self.mode),
            n = fusion_mode_str(self.nav_mode),
            g = b2u(self.gps_healthy),
            v = b2u(self.vio_healthy),
            t = b2u(self.terrain_healthy),
            u = self.horiz_uncert_m,
        )
    }

    /// Decode a [`to_json`]-produced object. Returns `None` if any expected key
    /// is missing or malformed. Tolerant of field order.
    pub fn from_json(s: &str) -> Option<WebTelemetry> {
        let roll = f64_field(s, "roll")?;
        let pitch = f64_field(s, "pitch")?;
        let yaw = f64_field(s, "yaw")?;
        let pn = f64_field(s, "pn")?;
        let pe = f64_field(s, "pe")?;
        let pd = f64_field(s, "pd")?;
        let vn = f64_field(s, "vn")?;
        let ve = f64_field(s, "ve")?;
        let vd = f64_field(s, "vd")?;
        let batt = f64_field(s, "batt")?;
        let unc = f64_field(s, "unc")?;
        let mode = str_field(s, "mode").and_then(flight_mode_from_str)?;
        let nav = str_field(s, "nav").and_then(fusion_mode_from_str)?;
        let gps = bool_field(s, "gps")?;
        let vio = bool_field(s, "vio")?;
        let ter = bool_field(s, "ter")?;
        Some(WebTelemetry {
            roll,
            pitch,
            yaw,
            pos: Vector3::new(pn, pe, pd),
            vel: Vector3::new(vn, ve, vd),
            battery: batt,
            mode,
            nav_mode: nav,
            gps_healthy: gps,
            vio_healthy: vio,
            terrain_healthy: ter,
            horiz_uncert_m: unc,
        })
    }

    /// A nominal, fully-healthy sample — handy for demos and offline tests.
    pub fn sample() -> Self {
        let t = Telemetry::zeroed();
        let nav = NavHealth {
            mode: FusionMode::GpsAided,
            gps: SourceStatus::Healthy,
            vio: SourceStatus::Healthy,
            depth: SourceStatus::Lost,
            terrain: SourceStatus::Healthy,
            horiz_uncert_m: 0.4,
            vert_uncert_m: 0.4,
            time_since_aiding_s: 0.1,
        };
        Self::from_telemetry(&t, &nav)
    }
}

fn b2u(b: bool) -> u8 {
    if b { 1 } else { 0 }
}

fn flight_mode_str(m: FlightMode) -> &'static str {
    match m {
        FlightMode::Disarmed => "Disarmed",
        FlightMode::Armed => "Armed",
        FlightMode::Takeoff => "Takeoff",
        FlightMode::PositionHold => "PositionHold",
        FlightMode::Land => "Land",
        FlightMode::Failsafe => "Failsafe",
        FlightMode::Glide => "Glide",
    }
}

fn flight_mode_from_str(s: &str) -> Option<FlightMode> {
    match s {
        "Disarmed" => Some(FlightMode::Disarmed),
        "Armed" => Some(FlightMode::Armed),
        "Takeoff" => Some(FlightMode::Takeoff),
        "PositionHold" => Some(FlightMode::PositionHold),
        "Land" => Some(FlightMode::Land),
        "Failsafe" => Some(FlightMode::Failsafe),
        _ => None,
    }
}

fn fusion_mode_str(m: FusionMode) -> &'static str {
    match m {
        FusionMode::GpsAided => "GpsAided",
        FusionMode::Coast => "Coast",
        FusionMode::VisualAided => "VisualAided",
        FusionMode::TerrainAided => "TerrainAided",
    }
}

fn fusion_mode_from_str(s: &str) -> Option<FusionMode> {
    match s {
        "GpsAided" => Some(FusionMode::GpsAided),
        "Coast" => Some(FusionMode::Coast),
        "VisualAided" => Some(FusionMode::VisualAided),
        "TerrainAided" => Some(FusionMode::TerrainAided),
        _ => None,
    }
}

/// Find `"key":<number>` and parse the number.
fn f64_field(json: &str, key: &str) -> Option<f64> {
    let needle = format!("\"{key}\":");
    let start = json.find(&needle)? + needle.len();
    let rest = &json[start..];
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    rest[..end].trim().parse::<f64>().ok()
}

/// Find `"key":<true|false>`.
fn bool_field(json: &str, key: &str) -> Option<bool> {
    let needle = format!("\"{key}\":");
    let start = json.find(&needle)? + needle.len();
    let rest = &json[start..];
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    match rest[..end].trim() {
        "true" | "1" => Some(true),
        "false" | "0" => Some(false),
        _ => None,
    }
}

/// Find `"key":"<value>"` and return the inner string (unquoted).
fn str_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let needle = format!("\"{key}\":");
    let start = json.find(&needle)? + needle.len();
    let rest = &json[start..];
    let q1 = rest.find('"')?;
    let after = &rest[q1 + 1..];
    let q2 = after.find('"')?;
    Some(&after[..q2])
}

/// The `leptos` browser front-end. Only compiled with `--features web`
/// (after `cargo add leptos`). Kept in its own module so the host/test build of
/// `tpt-web` needs no wasm toolchain.
#[cfg(feature = "web")]
pub mod ui;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_round_trips() {
        let t = WebTelemetry::sample();
        let j = t.to_json();
        let back = WebTelemetry::from_json(&j).expect("decode");
        assert_eq!(t, back);
    }

    #[test]
    fn json_is_flat_and_deterministic() {
        let j = WebTelemetry::sample().to_json();
        assert!(j.starts_with('{') && j.ends_with('}'));
        assert!(j.contains("\"mode\":\"Disarmed\""));
        assert!(j.contains("\"nav\":\"GpsAided\""));
        assert!(j.contains("\"gps\":1"));
    }

    #[test]
    fn malformed_json_rejected() {
        assert!(WebTelemetry::from_json("not json").is_none());
        assert!(WebTelemetry::from_json("{\"roll\":1.0}").is_none());
    }

    #[test]
    fn builds_from_gcs_telemetry_and_nav() {
        let t = Telemetry::zeroed();
        let nav = NavHealth {
            mode: FusionMode::VisualAided,
            gps: SourceStatus::Lost,
            vio: SourceStatus::Healthy,
            depth: SourceStatus::Lost,
            terrain: SourceStatus::Lost,
            horiz_uncert_m: 1.2,
            vert_uncert_m: 1.2,
            time_since_aiding_s: 0.2,
        };
        let w = WebTelemetry::from_telemetry(&t, &nav);
        assert!(!w.gps_healthy);
        assert!(w.vio_healthy);
        assert_eq!(w.nav_mode, FusionMode::VisualAided);
        assert!((w.ground_speed()).abs() < 1e-9);
    }
}
