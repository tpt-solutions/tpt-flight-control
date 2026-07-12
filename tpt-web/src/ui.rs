//! Leptos browser front-end for the TPT dashboard (`spec.txt` §15.4).
//!
//! Compiled only with `--features web` after `cargo add leptos`. It renders a
//! live [`WebTelemetry`] signal as a small attitude / navigation panel. The
//! crate is intentionally split so the host- and CI-buildable core (the JSON
//! bridge + telemetry model in `lib.rs`) needs no wasm toolchain.

use crate::WebTelemetry;
use leptos::*;

/// A small live dashboard panel bound to a telemetry signal.
#[component]
pub fn Dashboard(cx: Scope, telemetry: Signal<WebTelemetry>) -> impl IntoView {
    let roll = move || telemetry.get().roll.to_degrees();
    let pitch = move || telemetry.get().pitch.to_degrees();
    let yaw = move || telemetry.get().yaw.to_degrees();
    let speed = move || telemetry.get().ground_speed();
    let batt = move || telemetry.get().battery * 100.0;
    let nav = move || flight_mode_label(telemetry.get().nav_mode);
    let gps = move || health_label(telemetry.get().gps_healthy);
    let vio = move || health_label(telemetry.get().vio_healthy);
    let ter = move || health_label(telemetry.get().terrain_healthy);
    let unc = move || telemetry.get().horiz_uncert_m;

    view! { cx,
        <div class="tpt-dashboard">
            <h1>"TPT Flight Control"</h1>
            <div class="row"><span class="k">"Roll"</span><span class="v">{roll}"°"</span></div>
            <div class="row"><span class="k">"Pitch"</span><span class="v">{pitch}"°"</span></div>
            <div class="row"><span class="k">"Yaw"</span><span class="v">{yaw}"°"</span></div>
            <div class="row"><span class="k">"Ground speed"</span><span class="v">{speed}" m/s"</span></div>
            <div class="row"><span class="k">"Battery"</span><span class="v">{batt}"%"</span></div>
            <hr/>
            <div class="row"><span class="k">"Nav mode"</span><span class="v">{nav}</span></div>
            <div class="row"><span class="k">"GPS"</span><span class="v">{gps}</span></div>
            <div class="row"><span class="k">"VIO"</span><span class="v">{vio}</span></div>
            <div class="row"><span class="k">"Terrain"</span><span class="v">{ter}</span></div>
            <div class="row">
                <span class="k">"Horiz σ"</span>
                <span class="v">{unc}" m"</span>
            </div>
        </div>
    }
}

fn flight_mode_label(m: tpt_sensor_fusion::FusionMode) -> &'static str {
    match m {
        tpt_sensor_fusion::FusionMode::GpsAided => "GPS-aided",
        tpt_sensor_fusion::FusionMode::Coast => "Coast (INS only)",
        tpt_sensor_fusion::FusionMode::VisualAided => "Visual-aided",
        tpt_sensor_fusion::FusionMode::TerrainAided => "Terrain-aided",
    }
}

fn health_label(healthy: bool) -> &'static str {
    if healthy { "OK" } else { "LOST" }
}
