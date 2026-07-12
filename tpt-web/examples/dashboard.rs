//! Example `leptos` front-end entry point for `tpt-web`.
//!
//! Build with:
//!
//! ```sh
//! cargo build --example dashboard --features web --target wasm32-unknown-unknown -p tpt-web
//! ```
//!
//! The `web` feature (and therefore `leptos`) is required; the example is not
//! built by the default `cargo build --workspace` / CI.

#[cfg(feature = "web")]
fn main() {
    use leptos::*;
    use tpt_web::WebTelemetry;

    // In a real deployment `telemetry` is driven by a WebSocket bridge feeding
    // `WebTelemetry::from_json` frames from the vehicle/GCS link.
    mount_to_body(|| {
        let (tel, _set) = create_signal(WebTelemetry::sample());
        let tel: Signal<WebTelemetry> = tel.into();
        view! { <tpt_web::ui::Dashboard telemetry=tel/> }
    });
}
