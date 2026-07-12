//! Example `leptos` front-end entry point for `tpt-web`.
//!
//! Build with:
//!
//! ```sh
//! cargo add leptos --features web
//! cargo build --example dashboard --features web --target wasm32-unknown-unknown -p tpt-web
//! ```
//!
//! The `web` feature (and therefore `leptos`) is required; the example is not
//! built by the default `cargo build --workspace` / CI.

#[cfg(feature = "web")]
fn main() {
    use leptos::*;
    use tpt_web::WebTelemetry;
    use tpt_web::ui::Dashboard;

    // In a real deployment `telemetry` is driven by a WebSocket bridge feeding
    // `WebTelemetry::from_json` frames from the vehicle/GCS link.
    mount_to_body(|cx| {
        let (tel, _set) = create_signal(cx, WebTelemetry::sample());
        let tel: Signal<WebTelemetry> = tel.into();
        view! { cx, <Dashboard telemetry=tel/> }
    });
}
