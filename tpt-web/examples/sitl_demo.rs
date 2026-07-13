//! WASM SITL-in-browser demo: runs a `tpt-sim` GPS-denied scenario in the
//! browser and feeds the live estimate into the `tpt-web` leptos dashboard.
//!
//! This is the "live, no-install demo" for TPT Flight Control: a real
//! closed-loop control stack (`GpsDeniedSim`) runs entirely client-side and
//! its navigation estimate is rendered by the [`tpt_web::ui::Dashboard`]
//! component. There is no vehicle, radio, or backend to install.
//!
//! ## Build & serve (one time: `cargo install trunk`)
//!
//! ```sh
//! trunk serve --release --example sitl_demo -p tpt-web --features web
//! ```
//!
//! Then open the printed `http://localhost:8080` URL. `trunk` compiles the
//! example to `wasm32-unknown-unknown`, optimizes it, and serves it with a
//! tiny static host. Alternatively, build the artifact directly:
//!
//! ```sh
//! cargo build --release --target wasm32-unknown-unknown \
//!     --example sitl_demo -p tpt-web --features web
//! ```

#[cfg(feature = "web")]
pub fn main() {
    use leptos::mount::mount_to_body;
    use leptos::prelude::*;
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::time::Duration;
    use tpt_core::PositionTarget;
    use tpt_sim::{GpsDeniedSim, Scenario};
    use tpt_web::WebTelemetry;

    mount_to_body(|| {
        let (telemetry, set_telemetry) = signal(WebTelemetry::sample());

        // One closed-loop SITL run, owned by the animation loop.
        let sim = Rc::new(RefCell::new({
            let mut s = GpsDeniedSim::new(Scenario::Jamming);
            let mut t = PositionTarget::origin();
            t.x = 5.0;
            t.y = 5.0;
            t.z = -2.0;
            s.set_target(t);
            s
        }));

        let sim2 = sim.clone();
        set_interval(
            move || {
                let mut s = sim2.borrow_mut();
                // Advance ~16 ms of sim time per animation frame.
                for _ in 0..16 {
                    s.step(0.001);
                }
                let (roll, pitch, yaw) = s.plant().quat.euler_angles();
                let tel = WebTelemetry {
                    roll,
                    pitch,
                    yaw,
                    pos: s.est_position(),
                    vel: s.plant().vel,
                    battery: 1.0,
                    mode: s.flight_mode(),
                    nav_mode: s.fusion_mode(),
                    gps_healthy: s.scenario().gps().available,
                    vio_healthy: s.scenario().vio().available,
                    terrain_healthy: false,
                    horiz_uncert_m: s.uncertainty(),
                };
                set_telemetry.set(tel);
            },
            Duration::from_millis(16),
        );

        let telemetry: Signal<WebTelemetry> = telemetry.into();
        view! { <tpt_web::ui::Dashboard telemetry=telemetry/> }
    });
}
