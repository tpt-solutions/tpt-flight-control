//! 5-minute SITL quickstart — run a GPS-denied scenario with **no hardware**.
//!
//! ```sh
//! cargo run -p tpt-sim --example gps_denied_quickstart
//! # or pick a scenario:
//! cargo run -p tpt-sim --example gps_denied_quickstart -- Jamming
//! cargo run -p tpt-sim --example gps_denied_quickstart -- TotalBlackout
//! ```
//!
//! This drives the same closed-loop control stack used by the unit tests
//! (`GpsDeniedSim`) through a representative GPS-denied / degraded operating
//! condition and prints the navigation result. It is pure simulation: there is
//! no vehicle, radio, or sensor to connect.

use std::env;
use tpt_core::PositionTarget;
use tpt_sim::{GpsDeniedSim, Scenario};

fn main() {
    let scenario = env::args()
        .nth(1)
        .map(|s| match s.as_str() {
            "Nominal" => Scenario::Nominal,
            "UrbanCanyon" => Scenario::UrbanCanyon,
            "Jamming" => Scenario::Jamming,
            "Indoor" => Scenario::Indoor,
            "SensorDegradation" => Scenario::SensorDegradation,
            "TotalBlackout" => Scenario::TotalBlackout,
            other => {
                eprintln!("unknown scenario '{other}'; using Jamming");
                Scenario::Jamming
            }
        })
        .unwrap_or(Scenario::Jamming);

    println!("TPT Flight Control — SITL quickstart (no hardware)");
    println!("scenario : {scenario:?}  (GPS-denied navigation)");
    println!("mission  : fly to waypoint (5, 5, 2 m above origin)");
    println!("---");

    let mut sim = GpsDeniedSim::new(scenario);
    let mut target = PositionTarget::origin();
    target.x = 5.0;
    target.y = 5.0;
    target.z = -2.0;
    sim.set_target(target);

    // 30 s of flight at 1 kHz inner loop (fast on a laptop; this is the
    // "5-minute" onboarding demo, not a 5-minute wall-clock run).
    sim.run(30.0, 0.001);

    let p = sim.plant().pos;
    let reached = (p.x - 5.0).abs() < 2.0 && (p.y - 5.0).abs() < 2.0 && (p.z + 2.0).abs() < 0.5;

    println!(
        "final pos    : N={:6.2}  E={:6.2}  D={:6.2} m",
        p.x, p.y, p.z
    );
    println!(
        "final vel    : {:6.2} m/s  (ground speed)",
        (sim.plant().vel.x.powi(2) + sim.plant().vel.y.powi(2)).sqrt()
    );
    println!("fusion mode  : {:?}", sim.fusion_mode());
    println!("flight mode  : {:?}", sim.flight_mode());
    println!("nav uncert   : {:.3} m (1σ)", sim.uncertainty());
    println!("peak attitude: {:.3} rad", sim.max_attitude_seen());
    println!("---");

    match sim.fusion_mode() {
        tpt_sensor_fusion::FusionMode::Coast => {
            println!("RESULT: GPS lost with no aiding — vehicle held level and");
            println!("        entered FAILSAFE (expected degraded behaviour).");
            assert_eq!(sim.flight_mode(), tpt_core::FlightMode::Failsafe);
        }
        mode => {
            println!("RESULT: navigated under {mode:?} and reached the waypoint: {reached}");
            assert!(reached, "vehicle should reach the waypoint under {mode:?}");
        }
    }
    println!("quickstart complete.");
}
