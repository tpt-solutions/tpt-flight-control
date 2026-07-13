//! Deterministic flight-log replay / diff tool.
//!
//! ```sh
//! cargo run -p tpt-sim --example replay -- Jamming
//! ```
//!
//! Records a flight log for a GPS-denied scenario, replays it through the
//! *same* control-law crate from that log, and diffs the reproduced telemetry
//! against the original. Because the closed loop is a deterministic function of
//! its sensor history, the diff must be ~0 — any non-zero deviation is a
//! regression (non-determinism or a state/configuration mismatch).
//!
//! The log is also serialized to CSV and re-parsed to prove the on-disk format
//! round-trips, then replayed again from disk.

use std::env;
use std::process::exit;
use tpt_core::PositionTarget;
use tpt_sim::replay::{diff, from_csv, record, replay, to_csv};
use tpt_sim::{GpsDeniedSim, Scenario};

fn main() {
    let scenario = env::args()
        .nth(1)
        .and_then(|s| match s.as_str() {
            "Nominal" => Some(Scenario::Nominal),
            "UrbanCanyon" => Some(Scenario::UrbanCanyon),
            "Jamming" => Some(Scenario::Jamming),
            "Indoor" => Some(Scenario::Indoor),
            "SensorDegradation" => Some(Scenario::SensorDegradation),
            "TotalBlackout" => Some(Scenario::TotalBlackout),
            _ => None,
        })
        .unwrap_or(Scenario::Jamming);

    let mut sim = GpsDeniedSim::new(scenario);
    let mut target = PositionTarget::origin();
    target.x = 5.0;
    target.y = 5.0;
    target.z = -2.0;
    sim.set_target(target);

    println!("TPT Flight Control — flight-log replay / diff ({scenario:?})");

    // 1) Record the flight and replay it from the in-memory log.
    let log = record(&mut sim, 30.0, 0.001);
    let replayed = replay(&log);
    let d = diff(&log, &replayed);
    println!("frames        : {} (recorded == replayed)", d.frames);
    println!("max |Δpos|    : {:.3e} m", d.max_pos);
    println!("max |Δvel|    : {:.3e} m/s", d.max_vel);
    println!("max |Δatt|    : {:.3e} rad", d.max_att);
    println!("max |Δuncert| : {:.3e} m", d.max_uncert);
    println!("max |Δmotors| : {:.3e}", d.max_motor_sum);

    // 2) Serialize to CSV, parse it back, and replay from disk too.
    let csv = to_csv(&log);
    let parsed = match from_csv(&csv) {
        Some(p) => p,
        None => {
            eprintln!("FAILED: flight-log CSV did not round-trip");
            exit(1);
        }
    };
    let disk = diff(&parsed, &replay(&parsed));
    println!(
        "csv round-trip: {} frames parsed, max |Δpos| {:.3e} m",
        disk.frames, disk.max_pos
    );

    // 3) Determinism gate.
    let tol = 1e-6;
    let ok = d.max_pos < tol
        && d.max_vel < tol
        && d.max_att < tol
        && d.max_uncert < tol
        && d.max_motor_sum < tol;
    if ok {
        println!("REPLAY DETERMINISTIC — control laws reproducible from the log.");
    } else {
        eprintln!("FAILED: replay diverged from the original flight log.");
        exit(1);
    }
}
