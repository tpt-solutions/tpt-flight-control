//! `tpt-sim` — runs hover and waypoint scenarios and prints the result.

use tpt_core::PositionTarget;
use tpt_sim::Sim;

fn main() {
    println!("TPT Flight Control — SITL scenarios (Phase 0/1)");

    // 1) Hover at the origin.
    let mut hover = Sim::new();
    hover.run(10.0, 0.001);
    let p = hover.plant();
    println!(
        "hover          -> pos = ({:.2}, {:.2}, {:.2}) m, |att| = {:.3} rad",
        p.pos.x, p.pos.y, p.pos.z, hover.max_attitude_seen()
    );

    // 2) Recover from an initial attitude disturbance.
    let mut rec = Sim::with_initial_attitude(0.21, 0.0, 0.0);
    rec.run(15.0, 0.001);
    let p = rec.plant();
    println!(
        "recover        -> pos = ({:.2}, {:.2}, {:.2}) m, final att = {:.3} rad",
        p.pos.x, p.pos.y, p.pos.z, rec.attitude().0
    );

    // 3) Fly to a waypoint (5, 5, 2 m above origin).
    let mut wp = Sim::new();
    let mut tgt = PositionTarget::origin();
    tgt.x = 5.0;
    tgt.y = 5.0;
    tgt.z = -2.0;
    wp.set_target(tgt);
    wp.run(25.0, 0.001);
    let p = wp.plant();
    println!(
        "waypoint (5,5) -> pos = ({:.2}, {:.2}, {:.2}) m, |att| = {:.3} rad",
        p.pos.x, p.pos.y, p.pos.z, wp.max_attitude_seen()
    );

    println!("Simulation scenarios complete.");
}
