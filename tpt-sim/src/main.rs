//! `tpt-sim` — runs the hover scenario and prints the result.

use tpt_sim::Sim;

fn main() {
    println!("TPT Flight Control — SITL hover scenario (Phase 0 milestone)");

    // Start level, then run 10 s and confirm a stable hover.
    let mut sim = Sim::new();
    sim.run(10.0, 0.001);

    let p = sim.plant();
    println!(
        "level start   -> pos = ({:.3}, {:.3}, {:.3}) m, |attitude| = {:.3} rad",
        p.pos.x,
        p.pos.y,
        p.pos.z,
        sim.max_attitude_seen()
    );

    // Start with a 12-degree initial roll disturbance; the controller must
    // recover to a stable hover.
    let mut sim2 = Sim::with_initial_attitude(0.21, 0.0, 0.0);
    sim2.run(10.0, 0.001);
    let p2 = sim2.plant();
    println!(
        "disturbed start-> pos = ({:.3}, {:.3}, {:.3}) m, |attitude| = {:.3} rad",
        p2.pos.x,
        p2.pos.y,
        p2.pos.z,
        sim2.max_attitude_seen()
    );

    println!("Motors (final): {:?}", sim2.motors());
    println!("Hover demonstration complete.");
}
