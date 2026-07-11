//! Phase 0/1 milestone tests: a virtual quadcopter hovers and navigates in
//! simulation.
//!
//! Runs the closed-loop harness (AHRS -> attitude controller -> envelope
//! protection -> position/guidance controller -> quad-X mixer -> plant) and
//! asserts stable hover, attitude recovery, and waypoint flight.

use tpt_core::PositionTarget;
use tpt_sim::Sim;

#[test]
fn hovers_from_level() {
    let mut sim = Sim::new();
    sim.run(10.0, 0.001);
    let p = sim.plant();

    assert!(p.pos.x.abs() < 0.3, "x drift {}", p.pos.x);
    assert!(p.pos.y.abs() < 0.3, "y drift {}", p.pos.y);
    assert!(p.pos.z.abs() < 0.3, "altitude drift {}", p.pos.z);
    assert!(sim.max_attitude_seen() < 0.05, "attitude {}", sim.max_attitude_seen());
    // At hover the collective thrust command is ~0.5; with the quad-X mixer
    // each motor sits at ~thrust/4 and the sum equals the collective.
    let sum: f64 = sim.motors().iter().sum();
    assert!((sum - 0.5).abs() < 0.1, "collective thrust {}", sum);
    for m in sim.motors() {
        assert!((0.0..=1.0).contains(m), "motor cmd {}", m);
    }
}

#[test]
fn recovers_from_attitude_disturbance() {
    // ~12 deg initial roll.
    let mut sim = Sim::with_initial_attitude(0.21, 0.0, 0.0);
    sim.run(15.0, 0.001);
    let p = sim.plant();
    let (roll, pitch, _yaw) = sim.attitude();

    // Attitude recovered, and the position-hold guidance brings it back to
    // the origin.
    assert!(roll.abs() < 0.04, "final roll {}", roll);
    assert!(pitch.abs() < 0.04, "final pitch {}", pitch);
    assert!(p.pos.x.abs() < 0.5, "x drift {}", p.pos.x);
    assert!(p.pos.y.abs() < 0.5, "y drift {}", p.pos.y);
    assert!(p.pos.z.abs() < 0.3, "altitude drift {}", p.pos.z);
    // It must have actually tilted to recover before settling level.
    assert!(sim.max_plant_attitude_seen() > 0.1, "should have tilted to recover");
}

#[test]
fn flies_to_waypoint() {
    let mut sim = Sim::new();
    let mut target = PositionTarget::origin();
    target.x = 5.0;
    target.y = 5.0;
    target.z = -2.0; // 2 m above origin (NED z is down-positive)
    sim.set_target(target);
    sim.run(25.0, 0.001);
    let p = sim.plant();

    assert!((p.pos.x - 5.0).abs() < 1.0, "x = {}", p.pos.x);
    assert!((p.pos.y - 5.0).abs() < 1.0, "y = {}", p.pos.y);
    assert!((p.pos.z + 2.0).abs() < 0.5, "z = {}", p.pos.z);
    assert!(sim.max_attitude_seen() < 0.4, "attitude {}", sim.max_attitude_seen());
}
