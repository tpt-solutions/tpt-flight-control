//! Phase 0 milestone test: a virtual quadcopter hovers in simulation.
//!
//! Runs the closed-loop harness (AHRS -> attitude controller -> envelope
//! protection -> quad-X mixer -> plant) and asserts a stable hover with and
//! without an initial attitude disturbance.

use tpt_sim::Sim;

#[test]
fn hovers_from_level() {
    let mut sim = Sim::new();
    sim.run(10.0, 0.001);
    let p = sim.plant();

    assert!(p.pos.z.abs() < 0.5, "altitude drift {}", p.pos.z);
    assert!(p.pos.x.abs() < 0.5, "x drift {}", p.pos.x);
    assert!(p.pos.y.abs() < 0.5, "y drift {}", p.pos.y);
    assert!(sim.max_attitude_seen() < 0.1, "attitude {}", sim.max_attitude_seen());
    // At hover the collective thrust command is ~0.5; with the quad-X mixer
    // each motor sits at ~thrust/4 and the sum equals the collective.
    let sum: f64 = sim.motors().iter().sum();
    assert!((sum - 0.5).abs() < 0.2, "collective thrust {}", sum);
    for m in sim.motors() {
        assert!((0.0..=1.0).contains(m), "motor cmd {}", m);
    }
}

#[test]
fn recovers_from_attitude_disturbance() {
    // ~12 deg initial roll.
    let mut sim = Sim::with_initial_attitude(0.21, 0.0, 0.0);
    sim.run(10.0, 0.001);
    let p = sim.plant();
    let (roll, pitch, yaw) = sim.attitude();

    assert!(roll.abs() < 0.06, "final roll {}", roll);
    assert!(pitch.abs() < 0.06, "final pitch {}", pitch);
    assert!(p.pos.z.abs() < 0.5, "altitude drift {}", p.pos.z);
}
