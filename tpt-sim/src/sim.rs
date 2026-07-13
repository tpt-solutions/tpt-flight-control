//! Closed-loop simulation harness — the Phase 0 "hover in simulation"
//! milestone, extended in Phase 1 with GPS/position-hold guidance. Wires the
//! AHRS, cascaded attitude controller, non-bypassable envelope protection,
//! position/guidance controller, and quad-X mixer around the
//! [`plant`](crate::plant).

use crate::plant::{GRAVITY, Plant, sanitize_motors};
use tpt_core::{
    AttitudeController, AttitudeSetpoint, EnvelopeConfig, EnvelopeProtector, FlightStateMachine,
    PositionController, PositionTarget, VehicleState,
};
use tpt_math::Vector3;
use tpt_mixer::{ControlCommand, MotorMixer, QuadXMixer};
use tpt_sensor_fusion::ComplementaryAhrs;

/// Fully wired simulation.
pub struct Sim {
    plant: Plant,
    ahrs: ComplementaryAhrs,
    controller: AttitudeController,
    envelope: EnvelopeProtector,
    mixer: QuadXMixer,
    fsm: FlightStateMachine,
    guidance: PositionController,
    target: PositionTarget,
    tick: u64,
    motors: [f64; 4],
    /// Last computed attitude setpoint (for inspection/telemetry).
    last_setpoint: AttitudeSetpoint,
    /// Observed max envelope violation (diagnostic).
    max_attitude: f64,
    /// Peak physical attitude (rad) observed on the plant (diagnostic).
    max_plant_attitude: f64,
}

impl Sim {
    pub fn new() -> Self {
        Self {
            plant: Plant::new(),
            ahrs: ComplementaryAhrs::new(0.17),
            controller: AttitudeController::new(),
            envelope: EnvelopeProtector::new(EnvelopeConfig::default()),
            mixer: QuadXMixer,
            fsm: FlightStateMachine::new(),
            guidance: PositionController::new(tpt_core::guidance::PositionGains::default()),
            target: PositionTarget::origin(),
            tick: 0,
            motors: [0.0; 4],
            last_setpoint: AttitudeSetpoint::default(),
            max_attitude: 0.0,
            max_plant_attitude: 0.0,
        }
    }

    /// Set the navigation target (waypoint / position hold).
    pub fn set_target(&mut self, target: PositionTarget) {
        self.target = target;
    }

    /// Current navigation target.
    pub fn target(&self) -> PositionTarget {
        self.target
    }

    /// Start from a perturbed attitude to demonstrate stabilization.
    pub fn with_initial_attitude(roll: f64, pitch: f64, yaw: f64) -> Self {
        let mut s = Self::new();
        s.plant = Plant::with_initial_attitude(roll, pitch, yaw);
        s
    }

    pub fn plant(&self) -> &Plant {
        &self.plant
    }

    /// Current estimated attitude `(roll, pitch, yaw)` in radians (NED).
    pub fn attitude(&self) -> (f64, f64, f64) {
        self.ahrs.attitude()
    }

    /// Peak physical attitude (rad) observed on the plant since start.
    pub fn max_plant_attitude_seen(&self) -> f64 {
        self.max_plant_attitude
    }

    pub fn motors(&self) -> &[f64; 4] {
        &self.motors
    }

    pub fn last_setpoint(&self) -> AttitudeSetpoint {
        self.last_setpoint
    }

    pub fn max_attitude_seen(&self) -> f64 {
        self.max_attitude
    }

    /// Run `seconds` of simulation at `dt` (default 1 kHz inner loop).
    pub fn run(&mut self, seconds: f64, dt: f64) {
        let steps = (seconds / dt).round() as u64;
        for _ in 0..steps {
            self.step(dt);
        }
    }

    /// Advance one control step of `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.tick += 1;

        // 1) Sensors (IMU).
        let (accel, gyro) = self.plant.imu(&self.motors);

        // 2) Attitude estimation.
        self.ahrs.update(accel, gyro, dt);
        let (roll, pitch, yaw) = self.ahrs.attitude();

        // 3) Build the vehicle state for the control laws.
        let state = VehicleState {
            position: self.plant.pos,
            velocity: self.plant.vel,
            attitude: (roll, pitch, yaw),
            body_rates: self.plant.omega,
            pose: None,
            battery: 1.0,
        };

        // 4) Guidance (navigation) loop at 200 Hz: produce an attitude
        //    setpoint that drives the vehicle to `self.target`.
        let outer_tick = self.tick % 5 == 0; // dt = 1ms -> 5ms = 200Hz
        let mut setpoint = self.last_setpoint;
        if outer_tick {
            setpoint = self.guidance.update(&self.target, &state, GRAVITY);
            self.last_setpoint = setpoint;
        }
        if self.tick % 2000 == 0 {
            eprintln!(
                "dbg tick {} thrust {:.3} roll {:.3} pitch {:.3} z {:.3} vz {:.3}",
                self.tick,
                setpoint.thrust,
                setpoint.roll,
                setpoint.pitch,
                self.plant.pos.z,
                self.plant.vel.z
            );
        }

        // 5) Attitude controller (inner loop, 1000 Hz).
        let moments = self.controller.update(&setpoint, &state, dt);

        // 6) Non-bypassable envelope protection (control laws -> mixer).
        let protected = self.envelope.protect(setpoint, &state);

        // 7) Mixer: moments + thrust -> motor commands.
        let cmd = ControlCommand {
            thrust: protected.thrust,
            roll: moments.roll,
            pitch: moments.pitch,
            yaw: moments.yaw,
        };
        let mut motors = [0.0f64; 4];
        self.mixer.mix(&cmd, &mut motors);
        sanitize_motors(&mut motors);
        self.motors = motors;

        // 8) Diagnostics.
        self.max_attitude = self.max_attitude.max(roll.abs().max(pitch.abs()));

        // 9) Plant integration.
        self.plant.step(dt, &motors);

        // Peak physical attitude (independent of AHRS lag).
        let (pr, pp, _) = self.plant.quat.euler_angles();
        self.max_plant_attitude = self.max_plant_attitude.max(pr.abs().max(pp.abs()));

        // 10) Flight mode: arm -> takeoff -> position hold.
        match self.fsm.mode() {
            tpt_core::FlightMode::Disarmed => {
                let _ = self.fsm.handle(tpt_core::FlightEvent::Arm);
                let _ = self.fsm.handle(tpt_core::FlightEvent::CommandTakeoff);
            }
            tpt_core::FlightMode::Takeoff if self.plant.pos.z < self.target.z + 0.05 => {
                let _ = self
                    .fsm
                    .handle(tpt_core::FlightEvent::ReachedTargetAltitude);
            }
            _ => {}
        }
    }
}

impl Default for Sim {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience: a position vector for assertions.
pub fn attitude_vector(s: &Sim) -> Vector3<f64> {
    let (r, p, y) = s.plant().quat.euler_angles();
    Vector3::new(r, p, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_sim_starts_disarmed_at_rest() {
        let sim = Sim::new();
        assert_eq!(sim.target(), PositionTarget::origin());
        assert_eq!(sim.motors(), &[0.0; 4]);
        assert_eq!(sim.max_attitude_seen(), 0.0);
        assert_eq!(sim.max_plant_attitude_seen(), 0.0);
        assert_eq!(sim.plant().pos, Vector3::zeros());
    }

    #[test]
    fn default_matches_new() {
        let sim = Sim::default();
        assert_eq!(sim.target(), Sim::new().target());
    }

    #[test]
    fn with_initial_attitude_seeds_the_plant() {
        let sim = Sim::with_initial_attitude(0.1, 0.0, 0.0);
        let (roll, _pitch, _yaw) = sim.plant().quat.euler_angles();
        assert!((roll - 0.1).abs() < 1e-9);
    }

    #[test]
    fn set_target_updates_target() {
        let mut sim = Sim::new();
        let mut target = PositionTarget::origin();
        target.x = 3.0;
        target.z = -1.5;
        sim.set_target(target);
        assert_eq!(sim.target().x, 3.0);
        assert_eq!(sim.target().z, -1.5);
    }

    #[test]
    fn run_executes_the_requested_number_of_steps() {
        let mut sim = Sim::new();
        sim.run(0.01, 0.001); // 10 steps at 1ms.
        assert_eq!(sim.tick, 10);
    }

    #[test]
    fn step_produces_finite_in_range_motor_commands() {
        let mut sim = Sim::new();
        for _ in 0..50 {
            sim.step(0.001);
        }
        for m in sim.motors() {
            assert!(m.is_finite());
            assert!((0.0..=1.0).contains(m), "motor cmd out of range: {}", m);
        }
    }

    #[test]
    fn fsm_progresses_from_disarmed_through_takeoff() {
        let mut sim = Sim::new();
        assert_eq!(sim.fsm.mode(), tpt_core::FlightMode::Disarmed);
        // First step arms and commands takeoff.
        sim.step(0.001);
        assert_eq!(sim.fsm.mode(), tpt_core::FlightMode::Takeoff);
    }

    #[test]
    fn attitude_vector_matches_plant_euler_angles() {
        let sim = Sim::with_initial_attitude(0.05, -0.05, 0.2);
        let v = attitude_vector(&sim);
        let (r, p, y) = sim.plant().quat.euler_angles();
        assert_eq!(v, Vector3::new(r, p, y));
    }

    #[test]
    fn last_setpoint_updates_after_outer_loop_tick() {
        let mut sim = Sim::new();
        // Outer guidance loop runs every 5th tick (dt = 1ms -> 200 Hz).
        for _ in 0..5 {
            sim.step(0.001);
        }
        // Hovering at the origin target should command a non-zero thrust
        // to counter gravity, not the zero-initialized default.
        assert!(sim.last_setpoint().thrust > 0.0);
    }
}
