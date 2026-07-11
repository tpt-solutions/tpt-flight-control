//! Time-triggered superloop supervisor for the bare-metal backend.
//!
//! Wires the [`tpt_abstractions`] sensor/OS traits to the platform-independent
//! flight core ([`tpt_core`]), AHRS ([`tpt_sensor_fusion`]) and quad-X mixer
//! ([`tpt_mixer`]) around a time-triggered scheduler (§4.2). Exactly the same
//! closed loop as `tpt-sim`, but driven here by real (or host-mirrored)
//! hardware instead of the physics plant.
//!
//! The supervisor is generic over the sensor/OS backend `B`, so it can be
//! exercised in unit tests with [`tpt_backend_bare_metal::board::Stm32Board`]
//! (host mirror) before flashing to silicon.

use tpt_abstractions::{Gnss, Imu, RadarAltimeter, os::RateGroup, os::RateGroups, os::Scheduler};
use tpt_core::{
    AttitudeController, AttitudeSetpoint, EnvelopeConfig, EnvelopeProtector, FlightStateMachine,
    PositionController, PositionTarget, VehicleState, guidance::PositionGains,
    TimeTriggeredScheduler,
};
use tpt_math::Vector3;
use tpt_mixer::{ControlCommand, MotorMixer, QuadXMixer};
use tpt_sensor_fusion::ComplementaryAhrs;

/// Gravitational acceleration used by the guidance loop (m/s^2).
pub const GRAVITY: f64 = 9.81;

/// Fully wired bare-metal flight supervisor.
pub struct Supervisor<B>
where
    B: Imu + Gnss + RadarAltimeter + Scheduler,
{
    board: B,
    sched: TimeTriggeredScheduler,
    ahrs: ComplementaryAhrs,
    att: AttitudeController,
    env: EnvelopeProtector,
    mixer: QuadXMixer,
    guide: PositionController,
    fsm: FlightStateMachine,
    target: PositionTarget,
    state: VehicleState,
    tick: u64,
    last_setpoint: AttitudeSetpoint,
}

impl<B> Supervisor<B>
where
    B: Imu + Gnss + RadarAltimeter + Scheduler,
{
    /// Construct from a sensor/OS backend and default control gains.
    pub fn new(board: B) -> Self {
        Self {
            board,
            sched: TimeTriggeredScheduler::new(),
            ahrs: ComplementaryAhrs::new(0.1),
            att: AttitudeController::new(),
            env: EnvelopeProtector::new(EnvelopeConfig::default()),
            mixer: QuadXMixer,
            guide: PositionController::new(PositionGains::default()),
            fsm: FlightStateMachine::new(),
            target: PositionTarget::origin(),
            state: VehicleState::default(),
            tick: 0,
            last_setpoint: AttitudeSetpoint::default(),
        }
    }

    /// Set the navigation target / waypoint.
    pub fn set_target(&mut self, target: PositionTarget) {
        self.target = target;
    }

    /// Immutable access to the sensor/OS backend.
    pub fn board(&self) -> &B {
        &self.board
    }
    /// Mutable access to the sensor/OS backend (e.g. to inject samples).
    pub fn board_mut(&mut self) -> &mut B {
        &mut self.board
    }

    /// Current estimated attitude `(roll, pitch, yaw)` in radians (NED).
    pub fn attitude(&self) -> (f64, f64, f64) {
        self.ahrs.attitude()
    }

    /// Current navigation target.
    pub fn target(&self) -> PositionTarget {
        self.target
    }

    /// Advance one control step of `dt` seconds, returning the motor commands.
    ///
    /// Reads sensors from `B`, runs the 1000 Hz attitude loop and the 200 Hz
    /// guidance loop gated by the time-triggered scheduler, enforces the
    /// non-bypassable envelope, mixes, and returns four normalized `[0,1]`
    /// motor commands for the actuator layer.
    pub fn tick(&mut self, dt: f64) -> [f64; 4] {
        self.tick += 1;

        // 1) Sensors via the abstraction traits.
        let accel = self.board.read_accelerometer().unwrap_or_default();
        let gyro = self.board.read_gyroscope().unwrap_or_default();
        let (roll, pitch, yaw) = self.ahrs.attitude();
        let pos = self
            .board
            .read_position()
            .map(|g| Vector3::new(g.lat_deg, g.lon_deg, g.alt_m))
            .unwrap_or_default();
        let vel = self.board.read_velocity().unwrap_or_default();

        // 2) Estimate attitude.
        self.ahrs.update(accel, gyro, dt);

        // 3) Build the vehicle state for the control laws.
        self.state = VehicleState {
            position: pos,
            velocity: vel,
            attitude: (roll, pitch, yaw),
            body_rates: gyro,
            pose: None,
            battery: 1.0,
        };

        // 4) Time-triggered dispatch: guidance at 200 Hz, attitude at 1000 Hz.
        let now = self.board.monotonic_micros().unwrap_or(0);
        let due: RateGroups = self.sched.poll(now);
        let mut setpoint = self.last_setpoint;
        if due.is_due(RateGroup::R200Hz) {
            setpoint = self.guide.update(&self.target, &self.state, GRAVITY);
            self.last_setpoint = setpoint;
        }

        // 5) Attitude controller (inner loop).
        let moments = self.att.update(&setpoint, &self.state, dt);

        // 6) Non-bypassable envelope protection.
        let protected = self.env.protect(setpoint, &self.state);

        // 7) Mixer: moment + thrust -> motor commands.
        let cmd = ControlCommand {
            thrust: protected.thrust,
            roll: moments.roll,
            pitch: moments.pitch,
            yaw: moments.yaw,
        };
        let mut motors = [0.0f64; 4];
        self.mixer.mix(&cmd, &mut motors);

        // 8) Flight mode progression (arm -> takeoff -> position hold).
        match self.fsm.mode() {
            tpt_core::FlightMode::Disarmed => {
                let _ = self.fsm.handle(tpt_core::FlightEvent::Arm);
                let _ = self.fsm.handle(tpt_core::FlightEvent::CommandTakeoff);
            }
            _ => {}
        }

        motors
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::Stm32Board;

    #[test]
    fn hovers_with_stationary_imu() {
        let mut sup = Supervisor::new(Stm32Board::new());
        sup.board_mut().set_stationary_imu();
        let mut last = [0.0f64; 4];
        for _ in 0..10_000 {
            last = sup.tick(0.001);
        }
        // At hover the collective thrust command is ~0.5; each motor ~0.125.
        let sum: f64 = last.iter().sum();
        assert!((sum - 0.5).abs() < 0.1, "collective thrust {}", sum);
        for m in last {
            assert!((0.0..=1.0).contains(&m), "motor cmd {}", m);
        }
        // AHRS should estimate a level attitude.
        let (r, p, _) = sup.attitude();
        assert!(r.abs() < 0.02, "roll {}", r);
        assert!(p.abs() < 0.02, "pitch {}", p);
    }

    #[test]
    fn applies_waypoint_target() {
        let mut sup = Supervisor::new(Stm32Board::new());
        sup.board_mut().set_stationary_imu();
        let mut tgt = PositionTarget::origin();
        tgt.x = 5.0;
        tgt.y = 5.0;
        tgt.z = -2.0;
        sup.set_target(tgt);
        // Run long enough for the guidance loop to demand tilt. We cannot
        // observe position here (no plant), but the attitude estimate must
        // remain bounded and finite.
        for _ in 0..5_000 {
            let m = sup.tick(0.001);
            assert!(m.iter().all(|v| v.is_finite()));
        }
        let (r, p, _) = sup.attitude();
        assert!(r.abs() < 0.6 && p.abs() < 0.6);
    }
}
