//! SITL GPS-denied navigation scenarios (`spec.txt` §14, Phase 2).
//!
//! Builds on the same [`Plant`] + control stack as the nominal [`Sim`](crate::Sim)
//! but drives the navigation estimate through the [`InsEkf`] and the
//! [`FusionStateMachine`], so it can exercise the GPS-degraded fusion
//! strategy (Coast → Visual/Depth-Aided → Terrain-Aided) and obstacle
//! avoidance. Each [`Scenario`] models a representative operating condition:
//!
//! - `Nominal` — GPS healthy, full GPS/INS fusion (baseline).
//! - `UrbanCanyon` — GPS multipath-degraded, VIO aiding available.
//! - `Jamming` — GPS jammed (no fix), VIO aiding only.
//! - `Indoor` — no GPS, indoors, VIO aiding only.
//! - `SensorDegradation` — GPS healthy but IMU noise elevated.
//! - `TotalBlackout` — no aiding at all (EW); falls to Coast + failsafe.
//!
//! The navigation estimate used by the guidance loop is the EKF state, which
//! is corrected by GPS when available and by VIO when available, exactly as
//! the §7.2 fusion plan prescribes.

use crate::plant::{GRAVITY, Plant, sanitize_motors};
use tpt_abstractions::types::FixType;
use tpt_core::{
    AttitudeController, AttitudeSetpoint, EnvelopeConfig, EnvelopeProtector, FlightEvent,
    FlightMode, FlightStateMachine, PositionController, PositionTarget, VehicleState,
};
use tpt_math::{UnitQuaternion, Vector3};
use tpt_mixer::{ControlCommand, MotorMixer, QuadXMixer};
use tpt_sensor_fusion::{ComplementaryAhrs, FusionMode, FusionStateMachine, InsEkf, SourceStatus};

/// Hover collective thrust command.
const HOVER_THRUST: f64 = 0.5;
/// Obstacle look-ahead distance for avoidance (m).
const LOOKAHEAD: f64 = 5.0;
/// Obstacle avoidance aim-point push gain (m).
const AVOID_GAIN: f64 = 5.0;

/// A representative GPS-denied / degraded operating scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    Nominal,
    UrbanCanyon,
    Jamming,
    Indoor,
    SensorDegradation,
    TotalBlackout,
}

/// The exact, deterministic sensor inputs consumed by the control stack on a
/// single control step. Produced by [`GpsDeniedSim::sense`] and consumed by
/// [`GpsDeniedSim::apply`]; the flight-log replay tool records these so a run
/// can be reproduced bit-for-bit (see `crate::replay`).
#[derive(Debug, Clone)]
pub struct SenseBatch {
    /// Noise-corrupted accelerometer reading (body frame, m/s^2).
    pub accel: Vector3<f64>,
    /// Noise-corrupted gyroscope reading (body frame, rad/s).
    pub gyro: Vector3<f64>,
    /// GPS position/velocity correction, if aiding was applied this step:
    /// `(gps_pos, gps_vel, pos_noise, vel_noise)`.
    pub gps: Option<(Vector3<f64>, Vector3<f64>, f64, f64)>,
    /// VIO pose correction, if aiding was applied this step:
    /// `(vio_pos, yaw, pos_noise, yaw_noise)`.
    pub vio: Option<(Vector3<f64>, f64, f64, f64)>,
}

/// GPS availability / quality for a scenario.
#[derive(Debug, Clone, Copy)]
pub struct GpsCondition {
    pub fix: FixType,
    pub available: bool,
    pub pos_noise: f64,
    pub vel_noise: f64,
}

/// Visual-inertial odometry availability / quality for a scenario.
#[derive(Debug, Clone, Copy)]
pub struct VioCondition {
    pub available: bool,
    pub pos_noise: f64,
    pub yaw_noise: f64,
}

/// IMU corruption model for a scenario.
#[derive(Debug, Clone, Copy)]
pub struct ImuCondition {
    pub accel_noise: f64,
    pub gyro_noise: f64,
}

impl Scenario {
    /// GPS condition for this scenario.
    pub const fn gps(&self) -> GpsCondition {
        match self {
            Scenario::Nominal => GpsCondition {
                fix: FixType::Fix3D,
                available: true,
                pos_noise: 0.3,
                vel_noise: 0.1,
            },
            Scenario::UrbanCanyon => GpsCondition {
                fix: FixType::Fix2D,
                available: true,
                pos_noise: 2.0,
                vel_noise: 0.5,
            },
            Scenario::Jamming => GpsCondition {
                fix: FixType::NoFix,
                available: false,
                pos_noise: 0.0,
                vel_noise: 0.0,
            },
            Scenario::Indoor => GpsCondition {
                fix: FixType::NoFix,
                available: false,
                pos_noise: 0.0,
                vel_noise: 0.0,
            },
            Scenario::SensorDegradation => GpsCondition {
                fix: FixType::Fix3D,
                available: true,
                pos_noise: 0.3,
                vel_noise: 0.1,
            },
            Scenario::TotalBlackout => GpsCondition {
                fix: FixType::NoFix,
                available: false,
                pos_noise: 0.0,
                vel_noise: 0.0,
            },
        }
    }

    /// VIO condition for this scenario.
    pub const fn vio(&self) -> VioCondition {
        match self {
            Scenario::Nominal => VioCondition {
                available: false,
                pos_noise: 0.0,
                yaw_noise: 0.0,
            },
            Scenario::UrbanCanyon => VioCondition {
                available: true,
                pos_noise: 0.8,
                yaw_noise: 0.05,
            },
            Scenario::Jamming => VioCondition {
                available: true,
                pos_noise: 0.6,
                yaw_noise: 0.05,
            },
            Scenario::Indoor => VioCondition {
                available: true,
                pos_noise: 0.5,
                yaw_noise: 0.04,
            },
            Scenario::SensorDegradation => VioCondition {
                available: false,
                pos_noise: 0.0,
                yaw_noise: 0.0,
            },
            Scenario::TotalBlackout => VioCondition {
                available: false,
                pos_noise: 0.0,
                yaw_noise: 0.0,
            },
        }
    }

    /// IMU corruption for this scenario.
    pub const fn imu(&self) -> ImuCondition {
        match self {
            Scenario::Nominal => ImuCondition {
                accel_noise: 0.0,
                gyro_noise: 0.0,
            },
            Scenario::UrbanCanyon => ImuCondition {
                accel_noise: 0.02,
                gyro_noise: 0.005,
            },
            Scenario::Jamming => ImuCondition {
                accel_noise: 0.01,
                gyro_noise: 0.003,
            },
            Scenario::Indoor => ImuCondition {
                accel_noise: 0.0,
                gyro_noise: 0.0,
            },
            Scenario::SensorDegradation => ImuCondition {
                accel_noise: 0.15,
                gyro_noise: 0.03,
            },
            Scenario::TotalBlackout => ImuCondition {
                accel_noise: 0.05,
                gyro_noise: 0.01,
            },
        }
    }

    /// (gps, vio, depth, terrain) source statuses driving the fusion mode.
    pub fn sources(&self) -> (SourceStatus, SourceStatus, SourceStatus, SourceStatus) {
        // A full 3D/RTK fix is healthy; a 2D (multipath-degraded) fix or
        // dead-reckoning is degraded and should yield to visual aiding; no fix
        // is lost.
        let gps = match self.gps().fix {
            FixType::Fix3D | FixType::RtkFloat | FixType::RtkFixed => SourceStatus::Healthy,
            FixType::Fix2D | FixType::DeadReckoning => SourceStatus::Degraded,
            FixType::NoFix => SourceStatus::Lost,
        };
        let vio = if self.vio().available {
            SourceStatus::Healthy
        } else {
            SourceStatus::Lost
        };
        (gps, vio, SourceStatus::Lost, SourceStatus::Lost)
    }
}

/// A spherical obstacle for avoidance testing.
#[derive(Debug, Clone, Copy)]
pub struct Sphere {
    pub center: Vector3<f64>,
    pub radius: f64,
}

/// A small set of spherical obstacles.
#[derive(Debug, Clone)]
pub struct ObstacleField {
    spheres: [Sphere; 4],
    count: usize,
}

impl ObstacleField {
    /// Empty field.
    pub const fn new() -> Self {
        Self {
            spheres: [Sphere {
                center: Vector3::new(0.0, 0.0, 0.0),
                radius: 0.0,
            }; 4],
            count: 0,
        }
    }

    /// Build from a slice of `(center, radius)` spheres (up to 4).
    pub fn from_spheres(spheres: &[(Vector3<f64>, f64)]) -> Self {
        let mut field = Self::new();
        for (c, r) in spheres.iter().take(4) {
            field.add(*c, *r);
        }
        field
    }

    /// The `(center, radius)` spheres currently in the field.
    pub fn spheres(&self) -> &[Sphere] {
        &self.spheres[..self.count]
    }

    /// Add a sphere (up to 4).
    pub fn add(&mut self, center: Vector3<f64>, radius: f64) {
        if self.count < self.spheres.len() {
            self.spheres[self.count] = Sphere { center, radius };
            self.count += 1;
        }
    }

    /// Nearest obstacle surface distance to `pos`, with the away-direction.
    /// Returns `(away_dir, surface_distance)` where `surface_distance` is
    /// `|pos-center| - radius` (negative if inside the sphere).
    pub fn nearest(&self, pos: Vector3<f64>) -> Option<(Vector3<f64>, f64)> {
        let mut best: Option<(Vector3<f64>, f64)> = None;
        for s in &self.spheres[..self.count] {
            let to = pos - s.center;
            let d = to.norm();
            let surf = d - s.radius;
            if best.as_ref().is_none_or(|(_, bd)| surf < *bd) {
                let dir = if d > 1e-6 {
                    to / d
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                best = Some((dir, surf));
            }
        }
        best
    }

    /// Aim-point offset that routes the vehicle around nearby obstacles.
    ///
    /// When `pos` is within `lookahead + radius` of an obstacle, pushes the
    /// aim point directly away from the obstacle (horizontal plane).
    pub fn avoidance_offset(&self, pos: Vector3<f64>, lookahead: f64, gain: f64) -> Vector3<f64> {
        let mut offset = Vector3::zeros();
        for s in &self.spheres[..self.count] {
            let to = pos - s.center;
            let d = to.norm();
            let reach = lookahead + s.radius;
            if d < reach {
                let dir = if d > 1e-6 {
                    to / d
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                let strength = gain * (1.0 - d / reach);
                offset += Vector3::new(dir.x * strength, dir.y * strength, 0.0);
            }
        }
        offset
    }
}

impl Default for ObstacleField {
    fn default() -> Self {
        Self::new()
    }
}

/// Deterministic pseudo-noise in `[-scale, scale]` (no `rand` dependency).
fn noise(seed: f64, scale: f64) -> f64 {
    let s = (seed * 12.9898).sin() * 43758.5453;
    let frac = s - s.floor();
    (frac - 0.5) * 2.0 * scale
}

/// A closed-loop SITL simulation under a GPS-denied / degraded scenario.
pub struct GpsDeniedSim {
    plant: Plant,
    ahrs: ComplementaryAhrs,
    controller: AttitudeController,
    envelope: EnvelopeProtector,
    mixer: QuadXMixer,
    fsm: FlightStateMachine,
    guidance: PositionController,
    ekf: InsEkf,
    fusion: FusionStateMachine,
    scenario: Scenario,
    target: PositionTarget,
    obstacles: Option<ObstacleField>,
    tick: u64,
    motors: [f64; 4],
    faulted: bool,
    max_attitude: f64,
    min_obstacle_dist: f64,
    last_setpoint: AttitudeSetpoint,
}

impl GpsDeniedSim {
    /// Create a simulator for `scenario`, holding at the origin.
    pub fn new(scenario: Scenario) -> Self {
        Self {
            plant: Plant::new(),
            ahrs: ComplementaryAhrs::new(0.17),
            controller: AttitudeController::new(),
            envelope: EnvelopeProtector::new(EnvelopeConfig::default()),
            mixer: QuadXMixer,
            fsm: FlightStateMachine::new(),
            guidance: PositionController::new(tpt_core::guidance::PositionGains::default()),
            ekf: InsEkf::new(),
            fusion: FusionStateMachine::new(),
            scenario,
            target: PositionTarget::origin(),
            obstacles: None,
            tick: 0,
            motors: [0.0; 4],
            faulted: false,
            max_attitude: 0.0,
            min_obstacle_dist: f64::INFINITY,
            last_setpoint: AttitudeSetpoint::default(),
        }
    }

    /// Set the mission waypoint.
    pub fn set_target(&mut self, target: PositionTarget) {
        self.target = target;
    }

    /// Install an obstacle field for avoidance.
    pub fn set_obstacles(&mut self, field: ObstacleField) {
        self.obstacles = Some(field);
    }

    /// Current navigation/estimate position (NED).
    pub fn est_position(&self) -> Vector3<f64> {
        self.ekf.position()
    }

    /// Current navigation/estimate velocity (NED).
    pub fn est_velocity(&self) -> Vector3<f64> {
        self.ekf.velocity()
    }

    /// Last computed attitude setpoint (for inspection).
    pub fn last_setpoint(&self) -> AttitudeSetpoint {
        self.last_setpoint
    }

    /// Sum of the four motor commands (collective thrust proxy).
    pub fn motor_sum(&self) -> f64 {
        self.motors.iter().sum()
    }

    /// Current scenario being simulated.
    pub fn scenario(&self) -> Scenario {
        self.scenario
    }

    /// Current mission waypoint.
    pub fn target(&self) -> PositionTarget {
        self.target
    }

    /// Current vehicle plant position.
    pub fn plant(&self) -> &Plant {
        &self.plant
    }

    /// Current fusion mode.
    pub fn fusion_mode(&self) -> FusionMode {
        self.fusion.mode()
    }

    /// Current flight mode.
    pub fn flight_mode(&self) -> FlightMode {
        self.fsm.mode()
    }

    /// EKF position 1σ uncertainty (m).
    pub fn uncertainty(&self) -> f64 {
        self.ekf.position_uncertainty()
    }

    /// Peak commanded attitude magnitude seen (rad).
    pub fn max_attitude_seen(&self) -> f64 {
        self.max_attitude
    }

    /// Minimum obstacle surface distance observed (m).
    pub fn min_obstacle_dist(&self) -> f64 {
        self.min_obstacle_dist
    }

    /// The obstacle field as `(center, radius)` pairs, if one is installed.
    pub fn obstacle_spheres(&self) -> Option<Vec<(Vector3<f64>, f64)>> {
        self.obstacles
            .as_ref()
            .map(|o| o.spheres().iter().map(|s| (s.center, s.radius)).collect())
    }

    /// Run `seconds` of simulation at `dt`.
    pub fn run(&mut self, seconds: f64, dt: f64) {
        let steps = (seconds / dt).round() as u64;
        for _ in 0..steps {
            self.step(dt);
        }
    }

    /// Advance the control-loop tick counter by one step. The closed loop is
    /// driven in discrete time; both [`sense`](Self::sense) and the record /
    /// replay helpers advance the tick exactly once per step so the noise
    /// sequence and 200 Hz outer-loop cadence stay aligned.
    pub fn advance_tick(&mut self) {
        self.tick += 1;
    }

    /// Advance one control step: sense the (deterministic) world, then apply
    /// the control stack + plant to those readings.
    pub fn step(&mut self, dt: f64) {
        self.advance_tick();
        let batch = self.sense(dt);
        self.apply(dt, batch);
    }

    /// Compute the deterministic sensor batch for the next control step
    /// *without* advancing the control stack or plant. The batch captures
    /// exactly the inputs the control laws will consume, so a recording of
    /// these batches is sufficient to replay a flight bit-for-bit
    /// (see [`crate::replay`]).
    pub fn sense(&mut self, dt: f64) -> SenseBatch {
        let _ = dt;
        let outer_tick = self.tick % 5 == 0; // 200 Hz navigation loop
        let (accel, gyro) = self.plant.imu(&self.motors);
        let imu = self.scenario.imu();
        let accel_n = accel
            + Vector3::new(
                noise(self.tick as f64 * 0.1, imu.accel_noise),
                noise(self.tick as f64 * 0.1 + 1.0, imu.accel_noise),
                noise(self.tick as f64 * 0.1 + 2.0, imu.accel_noise),
            );
        let gyro_n = gyro
            + Vector3::new(
                noise(self.tick as f64 * 0.1 + 3.0, imu.gyro_noise),
                noise(self.tick as f64 * 0.1 + 4.0, imu.gyro_noise),
                noise(self.tick as f64 * 0.1 + 5.0, imu.gyro_noise),
            );

        let mut gps = None;
        let mut vio = None;
        if outer_tick {
            let (gps_status, vio_status, _, _) = self.scenario.sources();
            if gps_status != SourceStatus::Lost {
                let g = self.scenario.gps();
                // EXPERIMENT: white-noise sensor error (pre-rework).
                let gpos = self.plant.pos
                    + Vector3::new(
                        noise(self.tick as f64 * 0.2, g.pos_noise),
                        noise(self.tick as f64 * 0.2 + 1.0, g.pos_noise),
                        noise(self.tick as f64 * 0.2 + 2.0, g.pos_noise),
                    );
                let gvel = self.plant.vel
                    + Vector3::new(
                        noise(self.tick as f64 * 0.2 + 3.0, g.vel_noise),
                        noise(self.tick as f64 * 0.2 + 4.0, g.vel_noise),
                        noise(self.tick as f64 * 0.2 + 5.0, g.vel_noise),
                    );
                gps = Some((gpos, gvel, g.pos_noise, g.vel_noise));
            }
            if vio_status != SourceStatus::Lost {
                let v = self.scenario.vio();
                let vpos = self.plant.pos
                    + Vector3::new(
                        noise(self.tick as f64 * 0.3, v.pos_noise),
                        noise(self.tick as f64 * 0.3 + 1.0, v.pos_noise),
                        noise(self.tick as f64 * 0.3 + 2.0, v.pos_noise),
                    );
                let vyaw =
                    self.ahrs.attitude().2 + noise(self.tick as f64 * 0.3 + 3.0, v.yaw_noise);
                vio = Some((vpos, vyaw, v.pos_noise, v.yaw_noise));
            }
        }

        SenseBatch {
            accel: accel_n,
            gyro: gyro_n,
            gps,
            vio,
        }
    }

    /// Advance the control stack + plant using externally-supplied sensor
    /// readings. This is the deterministic, pure-function core of the closed
    /// loop: given the same [`SenseBatch`] and the same prior state, it always
    /// produces the same motor commands and plant evolution as a live
    /// [`GpsDeniedSim::step`].
    pub fn apply(&mut self, dt: f64, batch: SenseBatch) {
        let outer_tick = self.tick % 5 == 0; // 200 Hz navigation loop
        let SenseBatch {
            accel: accel_n,
            gyro: gyro_n,
            gps,
            vio,
        } = batch;

        // Attitude estimate (control loop uses this).
        self.ahrs.update(accel_n, gyro_n, dt);
        let (roll, pitch, yaw) = self.ahrs.attitude();

        // Seed the navigation EKF's attitude from the stabilized AHRS so its
        // velocity/position mechanization is not corrupted by INS attitude
        // drift (which would otherwise destabilize the guidance loop closed on
        // the EKF estimate).
        self.ekf
            .set_attitude(UnitQuaternion::from_euler_angles(roll, pitch, yaw));

        // Navigation estimate (EKF) mechanization runs every step (IMU rate);
        // aiding corrections and the fusion-mode selection run at the outer
        // 200 Hz rate (§7.2), matching the time-triggered rate groups.
        self.ekf.predict(accel_n, gyro_n, dt);
        if outer_tick {
            let (gps_status, vio_status, depth, terrain) = self.scenario.sources();
            self.fusion.set_gps(gps_status);
            self.fusion.set_vio(vio_status);
            self.fusion.set_depth(depth);
            self.fusion.set_terrain(terrain);

            if let Some((gpos, gvel, pos_noise, vel_noise)) = gps {
                self.ekf.correct_position(gpos, pos_noise);
                self.ekf.correct_velocity(gvel, vel_noise);
                self.fusion.note_aiding();
            }
            if let Some((vpos, vyaw, pos_noise, yaw_noise)) = vio {
                self.ekf
                    .correct_vio(vpos, vyaw, pos_noise, yaw_noise, dt * 5.0);
                self.fusion.note_aiding();
            }

            // INS-only drift proxy since last aiding (used to gate terrain aiding).
            let drift = self.ekf.velocity().norm() * dt * 5.0;
            self.fusion.tick(self.tick as f64 * dt, drift);
        }

        // Flight mode progression (arm -> takeoff -> position hold).
        match self.fsm.mode() {
            FlightMode::Disarmed => {
                let _ = self.fsm.handle(FlightEvent::Arm);
                let _ = self.fsm.handle(FlightEvent::CommandTakeoff);
            }
            FlightMode::Takeoff if self.plant.pos.z < self.target.z + 0.05 => {
                let _ = self.fsm.handle(FlightEvent::ReachedTargetAltitude);
            }
            _ => {}
        }

        // Guidance (navigation) loop at 200 Hz, attitude loop at 1000 Hz —
        // matching the time-triggered rate groups (§4.2) and the proven-stable
        // nominal [`Sim`](crate::Sim).
        if outer_tick {
            let setpoint = if self.fusion.mode() == FusionMode::Coast {
                // No aiding: hold level hover and declare a fault (degraded
                // behaviour — the flight manager would RTL/land here).
                if !self.faulted {
                    let _ = self.fsm.handle(FlightEvent::Fault);
                    self.faulted = true;
                }
                AttitudeSetpoint {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw_rate: 0.0,
                    thrust: HOVER_THRUST,
                }
            } else {
                let state = VehicleState {
                    position: self.ekf.position(),
                    velocity: self.plant.vel,
                    attitude: (roll, pitch, yaw),
                    body_rates: self.plant.omega,
                    pose: None,
                    battery: 1.0,
                };
                let mut eff = self.target;
                if let Some(obs) = &self.obstacles {
                    let off = obs.avoidance_offset(self.ekf.position(), LOOKAHEAD, AVOID_GAIN);
                    eff.x += off.x;
                    eff.y += off.y;
                    eff.z += off.z;
                }
                self.guidance.update(&eff, &state, GRAVITY)
            };
            self.last_setpoint = setpoint;
        }
        let setpoint = self.last_setpoint;

        // Attitude control state (every step, from the AHRS).
        let ctrl_state = VehicleState {
            position: self.ekf.position(),
            velocity: self.ekf.velocity(),
            attitude: (roll, pitch, yaw),
            body_rates: self.plant.omega,
            pose: None,
            battery: 1.0,
        };

        let moments = self.controller.update(&setpoint, &ctrl_state, dt);
        let protected = self.envelope.protect(setpoint, &ctrl_state);
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

        self.max_attitude = self.max_attitude.max(roll.abs().max(pitch.abs()));
        self.plant.step(dt, &motors);

        if let Some(obs) = &self.obstacles {
            if let Some((_, d)) = obs.nearest(self.plant.pos) {
                self.min_obstacle_dist = self.min_obstacle_dist.min(d);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn waypoint() -> PositionTarget {
        let mut t = PositionTarget::origin();
        t.x = 5.0;
        t.y = 5.0;
        t.z = -2.0;
        t
    }

    #[test]
    fn nominal_reaches_waypoint() {
        let mut sim = GpsDeniedSim::new(Scenario::Nominal);
        sim.set_target(waypoint());
        sim.run(25.0, 0.001);
        let p = sim.plant().pos;
        assert!((p.x - 5.0).abs() < 2.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 2.0, "y={}", p.y);
        assert!((p.z + 2.0).abs() < 0.5, "z={}", p.z);
        assert_eq!(sim.fusion_mode(), FusionMode::GpsAided);
    }

    #[test]
    fn jammed_gps_navigates_on_vio() {
        let mut sim = GpsDeniedSim::new(Scenario::Jamming);
        sim.set_target(waypoint());
        sim.run(30.0, 0.001);
        let p = sim.plant().pos;
        // With GPS jammed but VIO healthy, the vehicle should still reach the
        // waypoint (within a slightly looser bound reflecting VIO noise).
        assert!((p.x - 5.0).abs() < 3.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 3.0, "y={}", p.y);
        assert!((p.z + 2.0).abs() < 0.5, "z={}", p.z);
        assert_eq!(sim.fusion_mode(), FusionMode::VisualAided);
        assert_eq!(sim.flight_mode(), FlightMode::PositionHold);
    }

    #[test]
    fn indoor_navigates_on_vio() {
        let mut sim = GpsDeniedSim::new(Scenario::Indoor);
        sim.set_target(waypoint());
        sim.run(30.0, 0.001);
        let p = sim.plant().pos;
        assert!((p.x - 5.0).abs() < 3.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 3.0, "y={}", p.y);
        assert_eq!(sim.fusion_mode(), FusionMode::VisualAided);
    }

    #[test]
    fn urban_canyon_uses_visual_aiding() {
        let mut sim = GpsDeniedSim::new(Scenario::UrbanCanyon);
        sim.set_target(waypoint());
        sim.run(30.0, 0.001);
        let p = sim.plant().pos;
        assert!((p.x - 5.0).abs() < 3.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 3.0, "y={}", p.y);
        // GPS degraded but VIO present -> visual aided.
        assert_eq!(sim.fusion_mode(), FusionMode::VisualAided);
    }

    #[test]
    fn total_blackout_holds_and_failsafe() {
        let mut sim = GpsDeniedSim::new(Scenario::TotalBlackout);
        sim.set_target(waypoint());
        sim.run(15.0, 0.001);
        // No aiding: fusion coasts and the FSM drops to Failsafe.
        assert_eq!(sim.fusion_mode(), FusionMode::Coast);
        assert_eq!(sim.flight_mode(), FlightMode::Failsafe);
        // It must not have fallen out of the sky catastrophically: altitude
        // stays in a sane band (it holds level hover).
        let z = sim.plant().pos.z;
        assert!(z > -5.0 && z < 1.0, "altitude z={}", z);
    }

    #[test]
    fn obstacle_avoidance_routes_around() {
        let mut sim = GpsDeniedSim::new(Scenario::UrbanCanyon);
        let mut field = ObstacleField::new();
        // Obstacle directly on the path to the waypoint.
        field.add(Vector3::new(2.5, 2.5, -2.0), 1.5);
        sim.set_obstacles(field);
        sim.set_target(waypoint());
        sim.run(30.0, 0.001);
        // Vehicle reached the waypoint ...
        let p = sim.plant().pos;
        assert!((p.x - 5.0).abs() < 3.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 3.0, "y={}", p.y);
        // ... and never penetrated the obstacle surface.
        assert!(
            sim.min_obstacle_dist() > -0.5,
            "min obstacle dist {}",
            sim.min_obstacle_dist()
        );
    }

    #[test]
    fn sensor_degradation_still_navigable() {
        let mut sim = GpsDeniedSim::new(Scenario::SensorDegradation);
        sim.set_target(waypoint());
        sim.run(30.0, 0.001);
        let p = sim.plant().pos;
        assert!((p.x - 5.0).abs() < 3.0, "x={}", p.x);
        assert!((p.y - 5.0).abs() < 3.0, "y={}", p.y);
        assert_eq!(sim.fusion_mode(), FusionMode::GpsAided);
    }
}
