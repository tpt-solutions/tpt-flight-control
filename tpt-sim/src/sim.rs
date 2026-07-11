//! Closed-loop simulation harness — the Phase 0 "hover in simulation"
//! milestone. Wires the AHRS, cascaded attitude controller, non-bypassable
//! envelope protection, and quad-X mixer around the [`plant`](crate::plant).

use crate::plant::{GRAVITY, Plant, sanitize_motors};
use tpt_core::{
    AttitudeController, AttitudeSetpoint, EnvelopeConfig, EnvelopeProtector, FlightStateMachine,
    VehicleState,
};
use tpt_math::Vector3;
use tpt_mixer::{ControlCommand, MotorMixer, QuadXMixer};
use tpt_sensor_fusion::ComplementaryAhrs;

/// Gains / targets for the outer (navigation) loop.
#[derive(Debug, Clone, Copy)]
pub struct OuterGains {
    /// Altitude proportional gain (fraction of thrust per meter).
    pub kp_alt: f64,
    /// Altitude derivative gain (fraction of thrust per m/s).
    pub kd_alt: f64,
    /// Yaw-hold gain (rad/s of yaw-rate per rad of yaw error).
    pub kp_yaw: f64,
    /// Horizontal position-hold proportional gain (1/s^2 per meter).
    pub kp_xy: f64,
    /// Horizontal position-hold derivative gain (1/s per m/s).
    pub kd_xy: f64,
    /// Collective thrust at hover (fraction, ~0.5).
    pub hover_thrust: f64,
    /// Target altitude in NED z (meters; default 0 = origin).
    pub target_z: f64,
}

impl Default for OuterGains {
    fn default() -> Self {
        Self {
            kp_alt: 0.08,
            kd_alt: 0.15,
            kp_yaw: 0.6,
            kp_xy: 0.5,
            kd_xy: 0.8,
            hover_thrust: 0.5,
            target_z: 0.0,
        }
    }
}

/// Fully wired simulation.
pub struct Sim {
    plant: Plant,
    ahrs: ComplementaryAhrs,
    controller: AttitudeController,
    envelope: EnvelopeProtector,
    mixer: QuadXMixer,
    fsm: FlightStateMachine,
    outer: OuterGains,
    tick: u64,
    motors: [f64; 4],
    /// Last computed attitude setpoint (for inspection/telemetry).
    last_setpoint: AttitudeSetpoint,
    /// Observed max envelope violation (diagnostic).
    max_attitude: f64,
}

impl Sim {
    pub fn new() -> Self {
        Self {
            plant: Plant::new(),
            ahrs: ComplementaryAhrs::new(0.04),
            controller: AttitudeController::new(),
            envelope: EnvelopeProtector::new(EnvelopeConfig::default()),
            mixer: QuadXMixer,
            fsm: FlightStateMachine::new(),
            outer: OuterGains::default(),
            tick: 0,
            motors: [0.0; 4],
            last_setpoint: AttitudeSetpoint::default(),
            max_attitude: 0.0,
        }
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

        // 3) Outer (navigation) loop at 200 Hz.
        let outer_tick = self.tick % 5 == 0; // dt = 1ms -> 5ms = 200Hz
        let mut setpoint = self.last_setpoint;
        if outer_tick {
            // Horizontal position hold (world NED, yaw ~ 0):
            //   a_x = -g·sin(pitch), a_y = +g·sin(roll)
            let x = self.plant.pos.x;
            let y = self.plant.pos.y;
            let vx = self.plant.vel.x;
            let vy = self.plant.vel.y;
            let ax = -(self.outer.kp_xy * x + self.outer.kd_xy * vx);
            let ay = -(self.outer.kp_xy * y + self.outer.kd_xy * vy);
            // a_x = -g·sin(pitch) ⇒ pitch = asin(-a_x/g); a_y = +g·sin(roll).
            let pitch_sp = (-ax / GRAVITY).clamp(-0.35, 0.35).asin();
            let roll_sp = (ay / GRAVITY).clamp(-0.35, 0.35).asin();

            let z = self.plant.pos.z;
            let vz = self.plant.vel.z;
            // Altitude hold: more thrust when below target (z > target_z).
            let thrust = self.outer.hover_thrust
                + self.outer.kp_alt * (z - self.outer.target_z)
                + self.outer.kd_alt * vz;
            let thrust = thrust.clamp(0.0, 1.0);
            let yaw_rate = -self.outer.kp_yaw * yaw;
            setpoint = AttitudeSetpoint {
                roll: roll_sp,
                pitch: pitch_sp,
                yaw_rate,
                thrust,
            };
            self.last_setpoint = setpoint;
        }

        // 4) Build vehicle state for the control laws.
        let state = VehicleState {
            position: self.plant.pos,
            velocity: self.plant.vel,
            attitude: (roll, pitch, yaw),
            body_rates: self.plant.omega,
            pose: None,
            battery: 1.0,
        };

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
        self.max_attitude = self
            .max_attitude
            .max(roll.abs().max(pitch.abs()));

        // 9) Plant integration.
        self.plant.step(dt, &motors);

        // 10) Flight mode: arm -> takeoff -> position hold.
        match self.fsm.mode() {
            tpt_core::FlightMode::Disarmed => {
                let _ = self.fsm.handle(tpt_core::FlightEvent::Arm);
                let _ = self.fsm.handle(tpt_core::FlightEvent::CommandTakeoff);
            }
            tpt_core::FlightMode::Takeoff => {
                if self.plant.pos.z < self.outer.target_z + 0.05 {
                    let _ = self.fsm.handle(tpt_core::FlightEvent::ReachedTargetAltitude);
                }
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
