//! Deterministic flight-log record / replay / diff (`spec.txt` §14).
//!
//! The closed loop in [`crate::GpsDeniedSim`] is a pure function of its initial
//! state and its sensor history: [`GpsDeniedSim::sense`] produces, for every
//! control step, the exact [`SenseBatch`] the control laws consume, and
//! [`GpsDeniedSim::apply`] advances the control stack + plant from that batch.
//! This module records those batches (a *flight log*), re-runs the *same*
//! control laws from the log, and diffs the reproduced telemetry against the
//! originally recorded frames.
//!
//! Because the control laws are deterministic, a faithful replay reproduces the
//! original telemetry bit-for-bit. Any non-zero diff therefore indicates
//! non-determinism or a state/configuration mismatch — exactly the class of
//! regression this tool exists to catch.

use crate::scenarios::{GpsDeniedSim, ObstacleField, Scenario, SenseBatch};
use tpt_core::{FlightMode, PositionTarget};
use tpt_math::Vector3;
use tpt_sensor_fusion::FusionMode;

/// One recorded control step: the sensor batch plus the telemetry the vehicle
/// reported as a result of the step.
#[derive(Debug, Clone)]
pub struct LogFrame {
    /// Sensor inputs that drove the step.
    pub batch: SenseBatch,
    /// Vehicle NED position after the step (m).
    pub pos: Vector3<f64>,
    /// Vehicle NED velocity after the step (m/s).
    pub vel: Vector3<f64>,
    /// Attitude after the step (rad), NED convention.
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    /// Active navigation fusion mode after the step.
    pub nav_mode: FusionMode,
    /// Active flight mode after the step.
    pub flight_mode: FlightMode,
    /// EKF position 1σ uncertainty (m).
    pub uncert: f64,
    /// Sum of the four motor commands (collective thrust proxy).
    pub motor_sum: f64,
}

/// A full recorded flight: configuration header + per-step frames.
#[derive(Debug, Clone)]
pub struct FlightLog {
    /// Scenario the run was executed under.
    pub scenario: Scenario,
    /// Mission waypoint.
    pub target: PositionTarget,
    /// Integration step (s).
    pub dt: f64,
    /// Number of control steps recorded.
    pub steps: u64,
    /// Optional obstacle field as `(center, radius)` pairs.
    pub obstacles: Option<Vec<(Vector3<f64>, f64)>>,
    /// Recorded frames, in step order.
    pub frames: Vec<LogFrame>,
}

/// Capture the telemetry the vehicle reports after one step.
fn capture(sim: &GpsDeniedSim) -> LogFrame {
    let (roll, pitch, yaw) = sim.plant().quat.euler_angles();
    LogFrame {
        batch: SenseBatch {
            // `capture` is called *after* `apply`, whose `SenseBatch` is not
            // retained by the sim; the replay path instead reads the batch from
            // the log, so this field is filled by the caller.
            accel: Vector3::zeros(),
            gyro: Vector3::zeros(),
            gps: None,
            vio: None,
        },
        pos: sim.plant().pos,
        vel: sim.plant().vel,
        roll,
        pitch,
        yaw,
        nav_mode: sim.fusion_mode(),
        flight_mode: sim.flight_mode(),
        uncert: sim.uncertainty(),
        motor_sum: sim.motor_sum(),
    }
}

/// Record `seconds` of a GPS-denied simulation into a [`FlightLog`].
///
/// Uses [`GpsDeniedSim::sense`] + [`GpsDeniedSim::apply`] so the recorded
/// [`SenseBatch`]es are exactly the inputs the replay consumes.
pub fn record(sim: &mut GpsDeniedSim, seconds: f64, dt: f64) -> FlightLog {
    let steps = (seconds / dt).round() as u64;
    let mut frames = Vec::with_capacity(steps as usize);
    for _ in 0..steps {
        sim.advance_tick();
        let batch = sim.sense(dt);
        sim.apply(dt, batch.clone());
        let mut frame = capture(sim);
        frame.batch = batch;
        frames.push(frame);
    }
    FlightLog {
        scenario: sim.scenario(),
        target: sim.target(),
        dt,
        steps,
        obstacles: sim.obstacle_spheres(),
        frames,
    }
}

/// Replay a [`FlightLog`] through a fresh [`GpsDeniedSim`] and return the
/// reproduced log. The reproduced telemetry is directly comparable to the
/// original via [`diff`].
pub fn replay(log: &FlightLog) -> FlightLog {
    let mut sim = GpsDeniedSim::new(log.scenario);
    sim.set_target(log.target);
    if let Some(spheres) = &log.obstacles {
        sim.set_obstacles(ObstacleField::from_spheres(spheres));
    }
    let mut frames = Vec::with_capacity(log.frames.len());
    for f in &log.frames {
        sim.advance_tick();
        sim.apply(log.dt, f.batch.clone());
        let mut frame = capture(&sim);
        frame.batch = f.batch.clone();
        frames.push(frame);
    }
    FlightLog {
        scenario: log.scenario,
        target: log.target,
        dt: log.dt,
        steps: log.steps,
        obstacles: log.obstacles.clone(),
        frames,
    }
}

/// Per-field worst-case deviation between an original log and its replay.
#[derive(Debug, Clone, Copy, Default)]
pub struct ReplayDiff {
    /// Maximum position deviation across all frames (m).
    pub max_pos: f64,
    /// Maximum velocity deviation across all frames (m/s).
    pub max_vel: f64,
    /// Maximum attitude (roll/pitch/yaw) deviation across all frames (rad).
    pub max_att: f64,
    /// Maximum EKF position-uncertainty deviation (m).
    pub max_uncert: f64,
    /// Maximum collective motor-sum deviation.
    pub max_motor_sum: f64,
    /// Number of frames compared.
    pub frames: usize,
}

/// Diff two logs frame-by-frame. The first argument is the original recording,
/// the second the replay. Frames are compared in order; mismatched lengths
/// compare only the shared prefix.
pub fn diff(original: &FlightLog, replayed: &FlightLog) -> ReplayDiff {
    let n = original.frames.len().min(replayed.frames.len());
    let mut d = ReplayDiff {
        frames: n,
        ..Default::default()
    };
    for i in 0..n {
        let a = &original.frames[i];
        let b = &replayed.frames[i];
        d.max_pos = d.max_pos.max((a.pos - b.pos).norm());
        d.max_vel = d.max_vel.max((a.vel - b.vel).norm());
        d.max_att = d.max_att.max(
            (a.roll - b.roll)
                .abs()
                .max((a.pitch - b.pitch).abs())
                .max((a.yaw - b.yaw).abs()),
        );
        d.max_uncert = d.max_uncert.max((a.uncert - b.uncert).abs());
        d.max_motor_sum = d.max_motor_sum.max((a.motor_sum - b.motor_sum).abs());
    }
    d
}

/// Serialize the log to a compact, dependency-free CSV so it can be saved and
/// reloaded (e.g. to diff a freshly-built control crate against a previously
/// recorded golden flight). Returns `(header, body)` lines.
pub fn to_csv(log: &FlightLog) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "# scenario={:?} dt={} steps={} target={},{},{} obstacles={}\n",
        log.scenario,
        log.dt,
        log.steps,
        log.target.x,
        log.target.y,
        log.target.z,
        log.obstacles.as_ref().map_or(0, |o| o.len()),
    ));
    if let Some(spheres) = &log.obstacles {
        for (c, r) in spheres {
            out.push_str(&format!("# obstacle {} {} {} {}\n", c.x, c.y, c.z, r));
        }
    }
    out.push_str(
        "# ax,ay,az,gx,gy,gz,gv,gnx,gny,gnz,gvx,gvy,gvz,gvn,gvn2,vvx,vvy,vvz,vvyaw,vvn,vvyn,px,py,pz,vx,vy,vz,roll,pitch,yaw,nav,flight,unc,motors\n",
    );
    for f in &log.frames {
        out.push_str(&frame_to_csv(f));
        out.push('\n');
    }
    out
}

/// CSV column layout used by [`to_csv`] / [`from_csv`].
const COLUMNS: usize = 33;

fn frame_to_csv(f: &LogFrame) -> String {
    let b = &f.batch;
    let (ax, ay, az) = (b.accel.x, b.accel.y, b.accel.z);
    let (gx, gy, gz) = (b.gyro.x, b.gyro.y, b.gyro.z);
    let (gnx, gny, gnz, gvx, gvy, gvz, gvn, gvn2) = match b.gps {
        Some((p, v, pn, vn)) => (p.x, p.y, p.z, v.x, v.y, v.z, pn, vn),
        None => (
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
            f64::NAN,
        ),
    };
    let (vvx, vvy, vvz, vvyaw, vvn, vvyn) = match b.vio {
        Some((p, yaw, pn, yn)) => (p.x, p.y, p.z, yaw, pn, yn),
        None => (f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN, f64::NAN),
    };
    format!(
        "{ax},{ay},{az},{gx},{gy},{gz},{gnx},{gny},{gnz},{gvx},{gvy},{gvz},{gvn},{gvn2},{vvx},{vvy},{vvz},{vvyaw},{vvn},{vvyn},{px},{py},{pz},{vx},{vy},{vz},{roll},{pitch},{yaw},{nav},{flight},{unc},{motors}",
        px = f.pos.x,
        py = f.pos.y,
        pz = f.pos.z,
        vx = f.vel.x,
        vy = f.vel.y,
        vz = f.vel.z,
        roll = f.roll,
        pitch = f.pitch,
        yaw = f.yaw,
        nav = nav_code(f.nav_mode),
        flight = flight_code(f.flight_mode),
        unc = f.uncert,
        motors = f.motor_sum,
    )
}

fn nav_code(m: FusionMode) -> u8 {
    match m {
        FusionMode::GpsAided => 0,
        FusionMode::Coast => 1,
        FusionMode::VisualAided => 2,
        FusionMode::TerrainAided => 3,
    }
}

fn flight_code(m: FlightMode) -> u8 {
    match m {
        FlightMode::Disarmed => 0,
        FlightMode::Armed => 1,
        FlightMode::Takeoff => 2,
        FlightMode::PositionHold => 3,
        FlightMode::Land => 4,
        FlightMode::Failsafe => 5,
        FlightMode::Glide => 6,
    }
}

fn nav_from_code(c: u8) -> FusionMode {
    match c {
        1 => FusionMode::Coast,
        2 => FusionMode::VisualAided,
        3 => FusionMode::TerrainAided,
        _ => FusionMode::GpsAided,
    }
}

fn flight_from_code(c: u8) -> FlightMode {
    match c {
        1 => FlightMode::Armed,
        2 => FlightMode::Takeoff,
        3 => FlightMode::PositionHold,
        4 => FlightMode::Land,
        5 => FlightMode::Failsafe,
        6 => FlightMode::Glide,
        _ => FlightMode::Disarmed,
    }
}

fn parse_f64(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(f64::NAN)
}

/// Parse a [`to_csv`]-produced flight log back into a [`FlightLog`].
///
/// Tolerant of the `#` comment/header lines; returns `None` if the body is
/// empty or malformed.
pub fn from_csv(text: &str) -> Option<FlightLog> {
    let mut scenario = Scenario::Nominal;
    let mut dt = 0.001;
    let mut steps = 0u64;
    let mut target = PositionTarget::origin();
    let mut obstacles: Option<Vec<(Vector3<f64>, f64)>> = None;
    let mut frames: Vec<LogFrame> = Vec::new();

    for line in text.lines() {
        if line.starts_with("# scenario=") {
            // `# scenario=Jamming dt=0.001 steps=30000 target=5,5,-2 obstacles=1`
            let body = &line[1..];
            for kv in body.split_whitespace() {
                if let Some(v) = kv.strip_prefix("scenario=") {
                    scenario = match v {
                        "Nominal" => Scenario::Nominal,
                        "UrbanCanyon" => Scenario::UrbanCanyon,
                        "Jamming" => Scenario::Jamming,
                        "Indoor" => Scenario::Indoor,
                        "SensorDegradation" => Scenario::SensorDegradation,
                        "TotalBlackout" => Scenario::TotalBlackout,
                        _ => Scenario::Nominal,
                    };
                } else if let Some(v) = kv.strip_prefix("dt=") {
                    dt = v.parse().unwrap_or(dt);
                } else if let Some(v) = kv.strip_prefix("steps=") {
                    steps = v.parse().unwrap_or(steps);
                } else if let Some(v) = kv.strip_prefix("target=") {
                    let parts: Vec<&str> = v.split(',').collect();
                    if parts.len() == 3 {
                        target = PositionTarget {
                            x: parse_f64(parts[0]),
                            y: parse_f64(parts[1]),
                            z: parse_f64(parts[2]),
                            ..PositionTarget::origin()
                        };
                    }
                } else if let Some(v) = kv.strip_prefix("obstacles=") {
                    let n: usize = v.parse().unwrap_or(0);
                    if n > 0 {
                        obstacles = Some(Vec::with_capacity(n));
                    }
                }
            }
        } else if let Some(rest) = line.strip_prefix("# obstacle ") {
            let body = rest.trim();
            let parts: Vec<&str> = body.split_whitespace().collect();
            if parts.len() == 4 {
                if let Some(o) = obstacles.as_mut() {
                    o.push((
                        Vector3::new(
                            parse_f64(parts[0]),
                            parse_f64(parts[1]),
                            parse_f64(parts[2]),
                        ),
                        parse_f64(parts[3]),
                    ));
                }
            }
        } else if line.starts_with('#') || line.trim().is_empty() {
            continue;
        } else {
            let c: Vec<&str> = line.split(',').collect();
            if c.len() != COLUMNS {
                continue;
            }
            // Skip the (commented) header row and any non-numeric line: the
            // first column is always a finite accelerometer reading in a real
            // frame.
            if c[0].parse::<f64>().is_err() {
                continue;
            }
            let f = LogFrame {
                batch: SenseBatch {
                    accel: Vector3::new(parse_f64(c[0]), parse_f64(c[1]), parse_f64(c[2])),
                    gyro: Vector3::new(parse_f64(c[3]), parse_f64(c[4]), parse_f64(c[5])),
                    gps: if c[6].parse::<f64>().is_ok_and(|v| v.is_nan()) {
                        None
                    } else {
                        Some((
                            Vector3::new(parse_f64(c[6]), parse_f64(c[7]), parse_f64(c[8])),
                            Vector3::new(parse_f64(c[9]), parse_f64(c[10]), parse_f64(c[11])),
                            parse_f64(c[12]),
                            parse_f64(c[13]),
                        ))
                    },
                    vio: if c[14].parse::<f64>().is_ok_and(|v| v.is_nan()) {
                        None
                    } else {
                        Some((
                            Vector3::new(parse_f64(c[14]), parse_f64(c[15]), parse_f64(c[16])),
                            parse_f64(c[17]),
                            parse_f64(c[18]),
                            parse_f64(c[19]),
                        ))
                    },
                },
                pos: Vector3::new(parse_f64(c[20]), parse_f64(c[21]), parse_f64(c[22])),
                vel: Vector3::new(parse_f64(c[23]), parse_f64(c[24]), parse_f64(c[25])),
                roll: parse_f64(c[26]),
                pitch: parse_f64(c[27]),
                yaw: parse_f64(c[28]),
                nav_mode: nav_from_code(parse_f64(c[29]) as u8),
                flight_mode: flight_from_code(parse_f64(c[30]) as u8),
                uncert: parse_f64(c[31]),
                motor_sum: parse_f64(c[32]),
            };
            frames.push(f);
        }
    }

    if frames.is_empty() {
        return None;
    }
    Some(FlightLog {
        scenario,
        target,
        dt,
        steps: steps.max(frames.len() as u64),
        obstacles,
        frames,
    })
}

/// Convenience: record + replay an in-memory log and return the diff. A
/// `max_pos` of ~0 (within tolerance) is the determinism guarantee.
pub fn record_and_diff(sim: &mut GpsDeniedSim, seconds: f64, dt: f64) -> (FlightLog, ReplayDiff) {
    let log = record(sim, seconds, dt);
    let replayed = replay(&log);
    let d = diff(&log, &replayed);
    (log, d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_core::PositionTarget;

    fn waypoint() -> PositionTarget {
        let mut t = PositionTarget::origin();
        t.x = 5.0;
        t.y = 5.0;
        t.z = -2.0;
        t
    }

    #[test]
    fn replay_is_bit_for_bit_deterministic() {
        let mut sim = GpsDeniedSim::new(Scenario::Jamming);
        sim.set_target(waypoint());
        let (log, d) = record_and_diff(&mut sim, 30.0, 0.001);
        assert_eq!(d.frames, log.frames.len());
        assert!(d.max_pos < 1e-9, "max_pos = {:e}", d.max_pos);
        assert!(d.max_vel < 1e-9, "max_vel = {:e}", d.max_vel);
        assert!(d.max_att < 1e-9, "max_att = {:e}", d.max_att);
        assert!(d.max_uncert < 1e-9, "max_uncert = {:e}", d.max_uncert);
        assert!(
            d.max_motor_sum < 1e-9,
            "max_motor_sum = {:e}",
            d.max_motor_sum
        );
    }

    #[test]
    fn csv_round_trips() {
        let mut sim = GpsDeniedSim::new(Scenario::UrbanCanyon);
        sim.set_target(waypoint());
        let mut field = ObstacleField::new();
        field.add(Vector3::new(2.5, 2.5, -2.0), 1.5);
        sim.set_obstacles(field);
        let log = record(&mut sim, 20.0, 0.001);
        let csv = to_csv(&log);
        let back = from_csv(&csv).expect("parse csv");
        assert_eq!(back.scenario, log.scenario);
        assert_eq!(back.frames.len(), log.frames.len());
        assert!(back.obstacles.is_some());
        assert_eq!(back.obstacles.as_ref().unwrap().len(), 1);
        let d = diff(&log, &back);
        assert!(d.max_pos < 1e-9, "csv max_pos = {:e}", d.max_pos);
    }

    #[test]
    fn replay_after_csv_is_still_deterministic() {
        let mut sim = GpsDeniedSim::new(Scenario::SensorDegradation);
        sim.set_target(waypoint());
        let log = record(&mut sim, 15.0, 0.001);
        let csv = to_csv(&log);
        let parsed = from_csv(&csv).expect("parse");
        let replayed = replay(&parsed);
        let d = diff(&parsed, &replayed);
        assert!(d.max_pos < 1e-9, "max_pos = {:e}", d.max_pos);
    }
}
