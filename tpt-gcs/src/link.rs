//! Protocol bridge between the GCS [`Telemetry`]/[`Command`] model and the
//! wire formats in [`tpt_protocols`].
//!
//! Two links are provided:
//! - **TPT-Link** (§12, Phase 3) — the native, compact, encrypted-capable
//!   link. Telemetry rides the `Telemetry` channel and commands the `Command`
//!   channel, both with CRC-16 integrity.
//! - **MAVLink v2** (§12, Phase 1) — telemetry is bridged onto the standard
//!   `ATTITUDE` and `GLOBAL_POSITION_INT` messages so the GCS interoperates
//!   with existing MAVLink ground stations.

use crate::command::Command;
use crate::telemetry::Telemetry;
use tpt_core::FlightMode;
use tpt_math::Vector3;
use tpt_protocols::mavlink;
use tpt_protocols::tptlink::{Channel, parse_plain, serialize_plain};
use tpt_sensor_fusion::FusionMode;

/// TPT-Link message id for a [`Telemetry`] frame.
pub const TELEMETRY_MSGID: u8 = 0x10;
/// TPT-Link message id for a [`Command`] frame.
pub const COMMAND_MSGID: u8 = 0x20;

// --- TPT-Link telemetry -----------------------------------------------------

/// Serialize a [`Telemetry`] frame (CRC-16 protected) into `out`.
pub fn serialize_telemetry(t: &Telemetry, seq: u16, out: &mut [u8]) -> Option<usize> {
    let mut p = [0u8; 64];
    let mut o = 0;
    for v in [
        t.roll,
        t.pitch,
        t.yaw,
        t.position.x,
        t.position.y,
        t.position.z,
        t.velocity.x,
        t.velocity.y,
        t.velocity.z,
        t.battery,
    ] {
        p[o..o + 4].copy_from_slice(&(v as f32).to_le_bytes());
        o += 4;
    }
    p[o] = t.mode as u8;
    o += 1;
    p[o] = t.nav_mode as u8;
    o += 1;
    serialize_plain(out, Channel::Telemetry, TELEMETRY_MSGID, seq, &p[..o])
}

/// Parse a [`Telemetry`] frame from a TPT-Link buffer.
pub fn parse_telemetry(buf: &[u8]) -> Option<Telemetry> {
    let (header, payload) = parse_plain(buf)?;
    if header.msgid != TELEMETRY_MSGID || payload.len() < 42 {
        return None;
    }
    let f = |i: usize| {
        f32::from_le_bytes([payload[i], payload[i + 1], payload[i + 2], payload[i + 3]]) as f64
    };
    let mode = match payload[40] {
        0 => FlightMode::Disarmed,
        1 => FlightMode::Armed,
        2 => FlightMode::Takeoff,
        3 => FlightMode::PositionHold,
        4 => FlightMode::Land,
        5 => FlightMode::Failsafe,
        _ => return None,
    };
    let nav_mode = match payload[41] {
        0 => FusionMode::GpsAided,
        1 => FusionMode::Coast,
        2 => FusionMode::VisualAided,
        3 => FusionMode::TerrainAided,
        _ => return None,
    };
    Some(Telemetry::new(
        f(0),
        f(4),
        f(8),
        Vector3::new(f(12), f(16), f(20)),
        Vector3::new(f(24), f(28), f(32)),
        f(36),
        mode,
        nav_mode,
    ))
}

// --- TPT-Link command -------------------------------------------------------

/// Serialize a [`Command`] frame (CRC-16 protected) into `out`.
pub fn serialize_command(c: &Command, seq: u16, out: &mut [u8]) -> Option<usize> {
    let mut p = [0u8; Command::MAX_LEN];
    let n = c.pack(&mut p)?;
    serialize_plain(out, Channel::Command, COMMAND_MSGID, seq, &p[..n])
}

/// Parse a [`Command`] frame from a TPT-Link buffer.
pub fn parse_command(buf: &[u8]) -> Option<Command> {
    let (header, payload) = parse_plain(buf)?;
    if header.msgid != COMMAND_MSGID {
        return None;
    }
    Command::unpack(payload)
}

// --- MAVLink v2 telemetry bridge -------------------------------------------

/// Bridge a [`Telemetry`] attitude onto a MAVLink `ATTITUDE` message.
pub fn to_mavlink_attitude(t: &Telemetry) -> mavlink::Attitude {
    mavlink::Attitude {
        roll: t.roll as f32,
        pitch: t.pitch as f32,
        yaw: t.yaw as f32,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    }
}

/// Bridge a [`Telemetry`] position onto a MAVLink `GLOBAL_POSITION_INT`.
///
/// Local NED meters are mapped to a synthetic geodetic offset for display:
/// `x` north and `y` east drive a small lat/lon delta, and `z` (down-positive)
/// drives the relative altitude (up-positive, mm). This is a GCS display aid,
/// not a geodetic transform.
pub fn to_mavlink_global_position(t: &Telemetry, time_boot_ms: u32) -> mavlink::GlobalPositionInt {
    // ~1.0 / 1.11e5 deg per meter at the equator, scaled to 1e7 integer deg.
    let lat = (t.position.x * 1.0 / 1.11e5 * 1e7) as i32;
    let lon = (t.position.y * 1.0 / 1.11e5 * 1e7) as i32;
    let relative_alt = (-t.position.z * 1000.0) as i32; // up-positive mm
    let hdg = (t.yaw.to_degrees().rem_euclid(360.0) * 100.0) as u16;
    mavlink::GlobalPositionInt {
        time_boot_ms,
        lat,
        lon,
        alt: 0,
        relative_alt,
        vx: (t.velocity.x * 100.0) as i16,
        vy: (t.velocity.y * 100.0) as i16,
        vz: (-t.velocity.z * 100.0) as i16,
        hdg,
    }
}

/// Extract `(roll, pitch, yaw)` from a MAVLink `ATTITUDE`.
pub fn from_mavlink_attitude(a: &mavlink::Attitude) -> (f64, f64, f64) {
    (a.roll as f64, a.pitch as f64, a.yaw as f64)
}

/// Extract `(relative_altitude_m, yaw_rad)` from a MAVLink `GLOBAL_POSITION_INT`.
pub fn from_mavlink_global_position(g: &mavlink::GlobalPositionInt) -> (f64, f64) {
    let alt_m = g.relative_alt as f64 / 1000.0;
    let yaw = (g.hdg as f64 / 100.0).to_radians();
    (alt_m, yaw)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Telemetry {
        Telemetry::new(
            0.1,
            -0.2,
            1.57,
            Vector3::new(5.0, 5.0, -2.0),
            Vector3::new(0.5, 0.0, 0.1),
            0.87,
            FlightMode::PositionHold,
            FusionMode::VisualAided,
        )
    }

    #[test]
    fn tptlink_telemetry_round_trip() {
        let t = sample();
        let mut out = [0u8; 128];
        let n = serialize_telemetry(&t, 7, &mut out).unwrap();
        let parsed = parse_telemetry(&out[..n]).unwrap();
        assert!((parsed.roll - t.roll).abs() < 1e-3);
        assert!((parsed.position.x - t.position.x).abs() < 1e-3);
        assert_eq!(parsed.mode, FlightMode::PositionHold);
        assert_eq!(parsed.nav_mode, FusionMode::VisualAided);
        assert!((parsed.battery - t.battery).abs() < 1e-3);
    }

    #[test]
    fn tptlink_telemetry_rejects_wrong_channel() {
        let mut out = [0u8; 128];
        let n = serialize_command(&Command::Arm, 0, &mut out).unwrap();
        assert!(parse_telemetry(&out[..n]).is_none());
    }

    #[test]
    fn tptlink_command_round_trip() {
        for c in [
            Command::Arm,
            Command::Land,
            Command::SetWaypoint {
                x: 1.0,
                y: 2.0,
                z: -3.0,
                yaw: 0.5,
            },
            Command::SetMode(FlightMode::Takeoff),
        ] {
            let mut out = [0u8; 128];
            let n = serialize_command(&c, 3, &mut out).unwrap();
            assert_eq!(parse_command(&out[..n]), Some(c));
        }
    }

    #[test]
    fn mavlink_attitude_round_trip() {
        let t = sample();
        let a = to_mavlink_attitude(&t);
        let (r, p, y) = from_mavlink_attitude(&a);
        assert!((r - t.roll).abs() < 1e-3);
        assert!((y - t.yaw).abs() < 1e-3);
        let _ = p;
    }

    #[test]
    fn mavlink_global_position_altitude() {
        let t = sample();
        let g = to_mavlink_global_position(&t, 1000);
        let (alt_m, yaw) = from_mavlink_global_position(&g);
        assert!((alt_m - 2.0).abs() < 1e-2); // relative alt up = +2 m
        assert!((yaw - t.yaw).abs() < 1e-2);
    }
}
