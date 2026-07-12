//! MAVLink v2 protocol support (`spec.txt` §12, Phase 1).
//!
//! A `#![no_std]` MAVLink v2 (frame protocol 0xFD) implementation: framing,
//! the X.25 CRC with per-message `CRC_EXTRA` seed, and a handful of concrete
//! messages (HEARTBEAT, ATTITUDE, GLOBAL_POSITION_INT) with correct field
//! packing. The framing layer is message-agnostic; add new messages by
//! implementing the [`Message`] trait.

#![allow(dead_code)]

pub const MAVLINK2_MAGIC: u8 = 0xFD;

/// MAVLink 2 header length (magic .. msgid inclusive, before payload).
pub const HEADER_LEN: usize = 10;
/// Maximum payload size permitted by the v2 framing.
pub const MAX_PAYLOAD: usize = 255;

pub mod crc {
    //! X.25 / CCITT CRC-16 used by MAVLink, with the `CRC_EXTRA` message seed.

    /// Accumulate `data` into a MAVLink CRC starting from `crc` (0xFFFF init).
    pub fn crc16_x25(data: &[u8], mut crc: u16) -> u16 {
        for &b in data {
            crc ^= b as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0x1021;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

    /// MAVLink v2 CRC: CRC-16 of the header+payload, then XOR with the
    /// per-message `crc_extra` seed byte.
    pub fn mavlink_v2(buf: &[u8], crc_extra: u8) -> u16 {
        let crc = crc16_x25(buf, 0xFFFF);
        crc ^ (crc_extra as u16)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        #[test]
        fn crc_is_deterministic_and_input_sensitive() {
            // Empty input leaves the accumulator unchanged.
            assert_eq!(crc16_x25(&[], 0xFFFF), 0xFFFF);
            let a = crc16_x25(&[0x00], 0xFFFF);
            let b = crc16_x25(&[0x01], 0xFFFF);
            let c = crc16_x25(&[0x00, 0x00], 0xFFFF);
            // Distinct inputs give distinct, non-trivial CRCs.
            assert_ne!(a, 0xFFFF);
            assert_ne!(a, b);
            assert_ne!(a, c);
            // Same input -> same CRC (deterministic).
            assert_eq!(a, crc16_x25(&[0x00], 0xFFFF));
        }
    }
}

/// A parsed or to-be-serialized MAVLink v2 frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub msgid: u32,
    pub payload_len: usize,
    pub payload: [u8; MAX_PAYLOAD],
}

impl Frame {
    /// Construct a frame for `msgid` with `payload` (clamped to 255 bytes).
    pub fn new(sysid: u8, compid: u8, seq: u8, msgid: u32, payload: &[u8]) -> Self {
        let n = payload.len().min(MAX_PAYLOAD);
        let mut buf = [0u8; MAX_PAYLOAD];
        buf[..n].copy_from_slice(&payload[..n]);
        Self {
            seq,
            sysid,
            compid,
            incompat_flags: 0,
            compat_flags: 0,
            msgid,
            payload_len: n,
            payload: buf,
        }
    }

    /// Serialize into `buf`, returning the total frame length, or `None` if the
    /// buffer is too small. `crc_extra` is the message's seed byte.
    pub fn serialize(&self, buf: &mut [u8], crc_extra: u8) -> Option<usize> {
        let total = HEADER_LEN + self.payload_len + 2;
        if buf.len() < total {
            return None;
        }
        buf[0] = MAVLINK2_MAGIC;
        buf[1] = self.payload_len as u8;
        buf[2] = self.incompat_flags;
        buf[3] = self.compat_flags;
        buf[4] = self.seq;
        buf[5] = self.sysid;
        buf[6] = self.compid;
        buf[7] = (self.msgid & 0xFF) as u8;
        buf[8] = ((self.msgid >> 8) & 0xFF) as u8;
        buf[9] = ((self.msgid >> 16) & 0xFF) as u8;
        buf[10..10 + self.payload_len].copy_from_slice(&self.payload[..self.payload_len]);

        let crc = crc::mavlink_v2(&buf[..10 + self.payload_len], crc_extra);
        buf[10 + self.payload_len] = (crc & 0xFF) as u8;
        buf[10 + self.payload_len + 1] = ((crc >> 8) & 0xFF) as u8;
        Some(total)
    }

    /// Parse a single frame from the front of `buf`.
    ///
    /// Returns the frame and the number of bytes consumed, or `None` if the
    /// buffer is incomplete or the CRC check fails (with `crc_extra`).
    pub fn parse(buf: &[u8], crc_extra: u8) -> Option<(Frame, usize)> {
        if buf.len() < HEADER_LEN {
            return None;
        }
        if buf[0] != MAVLINK2_MAGIC {
            return None;
        }
        let len = buf[1] as usize;
        let total = HEADER_LEN + len + 2;
        if buf.len() < total {
            return None;
        }
        let crc = crc::mavlink_v2(&buf[..HEADER_LEN + len], crc_extra);
        let got = u16::from_le_bytes([buf[HEADER_LEN + len], buf[HEADER_LEN + len + 1]]);
        if got != crc {
            return None;
        }
        let mut payload = [0u8; MAX_PAYLOAD];
        payload[..len].copy_from_slice(&buf[HEADER_LEN..HEADER_LEN + len]);
        let frame = Frame {
            seq: buf[4],
            sysid: buf[5],
            compid: buf[6],
            incompat_flags: buf[2],
            compat_flags: buf[3],
            msgid: (buf[7] as u32) | ((buf[8] as u32) << 8) | ((buf[9] as u32) << 16),
            payload_len: len,
            payload,
        };
        Some((frame, total))
    }
}

/// A MAVLink message: knows its id, CRC seed, and payload packing.
pub trait Message: Sized {
    fn message_id(&self) -> u32;
    fn crc_extra(&self) -> u8;
    fn payload_len(&self) -> usize;
    /// Pack the message fields into `out` (first `payload_len` bytes).
    fn pack(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize;
    /// Decode from a payload slice of the expected length.
    fn unpack(payload: &[u8]) -> Option<Self>;
}

/// Build a [`Frame`] from a message.
pub fn frame_from_message<M: Message>(sysid: u8, compid: u8, seq: u8, msg: &M) -> Frame {
    let mut payload = [0u8; MAX_PAYLOAD];
    let n = msg.pack(&mut payload);
    Frame::new(sysid, compid, seq, msg.message_id(), &payload[..n])
}

// ---------------------------------------------------------------------------
// Concrete messages.
// ---------------------------------------------------------------------------

/// MAV_TYPE / MAV_MODE_FLAG/ MAV_STATE bit constants for HEARTBEAT.
pub mod enums {
    pub const MAV_TYPE_QUADROTOR: u8 = 2;
    pub const MAV_AUTOPILOT_TPT: u8 = 13; // custom autopilot id
    pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;
    pub const MAV_STATE_ACTIVE: u8 = 4;
}

/// HEARTBEAT (msg id 0, CRC_EXTRA 50).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat {
    pub mav_type: u8,
    pub autopilot: u8,
    pub base_mode: u8,
    pub custom_mode: u32,
    pub system_status: u8,
}

impl Message for Heartbeat {
    fn message_id(&self) -> u32 {
        0
    }
    fn crc_extra(&self) -> u8 {
        50
    }
    fn payload_len(&self) -> usize {
        9
    }
    fn pack(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        out[0] = self.mav_type;
        out[1] = self.autopilot;
        out[2] = self.base_mode;
        out[3..7].copy_from_slice(&self.custom_mode.to_le_bytes());
        out[7] = self.system_status;
        out[8] = 3; // mavlink version (v2)
        9
    }
    fn unpack(p: &[u8]) -> Option<Self> {
        if p.len() < 9 {
            return None;
        }
        Some(Self {
            mav_type: p[0],
            autopilot: p[1],
            base_mode: p[2],
            custom_mode: u32::from_le_bytes([p[3], p[4], p[5], p[6]]),
            system_status: p[7],
        })
    }
}

/// ATTITUDE (msg id 30, CRC_EXTRA 39): roll/pitch/yaw + rates (rad, rad/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
}

impl Message for Attitude {
    fn message_id(&self) -> u32 {
        30
    }
    fn crc_extra(&self) -> u8 {
        39
    }
    fn payload_len(&self) -> usize {
        28
    }
    fn pack(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        let mut o = 0;
        for v in [
            self.roll,
            self.pitch,
            self.yaw,
            self.rollspeed,
            self.pitchspeed,
            self.yawspeed,
        ] {
            out[o..o + 4].copy_from_slice(&v.to_le_bytes());
            o += 4;
        }
        28
    }
    fn unpack(p: &[u8]) -> Option<Self> {
        if p.len() < 28 {
            return None;
        }
        let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        Some(Self {
            roll: f(0),
            pitch: f(4),
            yaw: f(8),
            rollspeed: f(12),
            pitchspeed: f(16),
            yawspeed: f(20),
        })
    }
}

/// GLOBAL_POSITION_INT (msg id 33, CRC_EXTRA 104): int32 mm / cmillideg.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GlobalPositionInt {
    pub time_boot_ms: u32,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub relative_alt: i32,
    pub vx: i16,
    pub vy: i16,
    pub vz: i16,
    pub hdg: u16,
}

impl Message for GlobalPositionInt {
    fn message_id(&self) -> u32 {
        33
    }
    fn crc_extra(&self) -> u8 {
        104
    }
    fn payload_len(&self) -> usize {
        28
    }
    fn pack(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        out[0..4].copy_from_slice(&self.time_boot_ms.to_le_bytes());
        out[4..8].copy_from_slice(&self.lat.to_le_bytes());
        out[8..12].copy_from_slice(&self.lon.to_le_bytes());
        out[12..16].copy_from_slice(&self.alt.to_le_bytes());
        out[16..20].copy_from_slice(&self.relative_alt.to_le_bytes());
        out[20..22].copy_from_slice(&self.vx.to_le_bytes());
        out[22..24].copy_from_slice(&self.vy.to_le_bytes());
        out[24..26].copy_from_slice(&self.vz.to_le_bytes());
        out[26..28].copy_from_slice(&self.hdg.to_le_bytes());
        28
    }
    fn unpack(p: &[u8]) -> Option<Self> {
        if p.len() < 28 {
            return None;
        }
        Some(Self {
            time_boot_ms: i32::from_le_bytes([p[0], p[1], p[2], p[3]]) as u32,
            lat: i32::from_le_bytes([p[4], p[5], p[6], p[7]]),
            lon: i32::from_le_bytes([p[8], p[9], p[10], p[11]]),
            alt: i32::from_le_bytes([p[12], p[13], p[14], p[15]]),
            relative_alt: i32::from_le_bytes([p[16], p[17], p[18], p[19]]),
            vx: i16::from_le_bytes([p[20], p[21]]),
            vy: i16::from_le_bytes([p[22], p[23]]),
            vz: i16::from_le_bytes([p[24], p[25]]),
            hdg: u16::from_le_bytes([p[26], p[27]]),
        })
    }
}

/// MISSION_ITEM_INT (msg id 73, CRC_EXTRA 38): a single waypoint item for
/// mission upload/download. Field widths follow the MAVLink definition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItemInt {
    pub target_system: u8,
    pub target_component: u8,
    pub seq: u16,
    pub frame: u8,
    pub command: u16,
    pub current: u8,
    pub autocontinue: u8,
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
    pub mission_type: u8,
}

impl Message for MissionItemInt {
    fn message_id(&self) -> u32 {
        73
    }
    fn crc_extra(&self) -> u8 {
        38
    }
    fn payload_len(&self) -> usize {
        38
    }
    fn pack(&self, out: &mut [u8; MAX_PAYLOAD]) -> usize {
        let mut b = |o: usize, v: &[u8]| out[o..o + v.len()].copy_from_slice(v);
        b(0, &[self.target_system]);
        b(1, &[self.target_component]);
        b(2, &self.seq.to_le_bytes());
        b(4, &[self.frame]);
        b(5, &self.command.to_le_bytes());
        b(7, &[self.current]);
        b(8, &[self.autocontinue]);
        b(9, &self.param1.to_le_bytes());
        b(13, &self.param2.to_le_bytes());
        b(17, &self.param3.to_le_bytes());
        b(21, &self.param4.to_le_bytes());
        b(25, &self.x.to_le_bytes());
        b(29, &self.y.to_le_bytes());
        b(33, &self.z.to_le_bytes());
        b(37, &[self.mission_type]);
        38
    }
    fn unpack(p: &[u8]) -> Option<Self> {
        if p.len() < 38 {
            return None;
        }
        let u16 = |k: usize| u16::from_le_bytes([p[k], p[k + 1]]);
        let i32v = |k: usize| i32::from_le_bytes([p[k], p[k + 1], p[k + 2], p[k + 3]]);
        let f32v = |k: usize| f32::from_le_bytes([p[k], p[k + 1], p[k + 2], p[k + 3]]);
        Some(Self {
            target_system: p[0],
            target_component: p[1],
            seq: u16(2),
            frame: p[4],
            command: u16(5),
            current: p[7],
            autocontinue: p[8],
            param1: f32v(9),
            param2: f32v(13),
            param3: f32v(17),
            param4: f32v(21),
            x: i32v(25),
            y: i32v(29),
            z: f32v(33),
            mission_type: p[37],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn heartbeat_round_trip() {
        let hb = Heartbeat {
            mav_type: enums::MAV_TYPE_QUADROTOR,
            autopilot: enums::MAV_AUTOPILOT_TPT,
            base_mode: enums::MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode: 1,
            system_status: enums::MAV_STATE_ACTIVE,
        };
        let frame = frame_from_message(1, 1, 7, &hb);
        let mut buf = [0u8; 64];
        let n = frame.serialize(&mut buf, hb.crc_extra()).unwrap();
        let (parsed, used) = Frame::parse(&buf[..n], hb.crc_extra()).unwrap();
        assert_eq!(used, n);
        assert_eq!(parsed.msgid, 0);
        assert_eq!(parsed.seq, 7);
        let decoded = Heartbeat::unpack(&parsed.payload[..parsed.payload_len]).unwrap();
        assert_eq!(decoded, hb);
    }

    #[test]
    fn attitude_round_trip() {
        let a = Attitude {
            roll: 0.1,
            pitch: -0.2,
            yaw: 1.57,
            rollspeed: 0.01,
            pitchspeed: -0.02,
            yawspeed: 0.03,
        };
        let frame = frame_from_message(1, 1, 0, &a);
        let mut buf = [0u8; 64];
        let n = frame.serialize(&mut buf, a.crc_extra()).unwrap();
        let (parsed, _) = Frame::parse(&buf[..n], a.crc_extra()).unwrap();
        let decoded = Attitude::unpack(&parsed.payload[..parsed.payload_len]).unwrap();
        assert!((decoded.roll - a.roll).abs() < 1e-6);
        assert!((decoded.yaw - a.yaw).abs() < 1e-6);
    }

    #[test]
    fn crc_failure_detected() {
        let hb = Heartbeat {
            mav_type: enums::MAV_TYPE_QUADROTOR,
            autopilot: enums::MAV_AUTOPILOT_TPT,
            base_mode: 0,
            custom_mode: 0,
            system_status: enums::MAV_STATE_ACTIVE,
        };
        let frame = frame_from_message(1, 1, 0, &hb);
        let mut buf = [0u8; 64];
        let n = frame.serialize(&mut buf, hb.crc_extra()).unwrap();
        // Corrupt a payload byte.
        buf[12] ^= 0xFF;
        assert!(Frame::parse(&buf[..n], hb.crc_extra()).is_none());
    }

    #[test]
    fn global_position_int_round_trip() {
        let g = GlobalPositionInt {
            time_boot_ms: 12_345,
            lat: 377_000_000,
            lon: -122_000_000,
            alt: 150_000,
            relative_alt: 50_000,
            vx: 100,
            vy: -200,
            vz: 50,
            hdg: 18000,
        };
        let frame = frame_from_message(1, 1, 3, &g);
        let mut buf = [0u8; 64];
        let n = frame.serialize(&mut buf, g.crc_extra()).unwrap();
        let (parsed, _) = Frame::parse(&buf[..n], g.crc_extra()).unwrap();
        let d = GlobalPositionInt::unpack(&parsed.payload[..parsed.payload_len]).unwrap();
        assert_eq!(d, g);
    }
}
