//! TPT-Link zero-copy binary telemetry protocol (`spec.txt` §12, Phase 3).
//!
//! A compact, allocation-free binary framing for high-rate onboard/companion
//! telemetry. Frames carry a fixed header, a variable payload, and either a
//! CRC-16 (plain) or a 16-byte ChaCha20-Poly1305 tag (encrypted). The wire
//! format is designed to be parsed in place (no copy) on the receiving side.
//!
//! Wire layout (little-endian fields):
//! ```text
//!  0..2  magic     0x5450 ("TP")
//!  2     flags     bit0 = encrypted
//!  3     channel   0=telemetry 1=command 2=map 3=health
//!  4     msgid     application message id
//!  5..7  seq       16-bit sequence number
//!  7..9  length    16-bit payload length
//!  9..   payload   `length` bytes
//!  ..+16 tag/CRC    16-byte AEAD tag (encrypted) or 2-byte CRC-16 (plain)
//! ```

use crate::chacha::{aead_decrypt, aead_encrypt};
use crate::mavlink::crc::crc16_x25;

/// TPT-Link magic bytes.
pub const MAGIC: [u8; 2] = [0x54, 0x50];
/// Size of the fixed header (before payload).
pub const HEADER_LEN: usize = 9;
/// Size of the authentication/signature trailer.
pub const TAG_LEN: usize = 16;

/// Frame channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Telemetry = 0,
    Command = 1,
    Map = 2,
    Health = 3,
}

impl Channel {
    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Channel::Telemetry),
            1 => Some(Channel::Command),
            2 => Some(Channel::Map),
            3 => Some(Channel::Health),
            _ => None,
        }
    }
}

/// Parsed TPT-Link frame header (payload is addressed by offset into the buffer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameHeader {
    pub encrypted: bool,
    pub channel: Channel,
    pub msgid: u8,
    pub seq: u16,
    pub length: u16,
}

/// Serialize a plaintext frame (CRC-16 protected) into `out`. Returns the total
/// length written, or `None` if `out` is too small.
pub fn serialize_plain(
    out: &mut [u8],
    channel: Channel,
    msgid: u8,
    seq: u16,
    payload: &[u8],
) -> Option<usize> {
    let total = HEADER_LEN + payload.len() + 2;
    if out.len() < total {
        return None;
    }
    out[0..2].copy_from_slice(&MAGIC);
    out[2] = 0x00; // plaintext
    out[3] = channel as u8;
    out[4] = msgid;
    out[5..7].copy_from_slice(&seq.to_le_bytes());
    out[7..9].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    out[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    let crc = crc16_x25(&out[..HEADER_LEN + payload.len()], 0xFFFF);
    out[HEADER_LEN + payload.len()..total].copy_from_slice(&crc.to_le_bytes());
    Some(total)
}

/// Parse a plaintext frame (validating CRC-16). On success returns the header
/// and a slice of the payload within `buf`.
pub fn parse_plain(buf: &[u8]) -> Option<(FrameHeader, &[u8])> {
    if buf.len() < HEADER_LEN + 2 {
        return None;
    }
    if buf[0..2] != MAGIC {
        return None;
    }
    let encrypted = buf[2] & 0x01 != 0;
    if encrypted {
        return None; // use parse_encrypted for those
    }
    let channel = Channel::from_u8(buf[3])?;
    let msgid = buf[4];
    let seq = u16::from_le_bytes([buf[5], buf[6]]);
    let length = u16::from_le_bytes([buf[7], buf[8]]) as usize;
    if buf.len() < HEADER_LEN + length + 2 {
        return None;
    }
    let body = &buf[..HEADER_LEN + length];
    let crc = crc16_x25(body, 0xFFFF);
    let got = u16::from_le_bytes([buf[HEADER_LEN + length], buf[HEADER_LEN + length + 1]]);
    if got != crc {
        return None;
    }
    let header = FrameHeader {
        encrypted: false,
        channel,
        msgid,
        seq,
        length: length as u16,
    };
    Some((header, &buf[HEADER_LEN..HEADER_LEN + length]))
}

/// Serialize an encrypted frame (ChaCha20-Poly1305). `payload` is encrypted in
/// place semantics: the function encrypts a copy into `out`. Returns the total
/// length written, or `None` if `out` is too small.
pub fn serialize_encrypted(
    out: &mut [u8],
    key: &[u8; 32],
    nonce: &[u8; 12],
    channel: Channel,
    msgid: u8,
    seq: u16,
    payload: &[u8],
) -> Option<usize> {
    let total = HEADER_LEN + payload.len() + TAG_LEN;
    if out.len() < total {
        return None;
    }
    out[0..2].copy_from_slice(&MAGIC);
    out[2] = 0x01; // encrypted
    out[3] = channel as u8;
    out[4] = msgid;
    out[5..7].copy_from_slice(&seq.to_le_bytes());
    out[7..9].copy_from_slice(&(payload.len() as u16).to_le_bytes());
    // Encrypt payload in place into the output buffer. The AAD covers the
    // fixed header fields (channel/msgid/seq/length); copy them out so the
    // mutable payload borrow does not alias the AAD borrow.
    out[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    let mut aad = [0u8; 6];
    aad.copy_from_slice(&out[3..HEADER_LEN]);
    let tag = aead_encrypt(
        key,
        nonce,
        &aad,
        &mut out[HEADER_LEN..HEADER_LEN + payload.len()],
    );
    out[HEADER_LEN + payload.len()..total].copy_from_slice(&tag);
    Some(total)
}

/// Parse and authenticate an encrypted frame. On success returns the header and
/// the decrypted payload (written into `buf` in place) and the length.
pub fn parse_encrypted(
    buf: &mut [u8],
    key: &[u8; 32],
    nonce: &[u8; 12],
) -> Option<(FrameHeader, usize)> {
    if buf.len() < HEADER_LEN + TAG_LEN {
        return None;
    }
    if buf[0..2] != MAGIC {
        return None;
    }
    if buf[2] & 0x01 == 0 {
        return None;
    }
    let channel = Channel::from_u8(buf[3])?;
    let msgid = buf[4];
    let seq = u16::from_le_bytes([buf[5], buf[6]]);
    let length = u16::from_le_bytes([buf[7], buf[8]]) as usize;
    if buf.len() < HEADER_LEN + length + TAG_LEN {
        return None;
    }
    let mut tag = [0u8; TAG_LEN];
    tag.copy_from_slice(&buf[HEADER_LEN + length..HEADER_LEN + length + TAG_LEN]);
    // AAD covers the header flags/channel/msgid/seq/length.
    let (head, tail) = buf.split_at_mut(HEADER_LEN);
    let ok = aead_decrypt(key, nonce, &head[3..HEADER_LEN], &mut tail[..length], &tag);
    if !ok {
        return None;
    }
    let header = FrameHeader {
        encrypted: true,
        channel,
        msgid,
        seq,
        length: length as u16,
    };
    Some((header, length))
}

/// TPT-Link Health-channel message id for a [`HealthReport`].
pub const MSG_HEALTH_REPORT: u8 = 1;

/// Maximum number of motor samples carried in a single [`HealthReport`].
pub const MAX_HEALTH_MOTORS: usize = 8;

/// Wire size (bytes) of a [`HealthReport`] payload.
pub const HEALTH_REPORT_LEN: usize =
    4 + 4 + 4 + 4 + 1 + MAX_HEALTH_MOTORS * (4 + 4 + 1); // 4 floats + count + motors

/// Per-motor health sample carried in a [`HealthReport`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorHealthSample {
    /// Winding / electronics temperature (°C).
    pub temperature_c: f32,
    /// Normalized mechanical load in `[0, 1]`.
    pub load: f32,
    /// Health flag: `1` = healthy, `0` = unhealthy / failed.
    pub healthy: u8,
}

impl MotorHealthSample {
    /// A zeroed (unknown) sample.
    pub const fn zero() -> Self {
        Self {
            temperature_c: 0.0,
            load: 0.0,
            healthy: 0,
        }
    }
}

/// Compact vehicle health / prognostics report (`spec.txt` §16.3, resilience
/// roadmap) carried on [`Channel::Health`].
///
/// Encodes the battery state-of-charge / cell-voltage / temperature / RUL
/// estimate alongside up to [`MAX_HEALTH_MOTORS`] per-motor samples, so a GCS
/// or companion computer can trend predictive-health telemetry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HealthReport {
    /// Battery state of charge in `[0, 1]`.
    pub battery_soc: f32,
    /// Minimum cell voltage under load (V).
    pub battery_cell_v_min: f32,
    /// Pack temperature (°C).
    pub battery_temp_c: f32,
    /// Battery remaining-useful-life estimate in `[0, 1]` (1 = new).
    pub battery_rul: f32,
    /// Number of valid motor samples in [`Self::motors`].
    pub motor_count: u8,
    /// Per-motor samples (only the first `motor_count` are meaningful).
    pub motors: [MotorHealthSample; MAX_HEALTH_MOTORS],
}

impl HealthReport {
    /// An empty report (all-zero, nominal battery).
    pub const fn empty() -> Self {
        Self {
            battery_soc: 1.0,
            battery_cell_v_min: 0.0,
            battery_temp_c: 0.0,
            battery_rul: 1.0,
            motor_count: 0,
            motors: [MotorHealthSample::zero(); MAX_HEALTH_MOTORS],
        }
    }

    /// Encode into `out` (needs at least [`HEALTH_REPORT_LEN`] bytes). Returns
    /// the bytes written, or `None` if `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < HEALTH_REPORT_LEN {
            return None;
        }
        let mut o = 0usize;
        let put_f32 = |out: &mut [u8], o: &mut usize, v: f32| {
            out[*o..*o + 4].copy_from_slice(&v.to_le_bytes());
            *o += 4;
        };
        put_f32(out, &mut o, self.battery_soc);
        put_f32(out, &mut o, self.battery_cell_v_min);
        put_f32(out, &mut o, self.battery_temp_c);
        put_f32(out, &mut o, self.battery_rul);
        let count = self.motor_count.min(MAX_HEALTH_MOTORS as u8);
        out[o] = count;
        o += 1;
        for i in 0..count as usize {
            let m = self.motors[i];
            put_f32(out, &mut o, m.temperature_c);
            put_f32(out, &mut o, m.load);
            out[o] = m.healthy;
            o += 1;
        }
        // Pad any unused motor slots so the wire size is constant.
        for _ in count as usize..MAX_HEALTH_MOTORS {
            put_f32(out, &mut o, 0.0);
            put_f32(out, &mut o, 0.0);
            out[o] = 0;
            o += 1;
        }
        Some(o)
    }

    /// Decode a [`HealthReport`] from a payload slice.
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() < HEALTH_REPORT_LEN {
            return None;
        }
        let mut o = 0usize;
        let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]);
        let battery_soc = f(o);
        o += 4;
        let battery_cell_v_min = f(o);
        o += 4;
        let battery_temp_c = f(o);
        o += 4;
        let battery_rul = f(o);
        o += 4;
        let motor_count = (p[o].min(MAX_HEALTH_MOTORS as u8)) as usize;
        o += 1;
        let mut motors = [MotorHealthSample::zero(); MAX_HEALTH_MOTORS];
        let mut i = 0usize;
        while i < motor_count {
            let temperature_c = f(o);
            o += 4;
            let load = f(o);
            o += 4;
            let healthy = p[o];
            o += 1;
            motors[i] = MotorHealthSample {
                temperature_c,
                load,
                healthy,
            };
            i += 1;
        }
        // The unused (padded) motor slots are simply not read; `decode` returns
        // the decoded struct and does not need the final cursor.
        Some(Self {
            battery_soc,
            battery_cell_v_min,
            battery_temp_c,
            battery_rul,
            motor_count: motor_count as u8,
            motors,
        })
    }
}

/// Serialize a [`HealthReport`] as a plaintext (CRC-16) TPT-Link frame on
/// [`Channel::Health`]. Returns the total bytes written into `out`, or `None`
/// if `out` is too small.
pub fn serialize_health(out: &mut [u8], seq: u16, report: &HealthReport) -> Option<usize> {
    let mut payload = [0u8; HEALTH_REPORT_LEN];
    let n = report.encode(&mut payload)?;
    serialize_plain(out, Channel::Health, MSG_HEALTH_REPORT, seq, &payload[..n])
}

/// Parse a plaintext TPT-Link frame on [`Channel::Health`] into a
/// [`HealthReport`]. Returns the frame header and the decoded report, or `None`
/// if the frame is malformed / not a health report.
pub fn parse_health(buf: &[u8]) -> Option<(FrameHeader, HealthReport)> {
    let (header, payload) = parse_plain(buf)?;
    if header.channel != Channel::Health || header.msgid != MSG_HEALTH_REPORT {
        return None;
    }
    let report = HealthReport::decode(payload)?;
    Some((header, report))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_round_trip() {
        let mut out = [0u8; 64];
        let payload = b"attitude 1.57";
        let n = serialize_plain(&mut out, Channel::Telemetry, 7, 42, payload).unwrap();
        let (h, p) = parse_plain(&out[..n]).unwrap();
        assert_eq!(h.channel, Channel::Telemetry);
        assert_eq!(h.msgid, 7);
        assert_eq!(h.seq, 42);
        assert_eq!(p, payload);
    }

    #[test]
    fn plain_crc_rejects_corruption() {
        let mut out = [0u8; 64];
        let payload = b"hello";
        let n = serialize_plain(&mut out, Channel::Command, 1, 0, payload).unwrap();
        out[10] ^= 0xFF;
        assert!(parse_plain(&out[..n]).is_none());
    }

    #[test]
    fn encrypted_round_trip() {
        let key = [3u8; 32];
        let nonce = [9u8; 12];
        let mut out = [0u8; 128];
        let payload = b"steer 12.5, climb 3.0";
        let _n =
            serialize_encrypted(&mut out, &key, &nonce, Channel::Command, 2, 7, payload).unwrap();
        let mut buf = out;
        let (h, len) = parse_encrypted(&mut buf, &key, &nonce).unwrap();
        assert_eq!(h.channel, Channel::Command);
        assert_eq!(&buf[HEADER_LEN..HEADER_LEN + len], payload);
    }

    #[test]
    fn encrypted_rejects_wrong_key() {
        let key = [3u8; 32];
        let nonce = [9u8; 12];
        let mut out = [0u8; 128];
        let payload = b"secret move";
        let _n =
            serialize_encrypted(&mut out, &key, &nonce, Channel::Command, 2, 7, payload).unwrap();
        let mut buf = out;
        let bad = [0u8; 32];
        assert!(parse_encrypted(&mut buf, &bad, &nonce).is_none());
    }

    #[test]
    fn health_report_round_trip() {
        let mut report = HealthReport::empty();
        report.battery_soc = 0.82;
        report.battery_cell_v_min = 3.65;
        report.battery_temp_c = 31.0;
        report.battery_rul = 0.74;
        report.motor_count = 4;
        for i in 0..4 {
            report.motors[i] = MotorHealthSample {
                temperature_c: 40.0 + i as f32,
                load: 0.5,
                healthy: 1,
            };
        }
        let mut out = [0u8; 256];
        let n = serialize_health(&mut out, 11, &report).unwrap();
        let (h, got) = parse_health(&out[..n]).unwrap();
        assert_eq!(h.channel, Channel::Health);
        assert_eq!(h.msgid, MSG_HEALTH_REPORT);
        assert_eq!(h.seq, 11);
        assert_eq!(got, report);
    }

    #[test]
    fn health_report_rejects_bad_frame() {
        let mut out = [0u8; 256];
        let n = serialize_health(&mut out, 1, &HealthReport::empty()).unwrap();
        out[3] = Channel::Telemetry as u8; // wrong channel
        assert!(parse_health(&out[..n]).is_none());
    }

    #[test]
    fn health_encode_decode_constant_size() {
        let report = HealthReport::empty();
        let mut buf = [0u8; HEALTH_REPORT_LEN];
        let n = report.encode(&mut buf).unwrap();
        assert_eq!(n, HEALTH_REPORT_LEN);
        assert_eq!(HealthReport::decode(&buf).unwrap(), report);
    }
}
