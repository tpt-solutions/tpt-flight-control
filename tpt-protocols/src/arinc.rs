//! ARINC 429 / AFDX transport-category integration protocols (`spec.txt` §12,
//! Phase 5).
//!
//! These implement the wire formats used by transport-category aircraft
//! avionics:
//! - **ARINC 429** — the classic unidirectional 32-bit label总线. We support
//!   label (octal) encoding, SDI, 19-bit data field, SSM, and odd parity, with
//!   BNR (scaled binary) and BCD helpers.
//! - **AFDX** — Avionics Full-Duplex Switched Ethernet. We implement a
//!   simplified frame (virtual link id, sequence, payload, CRC-16) sufficient
//!   for end-system to end-system telemetry without a full Ethernet stack.
//!
//! The module is `#![no_std]` and allocation-free so it can be linked into the
//! `tpt-transport` profile on a certified target.

use crate::mavlink::crc::crc16_x25;

// ---------------------------------------------------------------------------
// ARINC 429
// ---------------------------------------------------------------------------

/// An ARINC 429 word: 32 bits, LSB-first bit numbering.
///
/// ```text
/// bit  1..8   label      (8 bits, transmitted LSB-first, octal display)
/// bit  9..10  SDI        (source/destination identifier, 2 bits)
/// bit 11..29  data       (19 bits)
/// bit 30..31  SSM        (sign/status matrix, 2 bits)
/// bit 32      parity     (odd parity over bits 1..31)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Arinc429Word {
    /// Label number (0..=255). Stored as the raw label value.
    pub label: u8,
    /// Source/Destination identifier (0..=3).
    pub sdi: u8,
    /// 19-bit data field.
    pub data: u32,
    /// Sign/Status Matrix (0..=3).
    pub ssm: u8,
}

impl Arinc429Word {
    /// Construct a word with odd parity computed. `data` is masked to 19 bits.
    pub fn new(label: u8, sdi: u8, data: u32, ssm: u8) -> Self {
        Self {
            label: label & 0xFF,
            sdi: sdi & 0x03,
            data: data & 0x7_FFFF,
            ssm: ssm & 0x03,
        }
    }

    /// Pack into the 32-bit on-wire representation (LSB-first bit order).
    pub fn pack(&self) -> u32 {
        let mut bits: u32 = 0;
        // Label occupies bits 1..8 (bit index 0..7).
        bits |= u32::from(self.label) & 0xFF;
        // SDI bits 9..10 (index 8..9).
        bits |= u32::from(self.sdi & 0x03) << 8;
        // Data bits 11..29 (index 10..28).
        bits |= (self.data & 0x7_FFFF) << 10;
        // SSM bits 30..31 (index 29..30).
        bits |= u32::from(self.ssm & 0x03) << 29;
        // Odd parity over bits 1..31.
        bits |= u32::from(odd_parity(bits & 0x7FFF_FFFF)) << 31;
        bits
    }

    /// Parse a 32-bit word, returning `None` if the parity check fails.
    pub fn parse(raw: u32) -> Option<Self> {
        let body = raw & 0x7FFF_FFFF;
        if odd_parity(body) != ((raw >> 31) & 0x01) as u8 {
            return None;
        }
        Some(Self {
            label: (body & 0xFF) as u8,
            sdi: ((body >> 8) & 0x03) as u8,
            data: (body >> 10) & 0x7_FFFF,
            ssm: ((body >> 29) & 0x03) as u8,
        })
    }

    /// Decode the data field as a signed BNR value scaled by `lsb` (per bit).
    ///
    /// The 19-bit field is interpreted as two's-complement: bit 10 is the sign
    /// and bits 11..28 the magnitude. Returns the physical value.
    pub fn decode_bnr(&self, lsb: f64) -> f64 {
        let raw = self.data & 0x7_FFFF;
        // Sign bit is the top bit of the 19-bit field.
        let sign = (raw >> 18) & 0x01;
        let mag = raw & 0x3_FFFF; // lower 18 bits magnitude
        let signed = if sign == 1 {
            -(mag as f64)
        } else {
            mag as f64
        };
        signed * lsb
    }

    /// Encode a signed BNR value with the given `lsb` resolution into the data
    /// field (clamped to the 18-bit magnitude range). Returns a new word.
    pub fn encode_bnr(&self, value: f64, lsb: f64) -> Self {
        // Round half away from zero using only core methods (no_std-safe).
        let r = value / lsb;
        let steps = (r + 0.5 * r.signum()) as i64;
        let mag = (steps.abs() as u32) & 0x3_FFFF;
        let sign = if steps < 0 { 0x01 } else { 0x00 };
        let data = (sign << 18) | mag; // 19-bit field
        Self::new(self.label, self.sdi, data, self.ssm)
    }

    /// Decode the data field as 5-group BCD (4 digits of BCD in the lower
    /// 16 bits, each nibble 0..=9). Returns the integer value.
    pub fn decode_bcd(&self) -> u32 {
        let mut value: u32 = 0;
        let mut mult: u32 = 1;
        for nibble in 0..4u32 {
            let d = (self.data >> (nibble * 4)) & 0x0F;
            value += d.min(9) * mult;
            mult *= 10;
        }
        value
    }

    /// ARINC 429 label rendered as the traditional 3-digit octal string
    /// (label bits are transmitted LSB-first, so the conventional octal is the
    /// bit-reversed value).
    pub fn label_octal(&self) -> u8 {
        reverse_bits_8(self.label)
    }
}

/// Compute odd parity (1 if number of set bits is even) of a 31-bit value.
fn odd_parity(v: u32) -> u8 {
    let mut x = v;
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    // odd parity: result bit = NOT (popcount mod 2)
    ((!x) & 0x01) as u8
}

/// Reverse the 8 bits of `b` (ARINC label is transmitted LSB-first).
fn reverse_bits_8(mut b: u8) -> u8 {
    let mut out = 0u8;
    for _ in 0..8 {
        out = (out << 1) | (b & 0x01);
        b >>= 1;
    }
    out
}

/// A single ARINC 429 receiver channel (last-value + small history).
///
/// ARINC 429 is unidirectional; this models one receive line holding the most
/// recent word and a bounded history for diagnostic/retry paths.
#[derive(Debug, Clone)]
pub struct Arinc429Channel {
    history: [u32; 4],
    count: usize,
}

impl Arinc429Channel {
    /// Create an empty channel.
    pub const fn new() -> Self {
        Self {
            history: [0u32; 4],
            count: 0,
        }
    }

    /// Receive the latest valid word (parity-checked), or `None` if empty.
    pub fn receive(&self) -> Option<Arinc429Word> {
        if self.count == 0 {
            return None;
        }
        Arinc429Word::parse(self.history[(self.count - 1) % 4])
    }

    /// Push a raw wire word; returns `None` if its parity is invalid.
    pub fn ingest(&mut self, raw: u32) -> Option<Arinc429Word> {
        let word = Arinc429Word::parse(raw)?;
        self.history[self.count % 4] = raw;
        self.count = self.count.wrapping_add(1);
        Some(word)
    }

    /// Number of words ingested since creation.
    pub const fn count(&self) -> usize {
        self.count
    }
}

impl Default for Arinc429Channel {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// AFDX
// ---------------------------------------------------------------------------

/// Maximum AFDX payload size (simplified; real AFDX frames are larger).
pub const AFDX_MAX_PAYLOAD: usize = 1471;
/// AFDX ethertype used by TPT telemetry virtual links.
pub const AFDX_ETHERTYPE: u16 = 0x8911;

/// An AFDX frame for a single virtual link (VL).
///
/// Layout (little-endian):
/// ```text
///  0..2  ethertype   0x8911
///  2..4  vl_id       virtual link identifier
///  4..6  seq         16-bit sequence number
///  6..8  length      16-bit payload length
///  8..   payload     `length` bytes
///  ..+2  crc16       CRC-16/X25 over the header+payload
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AfdxFrame {
    pub vl_id: u16,
    pub seq: u16,
    pub length: u16,
    pub payload: [u8; AFDX_MAX_PAYLOAD],
}

impl AfdxFrame {
    /// Build a frame for `vl_id`/`seq` with `payload` (clamped to max).
    pub fn new(vl_id: u16, seq: u16, payload: &[u8]) -> Self {
        let n = payload.len().min(AFDX_MAX_PAYLOAD);
        let mut buf = [0u8; AFDX_MAX_PAYLOAD];
        buf[..n].copy_from_slice(&payload[..n]);
        Self {
            vl_id,
            seq,
            length: n as u16,
            payload: buf,
        }
    }

    /// Serialize into `out`, returning the total length or `None` if too small.
    pub fn serialize(&self, out: &mut [u8]) -> Option<usize> {
        let total = 8 + self.length as usize + 2;
        if out.len() < total {
            return None;
        }
        out[0..2].copy_from_slice(&AFDX_ETHERTYPE.to_le_bytes());
        out[2..4].copy_from_slice(&self.vl_id.to_le_bytes());
        out[4..6].copy_from_slice(&self.seq.to_le_bytes());
        out[6..8].copy_from_slice(&self.length.to_le_bytes());
        out[8..8 + self.length as usize].copy_from_slice(&self.payload[..self.length as usize]);
        let crc = crc16_x25(&out[..8 + self.length as usize], 0xFFFF);
        out[8 + self.length as usize..total].copy_from_slice(&crc.to_le_bytes());
        Some(total)
    }

    /// Parse and authenticate a frame from the front of `buf`.
    pub fn parse(buf: &[u8]) -> Option<(Self, usize)> {
        let total = 8 + 2;
        if buf.len() < total {
            return None;
        }
        if u16::from_le_bytes([buf[0], buf[1]]) != AFDX_ETHERTYPE {
            return None;
        }
        let vl_id = u16::from_le_bytes([buf[2], buf[3]]);
        let seq = u16::from_le_bytes([buf[4], buf[5]]);
        let length = u16::from_le_bytes([buf[6], buf[7]]) as usize;
        if buf.len() < 8 + length + 2 {
            return None;
        }
        let crc = crc16_x25(&buf[..8 + length], 0xFFFF);
        let got = u16::from_le_bytes([buf[8 + length], buf[8 + length + 1]]);
        if got != crc {
            return None;
        }
        let mut payload = [0u8; AFDX_MAX_PAYLOAD];
        payload[..length].copy_from_slice(&buf[8..8 + length]);
        Some((
            Self {
                vl_id,
                seq,
                length: length as u16,
                payload,
            },
            total + length,
        ))
    }
}

/// An AFDX end system: a transmit/receive buffer for one virtual link.
#[derive(Debug, Clone)]
pub struct AfdxEndSystem {
    vl_id: u16,
    seq: u16,
    tx: [u8; 8 + AFDX_MAX_PAYLOAD + 2],
    tx_len: usize,
}

impl AfdxEndSystem {
    /// Create an end system bound to `vl_id`.
    pub const fn new(vl_id: u16) -> Self {
        Self {
            vl_id,
            seq: 0,
            tx: [0u8; 8 + AFDX_MAX_PAYLOAD + 2],
            tx_len: 0,
        }
    }

    /// Build and stage a frame for `payload`, returning the on-wire bytes.
    pub fn send(&mut self, payload: &[u8]) -> &[u8] {
        let frame = AfdxFrame::new(self.vl_id, self.seq, payload);
        self.seq = self.seq.wrapping_add(1);
        self.tx_len = frame.serialize(&mut self.tx).unwrap_or(0);
        &self.tx[..self.tx_len]
    }

    /// Sequence number of the next frame to be sent.
    pub const fn next_seq(&self) -> u16 {
        self.seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arinc_parity_round_trip() {
        let w = Arinc429Word::new(0o203, 0, 12345, 0b11);
        let raw = w.pack();
        let parsed = Arinc429Word::parse(raw).expect("parity valid");
        assert_eq!(parsed.label, 0o203);
        assert_eq!(parsed.data, 12345);
        assert_eq!(parsed.ssm, 0b11);
    }

    #[test]
    fn arinc_parity_rejects_corruption() {
        let w = Arinc429Word::new(0o203, 1, 999, 2);
        let mut raw = w.pack();
        raw ^= 1 << 5; // flip a data bit -> parity mismatch
        assert!(Arinc429Word::parse(raw).is_none());
    }

    #[test]
    fn arinc_bnr_scales() {
        let w = Arinc429Word::new(0o044, 0, 0, 0);
        // value 12.5 with LSB 0.25 -> 50 steps
        let enc = w.encode_bnr(12.5, 0.25);
        assert!((enc.decode_bnr(0.25) - 12.5).abs() < 1e-9);
        let neg = w.encode_bnr(-3.0, 0.25);
        assert!((neg.decode_bnr(0.25) + 3.0).abs() < 1e-9);
    }

    #[test]
    fn arinc_label_octal() {
        // label 0o001 -> reversed bits = 0b1000_0000 = 0o200
        let w = Arinc429Word::new(0o001, 0, 0, 0);
        assert_eq!(w.label_octal(), 0o200);
    }

    #[test]
    fn arinc_channel_ingest() {
        let mut ch = Arinc429Channel::new();
        assert!(ch.receive().is_none());
        let w = Arinc429Word::new(0o100, 0, 7, 1);
        let got = ch.ingest(w.pack()).unwrap();
        assert_eq!(got.data, 7);
        assert_eq!(ch.count(), 1);
        // Bad parity rejected.
        assert!(ch.ingest(w.pack() ^ (1 << 15)).is_none());
    }

    #[test]
    fn afdx_round_trip() {
        let mut es = AfdxEndSystem::new(42);
        let wire = es.send(b"afdx-payload");
        let (f, used) = AfdxFrame::parse(wire).unwrap();
        assert_eq!(used, wire.len());
        assert_eq!(f.vl_id, 42);
        assert_eq!(&f.payload[..f.length as usize], b"afdx-payload");
    }

    #[test]
    fn afdx_rejects_bad_crc() {
        let mut es = AfdxEndSystem::new(1);
        let wire = es.send(b"hello world");
        let mut bad = [0u8; 8 + AFDX_MAX_PAYLOAD + 2];
        bad[..wire.len()].copy_from_slice(wire);
        bad[10] ^= 0xFF;
        assert!(AfdxFrame::parse(&bad[..wire.len()]).is_none());
    }
}
