//! Swarm coordination foundation (`spec.txt` §15, resilience roadmap).
//!
//! Feature-gated behind `swarm` on `tpt-core` (which also enables the
//! `tpt-protocols` dependency so peer telemetry can ride the existing TPT-Link
//! [`Channel::Telemetry`](tpt_protocols::tptlink::Channel) framing).
//!
//! Provides:
//! - [`PeerTelemetry`] — the per-peer state shared over the link, with
//!   [`serialize_peer`]/[`parse_peer`] that wrap the payload in a TPT-Link
//!   frame exactly like the rest of the telemetry path.
//! - [`SwarmNetwork`] — a fixed-capacity store of recently-received peer
//!   telemetry (`no_std`, allocation-free).
//! - [`RelativePositionController`] — a proportional controller that holds a
//!   desired relative offset to a peer (the basis for formation flight).

use tpt_math::Vector3;
use tpt_protocols::tptlink::{self, Channel, FrameHeader};

/// TPT-Link Telemetry-channel message id for a [`PeerTelemetry`] frame.
pub const MSG_PEER_TELEMETRY: u8 = 1;

/// Wire size (bytes) of a [`PeerTelemetry`] payload.
pub const PEER_TELEMETRY_LEN: usize = 1 + 7 * 4; // id + 7 f32

/// Shared state of one swarm peer, broadcast over the link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PeerTelemetry {
    /// Peer identifier (0..=255).
    pub id: u8,
    /// Local-frame position (m).
    pub position: Vector3<f64>,
    /// Local-frame velocity (m/s).
    pub velocity: Vector3<f64>,
    /// Heading / yaw (rad).
    pub heading: f64,
}

impl PeerTelemetry {
    /// Encode into `out` (needs at least [`PEER_TELEMETRY_LEN`] bytes). Returns
    /// bytes written, or `None` if `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < PEER_TELEMETRY_LEN {
            return None;
        }
        out[0] = self.id;
        let mut o = 1usize;
        let mut put = |o: &mut usize, v: f64| {
            out[*o..*o + 4].copy_from_slice(&(v as f32).to_le_bytes());
            *o += 4;
        };
        put(&mut o, self.position.x);
        put(&mut o, self.position.y);
        put(&mut o, self.position.z);
        put(&mut o, self.velocity.x);
        put(&mut o, self.velocity.y);
        put(&mut o, self.velocity.z);
        put(&mut o, self.heading);
        Some(o)
    }

    /// Decode a [`PeerTelemetry`] from a payload slice.
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() < PEER_TELEMETRY_LEN {
            return None;
        }
        let id = p[0];
        let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]) as f64;
        Some(Self {
            id,
            position: Vector3::new(f(1), f(5), f(9)),
            velocity: Vector3::new(f(13), f(17), f(21)),
            heading: f(25),
        })
    }
}

/// Serialize a [`PeerTelemetry`] as a plaintext TPT-Link frame on
/// [`Channel::Telemetry`]. Returns total bytes written, or `None`.
pub fn serialize_peer(out: &mut [u8], seq: u16, peer: &PeerTelemetry) -> Option<usize> {
    let mut payload = [0u8; PEER_TELEMETRY_LEN];
    let n = peer.encode(&mut payload)?;
    tptlink::serialize_plain(
        out,
        Channel::Telemetry,
        MSG_PEER_TELEMETRY,
        seq,
        &payload[..n],
    )
}

/// Parse a TPT-Link frame on [`Channel::Telemetry`] into a [`PeerTelemetry`].
pub fn parse_peer(buf: &[u8]) -> Option<(FrameHeader, PeerTelemetry)> {
    let (header, payload) = tptlink::parse_plain(buf)?;
    if header.channel != Channel::Telemetry || header.msgid != MSG_PEER_TELEMETRY {
        return None;
    }
    let peer = PeerTelemetry::decode(payload)?;
    Some((header, peer))
}

/// Fixed-capacity store of recently-received peer telemetry (`N` peers).
#[derive(Debug, Clone, Copy)]
pub struct SwarmNetwork<const N: usize> {
    peers: [Option<PeerTelemetry>; N],
}

impl<const N: usize> SwarmNetwork<N> {
    /// Create an empty network.
    pub const fn new() -> Self {
        Self { peers: [None; N] }
    }

    /// Ingest a peer's telemetry, keyed by `id` modulo `N`. Returns the slot
    /// index used.
    pub fn ingest(&mut self, peer: PeerTelemetry) -> usize {
        let slot = (peer.id as usize) % N;
        self.peers[slot] = Some(peer);
        slot
    }

    /// Number of peers currently tracked.
    pub fn peer_count(&self) -> usize {
        self.peers.iter().filter(|p| p.is_some()).count()
    }

    /// Fetch the latest telemetry for peer `id`, if known.
    pub fn peer(&self, id: u8) -> Option<PeerTelemetry> {
        self.peers[(id as usize) % N]
    }

    /// Iterate over known peers (yields `None` slots are skipped).
    pub fn iter(&self) -> impl Iterator<Item = &PeerTelemetry> {
        self.peers.iter().filter_map(|p| p.as_ref())
    }
}

impl<const N: usize> Default for SwarmNetwork<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Proportional relative-position-keeping controller.
///
/// Produces a velocity setpoint (m/s) that drives `own_pos` toward the desired
/// slot `peer_pos + offset`. This is the shared primitive for both station-
/// keeping and formation flight (§15).
#[derive(Debug, Clone, Copy)]
pub struct RelativePositionController {
    /// Proportional gain (1/s).
    kp: f64,
}

impl RelativePositionController {
    /// Create with the given proportional gain.
    pub const fn new(kp: f64) -> Self {
        Self { kp }
    }

    /// Velocity setpoint to hold `offset` relative to `peer_pos`, given the
    /// vehicle's own position `own_pos`. A pure proportional law (no
    /// derivative) keeps the allocation-free, deterministic path; the outer
    /// guidance loop clamps the resulting speed.
    pub fn update(
        &self,
        own_pos: Vector3<f64>,
        peer_pos: Vector3<f64>,
        offset: Vector3<f64>,
    ) -> Vector3<f64> {
        (peer_pos + offset - own_pos) * self.kp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn peer_telemetry_round_trip_over_tptlink() {
        let peer = PeerTelemetry {
            id: 7,
            position: Vector3::new(1.0, 2.0, -3.0),
            velocity: Vector3::new(0.5, -0.5, 0.1),
            heading: 1.57,
        };
        let mut out = [0u8; 128];
        let n = serialize_peer(&mut out, 3, &peer).unwrap();
        let (h, got) = parse_peer(&out[..n]).unwrap();
        assert_eq!(h.channel, Channel::Telemetry);
        assert_eq!(h.msgid, MSG_PEER_TELEMETRY);
        assert_eq!(h.seq, 3);
        // Wire format is f32 (lossy); compare within tolerance.
        assert_eq!(got.id, peer.id);
        assert!((got.position.x - peer.position.x).abs() < 1e-3);
        assert!((got.position.y - peer.position.y).abs() < 1e-3);
        assert!((got.position.z - peer.position.z).abs() < 1e-3);
        assert!((got.heading - peer.heading).abs() < 1e-3);
    }

    #[test]
    fn swarm_network_tracks_peers() {
        let mut net: SwarmNetwork<8> = SwarmNetwork::new();
        assert_eq!(net.peer_count(), 0);
        net.ingest(PeerTelemetry {
            id: 1,
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            heading: 0.0,
        });
        net.ingest(PeerTelemetry {
            id: 2,
            position: Vector3::new(5.0, 0.0, 0.0),
            velocity: Vector3::zeros(),
            heading: 0.0,
        });
        assert_eq!(net.peer_count(), 2);
        assert!(net.peer(1).is_some());
        assert!(net.peer(2).is_some());
        assert_eq!(net.peer(1).unwrap().position, Vector3::zeros());
    }

    #[test]
    fn relative_controller_closes_offset() {
        let ctrl = RelativePositionController::new(0.5);
        // Want to be 2 m behind (=-x) the peer. Own at 0, peer at 0, offset -x.
        let v = ctrl.update(
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::new(-2.0, 0.0, 0.0),
        );
        assert!((v.x + 1.0).abs() < 1e-9, "v.x {}", v.x);
        // Once at the slot, command goes to zero.
        let v2 = ctrl.update(
            Vector3::new(-2.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::new(-2.0, 0.0, 0.0),
        );
        assert!(v2.norm() < 1e-9);
    }
}
