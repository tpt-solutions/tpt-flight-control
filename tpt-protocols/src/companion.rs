//! Companion-compute offload path (`spec.txt` §8.1, §8.3, Phase 3).
//!
//! For heavy 3D mapping the `tpt-core` flight controller stays lightweight: a
//! companion computer (e.g. NVIDIA Jetson / Orin) runs the dense VIO / SLAM
//! pipeline and publishes a compact **"Local Pose + Obstacle Cloud"** back to
//! the flight controller over a high-speed internal bus (Ethernet/UDP or
//! PCIe). This module defines that wire contract:
//!
//! * [`LocalPose`] — the 6-DOF pose estimate (position + orientation) plus a
//!   scalar position uncertainty and a capture timestamp.
//! * [`ObstacleCloud`] — a bounded set of obstacle points in the local frame.
//!
//! Both are framed on the TPT-Link [`Channel::Map`](crate::tptlink::Channel)
//! (plaintext CRC or ChaCha20-Poly1305 authenticated), so the same transport
//! and integrity guarantees used for telemetry apply to the offload link. A
//! received cloud can be ingested straight into any
//! [`SpatialMap`](tpt_abstractions::spatial::SpatialMap) implementer via
//! [`ObstacleCloud::ingest_into`], matching the spec's "passing only the pose
//! estimate back to the core via the `SpatialMap` trait".

use crate::tptlink::{self, Channel, FrameHeader};
use tpt_abstractions::spatial::SpatialMap;
use tpt_abstractions::types::{Landmark, Point3D, Pose6DOF};
use tpt_math::{UnitQuaternion, Vector3};

/// TPT-Link Map-channel message id for a [`LocalPose`] update.
pub const MSG_LOCAL_POSE: u8 = 1;
/// TPT-Link Map-channel message id for an [`ObstacleCloud`] update.
pub const MSG_OBSTACLE_CLOUD: u8 = 2;

/// Serialized size (bytes) of a [`LocalPose`] payload.
pub const LOCAL_POSE_LEN: usize = 3 * 4 + 4 * 4 + 4 + 8; // 40

/// Bytes per obstacle point on the wire (x, y, z as `f32`).
pub const POINT_LEN: usize = 12;

/// Maximum obstacle points carried in a single wire frame. Companions
/// voxel-downsample before publishing, so one frame covers the local avoidance
/// horizon; larger clouds are split across sequential frames.
pub const MAX_FRAME_POINTS: usize = 256;

/// A lightweight 6-DOF pose estimate published by the companion computer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPose {
    /// Position in the local navigation frame (m).
    pub position: Vector3<f64>,
    /// Orientation as a unit quaternion (w, x, y, z).
    pub orientation: UnitQuaternion<f64>,
    /// Scalar 1-sigma position uncertainty (m), used by the fusion gate.
    pub pos_stddev_m: f64,
    /// Capture timestamp (monotonic microseconds on the companion clock).
    pub timestamp_us: u64,
}

impl LocalPose {
    /// Encode into `out` (needs at least [`LOCAL_POSE_LEN`] bytes). Returns the
    /// number of bytes written, or `None` if `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < LOCAL_POSE_LEN {
            return None;
        }
        let mut o = 0usize;
        let mut put = |o: &mut usize, v: f32| {
            out[*o..*o + 4].copy_from_slice(&v.to_le_bytes());
            *o += 4;
        };
        put(&mut o, self.position.x as f32);
        put(&mut o, self.position.y as f32);
        put(&mut o, self.position.z as f32);
        let q = self.orientation.quaternion();
        put(&mut o, q.w as f32);
        put(&mut o, q.i as f32);
        put(&mut o, q.j as f32);
        put(&mut o, q.k as f32);
        put(&mut o, self.pos_stddev_m as f32);
        out[o..o + 8].copy_from_slice(&self.timestamp_us.to_le_bytes());
        o += 8;
        Some(o)
    }

    /// Decode from a payload slice (at least [`LOCAL_POSE_LEN`] bytes).
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() < LOCAL_POSE_LEN {
            return None;
        }
        let f = |i: usize| f32::from_le_bytes([p[i], p[i + 1], p[i + 2], p[i + 3]]) as f64;
        let position = Vector3::new(f(0), f(4), f(8));
        let orientation =
            UnitQuaternion::from_quaternion(tpt_math::Quaternion::new(f(12), f(16), f(20), f(24)));
        let pos_stddev_m = f(28);
        let timestamp_us =
            u64::from_le_bytes([p[32], p[33], p[34], p[35], p[36], p[37], p[38], p[39]]);
        Some(Self {
            position,
            orientation,
            pos_stddev_m,
            timestamp_us,
        })
    }

    /// Convert to the abstraction-layer [`Pose6DOF`] consumed by `SpatialMap`.
    pub fn to_pose6dof(&self) -> Pose6DOF {
        Pose6DOF {
            position: self.position,
            orientation: self.orientation,
        }
    }
}

/// A bounded set of obstacle points published by the companion computer.
///
/// `N` bounds the on-stack capacity (and therefore the maximum points per
/// frame); the companion is expected to voxel-downsample before publishing so a
/// modest `N` covers the local avoidance horizon.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObstacleCloud<const N: usize> {
    points: [Point3D; N],
    len: usize,
}

impl<const N: usize> Default for ObstacleCloud<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ObstacleCloud<N> {
    /// An empty cloud.
    pub const fn new() -> Self {
        Self {
            points: [Vector3::new(0.0, 0.0, 0.0); N],
            len: 0,
        }
    }

    /// Build a cloud from a slice of points (truncated to `N`).
    pub fn from_points(pts: &[Point3D]) -> Self {
        let mut c = Self::new();
        for &p in pts {
            if !c.push(p) {
                break;
            }
        }
        c
    }

    /// Append a point; returns `false` when full.
    pub fn push(&mut self, p: Point3D) -> bool {
        if self.len >= N {
            return false;
        }
        self.points[self.len] = p;
        self.len += 1;
        true
    }

    /// Number of points held.
    pub const fn len(&self) -> usize {
        self.len
    }
    /// Whether the cloud is empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
    /// The valid points.
    pub fn points(&self) -> &[Point3D] {
        &self.points[..self.len]
    }

    /// Encode into `out`: `[u16 count][count * (x,y,z as f32)]`. Returns the
    /// number of bytes written, or `None` if `out` is too small.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let total = 2 + self.len * POINT_LEN;
        if out.len() < total {
            return None;
        }
        out[0..2].copy_from_slice(&(self.len as u16).to_le_bytes());
        let mut o = 2usize;
        for p in self.points() {
            out[o..o + 4].copy_from_slice(&(p.x as f32).to_le_bytes());
            out[o + 4..o + 8].copy_from_slice(&(p.y as f32).to_le_bytes());
            out[o + 8..o + 12].copy_from_slice(&(p.z as f32).to_le_bytes());
            o += POINT_LEN;
        }
        Some(total)
    }

    /// Decode from a payload slice. Points beyond `N` are dropped.
    pub fn decode(p: &[u8]) -> Option<Self> {
        if p.len() < 2 {
            return None;
        }
        let count = u16::from_le_bytes([p[0], p[1]]) as usize;
        if p.len() < 2 + count * POINT_LEN {
            return None;
        }
        let mut c = Self::new();
        for i in 0..count {
            let b = 2 + i * POINT_LEN;
            let f = |k: usize| f32::from_le_bytes([p[k], p[k + 1], p[k + 2], p[k + 3]]) as f64;
            if !c.push(Vector3::new(f(b), f(b + 4), f(b + 8))) {
                break;
            }
        }
        Some(c)
    }

    /// Ingest this cloud into a [`SpatialMap`] as a keyframe at `pose`, treating
    /// each point as a landmark observation. This is the flight-controller-side
    /// bridge that turns the companion offload back into the onboard obstacle
    /// map the avoidance layer queries.
    pub fn ingest_into<M, const L: usize>(
        &self,
        map: &mut M,
        pose: Pose6DOF,
    ) -> Result<(), M::Error>
    where
        M: SpatialMap,
    {
        let mut lms = [Landmark {
            position: Vector3::new(0.0, 0.0, 0.0),
            descriptor: 0,
        }; L];
        let n = self.len.min(L);
        for (dst, src) in lms.iter_mut().zip(self.points.iter()).take(n) {
            *dst = Landmark {
                position: *src,
                descriptor: 0,
            };
        }
        map.insert_keyframe(pose, &lms[..n])
    }
}

// ---------------------------------------------------------------------------
// TPT-Link Map-channel framing helpers.
// ---------------------------------------------------------------------------

/// Serialize a [`LocalPose`] into a plaintext TPT-Link Map-channel frame.
pub fn serialize_pose(out: &mut [u8], seq: u16, pose: &LocalPose) -> Option<usize> {
    let mut payload = [0u8; LOCAL_POSE_LEN];
    let n = pose.encode(&mut payload)?;
    tptlink::serialize_plain(out, Channel::Map, MSG_LOCAL_POSE, seq, &payload[..n])
}

/// Serialize an [`ObstacleCloud`] into a plaintext TPT-Link Map-channel frame.
///
/// The cloud must hold at most [`MAX_FRAME_POINTS`] points to fit the wire
/// scratch buffer; larger clouds should be split across sequential frames.
pub fn serialize_cloud<const N: usize>(
    out: &mut [u8],
    seq: u16,
    cloud: &ObstacleCloud<N>,
) -> Option<usize> {
    // Encode the cloud into a fixed scratch buffer (sized independently of the
    // generic `N`, which cannot appear in a const array length on stable Rust).
    let mut payload = [0u8; 2 + MAX_FRAME_POINTS * POINT_LEN];
    let n = cloud.encode(&mut payload)?;
    tptlink::serialize_plain(out, Channel::Map, MSG_OBSTACLE_CLOUD, seq, &payload[..n])
}

/// A parsed Map-channel companion message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CompanionMsg<const N: usize> {
    Pose(LocalPose),
    Cloud(ObstacleCloud<N>),
}

/// Parse a plaintext TPT-Link Map-channel companion frame. Returns the frame
/// header and the decoded message, or `None` on any framing / channel / id
/// mismatch or decode failure.
pub fn parse<const N: usize>(buf: &[u8]) -> Option<(FrameHeader, CompanionMsg<N>)> {
    let (header, payload) = tptlink::parse_plain(buf)?;
    if header.channel != Channel::Map {
        return None;
    }
    let msg = match header.msgid {
        MSG_LOCAL_POSE => CompanionMsg::Pose(LocalPose::decode(payload)?),
        MSG_OBSTACLE_CLOUD => CompanionMsg::Cloud(ObstacleCloud::<N>::decode(payload)?),
        _ => return None,
    };
    Some((header, msg))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_pose() -> LocalPose {
        LocalPose {
            position: Vector3::new(12.5, -3.25, -7.0),
            orientation: UnitQuaternion::from_euler_angles(0.05, -0.1, 1.2),
            pos_stddev_m: 0.35,
            timestamp_us: 123_456_789,
        }
    }

    #[test]
    fn local_pose_round_trip() {
        let p = sample_pose();
        let mut buf = [0u8; LOCAL_POSE_LEN];
        let n = p.encode(&mut buf).unwrap();
        assert_eq!(n, LOCAL_POSE_LEN);
        let d = LocalPose::decode(&buf).unwrap();
        assert!((d.position - p.position).norm() < 1e-4);
        assert_eq!(d.timestamp_us, p.timestamp_us);
        assert!((d.pos_stddev_m - p.pos_stddev_m).abs() < 1e-5);
        // Orientation survives the f32 round trip (angle between quats small).
        let dot = d
            .orientation
            .quaternion()
            .dot(p.orientation.quaternion())
            .abs();
        assert!(dot > 0.9999, "quat dot {dot}");
    }

    #[test]
    fn obstacle_cloud_round_trip() {
        let pts = [
            Vector3::new(1.0, 2.0, -3.0),
            Vector3::new(-4.5, 0.25, 10.0),
            Vector3::new(0.0, 0.0, 0.0),
        ];
        let cloud: ObstacleCloud<16> = ObstacleCloud::from_points(&pts);
        assert_eq!(cloud.len(), 3);
        let mut buf = [0u8; 2 + 16 * POINT_LEN];
        let n = cloud.encode(&mut buf).unwrap();
        let d = ObstacleCloud::<16>::decode(&buf[..n]).unwrap();
        assert_eq!(d.len(), 3);
        for (got, want) in d.points().iter().zip(pts.iter()) {
            assert!((got - want).norm() < 1e-4);
        }
    }

    #[test]
    fn cloud_truncates_at_capacity() {
        let pts = [Vector3::new(1.0, 0.0, 0.0); 8];
        let cloud: ObstacleCloud<4> = ObstacleCloud::from_points(&pts);
        assert_eq!(cloud.len(), 4);
    }

    #[test]
    fn pose_frame_over_tptlink_round_trip() {
        let p = sample_pose();
        let mut buf = [0u8; 128];
        let n = serialize_pose(&mut buf, 42, &p).unwrap();
        let (header, msg) = parse::<16>(&buf[..n]).unwrap();
        assert_eq!(header.channel, Channel::Map);
        assert_eq!(header.msgid, MSG_LOCAL_POSE);
        assert_eq!(header.seq, 42);
        match msg {
            CompanionMsg::Pose(d) => assert!((d.position - p.position).norm() < 1e-4),
            _ => panic!("expected pose"),
        }
    }

    #[test]
    fn cloud_frame_over_tptlink_round_trip() {
        let pts = [Vector3::new(5.0, 1.0, -2.0), Vector3::new(6.0, -1.0, -2.0)];
        let cloud: ObstacleCloud<16> = ObstacleCloud::from_points(&pts);
        let mut buf = [0u8; 256];
        let n = serialize_cloud(&mut buf, 7, &cloud).unwrap();
        let (header, msg) = parse::<16>(&buf[..n]).unwrap();
        assert_eq!(header.msgid, MSG_OBSTACLE_CLOUD);
        match msg {
            CompanionMsg::Cloud(c) => {
                assert_eq!(c.len(), 2);
                assert!((c.points()[1] - pts[1]).norm() < 1e-4);
            }
            _ => panic!("expected cloud"),
        }
    }

    #[test]
    fn corrupt_frame_rejected() {
        let p = sample_pose();
        let mut buf = [0u8; 128];
        let n = serialize_pose(&mut buf, 1, &p).unwrap();
        buf[12] ^= 0xFF; // flip a payload byte -> CRC mismatch
        assert!(parse::<16>(&buf[..n]).is_none());
    }

    // A minimal in-test SpatialMap to verify cloud ingestion without pulling in
    // `tpt-mapping` (which would create a crate dependency cycle).
    struct MockMap {
        pose: Pose6DOF,
        count: usize,
    }
    impl SpatialMap for MockMap {
        type Error = core::convert::Infallible;
        fn insert_keyframe(
            &mut self,
            pose: Pose6DOF,
            landmarks: &[Landmark],
        ) -> Result<(), Self::Error> {
            self.pose = pose;
            self.count += landmarks.len();
            Ok(())
        }
        fn query_obstacles(
            &self,
            _bbox: &tpt_abstractions::types::BoundingBox,
            _out: &mut [Point3D],
        ) -> Result<usize, Self::Error> {
            Ok(0)
        }
        fn get_local_pose(&self) -> Result<Pose6DOF, Self::Error> {
            Ok(self.pose)
        }
        fn cull_distant_data(&mut self, _current_pos: Point3D, _max_radius: f64) {}
    }

    #[test]
    fn cloud_ingests_into_spatial_map() {
        let pts = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ];
        let cloud: ObstacleCloud<16> = ObstacleCloud::from_points(&pts);
        let mut map = MockMap {
            pose: Pose6DOF::origin(),
            count: 0,
        };
        let pose = sample_pose().to_pose6dof();
        cloud.ingest_into::<_, 16>(&mut map, pose).unwrap();
        assert_eq!(map.count, 3);
        assert_eq!(map.get_local_pose().unwrap().position, pose.position);
    }
}
