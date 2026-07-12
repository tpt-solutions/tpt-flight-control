//! LiDAR / vision SLAM backend (`spec.txt` §8.1, §8.3, Phase 3).
//!
//! Keyframe management plus a planar (2D) Iterative Closest Point (ICP) scan
//! matcher used for LiDAR odometry / loop closure. The 2D ICP uses a closed-form
//! 2×2 SVD (no external LA dependency, `no_std`-safe) and assumes index-aligned
//! correspondences between the current scan and the matched keyframe — the usual
//! setup for a downsampled, ordered LiDAR scan.

use tpt_math::Vector2;

use tpt_abstractions::{
    spatial::SpatialMap,
    types::{BoundingBox, Landmark, Point3D, Pose6DOF},
};

/// A SLAM keyframe: a planar scan (`P` points in the keyframe-local frame) taken/// at a pose `(x, y, yaw)` in the local map frame.
#[derive(Debug, Clone, Copy)]
pub struct Keyframe<const P: usize> {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
    pub points: [Vector2<f64>; P],
    pub count: usize,
}

impl<const P: usize> Keyframe<P> {
    /// Create an empty keyframe at the given pose.
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            points: [Vector2::new(0.0, 0.0); P],
            count: 0,
        }
    }

    /// Append a scan point (up to `P`). Returns `false` when full.
    pub fn push(&mut self, p: Vector2<f64>) -> bool {
        if self.count >= P {
            return false;
        }
        self.points[self.count] = p;
        self.count += 1;
        true
    }
}

/// Fixed-capacity keyframe graph (ring buffer of `K` keyframes).
#[derive(Debug, Clone)]
pub struct KeyframeGraph<const K: usize, const P: usize> {
    frames: [Keyframe<P>; K],
    len: usize,
}

impl<const K: usize, const P: usize> Default for KeyframeGraph<K, P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: usize, const P: usize> KeyframeGraph<K, P> {
    /// Create an empty graph.
    pub const fn new() -> Self {
        Self {
            frames: [Keyframe::<P>::new(0.0, 0.0, 0.0); K],
            len: 0,
        }
    }

    /// Add a keyframe, overwriting the oldest when full (sliding window).
    pub fn add(&mut self, kf: Keyframe<P>) {
        if self.len < K {
            self.frames[self.len] = kf;
            self.len += 1;
        } else {
            // Shift left by one (sliding window).
            for i in 1..K {
                self.frames[i - 1] = self.frames[i];
            }
            self.frames[K - 1] = kf;
        }
    }

    /// Number of stored keyframes.
    pub const fn len(&self) -> usize {
        self.len
    }
    /// Whether the graph is empty.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
    /// Access keyframe `i`.
    pub fn get(&self, i: usize) -> Option<&Keyframe<P>> {
        if i < self.len {
            Some(&self.frames[i])
        } else {
            None
        }
    }
}

/// Planar scan matcher using closed-form 2D ICP.
pub struct ScanMatcher;

impl ScanMatcher {
    /// Recover the planar transform `(dx, dy, dyaw)` that best aligns `source`
    /// onto `target` (index-aligned correspondences). Iterate up to `iters`
    /// times. Returns the accumulated transform: `target ≈ R(dyaw) * source + t`.
    pub fn icp_2d(
        source: &[Vector2<f64>],
        target: &[Vector2<f64>],
        iters: usize,
    ) -> (f64, f64, f64) {
        let n = source.len().min(target.len());
        if n < 2 {
            return (0.0, 0.0, 0.0);
        }
        // Accumulated estimate: source' = R * source + t.
        let (mut r00, mut r01, mut r10, mut r11) = (1.0f64, 0.0, 0.0, 1.0);
        let (mut tx, mut ty) = (0.0f64, 0.0f64);

        for _ in 0..iters {
            // Centroids of the current source estimate and the target.
            let mut msx = 0.0;
            let mut msy = 0.0;
            let mut mtx = 0.0;
            let mut mty = 0.0;
            for i in 0..n {
                let sx = r00 * source[i].x + r01 * source[i].y + tx;
                let sy = r10 * source[i].x + r11 * source[i].y + ty;
                msx += sx;
                msy += sy;
                mtx += target[i].x;
                mty += target[i].y;
            }
            let msx = msx / n as f64;
            let msy = msy / n as f64;
            let mtx = mtx / n as f64;
            let mty = mty / n as f64;

            // For planar alignment the optimal incremental yaw is the angle of
            // the complex cross-covariance sum (no SVD needed):
            //   θ = atan2(Σ (p×q)_z, Σ p·q),  p = s'-ms, q = t-mt.
            let mut cross = 0.0;
            let mut dot = 0.0;
            for i in 0..n {
                let px = r00 * source[i].x + r01 * source[i].y + tx - msx;
                let py = r10 * source[i].x + r11 * source[i].y + ty - msy;
                let qx = target[i].x - mtx;
                let qy = target[i].y - mty;
                cross += px * qy - py * qx;
                dot += px * qx + py * qy;
            }
            let theta = libm::atan2(cross, dot);
            let (ic, ir) = (libm::sin(theta), libm::cos(theta));
            // Compose: R_new = R_inc * R_old.
            let nr00 = ir * r00 - ic * r10;
            let nr01 = ir * r01 - ic * r11;
            let nr10 = ic * r00 + ir * r10;
            let nr11 = ic * r01 + ir * r11;
            // t_new = mt - R_inc * ms' + R_inc * t_old, where ms' is the centroid
            // of the already-transformed source and t_old the running translation.
            let ntx = mtx - (ir * msx - ic * msy) + (ir * tx - ic * ty);
            let nty = mty - (ic * msx + ir * msy) + (ic * tx + ir * ty);
            r00 = nr00;
            r01 = nr01;
            r10 = nr10;
            r11 = nr11;
            tx = ntx;
            ty = nty;
        }
        let cyaw = libm::atan2(r10, r00);
        (tx, ty, cyaw)
    }
}

/// A [`SpatialMap`] backed by the SLAM keyframe graph.
///
/// This wires the free-standing [`KeyframeGraph`] behind the
/// `tpt-abstractions` [`SpatialMap`] trait, closing the roadmap gap for the
/// SLAM keyframe map (the octree already provides `OctreeSpatialMap`). Keyframe
/// poses are stored as planar `(x, y, yaw)`; inserted landmarks are held in the
/// keyframe-local frame and rotated + translated into the world frame on query.
///
/// * `K` bounds the number of retained keyframes (sliding window).
/// * `P` bounds the points stored per keyframe.
#[derive(Debug, Clone)]
pub struct SlamSpatialMap<const K: usize, const P: usize> {
    graph: KeyframeGraph<K, P>,
    last_pose: Pose6DOF,
}

impl<const K: usize, const P: usize> Default for SlamSpatialMap<K, P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const K: usize, const P: usize> SlamSpatialMap<K, P> {
    /// Create an empty SLAM-backed map at the origin.
    pub fn new() -> Self {
        Self {
            graph: KeyframeGraph::new(),
            last_pose: Pose6DOF::origin(),
        }
    }

    /// Borrow the underlying keyframe graph (e.g. for scan-matching / loop
    /// closure against stored keyframes).
    pub const fn graph(&self) -> &KeyframeGraph<K, P> {
        &self.graph
    }

    /// Number of keyframes currently retained.
    pub const fn keyframes(&self) -> usize {
        self.graph.len()
    }

    /// Transform a keyframe-local planar point into the world frame.
    fn to_world(kf: &Keyframe<P>, p: Vector2<f64>) -> Point3D {
        let (s, c) = (libm::sin(kf.yaw), libm::cos(kf.yaw));
        Point3D::new(kf.x + c * p.x - s * p.y, kf.y + s * p.x + c * p.y, 0.0)
    }
}

impl<const K: usize, const P: usize> SpatialMap for SlamSpatialMap<K, P> {
    type Error = core::convert::Infallible;

    fn insert_keyframe(
        &mut self,
        pose: Pose6DOF,
        landmarks: &[Landmark],
    ) -> Result<(), Self::Error> {
        self.last_pose = pose;
        let (_, _, yaw) = pose.orientation.euler_angles();
        // Store the keyframe pose; world-frame landmarks are rotated into the
        // keyframe-local frame (inverse yaw) and projected onto the ground plane.
        let (s, c) = (libm::sin(yaw), libm::cos(yaw));
        let mut kf = Keyframe::<P>::new(pose.position.x, pose.position.y, yaw);
        for lm in landmarks {
            let dx = lm.position.x - pose.position.x;
            let dy = lm.position.y - pose.position.y;
            let local = Vector2::new(c * dx + s * dy, -s * dx + c * dy);
            if !kf.push(local) {
                break;
            }
        }
        self.graph.add(kf);
        Ok(())
    }

    fn query_obstacles(
        &self,
        bbox: &BoundingBox,
        out: &mut [Point3D],
    ) -> Result<usize, Self::Error> {
        let mut count = 0usize;
        for i in 0..self.graph.len() {
            let kf = match self.graph.get(i) {
                Some(k) => k,
                None => break,
            };
            for j in 0..kf.count {
                let w = Self::to_world(kf, kf.points[j]);
                if bbox.contains(&w) {
                    if count < out.len() {
                        out[count] = w;
                        count += 1;
                    } else {
                        return Ok(count);
                    }
                }
            }
        }
        Ok(count)
    }

    fn get_local_pose(&self) -> Result<Pose6DOF, Self::Error> {
        Ok(self.last_pose)
    }

    fn cull_distant_data(&mut self, current_pos: Point3D, max_radius: f64) {
        // Rebuild the graph keeping only keyframes whose origin is within the
        // sliding-window radius of the current position.
        let mut kept: KeyframeGraph<K, P> = KeyframeGraph::new();
        for i in 0..self.graph.len() {
            if let Some(kf) = self.graph.get(i) {
                let dx = kf.x - current_pos.x;
                let dy = kf.y - current_pos.y;
                if libm::sqrt(dx * dx + dy * dy) <= max_radius {
                    kept.add(*kf);
                }
            }
        }
        self.graph = kept;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icp_recovers_known_transform() {
        // Ground-truth transform: translate (1.0, 0.5), rotate 0.2 rad.
        let dyaw = 0.2;
        let (dx, dy) = (1.0, 0.5);
        let (s, c) = (libm::sin(dyaw), libm::cos(dyaw));
        let source: [Vector2<f64>; 5] = [
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, -1.0),
        ];
        let mut target = [Vector2::new(0.0, 0.0); 5];
        for i in 0..5 {
            target[i].x = c * source[i].x - s * source[i].y + dx;
            target[i].y = s * source[i].x + c * source[i].y + dy;
        }
        let (ex, ey, eyaw) = ScanMatcher::icp_2d(&source, &target, 20);
        assert!((ex - dx).abs() < 1e-6, "dx {}", ex);
        assert!((ey - dy).abs() < 1e-6, "dy {}", ey);
        assert!((eyaw - dyaw).abs() < 1e-6, "dyaw {}", eyaw);
    }

    #[test]
    fn keyframe_graph_slides() {
        let mut g: KeyframeGraph<3, 4> = KeyframeGraph::new();
        assert!(g.is_empty());
        g.add(Keyframe::new(0.0, 0.0, 0.0));
        g.add(Keyframe::new(1.0, 0.0, 0.0));
        g.add(Keyframe::new(2.0, 0.0, 0.0));
        g.add(Keyframe::new(3.0, 0.0, 0.0)); // pushes out the first
        assert_eq!(g.len(), 3);
        assert!((g.get(0).unwrap().x - 1.0).abs() < 1e-9);
        assert!((g.get(2).unwrap().x - 3.0).abs() < 1e-9);
    }
}

#[cfg(test)]
mod spatial_map_tests {
    use super::*;
    use tpt_abstractions::types::{BoundingBox, Landmark};
    use tpt_math::Vector3;

    fn map() -> SlamSpatialMap<8, 16> {
        SlamSpatialMap::new()
    }

    fn lm(x: f64, y: f64) -> Landmark {
        Landmark {
            position: Vector3::new(x, y, 0.0),
            descriptor: 0,
        }
    }

    #[test]
    fn keyframe_landmarks_query_back_in_world_frame() {
        let mut m = map();
        let pose = Pose6DOF::origin();
        m.insert_keyframe(pose, &[lm(2.0, 1.0), lm(-3.0, 4.0)])
            .unwrap();
        assert_eq!(m.keyframes(), 1);
        let bbox = BoundingBox {
            min: Vector3::new(-10.0, -10.0, -1.0),
            max: Vector3::new(10.0, 10.0, 1.0),
        };
        let mut out = [Point3D::zeros(); 16];
        let n = m.query_obstacles(&bbox, &mut out).unwrap();
        assert_eq!(n, 2);
        assert!((out[0] - Vector3::new(2.0, 1.0, 0.0)).norm() < 1e-9);
        assert!((out[1] - Vector3::new(-3.0, 4.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn rotated_keyframe_preserves_world_landmark() {
        let mut m = map();
        let pose = Pose6DOF {
            position: Vector3::new(5.0, 0.0, 0.0),
            orientation: tpt_math::UnitQuaternion::from_euler_angles(
                0.0,
                0.0,
                core::f64::consts::FRAC_PI_2,
            ),
        };
        m.insert_keyframe(pose, &[lm(6.0, 0.0)]).unwrap();
        let bbox = BoundingBox {
            min: Vector3::new(0.0, -10.0, -1.0),
            max: Vector3::new(10.0, 10.0, 1.0),
        };
        let mut out = [Point3D::zeros(); 4];
        let n = m.query_obstacles(&bbox, &mut out).unwrap();
        assert_eq!(n, 1);
        assert!(
            (out[0] - Vector3::new(6.0, 0.0, 0.0)).norm() < 1e-6,
            "{:?}",
            out[0]
        );
    }

    #[test]
    fn cull_drops_distant_keyframes() {
        let mut m = map();
        m.insert_keyframe(
            Pose6DOF {
                position: Vector3::new(0.0, 0.0, 0.0),
                orientation: tpt_math::UnitQuaternion::identity(),
            },
            &[lm(0.5, 0.0)],
        )
        .unwrap();
        m.insert_keyframe(
            Pose6DOF {
                position: Vector3::new(100.0, 0.0, 0.0),
                orientation: tpt_math::UnitQuaternion::identity(),
            },
            &[lm(100.5, 0.0)],
        )
        .unwrap();
        assert_eq!(m.keyframes(), 2);
        m.cull_distant_data(Vector3::zeros(), 10.0);
        assert_eq!(m.keyframes(), 1);
        assert!((m.graph().get(0).unwrap().x).abs() < 1e-9);
    }
}
