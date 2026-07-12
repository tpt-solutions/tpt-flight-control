//! LiDAR / vision SLAM backend (`spec.txt` §8.1, §8.3, Phase 3).
//!
//! Keyframe management plus a planar (2D) Iterative Closest Point (ICP) scan
//! matcher used for LiDAR odometry / loop closure. The 2D ICP uses a closed-form
//! 2×2 SVD (no external LA dependency, `no_std`-safe) and assumes index-aligned
//! correspondences between the current scan and the matched keyframe — the usual
//! setup for a downsampled, ordered LiDAR scan.

use tpt_math::Vector2;

/// A SLAM keyframe: a planar scan (`P` points in the keyframe-local frame) taken
/// at a pose `(x, y, yaw)` in the local map frame.
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
