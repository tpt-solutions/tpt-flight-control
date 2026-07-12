//! Visual-Inertial Odometry (VIO) front-end (`spec.txt` §8.1, Phase 2).
//!
//! A depth-aided monocular VIO estimator. Given 2D feature correspondences
//! between consecutive frames and a known altitude (from a downward range /
//! radar altimeter or depth sensor), it recovers the body-frame relative
//! translation and heading change. This is the "Visual/Depth-Aided" source used
//! by the GPS-degraded fusion state machine (`tpt-sensor-fusion::nav_health`).
//!
//! The estimator is stateless (calibration only); call [`VioEstimator::update`]
//! each frame with the latest matches. It uses a median aggregator so a few
//! outlier tracks do not corrupt the estimate.

use tpt_math::Vector3;

/// A 2D-2D feature correspondence in normalized image coordinates
/// (x right, y down, focal = 1).
#[derive(Debug, Clone, Copy)]
pub struct FeatureMatch {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

/// Relative pose estimated from one frame to the next.
#[derive(Debug, Clone, Copy, Default)]
pub struct RelativePose {
    /// Body-frame translation (m), x = forward, y = right, z = down.
    pub delta_pos: Vector3<f64>,
    /// Heading change (rad), about body z (down).
    pub delta_yaw: f64,
    /// Number of inlier matches that supported the estimate.
    pub inliers: usize,
}

/// Depth-aided monocular VIO estimator.
#[derive(Debug, Clone, Copy)]
pub struct VioEstimator {
    /// Focal length in pixels (used to convert pixel flow to bearing angle).
    focal: f64,
}

impl VioEstimator {
    /// Create with the camera focal length in pixels.
    pub const fn new(focal: f64) -> Self {
        Self { focal }
    }

    /// Estimate the relative pose from `matches` given the sensor `altitude` (m,
    /// height above the observed plane). Returns a zero pose if fewer than 3
    /// matches are supplied.
    pub fn update(&self, matches: &[FeatureMatch], altitude: f64) -> RelativePose {
        if matches.len() < 3 || altitude <= 0.0 {
            return RelativePose::default();
        }
        let n = matches.len();

        // Per-match body translation: for a downward camera at height `h`, a
        // feature's ground position relative to the camera is h * (bearing),
        // with bearing = pixel / focal. The camera displacement is the negative
        // mean change of those ground positions between frames.
        let mut dxs = heapless_vec(n);
        let mut dys = heapless_vec(n);
        let mut yaws = heapless_vec(n);
        for m in matches {
            // Bearing angle of the feature in the body x-y (ground) plane.
            let b0x = m.x0 / self.focal;
            let b0y = m.y0 / self.focal;
            let b1x = m.x1 / self.focal;
            let b1y = m.y1 / self.focal;
            // Ground displacement of the feature between frames.
            let gx = altitude * (b1x - b0x);
            let gy = altitude * (b1y - b0y);
            // Camera moved by the negative of the ground displacement.
            dxs.push(-gx);
            dys.push(-gy);
            // Heading change from the rotation of the bearing vector.
            let a0 = libm::atan2(b0y, b0x);
            let a1 = libm::atan2(b1y, b1x);
            let mut da = a1 - a0;
            while da > core::f64::consts::PI {
                da -= 2.0 * core::f64::consts::PI;
            }
            while da < -core::f64::consts::PI {
                da += 2.0 * core::f64::consts::PI;
            }
            yaws.push(da);
        }
        // Median aggregation for robustness.
        let dx = median(&mut dxs);
        let dy = median(&mut dys);
        let yaw = median(&mut yaws);
        let inliers = n;

        RelativePose {
            delta_pos: Vector3::new(dx, dy, 0.0),
            delta_yaw: yaw,
            inliers,
        }
    }
}

/// Tiny fixed-capacity f64 scratch buffer (no alloc).
struct Scratch {
    buf: [f64; 64],
    len: usize,
}
impl Scratch {
    fn new() -> Self {
        Self {
            buf: [0.0; 64],
            len: 0,
        }
    }
    fn push(&mut self, v: f64) {
        if self.len < self.buf.len() {
            self.buf[self.len] = v;
            self.len += 1;
        }
    }
}
fn heapless_vec(_n: usize) -> Scratch {
    Scratch::new()
}

/// Median of the buffer (sorts a copy; small N is fine).
fn median(s: &mut Scratch) -> f64 {
    if s.len == 0 {
        return 0.0;
    }
    let mut tmp = [0.0f64; 64];
    tmp[..s.len].copy_from_slice(&s.buf[..s.len]);
    // Selection-free insertion sort over the used prefix.
    for i in 1..s.len {
        let mut j = i;
        while j > 0 && tmp[j - 1] > tmp[j] {
            tmp.swap(j, j - 1);
            j -= 1;
        }
    }
    tmp[s.len / 2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pure_forward_motion_recovers_x_translation() {
        // Camera moves forward (+x) by 1 m at altitude 10 m, focal = 500.
        // A ground point straight ahead shifts backward in the image by
        // (dx_image) = focal * (delta_camera_x / altitude) projected.
        let est = VioEstimator::new(500.0);
        let matches = [
            FeatureMatch {
                x0: 0.0,
                y0: -20.0,
                x1: 0.0,
                y1: -20.0 - 50.0,
            }, // forward feature moves down by 50px
        ];
        // Need >=3 matches; duplicate with slight variation.
        let matches = [
            matches[0],
            FeatureMatch {
                x0: 30.0,
                y0: 10.0,
                x1: 30.0 - 5.0,
                y1: 10.0,
            },
            FeatureMatch {
                x0: -30.0,
                y0: 5.0,
                x1: -30.0 - 5.0,
                y1: 5.0,
            },
        ];
        let pose = est.update(&matches, 10.0);
        // Forward camera motion -> delta_pos.x > 0.
        assert!(pose.delta_pos.x > 0.0, "dx = {}", pose.delta_pos.x);
        assert!(pose.inliers == 3);
    }

    #[test]
    fn insufficient_matches_returns_zero() {
        let est = VioEstimator::new(500.0);
        let pose = est.update(&[], 10.0);
        assert_eq!(pose.inliers, 0);
        assert_eq!(pose.delta_pos, Vector3::zeros());
    }
}

/// Kani proof harnesses (`cargo kani -p tpt-mapping`, §16 / REQ-M-7).
///
/// Lives in `vio/mod.rs` (rather than a crate-wide module) so it can reach
/// the private `Scratch` type and its `len` field directly. The `kani`
/// crate is injected by the Kani compiler and is not in `Cargo.toml`.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// A single match (fewer than the required 3) yields the zero pose
    /// rather than a garbage estimate — the VIO front-end must fail safe.
    #[kani::proof]
    fn vio_single_match_is_zero() {
        let x0: f64 = kani::any();
        let y0: f64 = kani::any();
        let x1: f64 = kani::any();
        let y1: f64 = kani::any();
        kani::assume(x0.is_finite() && y0.is_finite() && x1.is_finite() && y1.is_finite());
        let m = FeatureMatch { x0, y0, x1, y1 };
        let pose = VioEstimator::new(500.0).update(&[m], 10.0);
        assert_eq!(pose.inliers, 0);
        assert_eq!(pose.delta_pos, Vector3::zeros());
    }

    /// A non-positive altitude yields the zero pose regardless of match
    /// count (a non-positive altitude makes the depth-scaling undefined).
    #[kani::proof]
    fn vio_nonpositive_altitude_is_zero() {
        let x0: f64 = kani::any();
        let y0: f64 = kani::any();
        let x1: f64 = kani::any();
        let y1: f64 = kani::any();
        let alt: f64 = kani::any();
        kani::assume(x0.is_finite() && y0.is_finite() && x1.is_finite() && y1.is_finite());
        kani::assume(alt.is_finite() && alt <= 0.0);
        let m = FeatureMatch { x0, y0, x1, y1 };
        let pose = VioEstimator::new(500.0).update(&[m, m, m], alt);
        assert_eq!(pose.inliers, 0);
        assert_eq!(pose.delta_pos, Vector3::zeros());
    }

    /// The fixed-capacity scratch buffer used by the median aggregator
    /// never writes past its 64-element backing array, no matter how many
    /// values are pushed (feature counts are attacker/sensor controlled and
    /// unbounded in principle, so this is the actual safety boundary for
    /// REQ-8.1-1's median aggregation).
    #[kani::proof]
    #[kani::unwind(67)]
    fn scratch_push_bounded_by_capacity() {
        let mut s = Scratch::new();
        for _ in 0..66 {
            let v: f64 = kani::any();
            kani::assume(v.is_finite());
            s.push(v);
            assert!(s.len <= 64);
        }
    }
}
