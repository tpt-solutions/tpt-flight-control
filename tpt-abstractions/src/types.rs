//! Shared data types used across the abstraction traits.

pub use tpt_math::Vector3;

/// A 3D point in the vehicle/local navigation frame (meters).
pub type Point3D = Vector3<f64>;

/// Geographic position (WGS-84-ish). Latitude/longitude in degrees,
/// altitude in meters above the reference ellipsoid.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoPosition {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
}

impl GeoPosition {
    pub const fn new(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        Self { lat_deg, lon_deg, alt_m }
    }
}

/// GNSS fix quality.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixType {
    NoFix,
    DeadReckoning,
    Fix2D,
    Fix3D,
    RtkFloat,
    RtkFixed,
}

/// Camera intrinsics (pinhole model).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// Image width / height in pixels.
    pub width: u32,
    pub height: u32,
}

/// Metadata returned alongside a captured camera frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameMetadata {
    pub sequence: u64,
    pub width: u32,
    pub height: u32,
    /// Capture timestamp in seconds (monotonic).
    pub timestamp_s: f64,
}

/// A sparse visual landmark observed in the environment.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landmark {
    pub position: Point3D,
    /// Descriptor id used for data association (0 = unmatched).
    pub descriptor: u32,
}

/// A 6-DOF pose: position in the local frame plus orientation quaternion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose6DOF {
    pub position: Point3D,
    pub orientation: tpt_math::UnitQuaternion<f64>,
}

impl Pose6DOF {
    /// Identity pose at the origin.
    pub fn origin() -> Self {
        Self {
            position: Point3D::zeros(),
            orientation: tpt_math::UnitQuaternion::identity(),
        }
    }
}

/// Axis-aligned bounding box for spatial queries (meters, local frame).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingBox {
    pub min: Point3D,
    pub max: Point3D,
}

impl BoundingBox {
    /// Returns `true` if `p` lies within the (inclusive) box.
    pub fn contains(&self, p: &Point3D) -> bool {
        p.x >= self.min.x && p.x <= self.max.x
            && p.y >= self.min.y && p.y <= self.max.y
            && p.z >= self.min.z && p.z <= self.max.z
    }
}
