//! Hardware & sensor abstraction traits (`spec.txt` §5.1).

use crate::types::{CameraIntrinsics, FixType, FrameMetadata, GeoPosition, Point3D};
use tpt_math::Vector3;

/// Inertial Measurement Unit.
pub trait Imu {
    type Error;
    /// Acceleration in m/s^2 (body frame).
    fn read_accelerometer(&mut self) -> Result<Vector3<f64>, Self::Error>;
    /// Angular rate in rad/s (body frame).
    fn read_gyroscope(&mut self) -> Result<Vector3<f64>, Self::Error>;
    /// Magnetic field in uT (body frame).
    fn read_magnetometer(&mut self) -> Result<Vector3<f64>, Self::Error>;
}

/// Global Navigation Satellite System.
pub trait Gnss {
    type Error;
    /// Geodetic position.
    fn read_position(&mut self) -> Result<GeoPosition, Self::Error>;
    /// Velocity in m/s (local tangent plane).
    fn read_velocity(&mut self) -> Result<Vector3<f64>, Self::Error>;
    /// Current fix quality.
    fn get_fix_type(&self) -> FixType;
    /// Anti-spoofing / anti-jamming integrity check (§19.1).
    fn is_jammed_or_spoofed(&self) -> bool;
}

/// Visual / camera sensor (for VIO and mapping).
pub trait VisualSensor {
    type Error;
    /// Capture a frame into `buffer`, returning its metadata.
    fn capture_frame(&mut self, buffer: &mut [u8]) -> Result<FrameMetadata, Self::Error>;
    /// Camera intrinsics.
    fn get_intrinsics(&self) -> &CameraIntrinsics;
}

/// LiDAR / depth sensor.
pub trait LidarSensor {
    type Error;
    /// Fill `buffer` with points, returning the number written.
    fn read_point_cloud(&mut self, buffer: &mut [Point3D]) -> Result<usize, Self::Error>;
}

/// Radar altimeter (critical for Terrain-Aided Navigation).
pub trait RadarAltimeter {
    type Error;
    /// Height above ground level in meters.
    fn read_altitude_agl(&mut self) -> Result<f64, Self::Error>;
}
