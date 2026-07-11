//! Spatial mapping & terrain database traits (`spec.txt` §5.2).
//!
//! These traits back the GPS-denied navigation subsystem. They are written to
//! be `no_std` and allocation-free: obstacle queries write into a caller-owned
//! buffer rather than returning an owned collection.

use crate::types::{BoundingBox, GeoPosition, Landmark, Point3D, Pose6DOF};

/// Onboard spatial map for GPS-denied navigation.
pub trait SpatialMap {
    type Error;

    /// Insert a new keyframe (pose + landmarks) into the map.
    fn insert_keyframe(&mut self, pose: Pose6DOF, landmarks: &[Landmark]) -> Result<(), Self::Error>;

    /// Query obstacles within `bbox`, writing up to `out.len()` points and
    /// returning the count written.
    fn query_obstacles(&self, bbox: &BoundingBox, out: &mut [Point3D]) -> Result<usize, Self::Error>;

    /// Current estimated pose relative to the map origin.
    fn get_local_pose(&self) -> Result<Pose6DOF, Self::Error>;

    /// Cull map data beyond `max_radius` from `current_pos` (sliding window).
    fn cull_distant_data(&mut self, current_pos: Point3D, max_radius: f64);
}

/// Terrain database (Digital Elevation Model) for TAN / TERCOM.
pub trait TerrainDatabase {
    type Error;
    /// Elevation in meters at the given lat/lon.
    fn get_elevation(&self, lat: f64, lon: f64) -> Result<f64, Self::Error>;
    /// A patch of elevations centered on `center` with radius `radius_m`,
    /// written into `out` (row-major, `out.len()` = rows*cols). Returns the
    /// grid dimensions as `(rows, cols)`.
    fn get_terrain_patch(
        &self,
        center: GeoPosition,
        radius_m: f64,
        out: &mut [f64],
    ) -> Result<(usize, usize), Self::Error>;
}
