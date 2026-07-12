//! Terrain-Aided Navigation / TERCOM (`spec.txt` §8.1, Phase 3).
//!
//! Terrain Contour Matching correlates a measured elevation profile (from a
//! radar altimeter fused with the baro/INS altitude) against a stored Digital
//! Elevation Model (DEM) to bound INS drift when GNSS is unavailable — the
//! "Terrain-Aided" mode of the fusion state machine
//! (`tpt-sensor-fusion::nav_health`).
//!
//! This module implements a Mean-Absolute-Difference (MAD) TERCOM correlator
//! over a local grid of north/east offsets. It is allocation-free and works
//! against any elevation source via a closure.

use libm::{cos, floor, sqrt};
use tpt_abstractions::{spatial::TerrainDatabase, types::GeoPosition};
use tpt_math::Vector2;

/// Approximate meters-per-degree of latitude (equirectangular local tangent).
const M_PER_DEG_LAT: f64 = 111_320.0;

/// Local-float helpers (the crate is `no_std`, so the `libm` backends are used
/// instead of the `std` float methods).
#[inline]
fn clampf(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
fn to_rad(deg: f64) -> f64 {
    deg * core::f64::consts::PI / 180.0
}

/// TERCOM correlator configuration.
#[derive(Debug, Clone, Copy)]
pub struct Tercom {
    /// Half-extent of the search grid (m) in north and east.
    pub search_radius_m: f64,
    /// Grid spacing (m).
    pub step_m: f64,
    /// Sampling spacing of the measured elevation profile along the track (m).
    pub profile_spacing_m: f64,
}

impl Tercom {
    /// Create a correlator with the given search radius and step.
    pub const fn new(search_radius_m: f64, step_m: f64, profile_spacing_m: f64) -> Self {
        Self {
            search_radius_m,
            step_m,
            profile_spacing_m,
        }
    }

    /// Correlate `profile` (elevation samples along a north-heading track
    /// starting at `origin`) against the DEM accessed via `dem(north_m, east_m)`.
    ///
    /// Returns the north/east offset (m) of the best match. The offset is added
    /// to `origin` to obtain the corrected position.
    pub fn correlate<F>(&self, profile: &[f64], origin_n: f64, origin_e: f64, dem: F) -> Vector2<f64>
    where
        F: Fn(f64, f64) -> f64,
    {
        let mut best = Vector2::new(0.0, 0.0);
        let mut best_mad = f64::MAX;
        let mut de = -self.search_radius_m;
        while de <= self.search_radius_m + 1e-9 {
            let mut dn = -self.search_radius_m;
            while dn <= self.search_radius_m + 1e-9 {
                let mut mad = 0.0;
                for (i, &meas) in profile.iter().enumerate() {
                    let north = origin_n + dn + (i as f64) * self.profile_spacing_m;
                    let east = origin_e + de;
                    let pred = dem(north, east);
                    mad += (meas - pred).abs();
                }
                mad /= profile.len().max(1) as f64;
                if mad < best_mad {
                    best_mad = mad;
                    best = Vector2::new(dn, de);
                }
                dn += self.step_m;
            }
            de += self.step_m;
        }
        best
    }
}

/// A [`TerrainDatabase`] backed by a `Fn(lat, lon) -> elevation` closure.
///
/// Intended for SITL/HIL where the DEM is generated analytically, and for
/// tests. Allocation-free; `get_terrain_patch` samples a square grid of
/// `side = floor(sqrt(out.len()))` cells centered on `center`.
#[derive(Clone, Copy)]
pub struct DemFn<F> {
    f: F,
}

impl<F> DemFn<F>
where
    F: Fn(f64, f64) -> f64,
{
    pub const fn new(f: F) -> Self {
        Self { f }
    }
}

impl<F> TerrainDatabase for DemFn<F>
where
    F: Fn(f64, f64) -> f64,
{
    type Error = core::convert::Infallible;

    fn get_elevation(&self, lat: f64, lon: f64) -> Result<f64, Self::Error> {
        Ok((self.f)(lat, lon))
    }

    fn get_terrain_patch(
        &self,
        center: GeoPosition,
        radius_m: f64,
        out: &mut [f64],
    ) -> Result<(usize, usize), Self::Error> {
        let side = sqrt(out.len() as f64) as usize;
        if side == 0 {
            return Ok((0, 0));
        }
        let cos_lat = clampf(cos(to_rad(center.lat_deg)), 1e-6, core::f64::INFINITY);
        let m_per_deg_lon = M_PER_DEG_LAT * cos_lat;
        let step = if side > 1 {
            (2.0 * radius_m) / (side as f64 - 1.0)
        } else {
            0.0
        };
        for r in 0..side {
            for c in 0..side {
                let north_m = (r as f64 - (side - 1) as f64 / 2.0) * step;
                let east_m = (c as f64 - (side - 1) as f64 / 2.0) * step;
                let lat = center.lat_deg + north_m / M_PER_DEG_LAT;
                let lon = center.lon_deg + east_m / m_per_deg_lon;
                let idx = r * side + c;
                out[idx] = (self.f)(lat, lon);
            }
        }
        Ok((side, side))
    }
}

/// A stored `rows × cols` elevation grid with a known origin and cell size.
///
/// Const-generic over the number of cells (`N = rows * cols`); data lives in
/// static RAM with no heap, suitable for embedded TAN.
#[derive(Debug, Clone)]
pub struct DemGrid<const N: usize> {
    rows: usize,
    cols: usize,
    /// Origin (top-left) latitude/longitude in degrees.
    origin_lat: f64,
    origin_lon: f64,
    /// Cell size in meters (square cells).
    cell_m: f64,
    data: [f64; N],
}

impl<const N: usize> DemGrid<N> {
    /// Build a grid from `rows*cols` ordered row-major elevations (north to
    /// south, west to east). Panics in debug if `data.len() != rows*cols`.
    pub const fn new(
        rows: usize,
        cols: usize,
        origin_lat: f64,
        origin_lon: f64,
        cell_m: f64,
        data: [f64; N],
    ) -> Self {
        Self {
            rows,
            cols,
            origin_lat,
            origin_lon,
            cell_m,
            data,
        }
    }

    /// Bilinear sample of the stored grid at `lat`/`lon` (m). Clamps to edges.
    pub fn sample(&self, lat: f64, lon: f64) -> f64 {
        let dn = (lat - self.origin_lat) * M_PER_DEG_LAT;
        let cos_lat = clampf(cos(to_rad(self.origin_lat)), 1e-6, core::f64::INFINITY);
        let de = (lon - self.origin_lon) * (M_PER_DEG_LAT * cos_lat);
        let fr = clampf(dn / self.cell_m, 0.0, (self.rows - 1) as f64);
        let fc = clampf(de / self.cell_m, 0.0, (self.cols - 1) as f64);
        let r0 = floor(fr) as usize;
        let c0 = floor(fc) as usize;
        let tr = (r0 + 1).min(self.rows - 1);
        let tc = (c0 + 1).min(self.cols - 1);
        let wr = fr - floor(fr);
        let wc = fc - floor(fc);
        let a = self.data[r0 * self.cols + c0];
        let b = self.data[r0 * self.cols + tc];
        let cc = self.data[tr * self.cols + c0];
        let d = self.data[tr * self.cols + tc];
        let top = a + (b - a) * wc;
        let bot = cc + (d - cc) * wc;
        top + (bot - top) * wr
    }
}

impl<const N: usize> TerrainDatabase for DemGrid<N> {
    type Error = core::convert::Infallible;

    fn get_elevation(&self, lat: f64, lon: f64) -> Result<f64, Self::Error> {
        Ok(self.sample(lat, lon))
    }

    fn get_terrain_patch(
        &self,
        center: GeoPosition,
        radius_m: f64,
        out: &mut [f64],
    ) -> Result<(usize, usize), Self::Error> {
        let side = sqrt(out.len() as f64) as usize;
        if side == 0 {
            return Ok((0, 0));
        }
        let cos_lat = clampf(cos(to_rad(center.lat_deg)), 1e-6, core::f64::INFINITY);
        let m_per_deg_lon = M_PER_DEG_LAT * cos_lat;
        let step = if side > 1 {
            (2.0 * radius_m) / (side as f64 - 1.0)
        } else {
            0.0
        };
        for r in 0..side {
            for c in 0..side {
                let north_m = (r as f64 - (side - 1) as f64 / 2.0) * step;
                let east_m = (c as f64 - (side - 1) as f64 / 2.0) * step;
                let lat = center.lat_deg + north_m / M_PER_DEG_LAT;
                let lon = center.lon_deg + east_m / m_per_deg_lon;
                out[r * side + c] = self.sample(lat, lon);
            }
        }
        Ok((side, side))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthetic DEM: rolling terrain, higher toward +north/+east.
    fn dem(north: f64, east: f64) -> f64 {
        0.01 * north + 0.02 * east + 0.0005 * (north * north + east * east)
    }

    #[test]
    fn correlates_true_offset() {
        let true_offset = Vector2::new(8.0, -6.0);
        // Build a measured profile starting from origin+true_offset.
        let origin = (0.0f64, 0.0f64);
        let spacing = 2.0;
        let n = 30usize;
        let mut profile = alloc_array(n);
        for i in 0..n {
            let north = origin.0 + true_offset.x + (i as f64) * spacing;
            let east = origin.1 + true_offset.y;
            profile[i] = dem(north, east);
        }
        let tercom = Tercom::new(20.0, 1.0, spacing);
        let est = tercom.correlate(&profile, origin.0, origin.1, dem);
        assert!((est.x - true_offset.x).abs() < 1.5, "dn {}", est.x);
        assert!((est.y - true_offset.y).abs() < 1.5, "de {}", est.y);
    }

    #[test]
    fn dem_fn_elevation_and_patch() {
        let dem = DemFn::new(dem);
        let center = GeoPosition::new(37.0, -122.0, 0.0);
        // The DEM closure evaluates directly through the trait.
        assert!(dem.get_elevation(0.0, 0.0).is_ok());
        let mut patch = [0.0f64; 25];
        let (rows, cols) = dem.get_terrain_patch(center, 10.0, &mut patch).unwrap();
        assert_eq!((rows, cols), (5, 5));
        // The center cell samples the DEM at the center coordinate.
        assert!((patch[12] - dem.get_elevation(center.lat_deg, center.lon_deg).unwrap()).abs() < 1e-9);
    }

    #[test]
    fn dem_grid_stored_and_patch() {
        // 3x3 ramp grid: elevation increases east and south.
        let data = [0.0, 1.0, 2.0, 10.0, 11.0, 12.0, 20.0, 21.0, 22.0];
        let grid = DemGrid::new(3, 3, 0.0, 0.0, 1.0, data);
        // Corner cell (2,2) sits 2 m east + 2 m south of the origin.
        let lat = 2.0 / 111_320.0;
        let lon = 2.0 / 111_320.0;
        assert!((grid.get_elevation(lat, lon).unwrap() - 22.0).abs() < 1e-6);
        let mut patch = [0.0f64; 9];
        let (r, c) = grid.get_terrain_patch(GeoPosition::new(0.0, 0.0, 0.0), 1.0, &mut patch).unwrap();
        assert_eq!((r, c), (3, 3));
        // Center of a 3x3 patch (radius 1 m) maps back to the grid origin cell.
        assert!((patch[4]).abs() < 1e-9);
    }
}

/// Fixed-capacity f64 array helper for tests (no alloc).
fn alloc_array(n: usize) -> heapless::Vec64 {
    heapless::Vec64::new(n)
}

mod heapless {
    pub struct Vec64 {
        buf: [f64; 64],
        len: usize,
    }
    impl Vec64 {
        pub fn new(n: usize) -> Self {
            Self {
                buf: [0.0; 64],
                len: n.min(64),
            }
        }
        pub fn len(&self) -> usize {
            self.len
        }
    }
    impl core::ops::Index<usize> for Vec64 {
        type Output = f64;
        fn index(&self, i: usize) -> &f64 {
            &self.buf[i]
        }
    }
    impl core::ops::IndexMut<usize> for Vec64 {
        fn index_mut(&mut self, i: usize) -> &mut f64 {
            &mut self.buf[i]
        }
    }
    impl core::ops::Deref for Vec64 {
        type Target = [f64];
        fn deref(&self) -> &[f64] {
            &self.buf[..self.len]
        }
    }
}
