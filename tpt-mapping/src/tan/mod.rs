//! Terrain-Aided Navigation / TERCOM (`spec.txt` §8.4, Phase 3).
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

use tpt_math::Vector2;

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
