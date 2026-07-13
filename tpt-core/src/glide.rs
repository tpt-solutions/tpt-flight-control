//! Engine-out glide guidance (`spec.txt` §6.4, resilience roadmap).
//!
//! Feature-gated behind `glide` on `tpt-core`. On total propulsion loss the
//! flight mode transitions to [`FlightMode::Glide`] (see [`crate::fsm`]) and
//! this module supplies:
//! - [`GlideController`] — flies best-glide speed/pitch to maximize endurance /
//!   range, holding wings level.
//! - [`GlideProfile`] — per-airframe-class best-glide parameters.
//! - [`best_landing_site`] — searches a [`TerrainDatabase`] within gliding
//!   range for the most reachable landing site.

use crate::state::{AttitudeSetpoint, VehicleState};
use tpt_abstractions::GeoPosition;
use tpt_abstractions::spatial::TerrainDatabase;
use tpt_math::{Vector3, clamp};

/// Earth parameters (shared convention with `crate::nav`).
const EARTH_RADIUS_M: f64 = 6_378_137.0;
const RAD2DEG: f64 = 180.0 / core::f64::consts::PI;

/// Glide performance profile for an airframe class.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GlideProfile {
    /// Best glide (trim) airspeed (m/s).
    pub best_glide_speed: f64,
    /// Best glide pitch attitude (rad, nose-up positive) for that airspeed.
    pub best_glide_pitch: f64,
    /// Lift-to-drag ratio (glide ratio) — horizontal reach per meter of altitude.
    pub glide_ratio: f64,
}

impl GlideProfile {
    /// Conventional fixed-wing best-glide profile.
    pub const fn fixed_wing() -> Self {
        Self {
            best_glide_speed: 18.0,
            best_glide_pitch: 0.06,
            glide_ratio: 12.0,
        }
    }

    /// eVTOL cruise (autorotation / shallow glide) profile.
    pub const fn evtol_cruise() -> Self {
        Self {
            best_glide_speed: 12.0,
            best_glide_pitch: 0.08,
            glide_ratio: 6.0,
        }
    }
}

/// Engine-out glide controller.
#[derive(Debug, Clone, Copy)]
pub struct GlideController {
    profile: GlideProfile,
    /// Pitch loop gain (rad per m/s of airspeed error).
    kp: f64,
    /// Pitch authority limit (rad).
    max_pitch: f64,
}

impl GlideController {
    /// Create with a profile and tunables.
    pub const fn new(profile: GlideProfile, kp: f64, max_pitch: f64) -> Self {
        Self {
            profile,
            kp,
            max_pitch,
        }
    }

    /// Best-glide attitude setpoint given the current state and the wind vector
    /// (m/s, world NED). Thrust is commanded to zero; roll is held level; pitch
    /// is modulated to hold best-glide airspeed (airspeed = groundspeed − wind).
    pub fn update(&self, state: &VehicleState, wind: Vector3<f64>) -> AttitudeSetpoint {
        let air_vx = state.velocity.x - wind.x;
        let air_vy = state.velocity.y - wind.y;
        let airspeed = libm::sqrt(air_vx * air_vx + air_vy * air_vy);
        let err = self.profile.best_glide_speed - airspeed;
        // Feed-forward the best-glide trim pitch, then modulate it to hold
        // airspeed: below best-glide speed -> nose down (negative pitch) to
        // trade altitude for airspeed; above -> nose up. Matches the stack's
        // `a_x = -g·sin(pitch)`.
        let pitch = clamp(
            self.profile.best_glide_pitch - self.kp * err,
            -self.max_pitch,
            self.max_pitch,
        );
        AttitudeSetpoint {
            roll: 0.0,
            pitch,
            yaw_rate: 0.0,
            thrust: 0.0,
        }
    }
}

/// Offset (meters, NED) to a geo position, equirectangular approximation.
fn meters_to_geo(origin: GeoPosition, dx: f64, dy: f64) -> GeoPosition {
    let lat0 = origin.lat_deg / RAD2DEG;
    let dlat = dy / EARTH_RADIUS_M;
    let dlon = dx / (EARTH_RADIUS_M * libm::cos(lat0));
    GeoPosition::new(
        origin.lat_deg + dlat * RAD2DEG,
        origin.lon_deg + dlon * RAD2DEG,
        origin.alt_m,
    )
}

/// Search a [`TerrainDatabase`] within gliding range for the most reachable
/// landing site.
///
/// Samples a `grid × grid` lattice of candidate sites out to
/// `altitude_agl * profile.glide_ratio` meters from `current` (ignoring wind /
/// it is the conservative still-air reach) and returns the candidate with the
/// **lowest** terrain elevation — the site that leaves the most altitude
/// margin and is therefore easiest to reach and flare into.
///
/// Returns `None` if no candidate could be evaluated (e.g. the terrain backend
/// errors everywhere) or if `altitude_agl <= 0`.
pub fn best_landing_site<T, E>(
    terrain: &T,
    current: GeoPosition,
    altitude_agl: f64,
    profile: GlideProfile,
    grid: usize,
) -> Option<GeoPosition>
where
    T: TerrainDatabase<Error = E>,
{
    if altitude_agl <= 0.0 || grid == 0 {
        return None;
    }
    let reach = altitude_agl * profile.glide_ratio;
    let step = reach / grid as f64;
    let mut best: Option<(f64, GeoPosition)> = None;
    for i in 0..=grid {
        for j in 0..=grid {
            let dx = (i as f64) * step - reach / 2.0;
            let dy = (j as f64) * step - reach / 2.0;
            let geo = meters_to_geo(current, dx, dy);
            match terrain.get_elevation(geo.lat_deg, geo.lon_deg) {
                Ok(elev) => {
                    let candidate = (elev, geo);
                    match best {
                        Some((b, _)) if b <= elev => {}
                        _ => best = Some(candidate),
                    }
                }
                Err(_) => continue,
            }
        }
    }
    best.map(|(_, g)| g)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_math::Vector3 as V3;

    struct FlatTerrain {
        /// Elevation (m) as a function of position: a gentle bowl lowest at the
        /// southwest corner, so the best site is found there.
        low_lat: f64,
        low_lon: f64,
    }
    impl TerrainDatabase for FlatTerrain {
        type Error = ();
        fn get_elevation(&self, lat: f64, lon: f64) -> Result<f64, ()> {
            // A bowl: lowest (best) terrain at the low corner, rising outward.
            let dlat = lat - self.low_lat;
            let dlon = lon - self.low_lon;
            Ok((dlat * dlat + dlon * dlon) * 1000.0)
        }
        fn get_terrain_patch(
            &self,
            _center: GeoPosition,
            _radius_m: f64,
            out: &mut [f64],
        ) -> Result<(usize, usize), ()> {
            for v in out.iter_mut() {
                *v = 0.0;
            }
            Ok((1, out.len().max(1)))
        }
    }

    #[test]
    fn glide_commands_zero_thrust_and_wings_level() {
        let c = GlideController::new(GlideProfile::fixed_wing(), 0.05, 0.3);
        let state = VehicleState {
            velocity: V3::new(18.0, 0.0, -1.0),
            ..VehicleState::default()
        };
        let sp = c.update(&state, V3::zeros());
        assert_eq!(sp.thrust, 0.0);
        assert_eq!(sp.roll, 0.0);
        // At exactly best-glide speed, pitch should be near the trim pitch.
        assert!((sp.pitch - GlideProfile::fixed_wing().best_glide_pitch).abs() < 1e-6);
    }

    #[test]
    fn glide_pitches_down_when_slow() {
        let c = GlideController::new(GlideProfile::fixed_wing(), 0.05, 0.3);
        let state = VehicleState {
            velocity: V3::new(5.0, 0.0, -1.0),
            ..VehicleState::default()
        };
        let sp = c.update(&state, V3::zeros());
        assert!(sp.pitch < 0.0, "slow -> nose down, got {}", sp.pitch);
    }

    #[test]
    fn glide_pitches_up_when_fast() {
        let c = GlideController::new(GlideProfile::fixed_wing(), 0.05, 0.3);
        let state = VehicleState {
            velocity: V3::new(30.0, 0.0, -1.0),
            ..VehicleState::default()
        };
        let sp = c.update(&state, V3::zeros());
        assert!(sp.pitch > 0.0, "fast -> nose up, got {}", sp.pitch);
    }

    #[test]
    fn best_landing_site_finds_lowest_corner() {
        let terrain = FlatTerrain {
            low_lat: 37.0,
            low_lon: -122.0,
        };
        let here = GeoPosition::new(37.01, -121.99, 500.0);
        let site = best_landing_site(&terrain, here, 300.0, GlideProfile::fixed_wing(), 8).unwrap();
        // Lowest terrain is toward the southwest corner (lower lat, lower lon).
        assert!(site.lat_deg < here.lat_deg, "lat {}", site.lat_deg);
        assert!(site.lon_deg < here.lon_deg, "lon {}", site.lon_deg);
    }

    #[test]
    fn best_landing_site_rejects_zero_altitude() {
        let terrain = FlatTerrain {
            low_lat: 0.0,
            low_lon: 0.0,
        };
        assert!(
            best_landing_site(
                &terrain,
                GeoPosition::new(0.0, 0.0, 0.0),
                0.0,
                GlideProfile::fixed_wing(),
                4,
            )
            .is_none()
        );
    }
}
