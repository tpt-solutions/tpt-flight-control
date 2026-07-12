//! Navigation health telemetry and the GPS-degraded fusion state machine.
//!
//! `spec.txt` §7.2 / §12.2 (Phase 2). Reports a real-time assessment of
//! navigation integrity so the flight manager can trigger degradations (e.g.
//! RTL, descend, hover) when the estimate is untrustworthy.

use crate::InsEkf;
use tpt_math::Vector3;

/// Whether a particular navigation source is currently usable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceStatus {
    /// Source is available and within tolerance.
    Healthy,
    /// Source is present but degraded (e.g. inflated uncertainty).
    Degraded,
    /// Source is unavailable or rejected (e.g. GPS jammed, VIO lost tracking).
    Lost,
}

/// Overall navigation health snapshot, serialized in telemetry (§12.2).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavHealth {
    /// Active fusion mode (see [`FusionMode`]).
    pub mode: FusionMode,
    /// GPS/GNSS status.
    pub gps: SourceStatus,
    /// Visual-inertial odometry status.
    pub vio: SourceStatus,
    /// Depth/obstacle-aiding status.
    pub depth: SourceStatus,
    /// Terrain-aided navigation status.
    pub terrain: SourceStatus,
    /// Estimated position horizontal 1σ uncertainty (m).
    pub horiz_uncert_m: f64,
    /// Estimated vertical 1σ uncertainty (m).
    pub vert_uncert_m: f64,
    /// Seconds since the last update from any aiding source (s).
    pub time_since_aiding_s: f64,
}

impl NavHealth {
    /// `true` if navigation is sufficiently trustworthy to continue the mission.
    pub fn is_navigable(&self) -> bool {
        self.mode != FusionMode::Coast
            && self.horiz_uncert_m < 5.0
            && self.time_since_aiding_s < 5.0
    }
}

/// GPS-degraded fusion mode (`spec.txt` §7.2 Phase 1/2 strategy).
///
/// Transitions are evaluated by [`FusionStateMachine`] each cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FusionMode {
    /// GNSS available; full GPS/INS fusion.
    GpsAided,
    /// GNSS lost; coasting on INS only with drift bounding from other sources.
    Coast,
    /// Visual / depth odometry aiding available (indoor, urban canyon).
    VisualAided,
    /// Terrain-aided (TERCOM/TAN) correction available over rough terrain.
    TerrainAided,
}

impl FusionMode {
    /// Per-cycle mode selection given the available sources and INS drift.
    ///
    /// Priority: GPS if healthy; else terrain if available and INS drift low;
    /// else visual/depth if available; else coast.
    pub fn select(gps: SourceStatus, vio: SourceStatus, depth: SourceStatus, terrain: SourceStatus, ins_drift_m: f64) -> FusionMode {
        if gps == SourceStatus::Healthy || gps == SourceStatus::Degraded {
            return FusionMode::GpsAided;
        }
        if terrain == SourceStatus::Healthy && ins_drift_m < 25.0 {
            return FusionMode::TerrainAided;
        }
        if vio == SourceStatus::Healthy || depth == SourceStatus::Healthy {
            return FusionMode::VisualAided;
        }
        FusionMode::Coast
    }
}

/// State machine that tracks fusion mode and aids the EKF accordingly.
///
/// It monitors source availability and INS drift, selects the active
/// [`FusionMode`], and reports [`NavHealth`]. The actual correction calls into
/// [`InsEkf`] are made by the owner via [`FusionStateMachine::mode`].
#[derive(Debug, Clone)]
pub struct FusionStateMachine {
    mode: FusionMode,
    gps: SourceStatus,
    vio: SourceStatus,
    depth: SourceStatus,
    terrain: SourceStatus,
    last_aiding_ts: f64,
    now: f64,
}

impl FusionStateMachine {
    /// Create in GPS-aided mode, all sources initially healthy.
    pub fn new() -> Self {
        Self {
            mode: FusionMode::GpsAided,
            gps: SourceStatus::Healthy,
            vio: SourceStatus::Healthy,
            depth: SourceStatus::Lost,
            terrain: SourceStatus::Lost,
            last_aiding_ts: 0.0,
            now: 0.0,
        }
    }

    /// Update source statuses (called when a source report arrives).
    pub fn set_gps(&mut self, s: SourceStatus) {
        self.gps = s;
    }
    pub fn set_vio(&mut self, s: SourceStatus) {
        self.vio = s;
    }
    pub fn set_depth(&mut self, s: SourceStatus) {
        self.depth = s;
    }
    pub fn set_terrain(&mut self, s: SourceStatus) {
        self.terrain = s;
    }

    /// Advance the clock and recompute the fusion mode.
    ///
    /// `ins_drift_m` is the estimated INS-only drift since last aiding (m),
    /// used to gate terrain aiding.
    pub fn tick(&mut self, t_s: f64, ins_drift_m: f64) {
        self.now = t_s;
        self.mode = FusionMode::select(self.gps, self.vio, self.depth, self.terrain, ins_drift_m);
    }

    /// Record that an aiding measurement was fused at the current time.
    pub fn note_aiding(&mut self) {
        self.last_aiding_ts = self.now;
    }

    /// Current active fusion mode.
    pub const fn mode(&self) -> FusionMode {
        self.mode
    }

    /// Build a [`NavHealth`] snapshot from the EKF and current sources.
    pub fn health(&self, ekf: &InsEkf) -> NavHealth {
        let unc = ekf.position_uncertainty();
        NavHealth {
            mode: self.mode,
            gps: self.gps,
            vio: self.vio,
            depth: self.depth,
            terrain: self.terrain,
            horiz_uncert_m: unc,
            vert_uncert_m: unc,
            time_since_aiding_s: (self.now - self.last_aiding_ts).max(0.0),
        }
    }
}

impl Default for FusionStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience: estimate INS-only drift magnitude from a velocity estimate.
pub fn ins_drift_estimate(velocity: Vector3<f64>, dt_since_aid: f64) -> f64 {
    velocity.norm() * dt_since_aid
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_prefers_gps_when_healthy() {
        assert_eq!(
            FusionMode::select(
                SourceStatus::Healthy,
                SourceStatus::Healthy,
                SourceStatus::Lost,
                SourceStatus::Healthy,
                0.0
            ),
            FusionMode::GpsAided
        );
    }

    #[test]
    fn mode_falls_back_to_visual_without_gps() {
        assert_eq!(
            FusionMode::select(
                SourceStatus::Lost,
                SourceStatus::Healthy,
                SourceStatus::Lost,
                SourceStatus::Lost,
                0.0
            ),
            FusionMode::VisualAided
        );
    }

    #[test]
    fn mode_coasts_when_all_lost() {
        assert_eq!(
            FusionMode::select(
                SourceStatus::Lost,
                SourceStatus::Lost,
                SourceStatus::Lost,
                SourceStatus::Lost,
                0.0
            ),
            FusionMode::Coast
        );
    }

    #[test]
    fn terrain_rejected_when_drift_too_high() {
        assert_eq!(
            FusionMode::select(
                SourceStatus::Lost,
                SourceStatus::Lost,
                SourceStatus::Lost,
                SourceStatus::Healthy,
                50.0
            ),
            FusionMode::Coast
        );
    }

    #[test]
    fn health_reports_unnavigable_in_coast() {
        let mut fsm = FusionStateMachine::new();
        fsm.set_gps(SourceStatus::Lost);
        fsm.set_vio(SourceStatus::Lost);
        fsm.tick(10.0, 0.0);
        assert_eq!(fsm.mode(), FusionMode::Coast);
        // A coalesced health check needs an EKF; just assert mode gating logic.
        assert!(!NavHealth {
            mode: FusionMode::Coast,
            gps: SourceStatus::Lost,
            vio: SourceStatus::Lost,
            depth: SourceStatus::Lost,
            terrain: SourceStatus::Lost,
            horiz_uncert_m: 0.1,
            vert_uncert_m: 0.1,
            time_since_aiding_s: 10.0,
        }
        .is_navigable());
    }
}
