//! Dissimilar navigation-source architecture for certification
//! (`spec.txt` §16.2, Phase 5).
//!
//! TPT never trusts GNSS alone. GPS is cross-checked against dissimilar
//! sources: VIO (camera or visual, an optical principle) and TAN
//! (radar-altimeter plus terrain database, a radiometric or geophysical
//! principle). A GPS spoof or jam is therefore detectable by disagreement with
//! an independent physical basis. This is the architecture required to argue
//! that navigation loss probability stays below the catastrophic threshold for
//! DAL-A.
//!
//! [`DissimilarNavMonitor`] ingests position estimates from each source with
//! their uncertainty and a bounded INS drift estimate. It detects when GNSS
//! contradicts a healthy dissimilar source (spoof/jam) and recommends a
//! [`FusionMode`] that yields to the dissimilar source.

use crate::nav_health::{FusionMode, SourceStatus};
use tpt_math::Vector3;

/// Kind of navigation source feeding the monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavSourceKind {
    /// Inertial (INS) dead-reckoning — not an independent truth source.
    Ins,
    /// GNSS (GPS/Galileo/…) — the primary but spoofable source.
    Gnss,
    /// Visual-Inertial Odometry — dissimilar (optical) source.
    Vio,
    /// Terrain-Aided Navigation (radar-altimeter vs DEM) — dissimilar source.
    Tan,
}

/// One source's position estimate in the local NED frame (meters).
#[derive(Debug, Clone, Copy)]
pub struct NavSample {
    pub kind: NavSourceKind,
    /// Local-frame position (NED, m).
    pub pos: Vector3<f64>,
    /// 1σ horizontal uncertainty (m).
    pub uncert_m: f64,
    /// Whether the source is currently available.
    pub available: bool,
}

impl NavSample {
    pub const fn new(
        kind: NavSourceKind,
        pos: Vector3<f64>,
        uncert_m: f64,
        available: bool,
    ) -> Self {
        Self {
            kind,
            pos,
            uncert_m,
            available,
        }
    }
}

/// Verdict of a dissimilar-source cross-check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DissimilarVerdict {
    /// Recommended fusion mode for the control manager.
    pub mode: FusionMode,
    /// `true` if GNSS is distrusted because it disagreed with a dissimilar source.
    pub gps_distrusted: bool,
    /// Largest disagreement observed between GNSS and a dissimilar source (m).
    pub max_disagreement_m: f64,
}

/// Cross-checks GNSS against dissimilar VIO/TAN sources.
#[derive(Debug, Clone)]
pub struct DissimilarNavMonitor {
    gps: NavSample,
    vio: NavSample,
    tan: NavSample,
    /// Bounded INS-only drift since last aiding (m); gates TAN trust.
    ins_drift_m: f64,
    /// Disagreement (m) beyond which a source is treated as untrustworthy,
    /// scaled by the combined uncertainty of the two sources.
    pub disagreement_sigma: f64,
}

impl DissimilarNavMonitor {
    pub fn new() -> Self {
        Self {
            gps: NavSample::new(NavSourceKind::Gnss, Vector3::zeros(), 1.0, false),
            vio: NavSample::new(NavSourceKind::Vio, Vector3::zeros(), 1.0, false),
            tan: NavSample::new(NavSourceKind::Tan, Vector3::zeros(), 1.0, false),
            ins_drift_m: 0.0,
            disagreement_sigma: 3.0,
        }
    }

    /// Submit a source estimate (replaces the previous sample of that kind).
    pub fn submit(&mut self, s: NavSample) {
        match s.kind {
            NavSourceKind::Gnss => self.gps = s,
            NavSourceKind::Vio => self.vio = s,
            NavSourceKind::Tan => self.tan = s,
            NavSourceKind::Ins => {}
        }
    }

    /// Record the bounded INS drift since the last aiding fix (m).
    pub fn set_ins_drift(&mut self, drift_m: f64) {
        self.ins_drift_m = drift_m;
    }

    /// Disagreement between GNSS and a dissimilar source, accounting for both
    /// sources' uncertainty (in "sigmas"). Returns `None` if either is
    /// unavailable.
    fn gps_dissimilarity(&self, other: &NavSample) -> Option<f64> {
        if !self.gps.available || !other.available {
            return None;
        }
        let dist = (self.gps.pos - other.pos).norm();
        let sigma = (self.gps.uncert_m + other.uncert_m).max(1e-6);
        Some(dist / sigma)
    }

    /// Run the cross-check and produce a verdict.
    pub fn evaluate(&self) -> DissimilarVerdict {
        let dis_vio = self.gps_dissimilarity(&self.vio);
        let dis_tan = self.gps_dissimilarity(&self.tan);

        let mut max_dis = 0.0f64;
        for d in [dis_vio, dis_tan].iter().flatten() {
            max_dis = max_dis.max(*d);
        }

        // GNSS is distrusted when it disagrees with *any* healthy dissimilar
        // source beyond `disagreement_sigma` sigmas.
        let gps_distrusted = dis_vio.is_some_and(|d| d > self.disagreement_sigma)
            || dis_tan.is_some_and(|d| d > self.disagreement_sigma);

        let mode = if !self.gps.available {
            // GNSS absent entirely: prefer TAN (if trusted) over VIO over coast,
            // mirroring [`FusionMode::select`] but with the dissimilar sources.
            if self.tan.available && self.ins_drift_m < 25.0 {
                FusionMode::TerrainAided
            } else if self.vio.available {
                FusionMode::VisualAided
            } else {
                FusionMode::Coast
            }
        } else if gps_distrusted {
            // GNSS present but contradicted by a dissimilar source: yield to the
            // dissimilar source rather than trusting the spoofed/jammed GPS.
            if dis_tan.is_some_and(|d| d > self.disagreement_sigma) && self.ins_drift_m < 25.0 {
                FusionMode::TerrainAided
            } else {
                FusionMode::VisualAided
            }
        } else {
            FusionMode::GpsAided
        };

        DissimilarVerdict {
            mode,
            gps_distrusted,
            max_disagreement_m: max_dis * self.gps.uncert_m.max(1e-6),
        }
    }
}

impl Default for DissimilarNavMonitor {
    fn default() -> Self {
        Self::new()
    }
}

/// Map a [`DissimilarNavMonitor`] verdict into the [`SourceStatus`] flags used
/// by [`crate::nav_health::NavHealth`], so the existing telemetry model carries
/// the dissimilar-source assessment.
pub fn verdict_to_status(v: &DissimilarVerdict) -> (SourceStatus, FusionMode) {
    let gps = if v.gps_distrusted {
        SourceStatus::Lost
    } else {
        SourceStatus::Healthy
    };
    (gps, v.mode)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trusts_gps_when_dissimilar_sources_agree() {
        let mut m = DissimilarNavMonitor::new();
        m.submit(NavSample::new(
            NavSourceKind::Gnss,
            Vector3::new(0.0, 0.0, 0.0),
            1.0,
            true,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Vio,
            Vector3::new(0.5, 0.0, 0.0),
            1.0,
            true,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Tan,
            Vector3::new(-0.3, 0.2, 0.0),
            1.0,
            true,
        ));
        let v = m.evaluate();
        assert!(!v.gps_distrusted);
        assert_eq!(v.mode, FusionMode::GpsAided);
    }

    #[test]
    fn detects_gps_spoof_vs_tan() {
        // TAN (radar/terrain) says we are near the origin; GPS has been spoofed
        // far away. They share no common failure mode, so the disagreement is a
        // spoof indicator.
        let mut m = DissimilarNavMonitor::new();
        m.submit(NavSample::new(
            NavSourceKind::Gnss,
            Vector3::new(500.0, 0.0, 0.0),
            1.0,
            true,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Tan,
            Vector3::new(0.0, 0.0, 0.0),
            2.0,
            true,
        ));
        m.set_ins_drift(1.0);
        let v = m.evaluate();
        assert!(v.gps_distrusted);
        assert_eq!(v.mode, FusionMode::TerrainAided);
    }

    #[test]
    fn detects_gps_spoof_vs_vio() {
        let mut m = DissimilarNavMonitor::new();
        m.submit(NavSample::new(
            NavSourceKind::Gnss,
            Vector3::new(0.0, 300.0, 0.0),
            1.0,
            true,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Vio,
            Vector3::new(0.0, 0.0, 0.0),
            1.0,
            true,
        ));
        let v = m.evaluate();
        assert!(v.gps_distrusted);
        assert_eq!(v.mode, FusionMode::VisualAided);
    }

    #[test]
    fn yields_to_vio_when_gps_absent() {
        let mut m = DissimilarNavMonitor::new();
        m.submit(NavSample::new(
            NavSourceKind::Gnss,
            Vector3::zeros(),
            1.0,
            false,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Vio,
            Vector3::zeros(),
            1.0,
            true,
        ));
        let v = m.evaluate();
        assert_eq!(v.mode, FusionMode::VisualAided);
    }

    #[test]
    fn coasts_when_all_absent() {
        let m = DissimilarNavMonitor::new();
        let v = m.evaluate();
        assert_eq!(v.mode, FusionMode::Coast);
        assert!(!v.gps_distrusted);
    }

    #[test]
    fn small_gps_error_not_flagged() {
        // Sub-sigma disagreement is normal noise, not a spoof.
        let mut m = DissimilarNavMonitor::new();
        m.submit(NavSample::new(
            NavSourceKind::Gnss,
            Vector3::new(1.5, 0.0, 0.0),
            5.0,
            true,
        ));
        m.submit(NavSample::new(
            NavSourceKind::Vio,
            Vector3::new(0.0, 0.0, 0.0),
            5.0,
            true,
        ));
        let v = m.evaluate();
        assert!(!v.gps_distrusted);
    }
}
