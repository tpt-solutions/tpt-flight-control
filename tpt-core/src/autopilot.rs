//! Autopilot Phase 1 (`spec.txt` §6.1, resilience roadmap).
//!
//! Feature-gated behind `autopilot` on `tpt-core`. Provides:
//! - [`WaypointSequencer`] — mission / waypoint sequencing with proximity-based
//!   advancement.
//! - [`GeofenceMonitor`] — drives the previously-dead-code
//!   [`EnvelopeProtector::inside_geofence`] into a real geofence-breach
//!   response (clamp-to-fence / climb-out).
//! - [`FailsafeManager`] — a defined failsafe behaviour: RTL-to-last-good or
//!   land-in-place.
//!
//! All types are `no_std` and allocation-free (fixed-capacity waypoint store).
//! They are building blocks: the outer autopilot loop calls
//! [`WaypointSequencer::advance`], mitigates geofence breaches, and drops to
//! [`FailsafeManager`] on a fault.

use crate::envelope::{EnvelopeConfig, EnvelopeProtector};
use crate::state::{PositionTarget, VehicleState};
use libm::sqrt;
use tpt_abstractions::types::BoundingBox;
use tpt_math::Vector3;

/// A mission waypoint sequencer with a fixed-capacity store (`M` waypoints).
///
/// Waypoints advance when the vehicle comes within `radius` (horizontal) and
/// `z_radius` (vertical) of the current target. The first waypoint is active
/// immediately after [`Self::arm`].
#[derive(Debug, Clone, Copy)]
pub struct WaypointSequencer<const M: usize> {
    waypoints: [PositionTarget; M],
    count: usize,
    index: usize,
    armed: bool,
}

impl<const M: usize> WaypointSequencer<M> {
    /// Create an empty sequencer (no waypoints, disarmed).
    pub const fn new() -> Self {
        Self {
            waypoints: [PositionTarget::origin(); M],
            count: 0,
            index: 0,
            armed: false,
        }
    }

    /// Maximum number of waypoints this sequencer can hold.
    pub const fn capacity(&self) -> usize {
        M
    }

    /// Append a waypoint. Returns `false` if the store is full.
    pub fn add(&mut self, wp: PositionTarget) -> bool {
        if self.count >= M {
            return false;
        }
        self.waypoints[self.count] = wp;
        self.count += 1;
        true
    }

    /// Clear all waypoints and reset to disarmed.
    pub fn reset(&mut self) {
        self.count = 0;
        self.index = 0;
        self.armed = false;
    }

    /// Arm the sequencer: the first waypoint becomes active.
    pub fn arm(&mut self) {
        self.armed = true;
        self.index = 0;
    }

    /// Disarm: pauses advancement but retains the mission.
    pub fn disarm(&mut self) {
        self.armed = false;
    }

    /// Whether the sequencer is armed and has at least one waypoint.
    pub const fn is_active(&self) -> bool {
        self.armed && self.count > 0
    }

    /// Index of the currently-active waypoint (None if inactive / complete).
    pub const fn current_index(&self) -> Option<usize> {
        if self.is_active() && self.index < self.count {
            Some(self.index)
        } else {
            None
        }
    }

    /// The currently-active waypoint target, or `None` if inactive / complete.
    pub fn current(&self) -> Option<PositionTarget> {
        self.current_index().map(|i| self.waypoints[i])
    }

    /// Whether all waypoints have been visited.
    pub const fn is_complete(&self) -> bool {
        self.count > 0 && self.index >= self.count
    }

    /// Advance the mission if the vehicle is within capture tolerance of the
    /// active waypoint. Returns `true` if a waypoint was completed on this
    /// call. `radius` is the horizontal capture radius (m), `z_radius` the
    /// vertical one (m).
    pub fn advance(&mut self, state: &VehicleState, radius: f64, z_radius: f64) -> bool {
        if !self.is_active() {
            return false;
        }
        let Some(wp) = self.current() else {
            return false;
        };
        let dx = state.position.x - wp.x;
        let dy = state.position.y - wp.y;
        let d_horiz = sqrt(dx * dx + dy * dy);
        let d_z = (state.position.z - wp.z).abs();
        if d_horiz <= radius && d_z <= z_radius {
            self.index += 1;
            true
        } else {
            false
        }
    }
}

impl<const M: usize> Default for WaypointSequencer<M> {
    fn default() -> Self {
        Self::new()
    }
}

/// Geofence breach severity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeofenceStatus {
    /// Vehicle is well inside the fence.
    Inside,
    /// Vehicle is within `warning_margin` of the fence boundary.
    Warning,
    /// Vehicle has breached (or will imminently breach) the fence.
    Breach,
}

/// Geofence monitor and breach-response generator.
///
/// Wraps [`EnvelopeProtector::inside_geofence`] (previously dead code) into a
/// real response: on a breach it produces a recovery [`PositionTarget`] that
/// clamps the desired position to the fence boundary and adds a small climb to
/// recover inside; on a warning it tightens the desired target toward the
/// fence interior.
#[derive(Debug, Clone, Copy)]
pub struct GeofenceMonitor {
    fence: BoundingBox,
    warning_margin: f64,
    envelope: EnvelopeProtector,
}

impl GeofenceMonitor {
    /// Create a monitor for `fence` with the given warning margin (m).
    pub fn new(fence: BoundingBox, warning_margin: f64) -> Self {
        Self {
            fence,
            warning_margin,
            envelope: EnvelopeProtector::new(EnvelopeConfig::default()),
        }
    }

    /// Classify the vehicle's position against the fence.
    pub fn status(&self, state: &VehicleState) -> GeofenceStatus {
        if !self.envelope.inside_geofence(state, &self.fence) {
            return GeofenceStatus::Breach;
        }
        let p = state.position;
        let near = |a: f64, lo: f64, hi: f64| (a - lo).abs() < self.warning_margin || (hi - a).abs() < self.warning_margin;
        if near(p.x, self.fence.min.x, self.fence.max.x)
            || near(p.y, self.fence.min.y, self.fence.max.y)
            || near(p.z, self.fence.min.z, self.fence.max.z)
        {
            GeofenceStatus::Warning
        } else {
            GeofenceStatus::Inside
        }
    }

    /// Produce a safe target given the pilot/autopilot `desired` target and the
    /// current state. On a breach the returned target is clamped to the fence
    /// interior (plus a small climb margin) so the vehicle recovers inside; on
    /// a warning the desired target is pulled toward the interior; inside, the
    /// desired target is passed through.
    pub fn mitigate(&self, state: &VehicleState, desired: PositionTarget) -> PositionTarget {
        match self.status(state) {
            GeofenceStatus::Inside => desired,
            GeofenceStatus::Warning => {
                let mut t = desired;
                t.x = pull_toward(self.fence.min.x, self.fence.max.x, t.x, self.warning_margin);
                t.y = pull_toward(self.fence.min.y, self.fence.max.y, t.y, self.warning_margin);
                t.z = pull_toward(self.fence.min.z, self.fence.max.z, t.z, self.warning_margin);
                t
            }
            GeofenceStatus::Breach => {
                // Clamp the *current* position to the interior and climb to a
                // recover altitude (mid-height of the fence) so the vehicle
                // regains margin.
                let cx = clamp_between(self.fence.min.x, self.fence.max.x, state.position.x);
                let cy = clamp_between(self.fence.min.y, self.fence.max.y, state.position.y);
                let mid_z = 0.5 * (self.fence.min.z + self.fence.max.z);
                PositionTarget {
                    x: cx,
                    y: cy,
                    z: mid_z,
                    vx: 0.0,
                    vy: 0.0,
                    vz: 0.0,
                    yaw: state.attitude.2,
                }
            }
        }
    }
}

/// Failsafe strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeStrategy {
    /// Return to the last known-good position, then hold/land there.
    RtlToLastGood,
    /// Land in place at the current horizontal position.
    LandInPlace,
}

/// Failsafe manager: captures the last-good position during nominal flight and
/// generates a recovery [`PositionTarget`] when triggered.
#[derive(Debug, Clone, Copy)]
pub struct FailsafeManager {
    strategy: FailsafeStrategy,
    last_good: Vector3<f64>,
    /// Safe recovery altitude (m, NED down-positive) to descend toward.
    safe_z: f64,
    triggered: bool,
}

impl FailsafeManager {
    /// Create a manager with the given strategy and safe altitude.
    pub fn new(strategy: FailsafeStrategy, safe_z: f64) -> Self {
        Self {
            strategy,
            last_good: Vector3::zeros(),
            safe_z,
            triggered: false,
        }
    }

    /// Capture the current position as "last good" (call during nominal,
    /// flight-capable operation).
    pub fn note_good_position(&mut self, state: &VehicleState) {
        if !self.triggered {
            self.last_good = state.position;
        }
    }

    /// Trigger the failsafe. Idempotent.
    pub fn trigger(&mut self) {
        self.triggered = true;
    }

    /// Whether the failsafe is active.
    pub const fn is_active(&self) -> bool {
        self.triggered
    }

    /// Clear the failsafe (e.g. after recovery / re-arm).
    pub fn clear(&mut self) {
        self.triggered = false;
    }

    /// Produce the recovery target. Before trigger this returns `None`; once
    /// triggered it returns a target that either returns to the last-good
    /// horizontal position (RTL) or holds the current horizontal position
    /// (land-in-place), descending toward `safe_z`.
    pub fn target(&self, state: &VehicleState) -> Option<PositionTarget> {
        if !self.triggered {
            return None;
        }
        match self.strategy {
            FailsafeStrategy::RtlToLastGood => Some(PositionTarget {
                x: self.last_good.x,
                y: self.last_good.y,
                z: self.safe_z,
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                yaw: state.attitude.2,
            }),
            FailsafeStrategy::LandInPlace => Some(PositionTarget {
                x: state.position.x,
                y: state.position.y,
                z: self.safe_z,
                vx: 0.0,
                vy: 0.0,
                vz: 0.0,
                yaw: state.attitude.2,
            }),
        }
    }
}

/// Clamp `v` into `[lo, hi]`.
fn clamp_between(lo: f64, hi: f64, v: f64) -> f64 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// Pull `v` away from the nearest fence boundary by `margin` (toward interior).
fn pull_toward(lo: f64, hi: f64, v: f64, margin: f64) -> f64 {
    if v - lo < margin {
        lo + margin
    } else if hi - v < margin {
        hi - margin
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state_at(x: f64, y: f64, z: f64) -> VehicleState {
        VehicleState {
            position: Vector3::new(x, y, z),
            ..VehicleState::default()
        }
    }

    #[test]
    fn waypoint_sequence_advances_and_completes() {
        let mut seq: WaypointSequencer<4> = WaypointSequencer::new();
        let mut w = PositionTarget::origin();
        w.x = 10.0;
        seq.add(w);
        let mut w2 = PositionTarget::origin();
        w2.x = 20.0;
        seq.add(w2);
        seq.arm();
        assert_eq!(seq.current_index(), Some(0));
        // Not yet at wp0.
        assert!(!seq.advance(&state_at(0.0, 0.0, 0.0), 1.0, 0.5));
        // Move to wp0.
        assert!(seq.advance(&state_at(10.0, 0.0, 0.0), 1.0, 0.5));
        assert_eq!(seq.current_index(), Some(1));
        assert!(seq.advance(&state_at(20.0, 0.0, 0.0), 1.0, 0.5));
        assert!(seq.is_complete());
        assert_eq!(seq.current_index(), None);
    }

    #[test]
    fn waypoint_store_rejects_overflow() {
        let mut seq: WaypointSequencer<1> = WaypointSequencer::new();
        assert!(seq.add(PositionTarget::origin()));
        assert!(!seq.add(PositionTarget::origin()));
    }

    #[test]
    fn geofence_breach_clamps_to_interior() {
        let fence = BoundingBox {
            min: Vector3::new(-5.0, -5.0, -10.0),
            max: Vector3::new(5.0, 5.0, 0.0),
        };
        let mon = GeofenceMonitor::new(fence, 1.0);
        // Vehicle outside the fence to the +x side.
        let breach_state = state_at(12.0, 0.0, -3.0);
        assert_eq!(mon.status(&breach_state), GeofenceStatus::Breach);
        let desired = PositionTarget::origin();
        let t = mon.mitigate(&breach_state, desired);
        // Recovered target must be inside the fence (clamped x, mid z).
        assert!(t.x <= fence.max.x);
        assert!(t.x >= fence.min.x);
        assert!((t.z - (-5.0)).abs() < 1e-9, "z {}", t.z);
    }

    #[test]
    fn geofence_warning_pulls_inward() {
        let fence = BoundingBox {
            min: Vector3::new(-5.0, -5.0, -10.0),
            max: Vector3::new(5.0, 5.0, 0.0),
        };
        let mon = GeofenceMonitor::new(fence, 1.0);
        // Vehicle near the +x boundary (within warning margin).
        let warn_state = state_at(4.5, 0.0, -3.0);
        assert_eq!(mon.status(&warn_state), GeofenceStatus::Warning);
        let mut desired = PositionTarget::origin();
        desired.x = 5.0; // pilot wants to push further out
        let t = mon.mitigate(&warn_state, desired);
        assert!(t.x < 5.0, "should be pulled inward, got {}", t.x);
    }

    #[test]
    fn failsafe_rtl_returns_to_last_good() {
        let mut fs = FailsafeManager::new(FailsafeStrategy::RtlToLastGood, -2.0);
        fs.note_good_position(&state_at(30.0, 40.0, -5.0));
        fs.trigger();
        let t = fs.target(&state_at(0.0, 0.0, -5.0)).unwrap();
        assert!((t.x - 30.0).abs() < 1e-9);
        assert!((t.y - 40.0).abs() < 1e-9);
        assert!((t.z + 2.0).abs() < 1e-9);
    }

    #[test]
    fn failsafe_land_in_place_holds_horizontal() {
        let mut fs = FailsafeManager::new(FailsafeStrategy::LandInPlace, 0.0);
        fs.trigger();
        let t = fs.target(&state_at(7.0, 8.0, -5.0)).unwrap();
        assert!((t.x - 7.0).abs() < 1e-9);
        assert!((t.y - 8.0).abs() < 1e-9);
        assert_eq!(t.z, 0.0);
    }
}
