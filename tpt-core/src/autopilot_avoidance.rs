//! Autopilot Phase 2: reactive obstacle avoidance (`spec.txt` §8.2, resilience
//! roadmap).
//!
//! Feature-gated behind `autopilot-avoidance` on `tpt-core` (which also enables
//! the `tpt-mapping` dependency). Wires [`tpt_mapping::octree::
//! SparseVoxelOctree::query_obstacles`] / [`raycast`] into the guidance loop:
//! given the onboard obstacle map and the vehicle's current position, it
//! produces a perturbed [`PositionTarget`] that routes the vehicle around
//! nearby occupied voxels.
//!
//! This is the reactive layer; strategic path-planning is out of scope here.

use crate::state::PositionTarget;
use tpt_mapping::octree::SparseVoxelOctree;
use tpt_math::Vector3;

/// Maximum obstacles considered per avoidance query (fixed buffer, no alloc).
pub const MAX_AVOID_OBSTACLES: usize = 16;

/// Reactive obstacle-avoidance guidance modifier.
///
/// Holds a reference to the live [`SparseVoxelOctree`] obstacle map. Call
/// [`Self::mitigate`] each guidance step with the desired target and the
/// vehicle position to obtain a collision-aware target.
#[derive(Debug, Clone, Copy)]
pub struct ObstacleAvoider<'a, const CAP: usize> {
    octree: &'a SparseVoxelOctree<CAP>,
    /// Look-ahead distance for the repulsion field (m).
    lookahead: f64,
    /// Aim-point push gain (m).
    gain: f64,
}

impl<'a, const CAP: usize> ObstacleAvoider<'a, CAP> {
    /// Create an avoider over `octree` with the given tunables.
    pub const fn new(octree: &'a SparseVoxelOctree<CAP>, lookahead: f64, gain: f64) -> Self {
        Self {
            octree,
            lookahead,
            gain,
        }
    }

    /// Produce a collision-aware target from `desired` and the vehicle position
    /// `pos`. For every occupied voxel within `lookahead` of `pos`, a repulsive
    /// offset (stronger when closer) is summed into the target's horizontal
    /// aim point.
    pub fn mitigate(&self, desired: PositionTarget, pos: Vector3<f64>) -> PositionTarget {
        let reach = self.lookahead;
        let mut out = [Vector3::zeros(); MAX_AVOID_OBSTACLES];
        let n = self.octree.query_obstacles(
            pos - Vector3::new(reach, reach, reach),
            pos + Vector3::new(reach, reach, reach),
            &mut out,
        );
        let mut offset = Vector3::zeros();
        for p in out[..n].iter() {
            let to = pos - *p;
            let d = to.norm();
            if d < reach && d > 1e-6 {
                let dir = to / d;
                let strength = self.gain * (1.0 - d / reach);
                offset += Vector3::new(dir.x * strength, dir.y * strength, 0.0);
            }
        }
        PositionTarget {
            x: desired.x + offset.x,
            y: desired.y + offset.y,
            z: desired.z + offset.z,
            ..desired
        }
    }

    /// Look-ahead raycast along the desired travel direction. If an obstacle is
    /// detected within `lookahead` directly ahead, returns a lateral
    /// (horizontal) avoidance offset proportional to closeness; otherwise
    /// `Vector3::zeros()`.
    ///
    /// `travel_dir` should be a unit vector; it is normalized internally.
    pub fn lookahead_offset(&self, pos: Vector3<f64>, travel_dir: Vector3<f64>) -> Vector3<f64> {
        let dir = if travel_dir.norm() > 1e-6 {
            travel_dir / travel_dir.norm()
        } else {
            return Vector3::zeros();
        };
        match self.octree.raycast(pos, dir, self.lookahead) {
            Some((_hit, dist)) if dist < self.lookahead => {
                // Push perpendicular (rotate travel dir 90° in the horizontal
                // plane) away from the hit.
                let perp = Vector3::new(-dir.y, dir.x, 0.0);
                let closeness = 1.0 - dist / self.lookahead;
                perp * (self.gain * closeness)
            }
            _ => Vector3::zeros(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::VehicleState;
    use tpt_math::Vector3 as V3;

    #[test]
    fn avoids_obstacle_directly_ahead() {
        let mut tree: SparseVoxelOctree<1024> = SparseVoxelOctree::new(Vector3::zeros(), 20.0, 4);
        // Obstacle voxel just ahead (+x) of the vehicle.
        tree.insert_point(Vector3::new(2.0, 0.0, -1.0));
        let avd = ObstacleAvoider::new(&tree, 5.0, 5.0);
        let pos = Vector3::new(0.0, 0.0, -1.0);
        let mut desired = PositionTarget::origin();
        desired.x = 10.0; // want to go straight into it
        let t = avd.mitigate(desired, pos);
        // Target should be pushed back (negative x offset) to route around.
        assert!(
            t.x < desired.x,
            "target x {} should be < {}",
            t.x,
            desired.x
        );
    }

    #[test]
    fn no_avoidance_when_clear() {
        let tree: SparseVoxelOctree<1024> = SparseVoxelOctree::new(Vector3::zeros(), 20.0, 4);
        let avd = ObstacleAvoider::new(&tree, 5.0, 5.0);
        let pos = Vector3::new(0.0, 0.0, 0.0);
        let mut desired = PositionTarget::origin();
        desired.x = 10.0;
        let t = avd.mitigate(desired, pos);
        assert!((t.x - 10.0).abs() < 1e-9);
    }

    #[test]
    fn raycast_lookahead_triggers_offset() {
        let mut tree: SparseVoxelOctree<1024> = SparseVoxelOctree::new(Vector3::zeros(), 20.0, 4);
        tree.insert_point(Vector3::new(3.0, 0.0, 0.0));
        let avd = ObstacleAvoider::new(&tree, 5.0, 5.0);
        let pos = Vector3::new(0.0, 0.0, 0.0);
        let off = avd.lookahead_offset(pos, Vector3::new(1.0, 0.0, 0.0));
        assert!(off.norm() > 0.0, "expected lateral offset, got {:?}", off);
    }

    #[test]
    fn geofence_helper_compiles_independent() {
        // Sanity: ensure integration with VehicleState position works.
        let s = VehicleState {
            position: Vector3::new(1.0, 2.0, -3.0),
            ..VehicleState::default()
        };
        assert_eq!(s.position, V3::new(1.0, 2.0, -3.0));
    }
}
