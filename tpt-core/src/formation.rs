//! Formation flight for fuel savings (`spec.txt` §15, resilience roadmap).
//!
//! Feature-gated behind `formation` on `tpt-core` (depends on `swarm`). Holds a
//! trailing vehicle in the lead's **upwash** to cut induced drag — the
//! well-known "formation flight" drag-reduction effect. Two cruise profiles are
//! provided (fixed-wing and eVTOL-cruise), each defining a body-frame slot
//! behind/beside the lead where the trailing aircraft experiences upwash
//! rather than downwash.
//!
//! The controller builds on [`crate::swarm::RelativePositionController`] to
//! hold the computed world slot.

use crate::state::PositionTarget;
use crate::swarm::RelativePositionController;
use libm::{cos, sin};
use tpt_math::Vector3;

/// Cruise profile determining the upwash slot geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormationProfile {
    /// Conventional fixed-wing formation: trailing aircraft well behind and
    /// offset to one side, tucked just inboard of the lead's wingtip vortex
    /// (in the upwash) and slightly low.
    FixedWing,
    /// eVTOL cruise (rotor/wing transition): tighter, more symmetric slot
    /// behind and below the lead where the rotor/wing wash is most favourable.
    EvtolCruise,
}

impl FormationProfile {
    /// Body-frame upwash slot `(forward+, right+, down+)` relative to the lead
    /// (meters). Negative `x` = behind the lead.
    pub const fn upwash_slot(&self) -> Vector3<f64> {
        match self {
            FormationProfile::FixedWing => Vector3::new(-8.0, 3.0, 1.0),
            FormationProfile::EvtolCruise => Vector3::new(-5.0, 0.0, 1.5),
        }
    }
}

/// Formation controller holding a trailing vehicle in the lead's upwash.
#[derive(Debug, Clone, Copy)]
pub struct FormationController {
    profile: FormationProfile,
    rel: RelativePositionController,
}

impl FormationController {
    /// Create with the given profile and position-keeping gain.
    pub const fn new(profile: FormationProfile, kp: f64) -> Self {
        Self {
            profile,
            rel: RelativePositionController::new(kp),
        }
    }

    /// The body-frame upwash slot for the active profile.
    pub const fn slot(&self) -> Vector3<f64> {
        self.profile.upwash_slot()
    }

    /// World-frame slot position for the lead at `lead_pos` with heading
    /// `lead_heading` (rad, yaw about the NED down-axis). The body-frame slot
    /// is rotated into the world frame by the lead's yaw.
    pub fn slot_in_world(&self, lead_pos: Vector3<f64>, lead_heading: f64) -> Vector3<f64> {
        let s = self.profile.upwash_slot();
        // Rotate the horizontal (x,y) components by the lead's yaw; vertical
        // (z, down-positive) is unchanged.
        let (ch, sh) = (cos(lead_heading), sin(lead_heading));
        let x = s.x * ch - s.y * sh;
        let y = s.x * sh + s.y * ch;
        Vector3::new(lead_pos.x + x, lead_pos.y + y, lead_pos.z + s.z)
    }

    /// Compute the formation-hold target for the trailing vehicle at `own_pos`,
    /// given the lead's position and heading.
    pub fn update(
        &self,
        own_pos: Vector3<f64>,
        lead_pos: Vector3<f64>,
        lead_heading: f64,
    ) -> PositionTarget {
        let world_slot = self.slot_in_world(lead_pos, lead_heading);
        let vel = self.rel.update(own_pos, world_slot, Vector3::zeros());
        PositionTarget {
            x: world_slot.x,
            y: world_slot.y,
            z: world_slot.z,
            vx: vel.x,
            vy: vel.y,
            vz: vel.z,
            yaw: lead_heading,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_is_behind_the_lead_at_zero_heading() {
        let c = FormationController::new(FormationProfile::FixedWing, 0.5);
        let slot = c.slot_in_world(Vector3::new(100.0, 0.0, -10.0), 0.0);
        // Body +x is forward; the slot is behind => smaller world x.
        assert!(slot.x < 100.0, "slot.x {}", slot.x);
        // Offset to the right (+y) at zero heading.
        assert!(slot.y > 0.0, "slot.y {}", slot.y);
        // Slightly below (down-positive) => more negative z.
        assert!(slot.z > -10.0, "slot.z {}", slot.z);
    }

    #[test]
    fn slot_rotates_with_heading() {
        let c = FormationController::new(FormationProfile::EvtolCruise, 0.5);
        let slot0 = c.slot_in_world(Vector3::zeros(), 0.0);
        let slot90 = c.slot_in_world(Vector3::zeros(), core::f64::consts::FRAC_PI_2);
        // At 90° yaw the body +x (behind, -5) maps to world -y.
        assert!(slot90.y < 0.0, "slot90.y {}", slot90.y);
        assert!((slot0.x - slot90.y).abs() < 1e-9, "{} vs {}", slot0.x, slot90.y);
    }

    #[test]
    fn formation_target_holds_slot() {
        let c = FormationController::new(FormationProfile::FixedWing, 0.5);
        let lead = Vector3::new(50.0, 0.0, -8.0);
        let slot = c.slot_in_world(lead, 0.0);
        // Trailing vehicle already at the slot -> zero velocity setpoint.
        let t = c.update(slot, lead, 0.0);
        assert!(t.vx.abs() < 1e-9 && t.vy.abs() < 1e-9 && t.vz.abs() < 1e-9);
        // Far from the slot -> non-zero command toward it.
        let t2 = c.update(Vector3::zeros(), lead, 0.0);
        assert!(t2.vx != 0.0 || t2.vy != 0.0 || t2.vz != 0.0);
    }
}
