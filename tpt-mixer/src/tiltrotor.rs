//! Tilt-rotor hover-to-cruise transition logic (`spec.txt` §9.4, Phase 3).
//!
//! Manages the schedule that rotates wing/nacelle-mounted rotors from vertical
//! lift (hover, tilt = 0°) to forward thrust (cruise, tilt = 90°) as airspeed
//! increases. During the transition band the controller blends the multicopter
//! attitude loop with the fixed-wing attitude loop and the mixer is fed a tilt
//! angle so it can redirect thrust. The module is pure logic (`no_std`, no I/O);
//! the platform driver applies the resulting tilt command to the servos.

use tpt_math::clamp;

/// Transition phases of a tilt-rotor vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TiltPhase {
    /// Rotors vertical; multicopter control laws active.
    Hover,
    /// Rotors rotating; blended control, speed-building.
    Transition,
    /// Rotors horizontal; fixed-wing control laws active.
    Cruise,
}

/// Tilt-rotor transition manager.
#[derive(Debug, Clone)]
pub struct TiltRotor {
    /// Airspeed (m/s) at which transition begins.
    transition_in: f64,
    /// Airspeed (m/s) at which transition completes (cruise).
    transition_out: f64,
    /// Current nacelle tilt angle (rad, 0 = vertical, π/2 = horizontal).
    tilt: f64,
    /// Current phase.
    phase: TiltPhase,
}

impl TiltRotor {
    /// Create with the given transition airspeed band (e.g. 8 → 25 m/s).
    pub const fn new(transition_in: f64, transition_out: f64) -> Self {
        Self {
            transition_in,
            transition_out,
            tilt: 0.0,
            phase: TiltPhase::Hover,
        }
    }

    /// Update the transition state from the current `airspeed` (m/s).
    ///
    /// Returns the commanded nacelle tilt (rad). The tilt is linearly scheduled
    /// across the transition band and clamped to `[0, π/2]`.
    pub fn update(&mut self, airspeed: f64) -> f64 {
        let half_pi = core::f64::consts::FRAC_PI_2;
        if airspeed < self.transition_in {
            self.phase = TiltPhase::Hover;
            self.tilt = 0.0;
        } else if airspeed > self.transition_out {
            self.phase = TiltPhase::Cruise;
            self.tilt = half_pi;
        } else {
            self.phase = TiltPhase::Transition;
            let f = (airspeed - self.transition_in) / (self.transition_out - self.transition_in);
            self.tilt = clamp(f, 0.0, 1.0) * half_pi;
        }
        self.tilt
    }

    /// Current nacelle tilt (rad).
    pub const fn tilt(&self) -> f64 {
        self.tilt
    }

    /// Current phase.
    pub const fn phase(&self) -> TiltPhase {
        self.phase
    }

    /// Blend factor `β ∈ [0, 1]` weighting fixed-wing vs multicopter control:
    /// `0` in hover, `1` in cruise. Useful for gain scheduling in the controller.
    pub fn blend(&self) -> f64 {
        self.tilt / core::f64::consts::FRAC_PI_2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_at_low_speed() {
        let mut t = TiltRotor::new(8.0, 25.0);
        let tilt = t.update(2.0);
        assert_eq!(tilt, 0.0);
        assert_eq!(t.phase(), TiltPhase::Hover);
        assert_eq!(t.blend(), 0.0);
    }

    #[test]
    fn cruise_at_high_speed() {
        let mut t = TiltRotor::new(8.0, 25.0);
        let tilt = t.update(30.0);
        assert!((tilt - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert_eq!(t.phase(), TiltPhase::Cruise);
        assert_eq!(t.blend(), 1.0);
    }

    #[test]
    fn transition_midband_is_linear() {
        let mut t = TiltRotor::new(8.0, 28.0);
        let tilt = t.update(18.0); // halfway -> 45°
        assert!((tilt - core::f64::consts::FRAC_PI_4).abs() < 1e-9, "tilt={tilt}");
        assert_eq!(t.phase(), TiltPhase::Transition);
        assert!((t.blend() - 0.5).abs() < 1e-9);
    }
}
