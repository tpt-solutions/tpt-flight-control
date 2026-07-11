//! # tpt-sim
//!
//! Physics simulator and SITL environment for TPT Flight Control
//! (`spec.txt` §14). Provides the rigid-body [`plant`], a closed-loop
//! [`sim`] harness, and (in later phases) the GPS-denied scenarios
//! (Urban Canyon, Electronic Warfare, Indoor/Subterranean, Sensor
//! Degradation).
//!
//! **Phase 0 milestone:** a virtual quadcopter hovers in simulation.

#![allow(clippy::too_many_lines)]

pub mod plant;
pub mod sim;

pub use plant::Plant;
pub use sim::Sim;
