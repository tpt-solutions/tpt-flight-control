//! # tpt-sim
//!
//! Physics simulator and SITL environment for TPT Flight Control
//! (`spec.txt` §14). Provides the rigid-body [`plant`], a closed-loop
//! [`sim`] harness, and (in later phases) the GPS-denied scenarios
//! (Urban Canyon, Electronic Warfare, Indoor/Subterranean, Sensor
//! Degradation).
//!
//! **Phase 0 milestone:** a virtual quadcopter hovers in simulation.
//!
//! The [`replay`] module records a flight log, replays it through the *same*
//! control-law crate, and diffs the reproduced telemetry against the original —
//! a deterministic-regression check for the control stack.

#![allow(clippy::too_many_lines)]

pub mod plant;
pub mod replay;
pub mod scenarios;
pub mod sim;

/// DO-160 environmental-qualification SITL scenarios (`spec.txt` §16.3):
/// power-input transients / brownout and lightning/HIRF-induced EMI upsets,
/// exercised through the `tpt-core::redundancy` fault-persistence scrubber and
/// the `PowerSystem::brownout_active` trait method.
pub mod environment;

pub use plant::Plant;
pub use scenarios::{GpsDeniedSim, ObstacleField, Scenario, SenseBatch, Sphere};
pub use sim::Sim;
