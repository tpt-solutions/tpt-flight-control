//! # tpt-mapping
//!
//! Onboard mapping and GPS-denied navigation subsystem (`spec.txt` §8). This
//! crate groups the visual-inertial odometry, LiDAR SLAM, terrain-aided
//! navigation, and sparse-voxel-octree modules used to maintain safe flight
//! when GNSS is degraded, jammed, or unavailable.
//!
//! The individual backends are implemented in later phases:
//! - `vio` — Visual-Inertial Odometry (Phase 2)
//! - `slam` — LiDAR/Vision SLAM keyframe management (Phase 3)
//! - `tan` — Terrain-Aided Navigation / TERCOM (Phase 3)
//! - `octree` — Sparse Voxel Octree obstacle map (Phase 2)
//!
//! > **Status:** crate scaffolded in Phase -1; backends land in Phases 2-3.

#![no_std]

pub mod vio;
pub mod slam;
pub mod tan;
pub mod octree;
