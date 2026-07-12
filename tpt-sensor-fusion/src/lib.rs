//! # tpt-sensor-fusion
//!
//! Sensor fusion for attitude and navigation (`spec.txt` §7).
//!
//! Conventions used throughout this crate:
//! - World frame: **NED** (x = north, y = east, z = down).
//! - Body frame: x = forward, y = right, z = down.
//! - Quaternions rotate **body → world**.
//!
//! Modules:
//! - [`ahrs`] — complementary-filter AHRS (Phase 0).
//! - [`ekf`] — error-state EKF fusing IMU + GNSS + VIO (Phase 2).
//! - [`nav_health`] — navigation health telemetry + GPS-degraded fusion FSM (Phase 2).
//! - [`dissimilar`] — dissimilar (VIO/TAN) GNSS cross-check for certification (Phase 5, §16.2).

#![no_std]
#![forbid(unsafe_code)]

pub mod ahrs;
pub mod dissimilar;
pub mod ekf;
pub mod nav_health;

pub use ahrs::ComplementaryAhrs;
pub use dissimilar::{DissimilarNavMonitor, DissimilarVerdict, NavSample, NavSourceKind};
pub use ekf::InsEkf;
pub use nav_health::{FusionMode, FusionStateMachine, NavHealth, SourceStatus};
