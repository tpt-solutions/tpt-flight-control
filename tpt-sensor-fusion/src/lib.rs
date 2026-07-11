//! # tpt-sensor-fusion
//!
//! Sensor fusion for attitude and navigation (`spec.txt` §7). The Phase 0
//! deliverable is the complementary-filter AHRS in [`ahrs`]; later phases add
//! the EKF and the GPS-degraded fusion state machine.
//!
//! Conventions used throughout this crate:
//! - World frame: **NED** (x = north, y = east, z = down).
//! - Body frame: x = forward, y = right, z = down.
//! - Quaternions rotate **body → world**.

#![no_std]
#![forbid(unsafe_code)]

pub mod ahrs;

pub use ahrs::ComplementaryAhrs;
