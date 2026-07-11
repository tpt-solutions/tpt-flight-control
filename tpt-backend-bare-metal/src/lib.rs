//! # tpt-backend-bare-metal
//!
//! Bare-metal superloop backend targeting entry-class STM32 MCUs (F4/F7/H7).
//!
//! This crate provides the [`tpt-abstractions`] implementations (HAL, OS
//! scheduler shim) and hardware bring-up for the `tpt-micro` / `tpt-drone`
//! profiles. It is intended to run as a `#![no_std]` firmware image.
//!
//! > **Status:** scaffolded in Phase -1. The HAL drivers and superloop are
//! > implemented in Phase 1 (see `spec.txt` §10.1, §18).

#![no_std]

/// Hardware abstraction implementations for STM32 entry-class targets.
pub mod hal {
    //! Placeholder for GPIO / timer / peripheral drivers.
}

/// Superloop entry point and time-triggered dispatch glue.
pub mod superloop {
    //! Placeholder for the rate-group tick driver (§4.2).
}
