//! # tpt-backend-zephyr
//!
//! Zephyr RTOS backend for the `tpt-drone` / `tpt-uas` profiles (§11.1,
//! IEC 61508 SIL-2). Implements the [`tpt-abstractions`] OS traits on top of
//! Zephyr threads and the native POSIX-like API.
//!
//! > **Status:** scaffolded in Phase -1.

#![no_std]

pub mod os {
    //! Zephyr thread / pipe bindings (placeholder).
}
