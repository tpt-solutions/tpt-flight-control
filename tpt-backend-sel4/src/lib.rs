//! # tpt-backend-sel4
//!
//! seL4 microkernel backend for the `tpt-sovereign` stack
//! (Common Criteria EAL7+, §11.1). Implements the [`tpt-abstractions`] OS
//! traits via seL4 capabilities, endpoints, and isolated protection domains.
//!
//! > **Status:** scaffolded in Phase -1. Implemented in Phase 4.

#![no_std]

pub mod microkernel {
    //! seL4 capability / endpoint bindings (placeholder).
}
