//! # tpt-backend-sel4
//!
//! seL4 microkernel backend for the `tpt-sovereign` stack
//! (Common Criteria EAL7+, §11.1). Implements the [`tpt_abstractions`] OS
//! traits via seL4 capabilities, endpoints, and isolated protection domains.
//!
//! The model implemented here is a minimal, deterministic subset of the seL4
//! object model: capabilities carry a rights mask, endpoints provide
//! synchronous point-to-point IPC modelled as a [`PartitionChannel`], and
//! protection domains are isolated address spaces that only communicate
//! through capabilities they have been granted. It is `#![no_std]` and runs in
//! unit tests and on a seL4 stub without real syscalls.
//!
//! > **Status:** scaffolded in Phase -1, capability/endpoint model implemented
//! > in Phase 4.

#![no_std]

pub mod microkernel;

pub use microkernel::{
    CapRights, Endpoint, ProtectionDomain, Sel4Backend, Sel4Error, Sel4Scheduler,
};
