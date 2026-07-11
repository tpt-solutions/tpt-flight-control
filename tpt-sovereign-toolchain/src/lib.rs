//! # tpt-sovereign-toolchain
//!
//! Custom compiler-qualification wrapper for the TPT Sovereign stack
//! (§13, §16). Provides build harness and verified-subset linting used to
//! qualify the Rust toolchain for DO-178C / Common Criteria evidence.
//!
//! > **Status:** scaffolded in Phase -1. Implemented in Phase 4.

//! Marker confirming the sovereign verified-subset feature is enabled.
pub const SOVEREIGN_VERIFIED_SUBSET: bool = cfg!(feature = "verified-subset");
