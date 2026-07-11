//! # tpt-backend-pikeos
//!
//! SYSGO PikeOS backend with ARINC 653 partitioning for the `tpt-evtol` /
//! `tpt-regional` profiles (§11.1, DO-178C DAL-A). Implements the
//! [`tpt-abstractions`] OS traits via PikeOS partitions and
//! `PartitionChannel` sampling/queuing ports.
//!
//! > **Status:** scaffolded in Phase -1. Implemented in Phase 3.

#![no_std]

pub mod partition {
    //! ARINC 653 partition / port bindings (placeholder).
}
