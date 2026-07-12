//! # tpt-backend-pikeos
//!
//! SYSGO PikeOS backend with ARINC 653 partitioning for the `tpt-evtol` /
//! `tpt-regional` profiles (§11.1, DO-178C DAL-A). Implements the
//! [`tpt_abstractions`] OS traits via PikeOS partitions and
//! `PartitionChannel` sampling/queuing ports.
//!
//! The ARINC 653 model here is a faithful, allocation-free subset sufficient
//! for the flight core to run as a partitioned application: each
//! [`Partition`] owns a periodic window on the time-triggered schedule, and
//! partitions exchange data through [`SamplingPort`] / [`QueuingPort`]
//! channels. The backend is `#![no_std]` and implements no real syscalls — it
//! models the partition state machine and ports so the rest of the stack can
//! be exercised and unit-tested on the host.
//!
//! > **Status:** scaffolded in Phase -1, ARINC 653 partition model implemented
//! > in Phase 3.

#![no_std]

pub mod partition;

pub use partition::{
    Partition, PartitionChannelKind, PartitionError, PartitionMode, PikeOsBackend,
    PikeOsScheduler, QueuingPort, SamplingPort,
};
