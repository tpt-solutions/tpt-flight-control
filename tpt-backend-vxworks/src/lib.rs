//! # tpt-backend-vxworks
//!
//! Wind River VxWorks backend for the `tpt-transport` profile (§11.1, DO-178C
//! DAL-A). Provides the [`tpt_abstractions`] OS-trait implementations backed
//! by VxWorks Real-Time Processes (RTPs) and kernel message queues.
//!
//! The model implemented here is a `#![no_std]`-compatible subset: VxWorks
//! tasks (with priority + periodic rate), a message-queue [`PartitionChannel`],
//! a monotonic [`Scheduler`], and a partition health monitor that feeds the
//! flight manager's fault logic. No real VxWorks syscalls are issued — the
//! kernel is modelled deterministically so the transport stack can be
//! exercised and unit-tested on the host.
//!
//! > **Status:** implemented in Phase 5.

#![no_std]

use tpt_abstractions::os::RateGroup;
use tpt_abstractions::{PartitionChannel, Scheduler};

/// Errors surfaced by the VxWorks backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VxError {
    /// Message queue buffer too small / full / empty.
    QueueError,
    /// Task id is unknown.
    UnknownTask,
    /// Task is not in a runnable state.
    NotRunnable,
}

/// A VxWorks message queue implementing [`PartitionChannel`].
#[derive(Debug, Clone)]
pub struct MessageQueue {
    slots: [[u8; 48]; 8],
    lengths: [usize; 8],
    head: usize,
    tail: usize,
    count: usize,
}

impl MessageQueue {
    /// Create an empty message queue (depth 8).
    pub const fn new() -> Self {
        Self {
            slots: [[0u8; 48]; 8],
            lengths: [0; 8],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Current queue depth.
    pub const fn depth(&self) -> usize {
        self.count
    }
}

impl Default for MessageQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl PartitionChannel for MessageQueue {
    type Error = VxError;

    fn write(&mut self, data: &[u8]) -> Result<(), VxError> {
        if self.count >= self.slots.len() {
            return Err(VxError::QueueError);
        }
        if data.len() > self.slots[0].len() {
            return Err(VxError::QueueError);
        }
        self.slots[self.tail][..data.len()].copy_from_slice(data);
        self.lengths[self.tail] = data.len();
        self.tail = (self.tail + 1) % self.slots.len();
        self.count += 1;
        Ok(())
    }

    fn read(&mut self, out: &mut [u8]) -> Result<usize, VxError> {
        if self.count == 0 {
            return Err(VxError::QueueError);
        }
        let len = self.lengths[self.head];
        if out.len() < len {
            return Err(VxError::QueueError);
        }
        out[..len].copy_from_slice(&self.slots[self.head][..len]);
        self.head = (self.head + 1) % self.slots.len();
        self.count -= 1;
        Ok(len)
    }

    fn fresh(&self) -> bool {
        self.count > 0
    }
}

/// A VxWorks task (periodic RTP/kernel thread) with a priority and rate group.
#[derive(Debug, Clone, Copy)]
pub struct VxTask {
    id: u8,
    priority: u8,
    rate: RateGroup,
    runnable: bool,
    exec_us: u64,
}

impl VxTask {
    /// Create a task with the given id/priority/rate, runnable by default.
    pub const fn new(id: u8, priority: u8, rate: RateGroup) -> Self {
        Self {
            id,
            priority,
            rate,
            runnable: true,
            exec_us: 0,
        }
    }

    /// Task id.
    pub const fn id(&self) -> u8 {
        self.id
    }

    /// Task priority (lower = higher priority, VxWorks convention).
    pub const fn priority(&self) -> u8 {
        self.priority
    }

    /// The rate group this task belongs to.
    pub const fn rate(&self) -> RateGroup {
        self.rate
    }

    /// Whether the task is currently runnable.
    pub const fn runnable(&self) -> bool {
        self.runnable
    }

    /// Suspend the task (e.g. on fault).
    pub fn suspend(&mut self) {
        self.runnable = false;
    }

    /// Resume the task.
    pub fn resume(&mut self) {
        self.runnable = true;
    }

    /// Account execution time while runnable.
    pub fn tick_exec(&mut self, us: u64) {
        if self.runnable {
            self.exec_us += us;
        }
    }

    /// Total execution time.
    pub const fn exec_us(&self) -> u64 {
        self.exec_us
    }
}

/// Monotonic scheduler for the VxWorks task set.
#[derive(Debug, Clone, Copy)]
pub struct VxWorksScheduler {
    now_us: u64,
}

impl VxWorksScheduler {
    /// Create a scheduler at t = 0.
    pub const fn new() -> Self {
        Self { now_us: 0 }
    }

    /// Advance the clock.
    pub fn advance(&mut self, us: u64) {
        self.now_us += us;
    }
}

impl Default for VxWorksScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler for VxWorksScheduler {
    type Error = VxError;

    fn monotonic_micros(&self) -> Result<u64, VxError> {
        Ok(self.now_us)
    }
}

/// Partition health monitor feeding the flight-manager fault logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionHealth {
    /// Nominal.
    Healthy,
    /// Degraded but functional.
    Degraded,
    /// Failed; trigger fault handling.
    Failed,
}

/// The VxWorks backend: task set, message queues, scheduler, and a health
/// monitor per partition (transport-category fault tolerance).
#[derive(Debug, Clone)]
pub struct VxWorksBackend {
    tasks: [VxTask; 5],
    queues: [MessageQueue; 3],
    scheduler: VxWorksScheduler,
    health: [PartitionHealth; 5],
}

impl VxWorksBackend {
    /// Build the standard transport task set (one per rate group) plus three
    /// inter-partition queues and healthy initial health.
    pub fn new() -> Self {
        Self {
            tasks: [
                VxTask::new(0, 0, RateGroup::R1000Hz),
                VxTask::new(1, 1, RateGroup::R200Hz),
                VxTask::new(2, 2, RateGroup::R50Hz),
                VxTask::new(3, 3, RateGroup::R10Hz),
                VxTask::new(4, 4, RateGroup::R1Hz),
            ],
            queues: [
                MessageQueue::new(),
                MessageQueue::new(),
                MessageQueue::new(),
            ],
            scheduler: VxWorksScheduler::new(),
            health: [PartitionHealth::Healthy; 5],
        }
    }

    /// A task by rate group.
    pub fn task(&self, rate: RateGroup) -> &VxTask {
        &self.tasks[rate.index()]
    }

    /// Mutable task by rate group.
    pub fn task_mut(&mut self, rate: RateGroup) -> &mut VxTask {
        &mut self.tasks[rate.index()]
    }

    /// A message queue by index.
    pub fn queue(&mut self, idx: usize) -> &mut MessageQueue {
        &mut self.queues[idx]
    }

    /// Health of the partition for a given rate group.
    pub const fn health(&self, rate: RateGroup) -> PartitionHealth {
        self.health[rate.index()]
    }

    /// Update a partition's health.
    pub fn set_health(&mut self, rate: RateGroup, h: PartitionHealth) {
        self.health[rate.index()] = h;
        if h == PartitionHealth::Failed {
            self.tasks[rate.index()].suspend();
        } else {
            self.tasks[rate.index()].resume();
        }
    }

    /// The backend scheduler.
    pub const fn scheduler(&self) -> &VxWorksScheduler {
        &self.scheduler
    }

    /// Advance the schedule and account execution for runnable tasks.
    pub fn schedule(&mut self, us: u64) {
        self.scheduler.advance(us);
        for t in &mut self.tasks {
            t.tick_exec(us);
        }
    }

    /// True if all partitions are healthy (used by the transport FSM to clear
    /// the fault condition).
    pub fn all_healthy(&self) -> bool {
        self.health.iter().all(|h| *h == PartitionHealth::Healthy)
    }
}

impl Default for VxWorksBackend {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_round_trip() {
        let mut q = MessageQueue::new();
        q.write(b"telemetry").unwrap();
        let mut out = [0u8; 64];
        let n = q.read(&mut out).unwrap();
        assert_eq!(&out[..n], b"telemetry");
    }

    #[test]
    fn failed_partition_suspends_task() {
        let mut be = VxWorksBackend::new();
        be.set_health(RateGroup::R200Hz, PartitionHealth::Failed);
        assert_eq!(be.health(RateGroup::R200Hz), PartitionHealth::Failed);
        assert!(!be.task(RateGroup::R200Hz).runnable());
        be.set_health(RateGroup::R200Hz, PartitionHealth::Healthy);
        assert!(be.task(RateGroup::R200Hz).runnable());
    }

    #[test]
    fn schedule_accounts_exec() {
        let mut be = VxWorksBackend::new();
        be.schedule(2000);
        assert_eq!(be.task(RateGroup::R1000Hz).exec_us(), 2000);
        assert!(be.all_healthy());
    }

    #[test]
    fn scheduler_monotonic() {
        let mut s = VxWorksScheduler::new();
        s.advance(42);
        assert_eq!(s.monotonic_micros().unwrap(), 42);
    }
}
