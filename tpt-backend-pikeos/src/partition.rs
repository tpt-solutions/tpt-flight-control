//! ARINC 653 partition / port model for the PikeOS backend.
//!
//! Models the pieces of the ARINC 653 API the flight core actually uses:
//! partitions with a periodic schedule window, sampling ports (latest-value,
//! lossy) and queuing ports (FIFO, bounded). The simulation of the kernel is
//! deterministic and `#![no_std]`-friendly so it runs in unit tests and on a
//! partitioned target stub alike.

use tpt_abstractions::os::RateGroup;
use tpt_abstractions::{PartitionChannel, Scheduler};

/// Errors surfaced by the PikeOS backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionError {
    /// Port buffer too small for the message.
    BufferTooSmall,
    /// Queuing port is full.
    QueueFull,
    /// Port is empty (no fresh sample / nothing to dequeue).
    Empty,
    /// Port id is not known to the partition.
    UnknownPort,
    /// Partition is not in the `Running` mode.
    NotRunning,
}

/// ARINC 653 partition operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionMode {
    /// Partition is idle (not yet started).
    Idle,
    /// Partition is executing its periodic window.
    Running,
    /// Partition is blocked waiting on a port / timing.
    Waiting,
    /// Partition was stopped after a fault.
    Stopped,
}

/// Which ARINC 653 port flavour a [`PartitionChannel`] behaves as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionChannelKind {
    /// Sampling port: holds the most recent message only.
    Sampling,
    /// Queuing port: FIFO of messages, bounded depth.
    Queuing,
}

/// A sampling (latest-value) port implementing [`PartitionChannel`].
#[derive(Debug, Clone)]
pub struct SamplingPort {
    buf: [u8; 64],
    len: usize,
    fresh: bool,
}

impl SamplingPort {
    /// Create an empty sampling port.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 64],
            len: 0,
            fresh: false,
        }
    }
}

impl Default for SamplingPort {
    fn default() -> Self {
        Self::new()
    }
}

impl PartitionChannel for SamplingPort {
    type Error = PartitionError;

    fn write(&mut self, data: &[u8]) -> Result<(), PartitionError> {
        if data.len() > self.buf.len() {
            return Err(PartitionError::BufferTooSmall);
        }
        self.buf[..data.len()].copy_from_slice(data);
        self.len = data.len();
        self.fresh = true;
        Ok(())
    }

    fn read(&mut self, out: &mut [u8]) -> Result<usize, PartitionError> {
        if self.len == 0 {
            return Err(PartitionError::Empty);
        }
        if out.len() < self.len {
            return Err(PartitionError::BufferTooSmall);
        }
        out[..self.len].copy_from_slice(&self.buf[..self.len]);
        self.fresh = false;
        Ok(self.len)
    }

    fn fresh(&self) -> bool {
        self.fresh
    }
}

/// A queuing (FIFO) port implementing [`PartitionChannel`].
#[derive(Debug, Clone)]
pub struct QueuingPort {
    slots: [[u8; 32]; 4],
    lengths: [usize; 4],
    head: usize,
    tail: usize,
    count: usize,
}

impl QueuingPort {
    /// Create an empty queuing port (depth 4).
    pub const fn new() -> Self {
        Self {
            slots: [[0u8; 32]; 4],
            lengths: [0; 4],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Number of messages currently queued.
    pub const fn depth(&self) -> usize {
        self.count
    }
}

impl Default for QueuingPort {
    fn default() -> Self {
        Self::new()
    }
}

impl PartitionChannel for QueuingPort {
    type Error = PartitionError;

    fn write(&mut self, data: &[u8]) -> Result<(), PartitionError> {
        if self.count >= self.slots.len() {
            return Err(PartitionError::QueueFull);
        }
        if data.len() > self.slots[0].len() {
            return Err(PartitionError::BufferTooSmall);
        }
        self.slots[self.tail][..data.len()].copy_from_slice(data);
        self.lengths[self.tail] = data.len();
        self.tail = (self.tail + 1) % self.slots.len();
        self.count += 1;
        Ok(())
    }

    fn read(&mut self, out: &mut [u8]) -> Result<usize, PartitionError> {
        if self.count == 0 {
            return Err(PartitionError::Empty);
        }
        let len = self.lengths[self.head];
        if out.len() < len {
            return Err(PartitionError::BufferTooSmall);
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

/// An ARINC 653 partition: a periodic execution window bound to a rate group.
#[derive(Debug, Clone, Copy)]
pub struct Partition {
    id: u8,
    rate: RateGroup,
    mode: PartitionMode,
    /// Monotonic microseconds consumed while running.
    used_us: u64,
}

impl Partition {
    /// Create a partition bound to `rate` in the `Idle` mode.
    pub const fn new(id: u8, rate: RateGroup) -> Self {
        Self {
            id,
            rate,
            mode: PartitionMode::Idle,
            used_us: 0,
        }
    }

    /// Numeric partition id.
    pub const fn id(&self) -> u8 {
        self.id
    }

    /// Rate group this partition is scheduled under.
    pub const fn rate(&self) -> RateGroup {
        self.rate
    }

    /// Current mode.
    pub const fn mode(&self) -> PartitionMode {
        self.mode
    }

    /// Start (or restart) the partition.
    pub fn start(&mut self) {
        self.mode = PartitionMode::Running;
    }

    /// Stop the partition after a fault.
    pub fn stop(&mut self) {
        self.mode = PartitionMode::Stopped;
    }

    /// Account execution time (only while running).
    pub fn tick_exec(&mut self, us: u64) {
        if self.mode == PartitionMode::Running {
            self.used_us += us;
        }
    }

    /// Microseconds this partition has executed.
    pub const fn used_micros(&self) -> u64 {
        self.used_us
    }
}

/// A monotonic scheduler for the PikeOS partition set.
#[derive(Debug, Clone, Copy)]
pub struct PikeOsScheduler {
    now_us: u64,
}

impl PikeOsScheduler {
    /// Create a scheduler at t = 0.
    pub const fn new() -> Self {
        Self { now_us: 0 }
    }

    /// Advance the clock by `us` microseconds.
    pub fn advance(&mut self, us: u64) {
        self.now_us += us;
    }
}

impl Default for PikeOsScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler for PikeOsScheduler {
    type Error = PartitionError;

    fn monotonic_micros(&self) -> Result<u64, PartitionError> {
        Ok(self.now_us)
    }
}

/// The PikeOS backend: owns the partition table and the schedule.
#[derive(Debug, Clone)]
pub struct PikeOsBackend {
    partitions: [Partition; 5],
    scheduler: PikeOsScheduler,
}

impl PikeOsBackend {
    /// Build the standard 5-partition set (one per rate group), all idle.
    pub fn new() -> Self {
        Self {
            partitions: [
                Partition::new(0, RateGroup::R1000Hz),
                Partition::new(1, RateGroup::R200Hz),
                Partition::new(2, RateGroup::R50Hz),
                Partition::new(3, RateGroup::R10Hz),
                Partition::new(4, RateGroup::R1Hz),
            ],
            scheduler: PikeOsScheduler::new(),
        }
    }

    /// Reference to a partition by rate group.
    pub fn partition(&self, rate: RateGroup) -> &Partition {
        &self.partitions[rate.index()]
    }

    /// Mutable reference to a partition by rate group.
    pub fn partition_mut(&mut self, rate: RateGroup) -> &mut Partition {
        &mut self.partitions[rate.index()]
    }

    /// Start every partition (system start).
    pub fn start_all(&mut self) {
        for p in &mut self.partitions {
            p.start();
        }
    }

    /// Advance the schedule by `us` and account execution for running
    /// partitions whose window is due this pass.
    pub fn schedule(&mut self, us: u64) {
        self.scheduler.advance(us);
        for p in &mut self.partitions {
            if p.mode() == PartitionMode::Running {
                p.tick_exec(us);
            }
        }
    }

    /// The backend scheduler (for [`Scheduler`] trait use).
    pub const fn scheduler(&self) -> &PikeOsScheduler {
        &self.scheduler
    }
}

impl Default for PikeOsBackend {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sampling_port_latest_value() {
        let mut p = SamplingPort::new();
        assert!(!p.fresh());
        p.write(b"first").unwrap();
        p.write(b"second").unwrap();
        let mut out = [0u8; 16];
        let n = p.read(&mut out).unwrap();
        assert_eq!(&out[..n], b"second");
        assert!(!p.fresh());
    }

    #[test]
    fn queuing_port_fifo_order() {
        let mut p = QueuingPort::new();
        p.write(b"a").unwrap();
        p.write(b"b").unwrap();
        p.write(b"c").unwrap();
        assert_eq!(p.depth(), 3);
        let mut out = [0u8; 32];
        let n = p.read(&mut out).unwrap();
        assert_eq!(&out[..n], b"a");
        let n = p.read(&mut out).unwrap();
        assert_eq!(&out[..n], b"b");
    }

    #[test]
    fn queuing_port_full_rejected() {
        let mut p = QueuingPort::new();
        for _ in 0..4 {
            p.write(b"x").unwrap();
        }
        assert_eq!(p.write(b"x"), Err(PartitionError::QueueFull));
    }

    #[test]
    fn backend_schedules_partitions() {
        let mut be = PikeOsBackend::new();
        be.start_all();
        be.schedule(1000);
        assert_eq!(be.partition(RateGroup::R1000Hz).used_micros(), 1000);
        assert_eq!(be.partition(RateGroup::R1Hz).used_micros(), 1000);
        be.partition_mut(RateGroup::R200Hz).stop();
        be.schedule(1000);
        assert_eq!(be.partition(RateGroup::R200Hz).used_micros(), 1000);
    }

    #[test]
    fn scheduler_monotonic() {
        let mut s = PikeOsScheduler::new();
        s.advance(500);
        assert_eq!(s.monotonic_micros().unwrap(), 500);
    }
}
