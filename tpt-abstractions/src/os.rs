//! Operating-system / platform abstraction traits (`spec.txt` §5.3, §11).

/// Time-triggered rate groups (§4.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateGroup {
    /// Inner loop: IMU read, attitude, rate control (1000 Hz).
    R1000Hz,
    /// Outer loop: position/velocity, navigation, VIO (200 Hz).
    R200Hz,
    /// Guidance & mapping: waypoints, SLAM keyframes (50 Hz).
    R50Hz,
    /// Telemetry: logging, health (10 Hz).
    R10Hz,
    /// Background: config, battery, diagnostics (1 Hz).
    R1Hz,
}

/// Nominal period of each rate group, in microseconds.
pub const RATE_GROUP_PERIOD_US: [u64; 5] = [1_000, 5_000, 20_000, 100_000, 1_000_000];

impl RateGroup {
    /// Index into [`RATE_GROUP_PERIOD_US`].
    pub const fn index(&self) -> usize {
        match self {
            RateGroup::R1000Hz => 0,
            RateGroup::R200Hz => 1,
            RateGroup::R50Hz => 2,
            RateGroup::R10Hz => 3,
            RateGroup::R1Hz => 4,
        }
    }

    /// Period of this group in microseconds.
    pub const fn period_us(&self) -> u64 {
        RATE_GROUP_PERIOD_US[self.index()]
    }
}

/// Set of rate groups that are due in a given scheduling pass.
#[derive(Debug, Clone, Copy, Default)]
pub struct RateGroups {
    bits: u8,
}

impl RateGroups {
    /// Mark `g` as due.
    pub fn mark(&mut self, g: RateGroup) {
        self.bits |= 1u8 << g.index();
    }
    /// Whether `g` is due.
    pub fn is_due(&self, g: RateGroup) -> bool {
        (self.bits & (1u8 << g.index())) != 0
    }
    /// Clear all pending groups.
    pub fn clear(&mut self) {
        self.bits = 0;
    }
}

/// A time-triggered scheduler abstraction.
pub trait Scheduler {
    type Error;
    /// Monotonic time in microseconds.
    fn monotonic_micros(&self) -> Result<u64, Self::Error>;
}

/// ARINC 653 sampling/queuing port between partitions.
pub trait PartitionChannel {
    type Error;
    /// Write a message into the channel.
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    /// Read the latest message into `out`, returning bytes written.
    fn read(&mut self, out: &mut [u8]) -> Result<usize, Self::Error>;
    /// For sampling ports: whether fresh data is available since last read.
    fn fresh(&self) -> bool;
}

/// Pool allocator reporting used by certified profiles.
pub trait MemoryPool {
    type Error;
    /// Total capacity in bytes.
    fn capacity_bytes(&self) -> usize;
    /// Currently used bytes.
    fn used_bytes(&self) -> usize;
    /// Reset the pool to empty.
    fn reset(&mut self) -> Result<(), Self::Error>;
}

/// Vehicle power system monitoring.
pub trait PowerSystem {
    type Error;
    /// Main bus voltage in volts.
    fn bus_voltage(&self) -> f64;
    /// Available power budget in watts.
    fn available_power_w(&self) -> f64;
    /// Whether the power system is within nominal limits.
    fn is_nominal(&self) -> bool;
}
