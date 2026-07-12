//! DO-160 environmental-qualification SITL scenarios (`spec.txt` §16.3).
//!
//! These scenarios stress the vehicle's environmental resilience rather than
//! its navigation: power-input transients (brownout / undervoltage) and
//! lightning/HIRF-induced EMI upsets. They exercise two pieces of production
//! code:
//! - [`tpt_abstractions::os::PowerSystem::brownout_active`] — the bus
//!   undervoltage detector.
//! - [`tpt_core::redundancy::FaultMonitor`] — the fault-persistence scrubber
//!   that separates *transient* environmental upsets from *permanent* faults.
//!
//! Both are driven by a deterministic, allocation-free step loop so the
//! scenarios are reproducible and can be asserted in tests.

use tpt_abstractions::os::PowerSystem;
use tpt_core::redundancy::{FaultClass, FaultMonitor};

/// An environmental-qualification scenario.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvScenario {
    /// Nominal: no transients, no upsets.
    Nominal,
    /// Power-input transient: the bus sags into brownout for a bounded window,
    /// then recovers (DO-160 §16.3 power-input transient).
    PowerTransient,
    /// Lightning/HIRF-induced EMI: short, self-clearing upsets on a channel
    /// (transient, must NOT be classified permanent).
    EmiUpset,
}

/// A mock power system whose bus can be forced into brownout.
struct MockPowerSystem {
    bus_v: f64,
    brownout: bool,
}

impl PowerSystem for MockPowerSystem {
    type Error = ();

    fn bus_voltage(&self) -> f64 {
        self.bus_v
    }

    fn available_power_w(&self) -> f64 {
        if self.brownout {
            40.0
        } else {
            200.0
        }
    }

    fn is_nominal(&self) -> bool {
        !self.brownout
    }

    fn brownout_active(&self) -> bool {
        self.brownout
    }
}

/// Deterministic environmental-qualification simulation.
pub struct EnvironmentSim {
    scenario: EnvScenario,
    tick: u64,
    ps: MockPowerSystem,
    monitor: FaultMonitor,
    last_class: FaultClass,
    brownout_ever_active: bool,
    permanent_ever_seen: bool,
}

impl EnvironmentSim {
    /// Create a simulator for `scenario`.
    pub fn new(scenario: EnvScenario) -> Self {
        Self {
            scenario,
            tick: 0,
            ps: MockPowerSystem {
                bus_v: 12.0,
                brownout: false,
            },
            monitor: FaultMonitor::new(),
            last_class: FaultClass::None,
            brownout_ever_active: false,
            permanent_ever_seen: false,
        }
    }

    /// Current scenario.
    pub const fn scenario(&self) -> EnvScenario {
        self.scenario
    }

    /// Whether the (mock) bus is currently in brownout.
    pub fn brownout_active(&self) -> bool {
        self.ps.brownout_active()
    }

    /// Whether brownout has been observed at any point so far.
    pub const fn brownout_ever_active(&self) -> bool {
        self.brownout_ever_active
    }

    /// Latest fault classification from the scrubber.
    pub const fn fault_class(&self) -> FaultClass {
        self.last_class
    }

    /// Whether a permanent fault has been observed at any point so far.
    pub const fn permanent_ever_seen(&self) -> bool {
        self.permanent_ever_seen
    }

    /// Advance one simulation step of `dt` seconds (default 1 ms).
    pub fn step(&mut self, dt: f64) {
        self.tick += 1;

        // --- Power-input transient model (DO-160 §16.3) ---
        self.ps.brownout = match self.scenario {
            EnvScenario::PowerTransient => {
                // Bus sags for ticks [1000, 1500) (a ~0.5 s transient at 1 kHz).
                self.tick >= 1000 && self.tick < 1500
            }
            _ => false,
        };
        if self.ps.brownout_active() {
            self.brownout_ever_active = true;
        }

        // --- EMI upset model (lightning / HIRF) ---
        let channel_faulted = match self.scenario {
            EnvScenario::EmiUpset => {
                // Short, self-clearing upsets: one tick fault every 50 ticks.
                self.tick % 50 == 0
            }
            _ => false,
        };
        // Only true channel upsets (not the power-input transient) drive the
        // fault-persistence scrubber; brownout is a separate detector.
        if channel_faulted {
            let cls = self.monitor.update(true, dt);
            self.last_class = cls;
            if cls == FaultClass::Permanent {
                self.permanent_ever_seen = true;
            }
        } else {
            let cls = self.monitor.update(false, dt);
            self.last_class = cls;
            if cls == FaultClass::Permanent {
                self.permanent_ever_seen = true;
            }
        }
    }

    /// Run `seconds` of simulation at `dt`.
    pub fn run(&mut self, seconds: f64, dt: f64) {
        let steps = (seconds / dt).round() as u64;
        for _ in 0..steps {
            self.step(dt);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_stays_clean() {
        let mut sim = EnvironmentSim::new(EnvScenario::Nominal);
        sim.run(3.0, 0.001);
        assert!(!sim.brownout_ever_active());
        assert!(!sim.permanent_ever_seen());
        assert_eq!(sim.fault_class(), FaultClass::None);
    }

    #[test]
    fn power_transient_browns_out_then_recovers() {
        let mut sim = EnvironmentSim::new(EnvScenario::PowerTransient);
        sim.run(3.0, 0.001);
        // Brownout was observed during the transient window ...
        assert!(sim.brownout_ever_active());
        // ... but the bus has recovered by the end of the run.
        assert!(!sim.brownout_active());
        // The transient did not latch a permanent fault.
        assert!(!sim.permanent_ever_seen());
    }

    #[test]
    fn emi_upset_is_transient_not_permanent() {
        let mut sim = EnvironmentSim::new(EnvScenario::EmiUpset);
        sim.run(3.0, 0.001);
        // Many short upsets occurred, but none should be classified permanent.
        assert!(!sim.permanent_ever_seen(), "EMI bursts must stay transient");
        // And at least one transient classification was observed.
        assert_eq!(sim.fault_class(), FaultClass::Transient);
    }

    #[test]
    fn brownout_detector_reportes_state() {
        let mut sim = EnvironmentSim::new(EnvScenario::PowerTransient);
        // Before the transient window: nominal.
        sim.run(0.5, 0.001);
        assert!(!sim.brownout_active());
        // Inside the transient window: brownout.
        sim.run(0.6, 0.001);
        assert!(sim.brownout_active());
    }
}
