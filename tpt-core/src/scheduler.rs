//! Time-triggered rate-group scheduler (`spec.txt` §4.2).
//!
//! Drives the five rate groups (1000 / 200 / 50 / 10 / 1 Hz). The backend
//! calls [`TimeTriggeredScheduler::poll`] with the current monotonic time;
//! the returned [`RateGroups`] indicates which groups are due this pass.

use tpt_abstractions::os::{RateGroup, RateGroups, RATE_GROUP_PERIOD_US};

/// Stateful time-triggered scheduler. No allocation; fixed per-group state.
#[derive(Debug, Clone, Copy)]
pub struct TimeTriggeredScheduler {
    last_us: [u64; 5],
    started: bool,
    /// Count of rate groups that missed their deadline (for FDIR).
    missed: u64,
}

impl TimeTriggeredScheduler {
    pub const fn new() -> Self {
        Self {
            last_us: [0; 5],
            started: false,
            missed: 0,
        }
    }

    /// Number of missed deadlines observed since construction.
    pub const fn missed_deadlines(&self) -> u64 {
        self.missed
    }

    /// Poll at monotonic time `now_us`. Returns the set of groups due.
    ///
    /// On the first call, all groups are reported due so the system
    /// initializes promptly.
    pub fn poll(&mut self, now_us: u64) -> RateGroups {
        let mut due = RateGroups::default();
        if !self.started {
            self.started = true;
            for g in [
                RateGroup::R1000Hz,
                RateGroup::R200Hz,
                RateGroup::R50Hz,
                RateGroup::R10Hz,
                RateGroup::R1Hz,
            ] {
                self.last_us[g.index()] = now_us;
                due.mark(g);
            }
            return due;
        }

        for g in [
            RateGroup::R1000Hz,
            RateGroup::R200Hz,
            RateGroup::R50Hz,
            RateGroup::R10Hz,
            RateGroup::R1Hz,
        ] {
            let period = g.period_us();
            let last = self.last_us[g.index()];
            let elapsed = now_us.saturating_sub(last);
            if elapsed >= period {
                if elapsed > 2 * period {
                    self.missed += 1;
                }
                self.last_us[g.index()] = now_us;
                due.mark(g);
            }
        }
        due
    }
}

impl Default for TimeTriggeredScheduler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_poll_due_all() {
        let mut s = TimeTriggeredScheduler::new();
        let d = s.poll(0);
        assert!(d.is_due(RateGroup::R1000Hz));
        assert!(d.is_due(RateGroup::R1Hz));
    }

    #[test]
    fn rate_groups_fire_at_expected_intervals() {
        let mut s = TimeTriggeredScheduler::new();
        s.poll(0); // initialize
        // At 1000us: 1000Hz due, others not.
        let d = s.poll(1_000);
        assert!(d.is_due(RateGroup::R1000Hz));
        assert!(!d.is_due(RateGroup::R200Hz));
        // At 5000us: 1000Hz and 200Hz due.
        let d = s.poll(5_000);
        assert!(d.is_due(RateGroup::R1000Hz));
        assert!(d.is_due(RateGroup::R200Hz));
        assert!(!d.is_due(RateGroup::R50Hz));
        // At 20_000us: down to R50Hz due.
        let d = s.poll(20_000);
        assert!(d.is_due(RateGroup::R50Hz));
        assert!(d.is_due(RateGroup::R200Hz));
        assert!(!d.is_due(RateGroup::R10Hz));
    }
}
