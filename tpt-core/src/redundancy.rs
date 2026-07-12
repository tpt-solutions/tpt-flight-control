//! Triple / quad-redundant dissimilar architecture with consensus and
//! dissimilar-monitor voting (`spec.txt` §4.3, Phase 5).
//!
//! For the `tpt-regional` / `tpt-transport` profiles, TPT does not rely on a
//! single lane. Instead it runs three or four **dissimilar** channels — each a
//! separate, independently-implemented copy of the control / navigation
//! function (different algorithms, possibly different toolchains) — and a
//! cross-channel monitor compares their outputs. A [`Voter`] then reduces the
//! channel reports to a single, safe command.
//!
//! Two voting strategies are provided:
//! - [`MidValueSelect`] — the "mid-value select" of §4.3: among the healthy
//!   channels pick the most-central report (the median for scalar triples, and
//!   the most-central vector for multi-axis states). Rejects a single
//!   rogue/faulted channel.
//! - [`Consensus`] — requires a quorum of channels in close agreement; otherwise
//!   it reports a [`VoteStatus::Disagreement`] and emits the safe neutral value.
//! - [`MonitorVoter`] — a dedicated, dissimilar *monitor* channel cross-checks
//!   the active lanes; if the monitor disagrees with the active consensus it
//!   issues a [`VoteStatus::MonitorVeto`] (fail-safe).
//!
//! The whole layer is `#![no_std]` and allocation-free so it can sit in the
//! flight-critical path between the control laws and the mixer.

use tpt_math::Vector3;

/// A value that redundant channels can vote on.
///
/// Implementors define how two reports disagree (for monitors / consensus
/// gates) and how to combine them (for averaging / fallbacks). The `neutral`
/// element is the safe value emitted when voting cannot produce a trustworthy
/// result.
pub trait Votable: Copy {
    /// Pairwise disagreement magnitude (`>= 0`, `0` == identical).
    fn disagreement(&self, other: &Self) -> f64;
    /// The safe fallback / neutral element (e.g. zero thrust, level attitude).
    fn neutral() -> Self;
    /// Convex combination `self*(1-w) + other*w` with `w in [0, 1]`.
    fn combine(&self, other: &Self, w: f64) -> Self;
}

impl Votable for f64 {
    fn disagreement(&self, other: &Self) -> f64 {
        (self - other).abs()
    }
    fn neutral() -> Self {
        0.0
    }
    fn combine(&self, other: &Self, w: f64) -> Self {
        self * (1.0 - w) + other * w
    }
}

impl Votable for Vector3<f64> {
    fn disagreement(&self, other: &Self) -> f64 {
        (self - other).norm()
    }
    fn neutral() -> Self {
        Vector3::zeros()
    }
    fn combine(&self, other: &Self, w: f64) -> Self {
        self * (1.0 - w) + other * w
    }
}

/// One channel's output together with its own self-reported health.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelReport<T: Votable> {
    /// The channel's computed value.
    pub value: T,
    /// `true` if the channel considers itself healthy (passes its own BIT).
    pub healthy: bool,
}

impl<T: Votable> ChannelReport<T> {
    pub const fn new(value: T, healthy: bool) -> Self {
        Self { value, healthy }
    }
}

/// Outcome of a vote.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoteStatus {
    /// Enough channels agreed within tolerance; `value` is the consensus.
    Agreement,
    /// The channels disagreed beyond tolerance; `value` is the safe fallback.
    Disagreement,
    /// A dissimilar monitor vetoed the active lanes; `value` is the safe
    /// fallback (fail-safe).
    MonitorVeto,
    /// Too few healthy channels to vote at all.
    InsufficientChannels,
}

/// Result of reducing `N` channel reports to one command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VotedResult<T: Votable> {
    /// The voted value (or the safe neutral value on `Disagreement`/`MonitorVeto`).
    pub value: T,
    /// Why this value was produced.
    pub status: VoteStatus,
}

/// A strategy that reduces channel reports to a single [`VotedResult`].
pub trait Voter<T: Votable> {
    fn vote(&self, reports: &[ChannelReport<T>]) -> VotedResult<T>;
}

/// Maximum number of channels the voting helpers will consider. The actual
/// channel count is a const generic on [`RedundantComputer`]; this bound only
/// caps the scratch buffer used for disagreement accounting.
const MAX_CHANNELS: usize = 8;

/// Collect the healthy channel values into a fixed buffer, returning the count.
fn healthy_values<T: Votable>(reports: &[ChannelReport<T>]) -> ([T; MAX_CHANNELS], usize) {
    let mut out: [T; MAX_CHANNELS] = [T::neutral(); MAX_CHANNELS];
    let mut n = 0;
    for r in reports.iter() {
        if r.healthy && n < MAX_CHANNELS {
            out[n] = r.value;
            n += 1;
        }
    }
    (out, n)
}

/// Maximum pairwise disagreement among the first `n` values.
fn max_pairwise_disagreement<T: Votable>(vals: &[T], n: usize) -> f64 {
    let mut max = 0.0f64;
    for i in 0..n {
        for j in (i + 1)..n {
            let d = vals[i].disagreement(&vals[j]);
            if d > max {
                max = d;
            }
        }
    }
    max
}

/// Index of the most-central value (smallest summed disagreement to the rest).
/// Equals the median for scalar triples and the geometric center for vectors.
fn most_central_index<T: Votable>(vals: &[T], n: usize) -> usize {
    let mut best = 0usize;
    let mut best_score = f64::MAX;
    for i in 0..n {
        let mut score = 0.0f64;
        for j in 0..n {
            if i != j {
                score += vals[i].disagreement(&vals[j]);
            }
        }
        if score < best_score {
            best_score = score;
            best = i;
        }
    }
    best
}

/// Equal-weighted mean of the first `n` values.
///
/// Uses a running weight `1/(k+1)` for the `k`-th (0-indexed) value so the
/// recurrence `acc = acc·(1-w) + v·w` yields the true arithmetic mean rather
/// than a geometric decay toward the last sample.
fn average<T: Votable>(vals: &[T], n: usize) -> T {
    if n == 0 {
        return T::neutral();
    }
    let mut acc = T::neutral();
    for (k, v) in vals.iter().take(n).enumerate() {
        acc = acc.combine(v, 1.0 / (k as f64 + 1.0));
    }
    acc
}

/// Mid-value select (`spec.txt` §4.3 "Mid-value select" for the triple channel).
///
/// Picks the most-central healthy channel. For three scalar channels this is
/// exactly the median, so a single faulted/runaway channel is rejected. Requires
/// at least `min_healthy` healthy channels (default 3) or it reports
/// [`VoteStatus::InsufficientChannels`].
#[derive(Debug, Clone, Copy)]
pub struct MidValueSelect {
    /// Minimum healthy channels required to emit a value.
    pub min_healthy: usize,
}

impl Default for MidValueSelect {
    fn default() -> Self {
        Self { min_healthy: 3 }
    }
}

impl MidValueSelect {
    pub const fn new() -> Self {
        Self { min_healthy: 3 }
    }
    pub const fn with_min(mut self, min_healthy: usize) -> Self {
        self.min_healthy = min_healthy;
        self
    }
}

impl<T: Votable> Voter<T> for MidValueSelect {
    fn vote(&self, reports: &[ChannelReport<T>]) -> VotedResult<T> {
        let (buf, n) = healthy_values(reports);
        if n < self.min_healthy {
            return VotedResult {
                value: T::neutral(),
                status: VoteStatus::InsufficientChannels,
            };
        }
        let idx = most_central_index(&buf, n);
        VotedResult {
            value: buf[idx],
            status: VoteStatus::Agreement,
        }
    }
}

/// Consensus voting for triple/quad dissimilar channels (`spec.txt` §4.3
/// "Consensus + dissimilar monitor").
///
/// If at least `min_healthy` channels agree within `tolerance` (max pairwise
/// disagreement), the averaged value is returned with [`VoteStatus::Agreement`].
/// Otherwise the safe neutral value is returned with [`VoteStatus::Disagreement`].
#[derive(Debug, Clone, Copy)]
pub struct Consensus {
    /// Minimum healthy channels required to form a consensus.
    pub min_healthy: usize,
    /// Max allowed pairwise disagreement (in the [`Votable`] unit) for consensus.
    pub tolerance: f64,
}

impl Default for Consensus {
    fn default() -> Self {
        Self {
            min_healthy: 2,
            tolerance: 0.1,
        }
    }
}

impl Consensus {
    pub const fn new() -> Self {
        Self {
            min_healthy: 2,
            tolerance: 0.1,
        }
    }
    pub const fn with(mut self, min_healthy: usize, tolerance: f64) -> Self {
        self.min_healthy = min_healthy;
        self.tolerance = tolerance;
        self
    }
}

impl<T: Votable> Voter<T> for Consensus {
    fn vote(&self, reports: &[ChannelReport<T>]) -> VotedResult<T> {
        let (buf, n) = healthy_values(reports);
        if n < self.min_healthy {
            return VotedResult {
                value: T::neutral(),
                status: VoteStatus::InsufficientChannels,
            };
        }
        if max_pairwise_disagreement(&buf, n) <= self.tolerance {
            VotedResult {
                value: average(&buf, n),
                status: VoteStatus::Agreement,
            }
        } else {
            VotedResult {
                value: T::neutral(),
                status: VoteStatus::Disagreement,
            }
        }
    }
}

/// Dissimilar-monitor voting: one designated channel is a *monitor* that
/// cross-checks the active lanes (`spec.txt` §4.3 "dissimilar monitor").
///
/// The monitor is assumed to be implemented by a *dissimilar* means (different
/// algorithm / toolchain) so that a common-mode fault in the active lanes is
/// unlikely to also affect it. If the monitor is healthy and disagrees with the
/// active-lane consensus beyond `tolerance`, it issues a
/// [`VoteStatus::MonitorVeto`] and the safe neutral value is emitted
/// (fail-safe). Otherwise the active lanes are reduced by mid-value select.
#[derive(Debug, Clone, Copy)]
pub struct MonitorVoter {
    /// Index (into the channel array) of the dissimilar monitor channel.
    pub monitor_index: usize,
    /// Max allowed disagreement (in the [`Votable`] unit) before a veto.
    pub tolerance: f64,
}

impl Default for MonitorVoter {
    fn default() -> Self {
        Self {
            monitor_index: 0,
            tolerance: 0.1,
        }
    }
}

impl MonitorVoter {
    pub const fn new(monitor_index: usize) -> Self {
        Self {
            monitor_index,
            tolerance: 0.1,
        }
    }
    pub const fn with_tolerance(mut self, tolerance: f64) -> Self {
        self.tolerance = tolerance;
        self
    }
}

impl<T: Votable> Voter<T> for MonitorVoter {
    fn vote(&self, reports: &[ChannelReport<T>]) -> VotedResult<T> {
        let (buf, n) = healthy_values(reports);
        if n < 2 {
            return VotedResult {
                value: T::neutral(),
                status: VoteStatus::InsufficientChannels,
            };
        }

        // Separate the monitor from the active lanes.
        let monitor = reports
            .get(self.monitor_index)
            .filter(|r| r.healthy)
            .map(|r| r.value);

        let active: [T; MAX_CHANNELS] = {
            let mut out = [T::neutral(); MAX_CHANNELS];
            let mut k = 0;
            for (i, r) in reports.iter().enumerate() {
                if r.healthy && i != self.monitor_index && k < MAX_CHANNELS {
                    out[k] = r.value;
                    k += 1;
                }
            }
            out
        };
        let active_n = if reports.get(self.monitor_index).map(|r| r.healthy) == Some(true) {
            n - 1
        } else {
            n
        };

        if active_n == 0 {
            // Only the monitor is healthy: not enough active lanes to vote.
            return VotedResult {
                value: T::neutral(),
                status: VoteStatus::InsufficientChannels,
            };
        }

        let active_consensus = average(&active, active_n);
        let active_ok = max_pairwise_disagreement(&active, active_n) <= self.tolerance;

        match monitor {
            Some(m) if !active_ok => {
                // Active lanes disagree among themselves; if the monitor also
                // disagrees with them, we cannot trust anything -> veto.
                if m.disagreement(&active_consensus) > self.tolerance {
                    VotedResult {
                        value: T::neutral(),
                        status: VoteStatus::MonitorVeto,
                    }
                } else {
                    VotedResult {
                        value: active_consensus,
                        status: VoteStatus::Disagreement,
                    }
                }
            }
            Some(m) => {
                // Active lanes agree; the monitor must corroborate them.
                if m.disagreement(&active_consensus) > self.tolerance {
                    VotedResult {
                        value: T::neutral(),
                        status: VoteStatus::MonitorVeto,
                    }
                } else {
                    VotedResult {
                        value: active_consensus,
                        status: VoteStatus::Agreement,
                    }
                }
            }
            None => {
                // Monitor unavailable: fall back to mid-value of all healthy.
                let idx = most_central_index(&buf, n);
                let status = if active_ok {
                    VoteStatus::Agreement
                } else {
                    VoteStatus::Disagreement
                };
                VotedResult {
                    value: buf[idx],
                    status,
                }
            }
        }
    }
}

/// Aggregates `N` redundant channel reports and produces a voted command.
///
/// `N` is the physical channel count (3 for triple, 4 for quad). Channels are
/// identified by index `0..N`.
#[derive(Debug, Clone)]
pub struct RedundantComputer<const N: usize, T: Votable> {
    reports: [ChannelReport<T>; N],
}

impl<const N: usize, T: Votable> RedundantComputer<N, T> {
    /// All channels start uninitialized and reported unhealthy.
    pub fn new() -> Self {
        Self {
            reports: core::array::from_fn(|_| ChannelReport::new(T::neutral(), false)),
        }
    }

    /// Submit one channel's report. Out-of-range channels are ignored.
    pub fn submit(&mut self, ch: usize, value: T, healthy: bool) {
        if let Some(slot) = self.reports.get_mut(ch) {
            *slot = ChannelReport::new(value, healthy);
        }
    }

    /// Number of channels currently reporting healthy.
    pub fn healthy_count(&self) -> usize {
        self.reports.iter().filter(|r| r.healthy).count()
    }

    /// Run the given [`Voter`] over the current channel set.
    pub fn vote<V: Voter<T>>(&self, voter: &V) -> VotedResult<T> {
        voter.vote(&self.reports)
    }

    /// Convenience: mid-value select requiring at least `min_healthy` channels.
    pub fn mid_value(&self, min_healthy: usize) -> VotedResult<T> {
        MidValueSelect::new()
            .with_min(min_healthy)
            .vote(&self.reports)
    }
}

impl<const N: usize, T: Votable> Default for RedundantComputer<N, T> {
    fn default() -> Self {
        Self::new()
    }
}

/// Classification of a detected fault after scrubbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultClass {
    /// No fault observed on this channel.
    None,
    /// Transient upset (e.g. lightning indirect-effects / HIRF-induced SEU or
    /// glitch) that has been observed but has *not* persisted long enough to be
    /// declared permanent. The channel is still trusted and the upset is
    /// expected to self-clear (DO-160 §16.3 environmental transients).
    Transient,
    /// Persistent fault that has survived the scrub window — treated as a
    /// permanent hardware failure requiring channel removal / reconfiguration.
    Permanent,
}

/// Per-channel fault-persistence monitor (`spec.txt` §16.3, DO-160
/// environmental qualification).
///
/// Distinguishes **transient** environmental upsets (induced by lightning
/// indirect-effects or HIRF) from **permanent** faults. A transient upset is an
/// instantaneous deviation that self-clears; a permanent fault persists across
/// the configured scrub window.
///
/// The monitor is allocation-free and `no_std`. On every [`Self::update`] it
/// scrubs the channel:
/// - when faulted, a `persist_time` timer accumulates and a `strike` count
///   increments;
/// - when healthy, both decay toward zero with `recovery_rate`;
/// - a fault is classified [`FaultClass::Permanent`] only once `persist_time`
///   reaches `persist_threshold` **or** the total strike count exceeds
///   `max_transient_strikes` without clearing. A permanent fault latches until
///   [`Self::reset`].
#[derive(Debug, Clone, Copy)]
pub struct FaultMonitor {
    /// Accumulated fault time (s) since the last full recovery.
    persist_time: f64,
    /// Running strike count (decays when healthy).
    strikes: u32,
    /// Time (s) of accumulated fault before a fault is declared permanent.
    persist_threshold: f64,
    /// Max strikes before a fault is declared permanent.
    max_transient_strikes: u32,
    /// Fractional recovery per second of healthy operation.
    recovery_rate: f64,
    /// Latched permanent-fault flag.
    latched: bool,
    /// Last reported class.
    class: FaultClass,
}

impl FaultMonitor {
    /// Build with DO-160-style defaults (100 ms scrub window, fast recovery).
    pub const fn new() -> Self {
        Self {
            persist_time: 0.0,
            strikes: 0,
            persist_threshold: 0.1,
            max_transient_strikes: 16,
            recovery_rate: 2.0,
            latched: false,
            class: FaultClass::None,
        }
    }

    /// Configure the scrub window / sensitivity.
    pub const fn with_params(
        mut self,
        persist_threshold: f64,
        max_transient_strikes: u32,
        recovery_rate: f64,
    ) -> Self {
        self.persist_threshold = persist_threshold;
        self.max_transient_strikes = max_transient_strikes;
        self.recovery_rate = recovery_rate;
        self
    }

    /// Scrub one channel sample. `is_faulted` is the raw per-sample health of
    /// the channel; `dt` is the elapsed time since the previous sample. Returns
    /// the current [`FaultClass`].
    pub fn update(&mut self, is_faulted: bool, dt: f64) -> FaultClass {
        if self.latched {
            return FaultClass::Permanent;
        }
        if dt <= 0.0 {
            return self.class;
        }
        if is_faulted {
            self.persist_time += dt;
            self.strikes = self.strikes.saturating_add(1);
            if self.persist_time >= self.persist_threshold
                || self.strikes > self.max_transient_strikes
            {
                self.latched = true;
                self.class = FaultClass::Permanent;
            } else {
                self.class = FaultClass::Transient;
            }
        } else {
            self.persist_time = (self.persist_time - dt * self.recovery_rate).max(0.0);
            if self.strikes > 0 {
                self.strikes -= 1;
            }
            if self.persist_time == 0.0 && self.strikes == 0 {
                self.class = FaultClass::None;
            } else {
                self.class = FaultClass::Transient;
            }
        }
        self.class
    }

    /// Whether the channel is currently classified as a permanent fault.
    pub const fn is_permanent(&self) -> bool {
        self.latched
    }

    /// Whether the channel has shown a (possibly transient) fault recently.
    pub const fn is_transient(&self) -> bool {
        matches!(self.class, FaultClass::Transient)
    }

    /// Current classification.
    pub const fn class(&self) -> FaultClass {
        self.class
    }

    /// Accumulated (decaying) fault time — useful for telemetry / trending.
    pub const fn persist_time(&self) -> f64 {
        self.persist_time
    }

    /// Total strikes observed since the last [`Self::reset`].
    pub const fn strikes(&self) -> u32 {
        self.strikes
    }

    /// Clear all counters and the latched permanent-fault state (e.g. after a
    /// scheduled power-cycle / built-in test pass).
    pub fn reset(&mut self) {
        self.persist_time = 0.0;
        self.strikes = 0;
        self.latched = false;
        self.class = FaultClass::None;
    }
}

impl Default for FaultMonitor {
    fn default() -> Self {
        Self::new()
    }
}

/// Scrub a whole redundant channel set, classifying each channel and returning
/// the indices that should be **dropped** from voting (permanent faults).
///
/// `health[i]` is the raw per-sample health of channel `i`. Indices that are
/// permanent per the supplied [`FaultMonitor`]s are returned in `dropped` (up to
/// `dropped.len()`), and the count written is returned.
pub fn scrub_channels(
    monitors: &mut [FaultMonitor],
    health: &[bool],
    dt: f64,
    dropped: &mut [usize],
) -> usize {
    let mut n = 0usize;
    let len = monitors.len().min(health.len());
    for i in 0..len {
        let cls = monitors[i].update(!health[i], dt);
        if cls == FaultClass::Permanent && n < dropped.len() {
            dropped[n] = i;
            n += 1;
        }
    }
    n
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mid_value_rejects_single_rogue_channel() {
        // Triple channel: two agree at 1.0, one runs away to 99.0.
        let reports = [
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(99.0f64, true),
        ];
        let r = MidValueSelect::new().vote(&reports);
        assert_eq!(r.status, VoteStatus::Agreement);
        assert!((r.value - 1.0).abs() < 1e-9, "got {}", r.value);
    }

    #[test]
    fn mid_value_insufficient_channels() {
        let reports = [
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(2.0f64, false),
            ChannelReport::new(3.0f64, false),
        ];
        let r = MidValueSelect::new().vote(&reports);
        assert_eq!(r.status, VoteStatus::InsufficientChannels);
    }

    #[test]
    fn consensus_agrees_when_close() {
        let reports = [
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(1.05f64, true),
            ChannelReport::new(0.98f64, true),
        ];
        let r = Consensus::new().with(2, 0.1).vote(&reports);
        assert_eq!(r.status, VoteStatus::Agreement);
        assert!(r.value > 0.95 && r.value < 1.1);
    }

    #[test]
    fn consensus_failsafe_on_disagreement() {
        let reports = [
            ChannelReport::new(0.0f64, true),
            ChannelReport::new(5.0f64, true),
        ];
        let r = Consensus::new().with(2, 0.1).vote(&reports);
        assert_eq!(r.status, VoteStatus::Disagreement);
        assert_eq!(r.value, 0.0);
    }

    #[test]
    fn monitor_vetoes_active_lanes() {
        // Monitor (channel 0) disagrees with both active lanes.
        let reports = [
            ChannelReport::new(9.0f64, true), // monitor
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(1.0f64, true),
        ];
        let r = MonitorVoter::new(0).with_tolerance(0.1).vote(&reports);
        assert_eq!(r.status, VoteStatus::MonitorVeto);
    }

    #[test]
    fn monitor_corroborates_active_consensus() {
        let reports = [
            ChannelReport::new(1.0f64, true), // monitor
            ChannelReport::new(1.0f64, true),
            ChannelReport::new(1.0f64, true),
        ];
        let r = MonitorVoter::new(0).with_tolerance(0.1).vote(&reports);
        assert_eq!(r.status, VoteStatus::Agreement);
        assert!((r.value - 1.0).abs() < 1e-9);
    }

    #[test]
    fn redundant_computer_quad_mid_value() {
        let mut comp: RedundantComputer<4, f64> = RedundantComputer::new();
        comp.submit(0, 1.0, true);
        comp.submit(1, 1.0, true);
        comp.submit(2, 1.0, true);
        comp.submit(3, 99.0, true); // single fault
        let r = comp.mid_value(3);
        assert_eq!(r.status, VoteStatus::Agreement);
        assert!((r.value - 1.0).abs() < 1e-9);
    }

    #[test]
    fn vector_mid_value_rejects_rogue() {
        let good = Vector3::new(0.0, 0.0, 1.0);
        let rogue = Vector3::new(100.0, 100.0, 100.0);
        let reports = [
            ChannelReport::new(good, true),
            ChannelReport::new(good, true),
            ChannelReport::new(rogue, true),
        ];
        let r = MidValueSelect::new().vote(&reports);
        assert_eq!(r.status, VoteStatus::Agreement);
        assert_eq!(r.value, good);
    }

    #[test]
    fn single_lightning_spike_is_transient_not_permanent() {
        let mut m = FaultMonitor::new();
        // One 1 ms upset is classified Transient; it must NOT latch permanent,
        // and a few healthy samples scrub it back to None.
        assert_eq!(m.update(true, 0.001), FaultClass::Transient);
        assert!(!m.is_permanent());
        for _ in 0..5 {
            m.update(false, 0.001);
        }
        assert_eq!(m.class(), FaultClass::None);
        assert!(!m.is_permanent());
    }

    #[test]
    fn sustained_fault_latches_permanent() {
        let mut m = FaultMonitor::new();
        // A continuous fault accumulates persist_time past the 100 ms scrub
        // window and eventually latches Permanent (not on the very first sample).
        let mut saw_permanent = false;
        for _ in 0..200 {
            if m.update(true, 0.001) == FaultClass::Permanent {
                saw_permanent = true;
            }
        }
        assert!(saw_permanent);
        assert!(m.is_permanent());
        // Even after going healthy, a permanent fault stays latched.
        assert_eq!(m.update(false, 0.001), FaultClass::Permanent);
    }

    #[test]
    fn repeated_hirf_bursts_stay_transient() {
        let mut m = FaultMonitor::new().with_params(0.05, 16, 0.5);
        // 1 ms lightning/HIRF upsets separated by 50 ms healthy gaps. The decay
        // (0.5/s * 0.05 = 0.025 s per gap) far outpaces the 0.001 s burst, so
        // each upset is classified Transient and then fully scrubbed back to
        // None; the channel never reaches the 0.05 s persist threshold and so
        // never latches Permanent.
        for _ in 0..20 {
            assert_eq!(m.update(true, 0.001), FaultClass::Transient);
            assert_eq!(m.update(false, 0.05), FaultClass::None);
        }
        assert!(!m.is_permanent());
    }

    #[test]
    fn scrub_channels_drops_permanent_lanes() {
        let mut monitors = [FaultMonitor::new(); 3];
        let mut dropped = [0usize; 3];
        // Channel 1 is permanently faulted; others healthy.
        let health = [true, false, true];
        // Run long enough for channel 1 to latch.
        let mut n = 0;
        for _ in 0..200 {
            n = scrub_channels(&mut monitors, &health, 0.001, &mut dropped);
        }
        assert_eq!(n, 1);
        assert_eq!(dropped[0], 1);
    }

    #[test]
    fn reset_clears_latched_fault() {
        let mut m = FaultMonitor::new();
        for _ in 0..200 {
            m.update(true, 0.001);
        }
        assert!(m.is_permanent());
        m.reset();
        assert!(!m.is_permanent());
        assert_eq!(m.class(), FaultClass::None);
    }
}
