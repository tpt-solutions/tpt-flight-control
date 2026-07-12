# Architecture: Triple / Quad-Redundant Dissimilar Voting

`spec.txt` §4.3 (Phase 5) — implemented in `tpt-core::redundancy` (behind the
`triple-redundancy` feature).

## Why

For the `tpt-regional` / `tpt-transport` profiles, TPT must not depend on a
single lane. A single faulted channel (runaway integrator, stuck sensor,
common-mode software bug) must not command the vehicle. The design uses three
or four **dissimilar** channels — each an independent implementation of the
control / navigation function (different algorithms, and ideally different
toolchains) — and a cross-channel monitor that compares their outputs.

## Components

- `Votable` — a value type that can report a disagreement magnitude and be
  combined/averaged. Implemented for `f64` and `Vector3<f64>`.
- `ChannelReport<T>` — one channel's value plus its own self-reported health
  (Built-In Test).
- `Voter<T>` — a strategy that reduces `N` reports to a single `VotedResult`:
  - `MidValueSelect` — picks the most-central (median for scalar triples)
    channel; rejects a single rogue channel. Requires `min_healthy` healthy
    channels (default 3).
  - `Consensus` — requires a quorum (default 2) within `tolerance`; otherwise
    fails safe to the neutral value with `VoteStatus::Disagreement`.
  - `MonitorVoter` — a designated **dissimilar monitor** channel cross-checks
    the active lanes; if it disagrees it issues `VoteStatus::MonitorVeto` and
    the safe neutral value is emitted (fail-safe).
- `RedundantComputer<const N, T>` — aggregates `N` channels and runs any
  `Voter`. `N` is the physical channel count (3 = triple, 4 = quad).

## Safety properties (tested)

- A single faulted channel in a triple set is rejected by `MidValueSelect`
  (mid-value property).
- `Consensus` emits the neutral value when channels disagree beyond tolerance.
- `MonitorVoter` vetoes when the dissimilar monitor contradicts the active
  consensus.
- `InsufficientChannels` is reported when too few channels are healthy to vote.

## Integration point

The voted command is the single value that leaves the redundant computer and
is fed to the (non-bypassable) `EnvelopeProtector` before the mixer — i.e. the
voting layer sits *above* envelope protection in the control chain.
