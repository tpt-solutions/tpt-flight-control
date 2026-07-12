# System Safety Assessment (ARP 4754A / ARP 4761)

> Status: **Draft scaffold.** This document establishes the safety-assessment
> structure and traces the implemented architectural mitigations to the ARP
> 4754A / ARP 4761 activities. The *final* certification sign-off (PSSA/SSA
> closure, FHA, and the full failure-modes effects analysis) is performed by a
> designated Design Assurance organization and is **not** completable in code —
> see `todo.md` (Phase 5, "Complete ARP 4754A / ARP 4761 system safety
> assessment").

## Standards in brief

- **ARP 4754A** — Guidelines for Development of Civil Aircraft and Systems.
  Defines the *planning* process: Functional Hazard Assessment (FHA) →
  Preliminary System Safety Assessment (PSSA) → System Safety Assessment (SSA),
  and how DAL (Design Assurance Level) flows from severity to process rigor.
- **ARP 4761** — Guidelines and Methods for Conducting the Safety Assessment
  Process. Defines the *analysis* methods: FHA, PSSA, SSA, Fault Tree Analysis
  (FTA), Failure Modes and Effects Analysis (FMEA), and the common-cause /
  common-mode analysis (CCA / DFA / ZSA).

## TPT safety lifecycle mapping

| ARP activity | TPT artifact |
|---|---|
| Functional Hazard Assessment (FHA) | `spec.txt` §3 (Design Principles) + §16 (Certification) enumerate the hazard classes (GPS loss, envelope exceedance, channel fault). |
| Preliminary System Safety Assessment (PSSA) | `certification/traceability/matrix.md` (requirement → implementation → test); `docs/architecture-redundancy.md` and `docs/architecture-dissimilar-nav.md` justify the architectural mitigations. |
| System Safety Assessment (SSA) | `certification/software-accomplishment-summary.md` (DO-178C Annex A objective status) + the unit/HIL test suites referenced by the traceability matrix. |
| Fault Tree Analysis (FTA) | "Navigation loss" top event is bounded by the dissimilar-source cross-check (`tpt-sensor-fusion::dissimilar`) and the redundant voting layer (`tpt-core::redundancy`). |
| FMEA | Per-channel `ChannelReport::healthy` (BIT) reporting feeds `Voter` disagreement detection. |
| Common-Cause Analysis (CCA / DFA) | Dissimilar channels (different algorithms/toolchains) + a dissimilar **monitor** channel (`MonitorVoter`) specifically address common-mode faults. |

## Key safety properties already enforced in code

1. **Non-bypassable envelope protection** (`tpt-core::envelope`) — attitude,
   rate, climb, and Vne limits are enforced between the control laws and the
   mixer; there is no API to skip it.
2. **GPS-independent navigation** — the fusion state machine (§7.2) and the
   dissimilar-source monitor (§16.2) guarantee safe degradation (visual /
   terrain / coast) when GNSS is lost, jammed, or spoofed.
3. **Dissimilar redundancy** — triple/quad voting with a dissimilar monitor
   (`tpt-core::redundancy`) provides fault-tolerant command generation for
   `tpt-regional` / `tpt-transport`.
4. **Formal bounds** — Kani proof harnesses (`tpt-math`, `tpt-mapping`) bound
   the core math and mapping primitives (see `kani.yml`).

## Open items before certification sign-off

- [ ] Fully populate the FHA hazard list with severity / DAL assignments.
- [ ] Close the PSSA: show each hazard's mitigation satisfies its target DAL.
- [ ] Execute the FTA/CCA to the catastrophic-failure-rate target
      (< 1e-9 per flight hour for DAL-A).
- [ ] Independent review / approval by the Design Assurance organization.
