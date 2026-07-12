# Environmental Qualification (DO-160)

> Status: **Draft scaffold.** This document maps the environmental-hardening
> code already implemented to the DO-160 sections it is relevant to. It is
> **not** certification credit — DO-160 compliance requires accredited
> physical test-lab qualification of real hardware, which no software repo
> can produce. See `todo.md` (Resilience & Autonomy Roadmap, "Add DO-160
> environmental-qualification coverage").

## Standard in brief

DO-160 *Environmental Conditions and Test Procedures for Airborne
Equipment* defines test categories and pass/fail criteria for physical
hardware. The sections relevant to TPT's current scope:

- **§16 Power Input** — voltage sag/surge, brownout, transient response.
- **§20/21 RF Susceptibility / Emissions (EMI)** — equipment must tolerate
  ambient RF energy without upset.
- **§22 Lightning Induced Transient Susceptibility** — indirect-effects
  transients coupled onto wiring by nearby lightning strikes.
- **§23 Lightning Direct Effects** — for equipment in a direct-strike zone.
- **§4/§5/§6 Temperature/Altitude/Decompression, Vibration** — physical
  environmental stress, not addressed by this document (no code artifact
  applies).

## TPT artifact mapping

| DO-160 section | Failure mode | TPT artifact |
|---|---|---|
| §16 Power Input | Bus undervoltage / brownout | `tpt_abstractions::os::PowerSystem::brownout_active()` — reports bus sag so the vehicle sheds non-essential loads and degrades gracefully instead of resetting. |
| §20/21 EMI/RF Susceptibility, §22 Lightning Indirect Effects | Transient single-event upset (SEU) or glitch on a redundant channel | `tpt-core::redundancy::FaultMonitor` / `FaultClass` — classifies a channel fault as `Transient` (self-clears within the scrub window) or `Permanent` (persists, treated as a real hardware failure) via `scrub_channels`, so a transient upset does not trigger unnecessary channel removal from the voting set. |
| §16, §20/21, §22 (simulated) | End-to-end scenario coverage | `tpt-sim::environment::EnvScenario::PowerTransient` and `EnvScenario::EmiUpset` drive `EnvironmentSim`/`FaultMonitor` through both scenarios in SITL; both pass. |

Spec reference: `spec.txt` §16.3 ("DO-160 Environmental Qualification").

## What this is — and isn't

This is **software fault-injection simulation**: `tpt-sim` synthesizes the
*symptom* of a power transient or an EMI-induced upset (a glitched reading,
a sagging bus voltage) and verifies `FaultMonitor`/`PowerSystem` respond
correctly. It does not, and cannot, substitute for:

- An accredited DO-160 test lab exposing physical hardware to real
  transients, RF fields, or lightning-simulated waveforms per the standard's
  test procedures and pass/fail thresholds.
- Temperature/altitude/vibration qualification (§4/§5/§6) — there is
  currently no code or hardware artifact addressing these categories at
  all, since they are purely physical stress tests with no software
  analog.

## Open items before qualification credit

- [ ] Physical DO-160 test-lab qualification per applicable category, once
      real hardware exists (see `certification/hardware-assurance.md`).
- [ ] Temperature/altitude/vibration (§4/§5/§6) test plan — no current
      software or hardware artifact.
- [ ] Cross-reference the `FaultMonitor` scrub-window thresholds
      (`persist_threshold`, `max_transient_strikes`) against actual lab
      transient-response data once available, rather than the current
      DO-160-*style* defaults chosen without lab validation.
