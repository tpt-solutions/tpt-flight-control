# Plan for Software Aspects of Certification (PSAC)

> Status: **Draft scaffold.** This is the top-level plan document a real
> PSAC would expand into; it is **not** certification credit — a PSAC must
> be submitted to and accepted by the certification authority before the
> project it governs can earn DO-178C credit for the plans it describes.

## Purpose

The PSAC states, up front, *how* a software project intends to satisfy
DO-178C: the certification basis, the standards applied, the life-cycle
processes used, and the life-cycle data that will be produced as evidence.
Everything else in `certification/` is downstream of this document.

## Certification basis

| Vehicle profile | Target DAL | `certification::Dal` |
|---|---|---|
| `tpt-drone` / `tpt-micro` | C/D | `Dal::C` / `Dal::D` |
| `tpt-uas` | C | `Dal::C` |
| `tpt-evtol` | C/B | `Dal::C` / `Dal::B` |
| `tpt-transport` | A | `Dal::A` |

`Dal::failure_bound_per_flight_hour()` encodes the DO-178C Table A-1
catastrophic-failure-rate target per level (1e-9 for DAL-A down to 1e-3 for
DAL-D).

## Applicable standards (`spec.txt` §16.1)

- **DO-178C** (with **DO-330** tool qualification, **DO-332** object-oriented
  technology, **DO-333** formal methods supplements) — software
  certification.
- **DO-326A / ED-202A** — airworthiness security
  (`certification/airworthiness-security.md`).
- **DO-160** — environmental qualification
  (`certification/environmental-qualification.md`).
- **DO-254** — hardware design assurance, not currently applicable
  (`certification/hardware-assurance.md`).
- **ARP 4754A / ARP 4761** — system safety assessment
  (`certification/system-safety-assessment.md`).

## Software life cycle overview

1. **Planning** — this document, plus the Software Verification Plan
   (`certification/software-verification-plan.md`) and the Software
   Configuration Index (`certification/software-configuration-index.md`).
2. **Development** — requirements captured as `spec.txt` sections, traced
   to implementation via `certification/traceability/matrix.md` (Req ID
   scheme documented in `certification/traceability/README.md`).
3. **Verification** — `cargo test --workspace`, SITL scenarios in
   `tpt-sim`, and formal proof harnesses (`cargo kani` for `tpt-math`/
   `tpt-mapping`), per the Software Verification Plan.
4. **Configuration management** — `git`, `.github/workflows/ci.yml`,
   `deny.toml`, per the Software Configuration Index.
5. **Quality assurance** — CI gates (`-D warnings`, clippy, `cargo-deny`
   audit, SBOM generation); tool qualification rationale in
   `certification/ci-qualification.md`.
6. **Accomplishment summary** — status rollup against DO-178C objectives in
   `certification/software-accomplishment-summary.md`.

## Open items before this is a real PSAC

- [ ] Submission to, and acceptance by, the certification authority — a
      PSAC is only a plan of record once an authority has agreed to it.
- [ ] Formal Design Assurance organization sign-off (see
      `certification/path-to-type-certification.md`).
- [ ] Per-profile transition-criteria detail (entry/exit criteria between
      life-cycle phases) — not yet elaborated beyond the overview above.
