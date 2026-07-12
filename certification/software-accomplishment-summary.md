# Software Accomplishment Summary (SAS)

DO-178C Annex A / Schedule / Software Accomplishment Summary objective.

This document is the top-level, auditor-facing summary of how the TPT Flight
Control software accomplishment maps to the DO-178C objectives. It is generated
from — and must stay in sync with — the [Requirements Traceability
Matrix](traceability/matrix.md). It is **not** a certification credit by itself;
it records current status and the evidence each objective draws on.

> Status values: **Complete** — objective satisfied by existing evidence;
> **Partial** — objective started, evidence incomplete; **Not started** —
> no evidence yet.

## A. Software Planning Process (§§4.1, 11.1)

| Objective | Evidence | Status |
|---|---|---|
| Plan for software aspects of certification (PSAC) | [`plan-for-software-aspects-of-certification.md`](plan-for-software-aspects-of-certification.md) | Partial |
| Software development plan (SDP) | `spec.txt` (v0.2.0-DRAFT), `todo.md` milestone tracking | Partial |
| Software verification plan (SVP) | [`software-verification-plan.md`](software-verification-plan.md) | Partial |
| Software configuration management plan (SCMP) | `git`, `.github/workflows/ci.yml`, `deny.toml`, `scripts/` | Partial |
| Software quality assurance plan (SQAP) | CI gates (`-D warnings`, clippy, audit, SBOM) | Partial |

## B. Software Development Process (§§4.2, 11.2)

| Objective | Evidence | Status |
|---|---|---|
| Low-level requirements (LLR) derived from HL requirements | `tpt-abstractions` traits, `tpt-core` module docs citing `spec.txt` §refs | Partial |
| Software architecture | `tpt-abstractions` trait boundary; backend/sim/core layering | Partial |
| Source code | entire `src/` tree, `no_std` + `forbid(unsafe_code)` | Complete |
| Executable object code | `cargo build` (host + `thumbv7em`/`riscv32` `no_std`) | Complete |

## C. Software Verification Process (§§4.3, 11.3)

| Objective | Evidence | Status |
|---|---|---|
| Verify LLR satisfy HL requirements | `traceability/matrix.md` (every `REQ-*` row cites spec §) | Partial |
| Verify architecture satisfies LLR | trait contracts; backend impls per crate | Partial |
| Verify source code satisfies LLR | `cargo test --workspace` (all crates green) | Partial |
| Verify outputs of integration process | cross-crate `tpt-sim` SITL scenarios | Partial |
| Verify traceability | `traceability/matrix.md` | Partial |
| Verify test coverage (structural) | unit tests per requirement (see matrix "Verified" counts) | Partial |

## D. Appendices / Life-Cycle Data

| Objective | Evidence | Status |
|---|---|---|
| Appendices A/B objectives per DAL | this SAS + matrix | Partial |
| Software configuration index (SCI) | [`software-configuration-index.md`](software-configuration-index.md) | Partial |
| Software accomplishment summary (this) | this document | Complete |

## DAL Targets & Gaps

- **`tpt-drone` / `tpt-micro` (DAL-C/D):** control, estimation, mixing, mapping,
  and MAVLink/TPT-Link protocols are implemented and unit/SITL-verified
  (matrix shows the bulk of `Verified` rows). No formal DAL credit yet.
- **`tpt-evtol` (DAL-C/B):** DEP/tilt-rotor mixers, PikeOS/seL4 partitions, and
  sovereign toolchain wrapper are implemented and unit-tested; certification
  engagement (§Phase 4) is **Not started**.
- **`tpt-transport` (DAL-A):** ARINC 429/AFDX (Phase 5) and VxWorks backend are
  implemented; ARP 4754A/4761 safety assessment and DAL-A certification are
  **Not started**.

## Related certification packages

This SAS covers DO-178C only. Adjacent standards have their own scaffold
documents in this directory: [environmental-qualification.md](environmental-qualification.md)
(DO-160), [airworthiness-security.md](airworthiness-security.md)
(DO-326A/ED-202A), and [hardware-assurance.md](hardware-assurance.md)
(DO-254, not currently applicable). For what none of these documents —
individually or together — can substitute for, see
[path-to-type-certification.md](path-to-type-certification.md).

## Open Items (tracked in `todo.md`)

- Flash bare-metal backend to real hardware and achieve stable hover (REQ-18-1).
- Get a green `kani-compiler` run for the `tpt-math`/`tpt-mapping` proof
  harnesses authored under REQ-M-7/REQ-M-8 (§Phase 4); the `kani` CI job is
  `continue-on-error: true` until that first run is confirmed.
- Implement `VisualSensor`/`LidarSensor`/`RadarAltimeter`/`ControlSurface`
  backend implementers (currently `Gap` in the matrix).
- Begin DO-178C DAL-C and DAL-A certification engagements with partners.
