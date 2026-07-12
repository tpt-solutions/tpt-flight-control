# Path to Type Certification — What This Repository Can and Cannot Provide

> Status: **Reference document, not certification credit.** This answers
> one recurring question directly: *has every legal requirement been
> accounted for to use TPT on a transport-category aircraft (e.g. a
> Boeing 747-class installation)?* **No.** This document is the durable,
> specific answer to that question — what exists today, and what
> structurally cannot exist inside a code repository, no matter how
> thorough the documentation gets.

## What this repository provides

- **DO-178C software life-cycle data**: requirements traceability
  (`traceability/matrix.md`), a Software Accomplishment Summary
  (`software-accomplishment-summary.md`), a Plan for Software Aspects of
  Certification (`plan-for-software-aspects-of-certification.md`), a
  Software Verification Plan (`software-verification-plan.md`), a Software
  Configuration Index (`software-configuration-index.md`), and a DO-330
  tool-qualification rationale for the CI pipeline (`ci-qualification.md`).
- **ARP 4754A / ARP 4761 system-safety-assessment scaffold**
  (`system-safety-assessment.md`): FHA/PSSA/SSA/FTA/FMEA/CCA activities
  mapped to implemented architectural mitigations (envelope protection,
  dissimilar redundancy, dissimilar navigation, formal bounds).
- **DO-326A/ED-202A airworthiness-security mapping**
  (`airworthiness-security.md`): anti-spoofing, map-integrity, and
  authenticated-encryption mitigations mapped to the security process.
- **DO-160 environmental-qualification mapping**
  (`environmental-qualification.md`): fault-persistence scrubbing and
  brownout handling mapped to relevant DO-160 sections, simulated in SITL.
- **Licensing clarity**: Apache-2.0 for all source and reference hardware
  (`spec.txt` §20), DCO-signed contribution history, supply-chain
  auditing (`cargo-deny`, SBOM generation).

Every one of the documents above says, in its own status banner, that it is
**not** a certification credit by itself. That repetition is deliberate —
it is the single most important fact about this directory.

## What no repository can provide

| Requirement | Why code can't satisfy it | Who/what actually does |
|---|---|---|
| **Type Certificate (TC) or Supplemental Type Certificate (STC) integration** | A TC/STC is an approval of a specific aircraft configuration issued to an *applicant organization*, not a piece of software. TPT is a component supplier's input, not an airframe. | The airframe OEM (or an STC applicant) files with the certification authority and carries the TC/STC. |
| **Organization Designation Authorization (ODA) or Designated Engineering Representative (DER) relationship** | Software cannot hold a delegation of authority from the FAA/EASA; this is an organizational credential. | A company must apply for and be granted ODA/DER status, or contract with one that already holds it. |
| **Stage of Involvement (SOI) audits** | SOI audits are the certification authority's on-site/document review of the *applicant's* process execution at defined life-cycle milestones (SOI #1 planning, #2 development, #3 verification, #4 final). No audit can occur without an applicant and an authority engagement to audit. | Authority (FAA/EASA) reviewers, scheduled through the applicant's certification liaison. |
| **Physical DO-160 test-lab qualification** | Requires an accredited lab, real instrumented hardware, and the specific test waveforms/procedures in the standard (vibration tables, EMC chambers, lightning simulators). `environmental-qualification.md`'s SITL coverage simulates the *software response* to these events, not the physical stress itself. | An accredited DO-160 test laboratory, once real target hardware exists. |
| **DO-254 hardware verification** | Same category as DO-160 — physical hardware design assurance. Currently moot: no custom hardware exists yet (`hardware-assurance.md`). | A hardware design assurance process once custom hardware is built. |
| **ARP 4761 FTA/FMEA/CCA closure** | The scaffold in `system-safety-assessment.md` identifies *which* mitigations address *which* top events, but closing the analysis to the DAL-A 10⁻⁹/flight-hour target requires quantitative failure-rate data for real components (sensors, actuators, wiring) that don't exist as a bill of materials yet. | A Design Assurance organization performing the full PSSA/SSA. |
| **Human factors compliance (14 CFR 25.1302, 25.1322)** | Governs flight-deck information design, alerting, and crew workload — a systems-engineering and human-factors discipline evaluated against the actual cockpit integration, not the flight-control software in isolation. | Not addressed anywhere in this project; would require a flight-deck integration study specific to the host airframe. |
| **Instructions for Continued Airworthiness (ICA)** | ICA documents maintenance, inspection intervals, and airworthiness limitations for the *installed, certified* product — it doesn't exist until the product does. | Authored post-certification by the type-certificate holder. |
| **Production Certificate / conformity inspection** | Confirms manufactured units conform to the certified design — a manufacturing-quality-system activity, unrelated to source code. | A production approval holder and the authority's conformity inspectors. |
| **Flight test program** | DAL-A software behavior must be validated against the real aircraft in flight, including failure-injection flight test for the redundancy/envelope-protection claims made in `system-safety-assessment.md`. | A flight test organization with a certificated aircraft and test pilots. |

## Bottom line

This repository can shorten the software-evidence portion of a real
certification program — traceability, planning documents, and an honest
gap list all exist and are kept in sync with the code. It cannot, by
itself, get TPT onto a 747 or any other transport-category aircraft. That
requires an applicant organization with an actual FAA/EASA authority
relationship (ODA/DER), access to physical test labs, a Design Assurance
organization to close the safety analysis, and a flight test program — none
of which a software repository can create by writing more markdown, no
matter how complete the documentation becomes.

If and when a real certification engagement begins (`todo.md` Phase 3–5),
every document in `certification/` becomes the starting evidence package
for that engagement, not a substitute for it.
