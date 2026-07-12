# Requirements Traceability — Methodology

[`matrix.md`](matrix.md) links `spec.txt` requirements to their
implementation and verifying tests. This is not a certification credit on
its own — DO-178C requires an authority-recognized process and independent
audit — but it is the foundational life-cycle data (Annex A objectives
A-3/A-7: requirements-to-code and requirements-to-test traceability) that
every later certification objective in `todo.md` Phase 4/5 depends on.

## Why `spec.txt` has no native requirement IDs

`spec.txt` is a design narrative, not a numbered-clause requirements
document — it has no "shall" statements or `REQ-*` tags. The matrix
extracts discrete, testable requirement statements from sections that are
already itemized or contractual enough to support this (trait method
signatures in §5, the rate-group table in §4.2, the fusion-phase bullets in
§7.2, etc.), and paraphrases them into single testable sentences.

If `spec.txt` is ever restructured into a numbered-clause requirements
document, the `Req ID` scheme below should be kept stable so external
traceability (e.g. a future DOORS/Polarion import) doesn't need to be
rebuilt from scratch.

## ID scheme

`REQ-<spec section>-<sequence>`, e.g. `REQ-6.3-1`. The section number is
the `spec.txt` heading the requirement is drawn from (not a code-comment
citation — see the discrepancies note below). Sequence numbers are
per-section and have no significance beyond uniqueness. A few requirements
that don't map to a specific spec heading (e.g. math-library helpers, or
milestones from §18) use a short mnemonic prefix instead (`REQ-M-*` for
`tpt-math`, `REQ-18-*` for roadmap milestones).

## Columns

- **Spec Reference** — the `spec.txt` section plus enough quoted/paraphrased
  context to locate the source material.
- **Requirement Statement** — a single testable sentence. If the source
  prose describes several behaviors, split it into multiple rows rather
  than one compound requirement.
- **Implementation** — `crate::module::item`.
- **Verification** — the test function(s) that exercise it, or a note on
  why none exist yet.
- **Status** — `Verified` (implemented + tested in-crate), `Partial`
  (implemented but not tested in-crate — e.g. only a trait impl exists in a
  backend crate not yet covered by this matrix), `Gap` (not implemented, or
  implemented but known-broken), or `Not covered` (out of this pass's
  scope; needs a follow-up pass, not evidence of a real gap).

## How to update this when adding flight-critical code

When a PR adds or changes behavior in one of the in-scope crates
(`tpt-abstractions`, `tpt-math`, `tpt-core`, `tpt-sensor-fusion`,
`tpt-mixer`, `tpt-mapping`):

1. Add or update the corresponding row(s) in `matrix.md`.
2. If the change closes a `Gap`/`Partial` row, update its `Status` and cite
   the new test.
3. If the change touches code with no corresponding row yet, add one —
   don't let the matrix silently fall out of sync with the code.

This mirrors the existing project convention (documented in
`CONTRIBUTING.md`) of referencing the relevant `spec.txt` section in PR
descriptions and doc comments; the matrix is the structured, queryable
version of that same convention.

## Known `spec.txt` section-number discrepancies (as of this pass)

A few module doc comments cited `spec.txt` subsections that don't actually
exist in the document's heading numbering — apparently the authors' own
finer subdivision of prose that `spec.txt` never numbered that granularly.
These were corrected to the nearest real heading as part of building this
matrix:

- `tpt-mapping/src/tan/mod.rs`: cited §8.4 (doesn't exist) → corrected to
  §8.1 (TAN/TERCOM is the third bullet under "8.1 Navigation Modalities").
- `tpt-mixer/src/dep.rs`: cited §9.3 (doesn't exist) → corrected to §9.1
  ("9.1 Distributed Electric Propulsion (DEP) Mixer").
- `tpt-mixer/src/tiltrotor.rs`: cited §9.4 (doesn't exist) → corrected to
  §9.2 ("9.2 Mixer Strategies" lists "Tilt-rotor eVTOL (hover-to-cruise
  transition)").
- `tpt-mapping/src/slam/mod.rs`: cited only §8.3 → corrected to §8.1, §8.3,
  since the module covers both LiDAR SLAM scan matching (§8.1) and the
  keyframe graph's sliding-window memory management (§8.3).

`tpt-core/src/lib.rs` and `tpt-math/src/lib.rs` cite §3.6, referring to the
6th item in §3's numbered design-principle list ("Formal Verification
Ready") rather than a literal `### 3.6` heading — §3 is a flat numbered
list, not subsectioned. This is left as-is; it's an unambiguous reference,
just not a literal heading.
