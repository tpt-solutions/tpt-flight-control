# Software Verification Plan (SVP)

> Status: **Draft scaffold.** Records the verification methods and
> environment actually in use; it is **not** certification credit — DO-178C
> verification independence and authority review of this plan are separate,
> unmet objectives.

## Verification methods by objective class

| Objective class | Method | Environment |
|---|---|---|
| Low-level requirements satisfy high-level requirements | Requirements-based unit testing | `cargo test -p <crate>`, one test module per source module |
| Software architecture / trait contracts | Integration testing across trait boundary | Backend crates implement `tpt-abstractions` traits; board-level tests (e.g. `tpt-backend-bare-metal::board`) exercise the concrete implementation |
| Source code satisfies low-level requirements | Requirements-based testing + structural coverage (informal — see gaps) | `cargo test --workspace`, `cargo build --target thumbv7em-none-eabihf` / `riscv32imac-unknown-none-elf` for `no_std` targets |
| Integration process outputs | End-to-end scenario testing | `tpt-sim` SITL scenarios (`Nominal`/`UrbanCanyon`/`Jamming`/`Indoor`/`SensorDegradation`/`TotalBlackout`/`PowerTransient`/`EmiUpset`) |
| Mathematical/algorithmic bounds | Formal verification | Kani proof harnesses (`#[cfg(kani)]`) in `tpt-math`, `tpt-mapping` — authored, not yet executed under `kani-compiler` in this environment (Linux-only toolchain; see `README.md` Phase 4 status) |
| Requirements-to-code / requirements-to-test traceability | Manual matrix maintenance | `certification/traceability/matrix.md`, updated per the procedure in `certification/traceability/README.md` |

## Independence

DO-178C verification independence (the verifier is not the author, for
DAL-A/B objectives) is **not** currently established — this is an
open-source project without a separate, dedicated verification team. This
is recorded here rather than glossed over: it is a real gap, not a
documentation gap.

## Regression strategy

- CI (`.github/workflows/ci.yml`) runs the full test suite plus `no_std`
  cross-builds on every change; `-D warnings` makes new clippy findings a
  hard failure rather than a silent regression.
- `certification/traceability/matrix.md`'s per-row `Status` (`Verified`/
  `Partial`/`Gap`/`Not covered`) is the running record of verification
  coverage; the update procedure requires new/changed flight-critical code
  to update the matrix in the same PR.

## Open items

- [ ] Run the Kani harnesses under `kani-compiler` and record a green
      result (currently authored but unexecuted in this dev environment).
- [ ] Establish structural coverage measurement (MC/DC for DAL-A) — current
      testing is requirements-based only, with no coverage tool wired into
      CI.
- [ ] Establish verification independence for DAL-A/B objectives.
