# CI Pipeline as Qualified Development Tools (DO-330)

DO-330 *Software Tool Qualification Considerations* applies to any tool whose
outage could introduce or fail to detect an error in the software under
certification. This note records the qualification rationale for the automated
tooling in `.github/workflows/ci.yml`, supporting the evidence chain used by the
[Software Accomplishment Summary](software-accomplishment-summary.md).

> Classification per DO-330 §2: a tool is **TQCI** (qualifiable) when it can
> introduce an error, or **TQLC** (not qualifiable / not needed) when it can only
> fail to detect one. TQCI tools require a Tool Qualification Package (TQP).

## Tool Inventory

| Tool | CI job | DO-330 role | Classification |
|---|---|---|---|
| `rustfmt` | `fmt` | Formatting only — does not change semantics | TQLC (not needed) |
| `clippy` | `clippy` | Static analysis — can fail to detect defects | TQLC (not needed) |
| `cargo build` / `cargo test` | `build-and-test`, `no-std` | Verification execution — can fail to detect defects | TQLC (not needed) |
| `cargo-deny` | `audit` | Supply-chain / license check — can fail to detect an issue | TQLC (not needed) |
| `cargo-cyclonedx` | `sbom` | SBOM generation — evidence capture, no code effect | TQLC (not needed) |

## Rationale

None of the CI tools **generate or modify** the object code that becomes the
certifiable software; they only **analyze, build, or record**. Their failure
mode is "miss an error," which is the TQLC case (DO-330 §2.2). Per DO-330, a
TQLC tool does not require a Tool Qualification Package — the missed-error risk
is already covered by the independent verification objectives in DO-178C
(Annex A: source-code/integration verification, test coverage).

## Qualification Records (when a TQCI tool is introduced)

If a tool is later introduced that **can introduce an error** (e.g. a code
generator, or a qualified-compiler wrapper such as
`tpt-sovereign-toolchain`), it becomes TQCI and requires a TQP containing:

1. **Tool Operational Requirements (TOR)** — what the tool must do and its
   operating environment (pinned Rust toolchain, `Cargo.lock`, target triple).
2. **Tool Qualification Requirements (TQR)** — the DO-178C objectives the tool
   helps satisfy.
3. **Tool Configuration Index (TCI)** — exact version, `Cargo.lock`, and CI
   image digest.
4. **Tool Qualification Test Cases / Results (TQT)** — tests demonstrating the
   tool produces correct output, including negative cases.

The `tpt-sovereign-toolchain` crate (`Construct` / `VerifiedSubset` /
`QualificationReport`, 5/5 tests passing) is the seed for a TQCI package: its
`QualificationReport` is the artifact that would accompany a TQP.

## Reproducibility Controls

- `rust-version = "1.85"` and `edition = "2024"` pin the language baseline.
- `RUSTFLAGS: "-D warnings"` turns latent issues into hard CI failures.
- `cargo-deny --all-features check advisories licenses bans sources` enforces the
  dependency-vetting process (`deny.toml`, `scripts/audit.sh`,
  `scripts/vendor.sh`).
- `no_std` + `forbid(unsafe_code)` crates are rebuilt for
  `thumbv7em-none-eabihf` and `riscv32imac-unknown-none-elf` to prove the
  embedded build path independent of the host toolchain.
