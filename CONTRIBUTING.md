# Contributing to TPT Flight Control

Thank you for your interest in contributing to TPT Flight Control. This project
is safety- and flight-critical software, so we follow a disciplined
contribution process.

## Licensing

By contributing, you agree that your contributions are licensed under the
[Apache License 2.0](../LICENSE) and that you have the right to submit them
under those terms.

## Developer Certificate of Origin (DCO)

We require a **Developer Certificate of Origin (DCO) sign-off** on every commit.
This is a simple attestation that you wrote the patch or have the right to
contribute it, similar to the Linux kernel's `Signed-off-by` line.

Add a sign-off to each commit:

```text
git commit -s -m "tpt-mixer: add quadcopter X configuration"
```

This appends a line such as:

```text
Signed-off-by: Your Name <you@example.com>
```

Commits without a `Signed-off-by` trailer will be rejected by CI.

## Domain-Expert Review

Per the project governance, the following paths require review by a designated
domain expert (enforced via `CODEOWNERS`):

- `tpt-core/` — flight-critical control laws, envelope protection, scheduler
- `tpt-sensor-fusion/` — AHRS / EKF / navigation
- `tpt-mixer/` — actuator allocation and fault reallocation
- `tpt-mapping/` — VIO / SLAM / TAN (mapping-critical)
- `tpt-abstractions/` — trait contracts relied upon by certified profiles

Do not bypass the required reviewers. For certified profiles (`tpt-evtol`,
`tpt-transport`) changes may also require a corresponding certification
artifact update in `certification/`.

## Coding Standards

- Rust **2024 edition**, formatted with `cargo fmt` and linted with `cargo clippy`
  (warnings treated as errors in CI).
- Core crates (`tpt-math`, `tpt-abstractions`, `tpt-core`, `tpt-sensor-fusion`,
  `tpt-mixer`) must remain `#![no_std]` and free of heap allocation in hot paths.
- No panics in flight-critical code paths.
- Prefer deterministic, bounded execution suitable for formal verification.

## Running Checks Locally

```text
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo build --workspace
cargo test  --workspace
```

## Submitting Changes

1. Fork and create a feature branch.
2. Make committed, DCO-signed changes with clear, scoped commit messages.
3. Open a pull request describing the change and referencing any related spec
   section (e.g. `spec.txt §5`).
4. Ensure CI is green and required reviewers have approved.
