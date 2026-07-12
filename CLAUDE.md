# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

TPT Flight Control is an open-source (Apache-2.0), vertically integrated flight
control software platform written in Rust, intended to span the full range of
airborne vehicles — hobby drones through DAL-A transport-category aircraft —
from a single Cargo workspace. A trait-based abstraction layer
(`tpt-abstractions`) decouples vehicle-agnostic control/navigation logic from
hardware, RTOS, and certification-profile concerns, so the same core code
compiles as lightweight bare-metal drone firmware or as a partitioned,
DO-178C-oriented avionics build depending on which Cargo features are
enabled. The full design rationale, trait signatures, and roadmap live in
`spec.txt` (the source-of-truth design doc); `todo.md` tracks milestone
status against it — check both before assuming a described capability is
implemented versus still aspirational.

## Commands

Standard workspace checks (mirrors `.github/workflows/ci.yml`):

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo build --workspace
cargo test  --workspace
```

Run a single test:

```sh
cargo test -p tpt-sensor-fusion correct_vio          # by name, one crate
cargo test -p tpt-core --features triple-redundancy  # feature-gated tests
```

`no_std` core crates must also build for embedded targets (CI checks
`thumbv7em-none-eabihf` and `riscv32imac-unknown-none-elf`):

```sh
cargo build --target thumbv7em-none-eabihf -p tpt-core --features triple-redundancy
```

Formal verification (Kani) — separate from the main build because it needs
the Kani toolchain, not stable rustc; `kani` is intentionally *not* a
`Cargo.toml` dependency, so proof harnesses live behind `#[cfg(kani)]` and
are invisible to a normal build:

```sh
cargo install --locked kani && cargo kani setup
cargo kani -p tpt-math
cargo kani -p tpt-mapping
```

Supply-chain checks:

```sh
cargo deny --all-features check advisories licenses bans sources   # or scripts/audit.sh
scripts/vendor.sh
```

GUI/wasm crates are opt-in via feature flags so the default workspace build
stays toolchain-light:

```sh
cargo run -p tpt-gcs --features gui          # egui/eframe desktop GCS
cargo build -p tpt-web --features web --target wasm32-unknown-unknown --example dashboard  # leptos dashboard, after `cargo add leptos --features web`
```

## Architecture

**Layered model** (`spec.txt` §4): application/autopilot logic sits on top of
the flight control core (control laws, state machine, envelope protection),
which sits on the sensor fusion / GPS-denied navigation engine, which feeds
the actuator mixing engine — all talking to the outside world only through
the trait abstraction layer in `tpt-abstractions`, which different backend
crates implement. Never add a direct hardware/OS/RTOS dependency to a core
crate; add or extend a trait in `tpt-abstractions` instead and implement it
in the relevant backend.

**`no_std`, no-unsafe core vs. host tooling.** `tpt-math`, `tpt-abstractions`,
`tpt-core`, `tpt-mixer`, `tpt-sensor-fusion`, `tpt-mapping`, `tpt-protocols`,
and every `tpt-backend-*` crate are `#![no_std]` (several, including
`tpt-core`, are also `#![forbid(unsafe_code)]`) with no heap allocation in
hot paths — this is load-bearing for the DO-178C/formal-verification story,
not a style preference. `tpt-sim`, `tpt-gcs`, and `tpt-web` are ordinary
`std` host-side tools (simulator, ground station, browser dashboard) and are
not held to the same constraints.

**Crate map:**

| Crate | Role |
|---|---|
| `tpt-abstractions` | Trait contracts (`Imu`, `Gnss`, `VisualSensor`, `LidarSensor`, `RadarAltimeter`, `SpatialMap`, `TerrainDatabase`, `Motor`, `ControlSurface`, `Scheduler`, `PartitionChannel`, `MemoryPool`, `PowerSystem`) — the only thing core crates depend on to reach hardware/OS |
| `tpt-math` | Verified-friendly linear algebra, quaternions, Kalman primitives (`nalgebra` `no_std`); carries the `tpt-math` Kani harnesses |
| `tpt-core` | Control laws (`control`), envelope protection (`envelope`), flight mode FSM (`fsm`), rate-group scheduler (`scheduler`), guidance/nav (`guidance`, `nav`), and (behind `triple-redundancy`) voting/consensus (`redundancy`) |
| `tpt-sensor-fusion` | AHRS/EKF (`ahrs`, `ekf`), `NavHealth` reporting (`nav_health`), dissimilar-nav cross-checking (`dissimilar`) |
| `tpt-mapping` | GPS-denied navigation: `vio/`, `slam/`, `tan/` (TERCOM), `octree/` (sparse voxel octree); carries the `tpt-mapping` Kani harnesses |
| `tpt-mixer` | Actuator allocation: `mixer-quad`/`mixer-dep`/`mixer-tilt`/`mixer-surface` feature-gated strategies, fault-tolerant reallocation |
| `tpt-backend-bare-metal` / `-freertos` / `-zephyr` / `-pikeos` / `-sel4` / `-vxworks` | Trait implementations per target OS/RTOS; e.g. bare-metal has `hal.rs`/`board.rs`/`superloop.rs`, PikeOS has ARINC 653 `partition.rs`, seL4 has capability-based `microkernel.rs` |
| `tpt-sovereign-toolchain` | Compiler-qualification wrapper for the verified/sovereign stack |
| `tpt-protocols` | `mavlink.rs` (v2 framing), `tptlink.rs` (zero-copy binary telemetry), `arinc.rs` (429/AFDX), `chacha.rs`/`sha256.rs` (ChaCha20-Poly1305 auth encryption), `antispoof.rs` (RAIM), `integrity.rs` (signed map data), `companion.rs` (companion-compute offload) |
| `tpt-sim` | Physics sim + SITL, GPS-denied `Scenario`s (`Nominal`/`UrbanCanyon`/`Jamming`/`Indoor`/`SensorDegradation`/`TotalBlackout`) |
| `tpt-gcs` / `tpt-web` | Ground control station (egui, behind `gui` feature) and browser dashboard (leptos/wasm, behind `web` feature); `tpt-web`'s `WebTelemetry` is a dependency-free JSON bridge shared with `tpt-gcs`'s telemetry model |
| `reference-hardware`, `certification`, `docs` | Non-code or doc-only workspace members (KiCad designs; DO-178C artifacts and traceability matrix; architecture docs) |

**Feature-flag composition** is per-crate, not a single workspace-level
switch: vehicle profiles (`vehicle-drone`/`vehicle-evtol`/`vehicle-transport`
on `tpt-core`) each turn on a bundle of mixer strategy, fusion strategy, and
certification-related flags (`gps-denied-nav`, `tan-nav`, `arinc653`,
`triple-redundancy`); `tpt-mapping` is pulled in transitively only when
`gps-denied-nav`/`tan-nav` is enabled (`dep:tpt-mapping`). When adding a
feature, mirror this pattern rather than introducing a root-level
`[features]` table.

**Execution model** (`spec.txt` §4.2): a time-triggered rate-group scheduler
at 1000/200/50/10/1 Hz (inner rate loop → attitude → guidance/mapping →
telemetry → background), implemented in `tpt-core::scheduler`.

**Certification trail:** `certification/traceability/matrix.md` links
`spec.txt` requirements to implementation and tests; `CODEOWNERS` requires
domain-expert review on `tpt-core`, `tpt-sensor-fusion`, `tpt-mixer`,
`tpt-mapping`, `tpt-abstractions`, `certification/`, and
`tpt-protocols`/`tpt-sovereign-toolchain` (security-sensitive); every commit
requires DCO sign-off (`git commit -s`) or CI rejects it. Changes to a
certified profile (`tpt-evtol`, `tpt-transport`) may need a matching
`certification/` artifact update, not just code.
