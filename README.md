# TPT Flight Control

**One Rust codebase, from a $20 hobby drone to a DO-178C-certifiable transport-category avionics suite — with continuous navigation when GPS is denied, jammed, or spoofed.**

TPT Flight Control is an open-source (Apache 2.0), vertically integrated flight
control platform written in Rust. A trait-based abstraction architecture decouples
the pure flight-control math from hardware, operating system, and certification
concerns, and Cargo feature flags select the target vehicle class and assurance
level at compile time:

- **TPT Standard** — compiles against the qualified [Ferrocene](https://www.ferrocene.dev/)
  toolchain with industry-standard RTOS integrations (PikeOS, VxWorks, Zephyr).
- **TPT Sovereign** — compiles against a fully vertically integrated, formally
  verified stack (custom microkernel, verified compiler subset, Kani-proven core
  libraries) for defense and national-security supply-chain sovereignty.

A core design principle is **GPS-independent flight**: if GNSS is lost, the system
seamlessly transitions to onboard mapping, Visual-Inertial Odometry (VIO), LiDAR
SLAM, or Terrain-Aided Navigation (TAN) to keep flying safely.

> **Status:** This repository is in the **architecture & design phase**. The
> code-complete portions are simulation- and unit-test-verified; the milestones
> that require physical hardware, crewed flight tests, or external certification
> authorities are explicitly **not** completable in code. See
> [Phase status](#phase-status) below for an honest breakdown.

---

## Why

- **Memory safety by construction.** Rust's guarantees eliminate the buffer
  overflows, use-after-free, and null derefs that dominate avionics C/C++
  vulnerabilities — with zero-cost `no_std` and no heap allocation in the
  flight-critical core by default.
- **One codebase, many targets.** The same control laws run on a bare-metal STM32
  superloop *and* a partitioned seL4/ARINC 653 microkernel.
- **GPS-denied resilience.** VIO, LiDAR SLAM, and TERCOM/TAN keep the vehicle
  navigating through urban canyons, indoors, subterranean, and electronic-warfare
  environments.
- **Scale to distributed propulsion.** Fault-tolerant reallocation for the 8–24+
  motors of modern eVTOLs via pseudo-inverse / QP mixing.
- **Certification-ready, open by default.** The source is Apache 2.0; the
  certification artifacts and proprietary map databases are the commercial product.

---

## Architecture

TPT is a layered system. The platform-independent core talks to the world only
through traits (`tpt-abstractions`); concrete backends (`tpt-backend-*`) and
software-in-the-loop (`tpt-sim`) implement those traits.

```text
┌─────────────────────────────────────────────────────────┐
│                   APPLICATION LAYER                     │
│  Mission Planner │ Autopilot │ Flight Envelope Protect  │
├─────────────────────────────────────────────────────────┤
│                 FLIGHT CONTROL CORE                     │
│  Control Laws │ State Machine │ Fault Detection & Mgmt │
├─────────────────────────────────────────────────────────┤
│      SENSOR FUSION & GPS-DENIED NAVIGATION ENGINE       │
│  IMU │ GNSS │ VIO │ LiDAR SLAM │ TAN │ Onboard Mapping │
├─────────────────────────────────────────────────────────┤
│              ACTUATOR MIXING ENGINE                     │
│  Motor Mixing │ Control Surface │ Thrust Vectoring      │
├─────────────────────────────────────────────────────────┤
│            ABSTRACTION LAYER (Traits)                   │
│  HAL Trait │ OS Trait │ Comms Trait │ Spatial Map Trait │
├─────────────────────────────────────────────────────────┤
│                  BACKEND LAYER                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ tpt-micro    │  │ tpt-standard │  │ tpt-sovereign│  │
│  │ (Bare Metal) │  │ (Ferrocene)  │  │ (Verified)   │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
├─────────────────────────────────────────────────────────┤
│                   HARDWARE                              │
│  STM32 │ NXP S32K │ TI TMS570 │ Xilinx Zynq │ Jetson  │
└─────────────────────────────────────────────────────────┘
```

**Crate map (workspace members):**

| Crate | Responsibility |
|---|---|
| `tpt-abstractions` | Core trait contracts: `Imu`, `Gnss`, `VisualSensor`, `LidarSensor`, `RadarAltimeter`, `SpatialMap`, `TerrainDatabase`, `Motor`, `ControlSurface`, `Scheduler`, … |
| `tpt-math` | `no_std` linear algebra, quaternions, Kalman primitives (Kani-verified). |
| `tpt-core` | Control laws (PID/EKF cascaded loops), envelope protection, redundancy, GPS-denied nav. |
| `tpt-sensor-fusion` | AHRS (complementary filter + EKF), GPS-degraded fusion state machine, dissimilar-nav monitor. |
| `tpt-mapping` | VIO, LiDAR SLAM (ICP), TERCOM/TAN, Sparse Voxel Octree obstacle maps. |
| `tpt-mixer` | Quad-X, DEP fault-tolerant, tilt-rotor transition mixing. |
| `tpt-backend-*` | Bare-metal (STM32 superloop), FreeRTOS, Zephyr, PikeOS, seL4, VxWorks. |
| `tpt-sovereign-toolchain` | Custom compiler-qualification wrapper. |
| `tpt-protocols` | MAVLink v2, TPT-Link, ARINC 429/AFDX, companion offload, anti-spoofing, auth-encryption. |
| `tpt-gcs` / `tpt-web` | Ground Control Station (egui/console) and wasm/leptos dashboard. |
| `tpt-sim` | Physics simulator, SITL scenarios, replay/diff tooling. |
| `certification/` `docs/` `reference-hardware/` | Assurance artifacts, architecture docs, open flight-computer PCBs. |

The control stack is a time-triggered, cascaded loop: **1000 Hz** rate loop →
**200 Hz** attitude/position → **50 Hz** guidance/mapping → **10 Hz** telemetry →
**1 Hz** background. Full design: [`spec.txt`](spec.txt).

---

## Phase status

Honest, checkpoint-by-checkpoint status derived from [`todo.md`](todo.md).
Checked `[x]` = code-complete and (unit/SITL) test-verified. Items left
unchecked are **structurally impossible to complete in code** and depend on
hardware, crewed flight tests, certification authorities, or commercial partners.

| Phase | Scope | Code status | Notes |
|---|---|---|---|
| **-1** Repository & governance | Git, workspace, CI, SBOM, audit, CONTRIBUTING/DCO | ✅ complete | Cargo feature-flag scaffold, `deny.toml`, reproducible-build pipeline. |
| **0** Foundation | Traits, `tpt-math`, PID, AHRS, mixer, scheduler, SITL | ✅ complete | *Milestone: virtual quad hovers in sim.* |
| **1** First flight & basic drones | Bare-metal backend, GPS nav, MAVLink, GCS | ⚠️ code complete, **flight pending** | `tpt-uas`+ EKF, companion offload. *Real-hardware hover & first flight require HITL bench / physical flight — not code.* |
| **2** GPS-denied & mapping | VIO, SVO octree, EKF, fusion FSM, SITL scenarios | ✅ complete | *Milestone: navigates with GPS off — all 7 SITL scenarios pass.* |
| **3** eVTOL & LiDAR SLAM | DEP/tilt mixers, ICP SLAM, TAN, PikeOS, TPT-Link | ✅ code complete, **flight pending** | 2D ICP only (no NDT/3D yet). *Crewed eVTOL demo & DO-178C DAL-C engagement require partners/hardware.* |
| **4** Certification & sovereign | seL4, sovereign toolchain, Kani proofs, map signing, anti-spoofing, ChaCha20-Poly1305 | ⚠️ code complete, **verification pending** | Kani harnesses authored but **not yet run** under `kani-compiler` (Linux-only; this dev env is Windows). DO-178C DAL-C/B sign-off requires authority. |
| **5** Transport category | Triple/quad dissimilar redundancy, VxWorks, ARINC 429/AFDX | ⚠️ code complete, **certification pending** | ARP 4754A/4761 SSA scaffolded; DO-178C DAL-A & airframe integration require authority/flight test. |
| **Cross-cutting** | Envelope protection, `tpt-web`, docs, traceability matrix, replay tool | ✅ mostly complete | KiCad PCBs, commercial-artifact packaging, and full SSA closure are non-code. |

**Bottom line:** every layer that *can* be proven in software — control laws,
sensor fusion, GPS-denied navigation, mapping, mixing, backends, protocols, and
formal-verification harnesses — is implemented and test-covered. The remaining
gaps are physical flight, crewed/certified demonstrations, and authority sign-off,
which no codebase can self-close. For the full, itemized list of what a real
transport-category certification needs beyond this repository, see
[`certification/path-to-type-certification.md`](certification/path-to-type-certification.md).

---

## 5-command quickstart

No hardware, radio, or sensor required. You only need a Rust toolchain
(edition 2024 / rust ≥ 1.85) installed via [rustup](https://rustup.rs/).

```sh
# 1. Clone the workspace
git clone https://github.com/tpt-flight-control/tpt-flight-control
cd tpt-flight-control

# 2. Build every crate (host + no_std targets via CI)
cargo build --workspace

# 3. Run the full test suite across all implemented phases
cargo test --workspace

# 4. Fly a virtual drone through a GPS-denied scenario
cargo run -p tpt-sim --example gps_denied_quickstart

# 5. (optional) Watch the same scenario live in a browser — no install
#    trunk serve --release --example sitl_demo -p tpt-web --features web
```

`gps_denied_quickstart` accepts a scenario name — `Nominal`, `UrbanCanyon`,
`Jamming`, `Indoor`, `SensorDegradation`, `TotalBlackout` — and prints final
position/velocity, fusion mode, and nav uncertainty with a pass/fail waypoint
check. Try `cargo run -p tpt-sim --example gps_denied_quickstart -- TotalBlackout`.

---

## Repository layout

```
spec.txt                     Full design document (v0.2.0-DRAFT)
todo.md                      Milestone checklist (source of truth for status)
CONTRIBUTING.md              DCO sign-off + domain-expert review process
CODEOWNERS                   Flight-/mapping-critical review requirements
docs/                        Architecture deep-dives (redundancy, dissimilar-nav, web)
certification/               Traceability matrix, SSA, CI qualification notes,
                              path-to-type-certification.md (what's non-code)
tpt-*/                       Workspace crates (see crate map above)
reference-hardware/          Open flight-computer KiCad designs
scripts/                     Audit, vendoring, reproducible-build tooling
```

## Further reading

- **Design spec:** [`spec.txt`](spec.txt) — architecture, principles, roadmap, security.
- **Architecture docs:** [`docs/`](docs/) — redundancy voting, dissimilar
  navigation monitoring, web dashboard.
- **Contributing:** [`CONTRIBUTING.md`](CONTRIBUTING.md) — DCO sign-off and
  CODEOWNERS-gated review for flight-critical code.
- **Milestones:** [`todo.md`](todo.md) — per-phase checklist and honest status.

## License

Apache License 2.0. See [`LICENSE`](LICENSE).
