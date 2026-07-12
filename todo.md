# TPT Flight Control — Project Checklist

Derived from `spec.txt` (v0.2.0-DRAFT). High-level milestone tracking across the full roadmap.

---

## Phase -1: Repository & Governance Setup

- [x] Initialize git repository, `.gitignore` (Rust/Cargo), `LICENSE` (Apache 2.0)
- [x] Create root Cargo workspace (`Cargo.toml`) with feature-flag scaffold (`stack-bare-metal`, `stack-ferrocene`, `stack-sovereign`, `vehicle-drone`, `vehicle-evtol`, `vehicle-transport`, `gps-denied-nav`, `tan-nav`)
- [x] Scaffold repository structure (empty crates) per §17:
  - [x] `tpt-core/`
  - [x] `tpt-math/`
  - [x] `tpt-sensor-fusion/`
  - [x] `tpt-mapping/` (`vio/`, `slam/`, `tan/`, `octree/`)
  - [x] `tpt-mixer/`
  - [x] `tpt-abstractions/`
  - [x] `tpt-backend-bare-metal/`
  - [x] `tpt-backend-freertos/`
  - [x] `tpt-backend-zephyr/`
  - [x] `tpt-backend-pikeos/`
  - [x] `tpt-backend-sel4/`
  - [x] `tpt-sovereign-toolchain/`
  - [x] `tpt-sim/`
  - [x] `tpt-gcs/`
  - [x] `tpt-protocols/`
  - [x] `reference-hardware/`
  - [x] `certification/`
  - [x] `docs/`
- [x] Add `CONTRIBUTING.md` with DCO sign-off requirement
- [x] Add `CODEOWNERS` / domain-expert review requirement for flight-critical and mapping-critical code
- [x] Set up CI (build + test across `no_std` targets, lint, formatting)
- [x] Set up SBOM generation and reproducible build pipeline (§19.2)
- [x] Establish dependency vendoring/audit process for all Rust crates (`deny.toml`, `scripts/audit.sh`, `scripts/vendor.sh`, CI `audit` job)

---

## Phase 0: Foundation (Months 1-3)

- [x] Define core trait abstractions in `tpt-abstractions` (§5):
  - [x] `Imu`, `Gnss`, `VisualSensor`, `LidarSensor`, `RadarAltimeter`
  - [x] `SpatialMap`, `TerrainDatabase`
  - [x] `Motor`, `ControlSurface`, `Scheduler`, `PartitionChannel`, `MemoryPool`, `PowerSystem`
- [x] Implement `tpt-math` (verified-friendly linear algebra, quaternions, Kalman primitives on `nalgebra` `no_std`)
- [x] Implement basic PID controller with anti-windup in `tpt-core`
- [x] Implement complementary filter AHRS in `tpt-sensor-fusion`
- [x] Implement quadcopter mixer in `tpt-mixer`
- [x] Implement time-triggered rate-group scheduler (1000/200/50/10/1 Hz) per §4.2
- [x] Build basic physics simulator + SITL in `tpt-sim`
- [x] **Milestone: Virtual quadcopter hovers in simulation**

## Phase 1: First Flight & Basic Drones (Months 4-9)

- [x] Implement `tpt-backend-bare-metal` (STM32F4/F7/H7 superloop backend)
- [x] Bring up bare-metal HAL drivers for entry-class hardware (§10.1) (`hal.rs` dual host/MMIO register interface, `board.rs` bring-up, `superloop.rs` supervisor)
- [ ] Flash to real hardware, tune and achieve stable hover
- [x] Integrate `Gnss` trait implementation and GPS navigation (`board.rs` `Gnss` impl, `tpt-core::nav::GpsInsNavigator`)
- [x] Implement MAVLink v2 protocol support in `tpt-protocols` (`mavlink.rs`: v2 framing, CRC-16/X25, Heartbeat/Attitude/GlobalPositionInt/MissionItemInt)
- [x] Build basic `tpt-gcs` Ground Control Station (egui/iced) (GUI-free `Telemetry`/`Command`/`link` model + dependency-free `ConsoleGcs`, `egui`-based `GcsApp` behind `gui` feature, `src/bin/gcs.rs` runner)
- [ ] **Milestone: TPT flies a real drone**

## Phase 2: GPS-Denied & Mapping (Months 10-15)

- [x] Implement Visual-Inertial Odometry (VIO) in `tpt-mapping/vio` (`VioEstimator`/`FeatureMatch`/`RelativePose`; basic relative-pose estimator)
- [x] Implement Sparse Voxel Octree (SVO) obstacle map in `tpt-mapping/octree` (`SparseVoxelOctree`: insert/query/raycast)
- [x] Implement EKF in `tpt-sensor-fusion` (replacing/augmenting complementary filter), required for `tpt-uas`+ (`InsEkf`: predict/correct_position/correct_velocity)
- [x] Integrate VIO pose estimates into EKF for GPS-fallback (Phase 1/2 fusion strategy, §7.2) (`InsEkf::correct_vio`)
- [x] Implement GPS-degraded fusion state machine (Coast → Visual/Depth-Aided → Terrain-Aided) (`nav_health::FusionStateMachine`/`FusionMode`)
- [x] Add `NavHealth` telemetry struct and reporting (§12.2) (`nav_health::NavHealth`)
- [x] Build SITL GPS-denied scenarios in `tpt-sim` (`scenarios.rs`: `GpsDeniedSim` + `Scenario::{Nominal, UrbanCanyon, Jamming, Indoor, SensorDegradation, TotalBlackout}`, `ObstacleField` avoidance) — **but 3/8 tests currently fail**: `jammed_gps_navigates_on_vio`, `nominal_reaches_waypoint` (altitude drift), `urban_canyon_uses_visual_aiding` (stuck in `GpsAided` instead of switching to `VisualAided`)
- [ ] **Milestone: Drone successfully navigates and avoids obstacles with GPS turned off** — obstacle avoidance and indoor/total-blackout cases pass, but GPS-jamming aiding handoff is still broken (see failing tests above)

## Phase 3: eVTOL & LiDAR SLAM (Year 2)

- [x] Implement Distributed Electric Propulsion (DEP) mixer with fault-tolerant reallocation (pseudo-inverse / QP) in `tpt-mixer` (`DepMixer`: fail/restore/allocate, feature-gated `mixer-dep`)
- [x] Implement tilt-rotor hover-to-cruise transition logic (`TiltRotor`/`TiltPhase`, feature-gated `mixer-tilt`)
- [x] Implement LiDAR SLAM backend (ICP/NDT scan matching) in `tpt-mapping/slam` (`ScanMatcher::icp_2d` + `KeyframeGraph`; 2D ICP only so far, no NDT/3D yet)
- [x] Implement Terrain-Aided Navigation / TERCOM in `tpt-mapping/tan` (`Tercom::correlate`)
- [ ] Implement companion-compute offload path (Local Pose + Obstacle Cloud over Ethernet/UDP or PCIe) for Jetson/Orin
- [x] Implement `tpt-backend-pikeos` (PikeOS, ARINC 653 partitioning) (`partition.rs`: `Partition`/`SamplingPort`/`QueuingPort`/`PikeOsScheduler`/`PikeOsBackend`, 5/5 tests passing)
- [x] Implement TPT-Link zero-copy binary telemetry protocol (`tptlink`: plain framing works; ChaCha20-Poly1305-encrypted framing currently **broken** — `encrypted_round_trip` test fails, see Phase 4 crypto note)
- [ ] Begin DO-178C DAL-C certification engagement with an eVTOL partner
- [ ] **Milestone: TPT flies a crewed eVTOL prototype using onboard mapping**

## Phase 4: Certification & Sovereign Stack (Years 3-4)

- [ ] Achieve DO-178C DAL-C/B certification for `tpt-evtol` profile
- [x] Implement `tpt-backend-sel4` (seL4 microkernel backend) (`microkernel.rs`: `CapRights`/`Endpoint`/`ProtectionDomain`/`Sel4Scheduler`/`Sel4Backend`, 5/5 tests passing)
- [x] Implement `tpt-sovereign-toolchain` (custom compiler qualification wrapper) (`Construct`/`VerifiedSubset`/`QualificationReport` checker, 5/5 tests passing)
- [ ] Formally verify `tpt-math` using Kani/Creusot
- [ ] Formally verify `tpt-mapping` (VIO/SLAM/TAN bounds) using Kani/Creusot
- [x] Implement Map Data Integrity signing (cryptographic signatures on terrain/map databases, §19.1) (`integrity`: `MapManifest`, `build_manifest`/`sign`/`verify` over SHA-256 root hash)
- [x] Implement GNSS anti-spoofing integrity monitoring (§19.1) (`antispoof`: `RaimMonitor` fault detection, `GnssAuth` sign/verify tokens)
- [ ] Implement authenticated encryption (ChaCha20-Poly1305) for all comm links — **crypto bug**: `chacha::tests::poly1305_rfc8439` fails the RFC 8439 test vector (Poly1305 MAC doesn't match spec), which also breaks `tptlink::tests::encrypted_round_trip`; needs a fix before this is trustworthy, and it's still only wired into `tptlink`, not `mavlink`
- [ ] **Milestone: TPT Sovereign stack demonstrated for defense application**

## Phase 5: Transport Category (Years 5+)

- [ ] Implement triple/quad-redundant dissimilar architecture with consensus + dissimilar monitor voting (§4.3)
- [ ] Implement dissimilar navigation source architecture (VIO + TAN as dissimilar GNSS backups) for certification
- [x] Implement `tpt-backend-vxworks` support path for `tpt-transport` (`VxWorksBackend`: task set/message queues/scheduler/partition health monitor, 4/4 tests passing)
- [x] Implement ARINC 429 / AFDX protocol support in `tpt-protocols` (`arinc.rs`: `Arinc429Word`/`Arinc429Channel` BNR/BCD/parity, `AfdxFrame`/`AfdxEndSystem`, 7/7 tests passing)
- [ ] Complete ARP 4754A / ARP 4761 system safety assessment
- [ ] Achieve DO-178C DAL-A certification
- [ ] **Milestone: TPT integrated into a transport-category aircraft**

---

## Cross-Cutting / Ongoing

- [ ] Maintain flight envelope protection layer (non-bypassable, between control laws and mixer) as new vehicle classes are added
- [ ] Expand `tpt-web` (Rust wasm + leptos) dashboard alongside `tpt-gcs`
- [ ] Grow `docs/` (architecture docs, tutorials, API docs) alongside each phase
- [ ] Maintain `reference-hardware/` KiCad designs for open flight computers
- [ ] Track commercial/certification artifact packaging in `certification/` separately from open-source core
