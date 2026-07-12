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
- [x] Build SITL GPS-denied scenarios in `tpt-sim` (`scenarios.rs`: `GpsDeniedSim` + `Scenario::{Nominal, UrbanCanyon, Jamming, Indoor, SensorDegradation, TotalBlackout}`, `ObstacleField` avoidance) — all 7 scenario tests pass. Root-cause of the earlier 4/9 regression: (1) `InsEkf::predict` double-integrated the gyro quaternion on top of the AHRS-seeded attitude from `set_attitude` (attitude drift → runaway); (2) moment commands from the cascaded attitude controller were unbounded, so the quad-X mixer's per-motor `[0,1]` clamp *distorted the collective thrust* (a low-thrust command summed to >1 and launched the vehicle); (3) the `temporally-correlated` GPS/VIO measurement noise destabilized the position hold. Fixed by guarding the internal attitude integration when externally seeded, adding VIO velocity aiding to `InsEkf::correct_vio`, smoothly scaling mixer moments (`MOMENT_SCALE`) so the clamp never distorts thrust, retuning the attitude-controller gains, and reverting the measurement model to white noise.
- [x] **Milestone: Drone successfully navigates and avoids obstacles with GPS turned off** — all GPS-denied scenarios pass (`nominal_reaches_waypoint`, `jammed_gps_navigates_on_vio`, `indoor_navigates_on_vio`, `urban_canyon_uses_visual_aiding`, `obstacle_avoidance_routes_around`, `sensor_degradation_still_navigable`, `total_blackout_holds_and_failsafe`). `urban_canyon` now correctly selects `FusionMode::VisualAided` (a 2D/multipath-degraded GPS fix is reported `Degraded` and yields to VIO).

## Phase 3: eVTOL & LiDAR SLAM (Year 2)

- [x] Implement Distributed Electric Propulsion (DEP) mixer with fault-tolerant reallocation (pseudo-inverse / QP) in `tpt-mixer` (`DepMixer`: fail/restore/allocate, feature-gated `mixer-dep`)
- [x] Implement tilt-rotor hover-to-cruise transition logic (`TiltRotor`/`TiltPhase`, feature-gated `mixer-tilt`)
- [x] Implement LiDAR SLAM backend (ICP/NDT scan matching) in `tpt-mapping/slam` (`ScanMatcher::icp_2d` + `KeyframeGraph`; 2D ICP only so far, no NDT/3D yet)
- [x] Implement Terrain-Aided Navigation / TERCOM in `tpt-mapping/tan` (`Tercom::correlate`)
- [ ] Implement companion-compute offload path (Local Pose + Obstacle Cloud over Ethernet/UDP or PCIe) for Jetson/Orin
- [x] Implement `tpt-backend-pikeos` (PikeOS, ARINC 653 partitioning) (`partition.rs`: `Partition`/`SamplingPort`/`QueuingPort`/`PikeOsScheduler`/`PikeOsBackend`, 5/5 tests passing)
- [x] Implement TPT-Link zero-copy binary telemetry protocol (`tptlink`: plain and ChaCha20-Poly1305-encrypted framing both work; `encrypted_round_trip` passes)
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
- [ ] Implement authenticated encryption (ChaCha20-Poly1305) for all comm links — Poly1305 MAC now matches the RFC 8439 test vector (`chacha::tests::poly1305_rfc8439` and `tptlink::tests::encrypted_round_trip` both pass); still only wired into `tptlink`, not `mavlink`
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
  - [x] Build requirements traceability matrix (`spec.txt` requirement → implementation → test) in `certification/traceability/` — covers `tpt-abstractions`, `tpt-math`, `tpt-core`, `tpt-sensor-fusion`, `tpt-mixer`, `tpt-mapping`; see `certification/traceability/matrix.md` for the full gap list
  - [ ] Extend the traceability matrix to `tpt-protocols`, `tpt-gcs`, `tpt-sim`, backend crates, and `tpt-sovereign-toolchain`
  - [ ] Implement `SpatialMap` for `tpt-mapping`'s octree/VIO/SLAM keyframe map (currently free-standing modules, no implementer wires them behind the trait)
  - [ ] Implement `TerrainDatabase` for TAN's DEM access (`Tercom::correlate` currently takes a raw closure instead of the trait)
  - [ ] Implement `VisualSensor`/`LidarSensor`/`RadarAltimeter` backends, or confirm they're intentionally bypassed and update the traits/spec accordingly
  - [ ] Implement `ControlSurface` for a fixed-wing/eVTOL surface backend (no implementer yet)
  - [ ] Wire `Gnss::is_jammed_or_spoofed()` to the existing `tpt-protocols::antispoof::RaimMonitor` RAIM detection logic (algorithm exists and is tested, just not plumbed through the trait method)
  - [ ] Add unit/HIL tests for the `Imu`/`Gnss`/`Motor`/`Scheduler` trait implementations in `tpt-backend-bare-metal::board` (implemented but untested against the trait contract)
  - [ ] Add independent test coverage for `EnvelopeProtector::is_violated`'s attitude/rate/Vne violation paths (currently only the climb-rate path is exercised)
  - [ ] Write a Software Accomplishment Summary tracking DO-178C Annex A objective status, building on the traceability matrix
  - [ ] Document the clippy/rustfmt/`cargo-audit` CI pipeline as qualified development tools (DO-330-style tool qualification notes)
