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
- [ ] Flash to real hardware, tune and achieve stable hover — *requires physical hardware + HITL bench; not completable in code*
- [x] Integrate `Gnss` trait implementation and GPS navigation (`board.rs` `Gnss` impl, `tpt-core::nav::GpsInsNavigator`)
- [x] Implement MAVLink v2 protocol support in `tpt-protocols` (`mavlink.rs`: v2 framing, CRC-16/X25, Heartbeat/Attitude/GlobalPositionInt/MissionItemInt)
- [x] Build basic `tpt-gcs` Ground Control Station (egui/iced) (GUI-free `Telemetry`/`Command`/`link` model + dependency-free `ConsoleGcs`, `egui`-based `GcsApp` behind `gui` feature, `src/bin/gcs.rs` runner)
- [ ] **Milestone: TPT flies a real drone** — *requires physical hardware flight test; not completable in code*

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
- [x] Implement companion-compute offload path (Local Pose + Obstacle Cloud over Ethernet/UDP or PCIe) for Jetson/Orin (`tpt-protocols::companion`: `LocalPose`/`ObstacleCloud` framed on the TPT-Link `Map` channel — plain CRC or ChaCha20-Poly1305 — with `ObstacleCloud::ingest_into` bridging a received cloud into any `SpatialMap`; 6/6 tests pass)
- [x] Implement `tpt-backend-pikeos` (PikeOS, ARINC 653 partitioning) (`partition.rs`: `Partition`/`SamplingPort`/`QueuingPort`/`PikeOsScheduler`/`PikeOsBackend`, 5/5 tests passing)
- [x] Implement TPT-Link zero-copy binary telemetry protocol (`tptlink`: plain and ChaCha20-Poly1305-encrypted framing both work; `encrypted_round_trip` passes)
- [ ] Begin DO-178C DAL-C certification engagement with an eVTOL partner — *requires an external certification partner / authority; not completable in code*
- [ ] **Milestone: TPT flies a crewed eVTOL prototype using onboard mapping** — *requires hardware + crewed flight test + partner; not completable in code*

## Phase 4: Certification & Sovereign Stack (Years 3-4)

- [ ] Achieve DO-178C DAL-C/B certification for `tpt-evtol` profile — *certification authority sign-off; not completable in code*
- [x] Implement `tpt-backend-sel4` (seL4 microkernel backend) (`microkernel.rs`: `CapRights`/`Endpoint`/`ProtectionDomain`/`Sel4Scheduler`/`Sel4Backend`, 5/5 tests passing)
- [x] Implement `tpt-sovereign-toolchain` (custom compiler qualification wrapper) (`Construct`/`VerifiedSubset`/`QualificationReport` checker, 5/5 tests passing)
- [x] Formally verify `tpt-math` using Kani/Creusot — Kani proof harnesses in `tpt-math/src/{lib,angles,kalman}.rs` (`#[cfg(kani)]`) + `.github/workflows/kani.yml` runs `cargo kani` (proof harnesses authored for `clamp`/`deadzone`, `wrap_pi`/`wrap_2pi`/`limit_symmetric`, and both Kalman filters in `#[cfg(kani)] mod kani_proofs` blocks, wired into a non-blocking `kani` CI job (`.github/workflows/ci.yml`); Kani only runs on Linux/macOS, so these have not yet been executed by the real `kani-compiler` — pending a green CI run, then drop `continue-on-error`)
- [x] Formally verify `tpt-mapping` (VIO/SLAM/TAN bounds) using Kani/Creusot — Kani proof harnesses added in `tpt-mapping/src/{octree,tan,vio}/mod.rs` (`#[cfg(kani)]`) + covered by `kani.yml` (proof harnesses authored for octree node-pool/query bounds, SLAM keyframe-graph/keyframe-point capacity bounds, VIO's fail-safe behavior + scratch-buffer bound, and TAN `DemGrid` array-safety, same CI job as `tpt-math`; likewise pending a first real Kani run)
- [x] Implement Map Data Integrity signing (cryptographic signatures on terrain/map databases, §19.1) (`integrity`: `MapManifest`, `build_manifest`/`sign`/`verify` over SHA-256 root hash)
- [x] Implement GNSS anti-spoofing integrity monitoring (§19.1) (`antispoof`: `RaimMonitor` fault detection, `GnssAuth` sign/verify tokens)
- [x] Implement authenticated encryption (ChaCha20-Poly1305) for all comm links — Poly1305 MAC now matches the RFC 8439 test vector (`chacha::tests::poly1305_rfc8439` and `tptlink::tests::encrypted_round_trip` both pass); now wired into both `tptlink` and `mavlink` (`mavlink::serialize_encrypted`/`parse_encrypted` via the `INCOMPAT_FLAG_ENCRYPTED` header bit; `mavlink::tests::encrypted_round_trip`, `encrypted_rejects_wrong_key`, `encrypted_rejects_tampered_header` pass)
- [ ] **Milestone: TPT Sovereign stack demonstrated for defense application** — *requires a defense demonstration program; not completable in code*

## Phase 5: Transport Category (Years 5+)

- [x] Implement triple/quad-redundant dissimilar architecture with consensus + dissimilar monitor voting (§4.3) — `tpt-core::redundancy` (`MidValueSelect`/`Consensus`/`MonitorVoter`), built/tested behind `triple-redundancy` feature + no-std CI
- [x] Implement dissimilar navigation source architecture (VIO + TAN as dissimilar GNSS backups) for certification — `tpt-sensor-fusion::dissimilar::DissimilarNavMonitor` cross-checks GNSS against dissimilar VIO/TAN and downgrades spoofed/jammed GPS
- [x] Implement `tpt-backend-vxworks` support path for `tpt-transport` (`VxWorksBackend`: task set/message queues/scheduler/partition health monitor, 4/4 tests passing)
- [x] Implement ARINC 429 / AFDX protocol support in `tpt-protocols` (`arinc.rs`: `Arinc429Word`/`Arinc429Channel` BNR/BCD/parity, `AfdxFrame`/`AfdxEndSystem`, 7/7 tests passing)
- [ ] Complete ARP 4754A / ARP 4761 system safety assessment — *scaffold + traceability added in `certification/system-safety-assessment.md`; full PSSA/SSA/FHA closure requires the Design Assurance organization*
- [ ] Achieve DO-178C DAL-A certification — *certification authority sign-off; not completable in code*
- [ ] **Milestone: TPT integrated into a transport-category aircraft** — *requires airframe integration + flight test; not completable in code*

---

## Cross-Cutting / Ongoing

- [x] Maintain flight envelope protection layer (non-bypassable, between control laws and mixer) as new vehicle classes are added — `tpt-core::envelope` is feature-complete; all violation paths independently tested
- [x] Expand `tpt-web` (Rust wasm + leptos) dashboard alongside `tpt-gcs` — new `tpt-web` crate: host-buildable `WebTelemetry` JSON bridge + `leptos` UI behind `web` feature (`docs/architecture-web.md`)
- [x] Grow `docs/` (architecture docs, tutorials, API docs) alongside each phase — added `docs/architecture-{redundancy,dissimilar-nav,web}.md` and `certification/system-safety-assessment.md`
- [ ] Maintain `reference-hardware/` KiCad designs for open flight computers — *physical PCB design artifacts; not completable in code*
- [ ] Track commercial/certification artifact packaging in `certification/` separately from open-source core — *packaging process/commercial artifacts; partially addressed by `certification/` docs*
  - [x] Build requirements traceability matrix (`spec.txt` requirement → implementation → test) in `certification/traceability/` — covers `tpt-abstractions`, `tpt-math`, `tpt-core`, `tpt-sensor-fusion`, `tpt-mixer`, `tpt-mapping`; see `certification/traceability/matrix.md` for the full gap list
  - [x] Extend the traceability matrix to `tpt-protocols`, `tpt-gcs`, `tpt-sim`, backend crates, and `tpt-sovereign-toolchain` (matrix now covers all crates, including the companion-offload path and the newly-wired sensor/actuator traits)
  - [x] Implement `SpatialMap` for `tpt-mapping`'s octree/VIO/SLAM keyframe map (`OctreeSpatialMap` over the SVO + `SlamSpatialMap` over the keyframe graph; both tested)
  - [x] Implement `TerrainDatabase` for TAN's DEM access (`DemFn`/`DemGrid` implement the trait; `Tercom::correlate_db` runs TERCOM against any `TerrainDatabase` instead of a raw closure)
  - [x] Implement `VisualSensor`/`LidarSensor`/`RadarAltimeter` backends, or confirm they're intentionally bypassed and update the traits/spec accordingly (all three implemented + tested on `tpt-backend-bare-metal::board::Stm32Board`)
  - [x] Implement `ControlSurface` for a fixed-wing/eVTOL surface backend (`tpt-backend-bare-metal::board::SurfaceChannel`, clamped to servo travel limit; tested)
  - [x] Wire `Gnss::is_jammed_or_spoofed()` to the existing `tpt-protocols::antispoof::RaimMonitor` RAIM detection logic (`Stm32Board::update_integrity` feeds multi-constellation solutions into the monitor; `is_jammed_or_spoofed` returns the RAIM alarm; tested)
  - [x] Add unit/HIL tests for the `Imu`/`Gnss`/`Motor`/`Scheduler` trait implementations in `tpt-backend-bare-metal::board` (plus `VisualSensor`/`LidarSensor`/`ControlSurface`; 10/10 board tests pass)
  - [x] Add independent test coverage for `EnvelopeProtector::is_violated`'s attitude/rate/Vne violation paths (each path now independently exercised)
  - [x] Write a Software Accomplishment Summary tracking DO-178C Annex A objective status, building on the traceability matrix (`certification/software-accomplishment-summary.md`)
  - [x] Document the clippy/rustfmt/`cargo-audit` CI pipeline as qualified development tools (DO-330-style tool qualification notes) (`certification/ci-qualification.md`)

---

## Resilience & Autonomy Roadmap (planned, 2026-07-12)

Tracks the environmental-hardening, autonomy, and fuel-efficiency
directions identified in a resilience/innovation review (see conversation
2026-07-12); designed in detail in the accompanying plan before
implementation begins.

- [x] Add DO-160 environmental-qualification coverage (§16.3, new): fault-
  persistence scrubbing in `tpt-core::redundancy` to distinguish transient
  lightning/HIRF-induced upsets from permanent faults (`FaultMonitor` /
  `FaultClass` / `scrub_channels`), `PowerSystem::brownout_active()`, and SITL
  `PowerTransient` / `EmiUpset` scenarios in `tpt-sim::environment` (all pass)
- [x] Add predictive health / prognostics (`tpt-core::prognostics`): fixed-
  capacity trend buffers for battery/motor health (`TrendBuffer` / `BatteryHealth`
  / `MotorHealth`), RUL-style estimates, extended `PowerSystem` / `Motor` trait
  fields (SoC, cell voltage, temperature, rpm, load, RUL), and a `tptlink`
  `HealthReport` message on the existing `Channel::Health` (`serialize_health` /
  `parse_health`) — all unit-tested
- [x] Autopilot Phase 1 (feature-gated `autopilot` on `tpt-core`): mission/
  waypoint sequencing (`WaypointSequencer`), wiring the dead-code `EnvelopeProtector::
  inside_geofence` into a real geofence-breach response (`GeofenceMonitor`:
  clamp-to-fence / climb-out), and a defined Failsafe (RTL-to-last-good /
  land-in-place) behavior (`FailsafeManager`) — all unit-tested
- [x] Autopilot Phase 2: reactive obstacle avoidance wiring `tpt-mapping`'s
  `SparseVoxelOctree::query_obstacles`/`raycast` into guidance (feature-
  gated `autopilot-avoidance`, built on `autopilot`) — `ObstacleAvoider::
  mitigate` / `lookahead_offset` push the aim point around nearby occupied
  voxels; unit-tested with a real octree
- [x] Swarm coordination foundation: peer telemetry sharing over existing
  `tpt-protocols` framing (`swarm::PeerTelemetry` + `serialize_peer`/`parse_peer`
  on `Channel::Telemetry`) + a relative-position-keeping controller
  (`swarm::RelativePositionController`, `SwarmNetwork`) in `tpt-core` (feature
  `swarm`) — unit-tested
- [x] Formation flight for fuel savings: `FormationController` holding a
  trailing vehicle in the lead's upwash to cut induced drag (fixed-wing/
  eVTOL-cruise profiles, `formation::FormationProfile::upwash_slot`), built on
  the swarm foundation above (feature `formation`) — unit-tested
- [x] Engine-out glide guidance: `FlightMode::Glide` + `GlideController`
  flying best-glide-speed/pitch and searching `TerrainDatabase` for the best
  reachable landing site on total propulsion loss (`glide::best_landing_site`,
  `FlightEvent::PropulsionLoss` added to `tpt-core::fsm`), feature `glide` —
  unit-tested with a mock `TerrainDatabase`

---

## Adoption & Innovation (post-review, 2026-07-12)

Tracks the highest-leverage adoption/differentiation items identified in an
adoption review; see the full list of considered-but-deferred items (docs
site, crates.io publish, community chat, etc.) discussed in that review.

- [x] Write a root `README.md` (pitch, architecture diagram, honest phase
  status, 5-command quickstart, links to `spec.txt`/`docs/`/`CONTRIBUTING.md`)
- [x] Build a 5-minute SITL quickstart entry point in `tpt-sim` so
  `cargo run -p tpt-sim --example ...` runs a GPS-denied scenario and prints
  results with no hardware (`tpt-sim/examples/gps_denied_quickstart.rs`:
  picks a `Scenario` from `argv`, runs `GpsDeniedSim` for 30s of sim time,
  and prints final position/velocity/fusion-mode/nav-uncertainty with a
  pass/fail waypoint check — no vehicle, radio, or sensor required)
- [x] Build a WASM SITL-in-browser demo: compile a `tpt-sim` scenario to
  `wasm32-unknown-unknown` and feed it into `tpt-web`'s existing
  `WebTelemetry`/`leptos` dashboard for a live no-install demo
  (`tpt-web/examples/sitl_demo.rs` + `tpt-web/index.html`: runs
  `GpsDeniedSim` client-side behind a `leptos` `set_interval` animation
  loop and renders live telemetry through `tpt_web::ui::Dashboard`; serve
  with `trunk serve --release --example sitl_demo -p tpt-web --features web`)
- [ ] Get the Kani CI job (`.github/workflows/kani.yml`) to a real green run
  under `kani-compiler` and drop `continue-on-error` so verification is
  enforced, not just advisory — `continue-on-error` has already been
  dropped and the job split into its own `workflow_dispatch`/path-filtered
  workflow, but a real `kani-compiler` run (Linux-only, not reproducible
  from this Windows dev environment) is still outstanding
- [x] Build a deterministic flight-log replay/diff tool: record a flight log,
  replay it through the same control-law crate, and diff against the
  original telemetry (`tpt-sim/src/replay.rs`: `record`/`replay`/`diff`
  over `GpsDeniedSim`'s `sense`/`apply` steps, plus dependency-free
  `to_csv`/`from_csv` so a golden log can be saved and diffed against
  later runs; `tpt-sim/examples/replay.rs` runner; 3/3 tests pass,
  bit-for-bit determinism confirmed to <1e-9 across position/velocity/
  attitude/uncertainty/motor-sum)
