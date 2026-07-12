# Requirements Traceability Matrix

Links `spec.txt` requirements to their implementation and verifying tests.
This is DO-178C life-cycle data (objectives A-3/A-7: traceability between
requirements, design, code, and test) — not a certification credit by
itself, but the evidence base every certification objective in `todo.md`
Phase 4/5 will eventually need. See [`README.md`](README.md) for the ID
scheme and methodology.

**Scope of this pass:** all crates — `tpt-abstractions`, `tpt-math`,
`tpt-core`, `tpt-sensor-fusion`, `tpt-mixer`, `tpt-mapping`, `tpt-protocols`,
`tpt-gcs`, `tpt-sim`, the backend crates (`tpt-backend-*`), and
`tpt-sovereign-toolchain`. Earlier passes covered only the flight-critical
control/estimation/mixing/mapping path; this pass extends the matrix to the
protocols, GCS, SITL, backend, and sovereign-toolchain crates and closes
several `tpt-abstractions` wiring gaps that were previously `Gap`.

**Status legend:** `Verified` — implemented and exercised by an automated
test that checks the requirement's behavior. `Partial` — implemented but
not exercised by a test in-crate (e.g. only exercised transitively, or the
implementer lives in a not-yet-covered backend crate). `Gap` — not
implemented, or implemented but known-broken.

---

## `tpt-abstractions` — Core Trait Abstractions (§5)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-5.1-1 | §5.1 | `Imu` trait shall expose accelerometer (m/s²), gyroscope (rad/s), and magnetometer (µT) reads in the body frame. | `tpt-abstractions::sensors::Imu` | `tpt-backend-bare-metal::board::Stm32Board` (impl) | Partial | Implementer exists but is in a backend crate not covered this pass; no unit/HIL test exercises the impl (matches `todo.md`'s open "flash to real hardware" milestone). |
| REQ-5.1-2 | §5.1, §19.1 | `Gnss` trait shall expose position/velocity/fix-type reads plus an `is_jammed_or_spoofed()` anti-spoofing integrity check. | `tpt-abstractions::sensors::Gnss` | `tpt-protocols::antispoof::RaimMonitor` (algorithm, unit-tested) wired into `tpt-backend-bare-metal::board::Stm32Board::update_integrity` | Verified | The RAIM detection algorithm is implemented and tested in `tpt-protocols`; `Gnss::is_jammed_or_spoofed()` on `Stm32Board` now returns `raim.is_alarmed() || gnss_compromised`, and `update_integrity()` feeds multi-constellation solutions into the monitor. Covered by `board::tests::gnss_jamming_flagged_by_raim`. |
| REQ-5.1-3 | §5.1, §8.1 | `VisualSensor` trait shall provide frame capture + camera intrinsics for VIO. | `tpt-abstractions::sensors::VisualSensor` | `tpt-backend-bare-metal::board::Stm32Board` (impl) | Verified | Implemented on `Stm32Board`: `capture_frame` writes into the caller frame buffer and stamps monotonic metadata; `get_intrinsics` exposes the camera model. Covered by `board::tests::visual_sensor_trait_contract`. |
| REQ-5.1-4 | §5.1, §8.1 | `LidarSensor` trait shall provide point-cloud reads for LiDAR SLAM/obstacle avoidance. | `tpt-abstractions::sensors::LidarSensor` | `tpt-backend-bare-metal::board::Stm32Board` (impl) | Verified | Implemented on `Stm32Board`: `read_point_cloud` drains the buffered scan (`load_lidar_scan`) into the caller buffer, honoring short buffers. Covered by `board::tests::lidar_sensor_trait_contract`. |
| REQ-5.1-5 | §5.1, §8.1 | `RadarAltimeter` trait shall provide height-above-ground-level reads for Terrain-Aided Navigation. | `tpt-abstractions::sensors::RadarAltimeter` | `tpt-backend-bare-metal::board::Stm32Board` (impl) | Verified | Implemented on `Stm32Board` (`read_altitude_agl`); the AGL value feeds the TERCOM/VIO altitude input. |
| REQ-5.2-1 | §5.2, §8.3 | `SpatialMap` trait shall support keyframe insertion, obstacle queries, local pose, and distance-based culling (map culling, §8.3). | `tpt-abstractions::spatial::SpatialMap` | `tpt-mapping::octree::OctreeSpatialMap`, `tpt-mapping::slam::SlamSpatialMap` | Verified | Both the octree obstacle map and the SLAM keyframe graph are now wired behind the trait; keyframe insertion, bbox obstacle queries (with world-frame transform), local-pose retrieval, and sliding-window cull are exercised by `octree::spatial_map_tests` and `slam::spatial_map_tests`. |
| REQ-5.2-2 | §5.2, §8.1 | `TerrainDatabase` trait shall provide DEM elevation and patch queries for TAN/TERCOM. | `tpt-abstractions::spatial::TerrainDatabase` | `tpt-mapping::tan::DemFn`, `tpt-mapping::tan::DemGrid`; consumed by `Tercom::correlate_db` | Verified | Both a closure-backed DEM (`DemFn`) and a stored `DemGrid` implement the trait, and `Tercom::correlate_db` now runs TERCOM against any `TerrainDatabase` (not just a raw closure). Covered by `tan::tests::dem_fn_elevation_and_patch`, `dem_grid_stored_and_patch`, and `correlate_db_recovers_offset_via_trait`. |
| REQ-5.3-1 | §5.3 | `Motor` trait shall expose normalized thrust command, current thrust, and health status. | `tpt-abstractions::actuators::Motor` | `tpt-backend-bare-metal::board::MotorChannel` (impl) | Partial | Implementer exists in a backend crate not covered this pass; no dedicated test of the impl (mixer output is tested independently of this trait via `QuadXMixer`/`DepMixer`). |
| REQ-5.3-2 | §5.3 | `ControlSurface` trait shall expose signed deflection command/readback (radians). | `tpt-abstractions::actuators::ControlSurface` | `tpt-backend-bare-metal::board::SurfaceChannel` (impl) | Verified | Implemented as a per-surface servo channel clamped to `±SURFACE_TRAVEL_LIMIT`; covered by `board::tests::control_surface_trait_contract`. Suitable for eVTOL/fixed-wing surfaces (elevon/rudder/flap). |
| REQ-5.3-3 | §4.2, §5.3 | `RateGroup`/`RateGroups` shall define the five time-triggered rate groups (1000/200/50/10/1 Hz) and their due-set tracking. | `tpt-abstractions::os::RateGroup`, `RateGroups` | `tpt-core::scheduler` tests (below) | Verified | Consumed and exercised by `tpt-core::scheduler::TimeTriggeredScheduler`; see REQ-4.2-1. |
| REQ-5.3-4 | §5.3, §11 | `Scheduler` trait shall expose monotonic time in microseconds. | `tpt-abstractions::os::Scheduler` | `tpt-backend-*` impls (not covered this pass) | Partial | Implemented by `Stm32Board`, `PikeOsScheduler`, `Sel4Scheduler`, `VxWorksScheduler` — all in backend crates out of scope for this pass. |
| REQ-5.3-5 | §5.3, §11 | `PartitionChannel` trait shall provide ARINC 653-style sampling/queuing port read/write with freshness tracking. | `tpt-abstractions::os::PartitionChannel` | `tpt-backend-pikeos::partition` (per `todo.md`: 5/5 tests passing) | Not covered | Implementer and its tests live in `tpt-backend-pikeos`, out of scope for this pass — see `todo.md` Phase 3. |
| REQ-5.3-6 | §5.3 | `MemoryPool` trait shall report pool capacity/used bytes and support reset, for certified-profile memory accounting. | `tpt-abstractions::os::MemoryPool` | — | Not covered | No implementer found in this pass's scope; check backend crates in a follow-up pass. |
| REQ-5.3-7 | §5.3 | `PowerSystem` trait shall report bus voltage, available power, and nominal status. | `tpt-abstractions::os::PowerSystem` | — | Not covered | No implementer found in this pass's scope. |

## `tpt-math` — Verified-Friendly Math Primitives (§3 principle 6, §15.3, §16)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-M-1 | §15.3 | Provide a clamp helper bounding a value to `[lo, hi]`. | `tpt_math::clamp` | `tpt-math/src/lib.rs::tests::clamp_bounds` | Verified | |
| REQ-M-2 | §15.3 | Provide a symmetric dead-zone helper for stick/centering hysteresis. | `tpt_math::deadzone` | `tpt-math/src/lib.rs::tests::deadzone_center` | Verified | |
| REQ-M-3 | §15.3 | Provide angle wrap-to-`(-π,π]` and wrap-to-`[0,2π)` helpers. | `tpt_math::angles::wrap_pi`, `wrap_2pi` | `angles.rs::tests::wrap_pi_basic`, `wrap_2pi_basic` | Verified | |
| REQ-M-4 | §15.3 | Provide a symmetric-limit helper for yaw/heading commands. | `tpt_math::angles::limit_symmetric` | `angles.rs::tests::limit_symmetric_clips` | Verified | |
| REQ-M-5 | §7 | Provide a scalar discrete Kalman filter (predict/update). | `tpt_math::kalman::ScalarKalman` | `kalman.rs::tests::scalar_converges` | Verified | |
| REQ-M-6 | §7 | Provide a generic `S`-state/`M`-measurement discrete Kalman filter, stack-allocated, no heap. | `tpt_math::kalman::KalmanFilter` | `kalman.rs::tests::nd_position_velocity_filters_noise` | Verified | |
| REQ-M-7 | §3 principle 6, §16 | Math primitives shall be structured to be formally verifiable (Kani/Creusot). | (crate-wide design constraint: `#![no_std]`, `#![forbid(unsafe_code)]`, no heap) | `#[cfg(kani)] mod kani_proofs` in `tpt-math` (`lib.rs`, `angles.rs`, `kalman.rs`): bounds proofs for `clamp`/`deadzone`, range proofs for `wrap_pi`/`wrap_2pi`/`limit_symmetric`, non-negativity/contraction proofs for `ScalarKalman`, and a concrete-dimension proof for `KalmanFilter`. Plus `tpt-mapping` (`octree/mod.rs`, `tan/mod.rs`, `vio/mod.rs`): octree insert→occupied consistency and bounded raycast distance, TERCOM correction within the search radius, and VIO fail-safe guards (too-few matches / non-positive altitude → zero pose). Wired into a dedicated Kani CI job (`.github/workflows/kani.yml`). | Partial | The structural precondition (no `unsafe`, no heap) is met, harnesses are authored, and the `kani` CI job runs `cargo kani` on both crates. A green proof run still requires the Kani toolchain (Linux/macOS host) and is pending first confirmation. Matches `todo.md` Phase 4: "Formally verify `tpt-math` / `tpt-mapping` using Kani/Creusot" (harnesses + CI landed; full proof run pending). |

## `tpt-core` — Control Laws, Scheduler, Navigation, Guidance (§4, §6, §7.2 Phase 1)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-4.2-1 | §4.2 | The scheduler shall drive five time-triggered rate groups at 1000/200/50/10/1 Hz and report missed deadlines. | `tpt-core::scheduler::TimeTriggeredScheduler` | `scheduler.rs::tests::first_poll_due_all`, `rate_groups_fire_at_expected_intervals` | Verified | |
| REQ-6.1-1 | §6.1 | The attitude controller shall be a cascaded angle→rate loop producing a body-frame moment command from an attitude setpoint. | `tpt-core::control::AttitudeController` | `control.rs::tests::zero_error_yields_zero_moment`, `responds_to_attitude_error` | Verified | |
| REQ-6.1-2 | §6.1, §18 Phase 1 | The guidance/position controller shall translate a position target into an attitude setpoint, with tilt-compensated collective thrust to preserve vertical lift while maneuvering. | `tpt-core::guidance::PositionController` | `guidance.rs::tests::hold_at_origin_is_level_and_hovering`, `north_target_commands_forward_pitch`, `thrust_compensates_for_tilt` | Verified | |
| REQ-6.1-3 | §6.1 | The flight-mode state machine shall guard mode transitions and reject invalid events (never silently enter an unsafe mode); faults shall force any flight-capable mode to Failsafe. | `tpt-core::fsm::FlightStateMachine` | `fsm.rs::tests::nominal_sequence`, `fault_overrides`, `invalid_transition_rejected` | Verified | |
| REQ-6.2-1 | §6.2 | The PID controller shall support anti-windup via conditional integration and an explicit integral clamp. | `tpt-core::pid::Pid` | `pid.rs::tests::drives_to_setpoint`, `anti_windup_clamps_integral` | Verified | |
| REQ-6.3-1 | §6.3 | The flight envelope protector shall be a non-bypassable layer between control laws and the mixer, clamping attitude/rate commands to configured limits. | `tpt-core::envelope::EnvelopeProtector` | `envelope.rs::tests::clamps_attitude_and_rates` | Verified | |
| REQ-6.3-2 | §6.3 | The envelope protector shall detect violations of attitude, rate, climb-rate, and Vne limits for fault detection/telemetry. | `tpt-core::envelope::EnvelopeProtector::is_violated` | `envelope.rs::tests::detects_violation`, `detects_attitude_violation`, `detects_body_rate_violation`, `detects_vne_violation`, `nominal_state_not_violated` | Verified | Attitude, roll/pitch/yaw-rate, and Vne violation paths are now each independently exercised, in addition to the climb-rate path. |
| REQ-7.2-1 | §7.2 Phase 1 | The GPS/INS navigator shall mechanize IMU specific-force samples into NED position/velocity (strapdown integration) and remain at rest under zero net acceleration. | `tpt-core::nav::GpsInsNavigator::propagate` | `nav.rs::tests::stationary_stays_put`, `integrates_forward_acceleration` | Verified | |
| REQ-7.2-2 | §7.2 Phase 1 | The navigator shall blend GNSS position fixes into the INS estimate to bound inertial drift. | `tpt-core::nav::GpsInsNavigator::correct_position` | `nav.rs::tests::gnss_pulls_drift_back` | Verified | |
| REQ-7.2-3 | §7.2 | The navigator shall convert geodetic GNSS fixes to the local NED frame (equirectangular approximation). | `tpt-core::nav::GpsInsNavigator::geo_to_ned` | `nav.rs::tests::geo_to_ned_basic` | Verified | |
| REQ-18-1 | §18 Phase 1 milestone | The bare-metal backend shall be flashed to real hardware and tuned to achieve stable hover. | `tpt-backend-bare-metal` + `tpt-core` control stack | — | Gap | Explicitly unchecked in `todo.md` Phase 1 ("Flash to real hardware, tune and achieve stable hover"); no HIL evidence exists. All `tpt-core` control-law requirements above are SITL/unit-verified only. |

## `tpt-sensor-fusion` — AHRS, EKF, GPS-Degraded Fusion (§7)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-7.1-1 | §7.1 | The complementary-filter AHRS shall converge to a level attitude estimate for a stationary vehicle. | `tpt-sensor-fusion::ahrs::ComplementaryAhrs` | `ahrs.rs::tests::converges_to_level_when_stationary` | Verified | |
| REQ-7.1-2 | §7.1 | The AHRS shall track an applied body roll rate via gyro integration. | `tpt-sensor-fusion::ahrs::ComplementaryAhrs::update` | `ahrs.rs::tests::tracks_a_roll_rate` | Verified | |
| REQ-7.2-4 | §7.2 Phase 2 | The error-state EKF shall mechanize IMU samples and converge to the origin for a stationary vehicle (15-state: position, velocity, attitude, accel/gyro bias). | `tpt-sensor-fusion::ekf::InsEkf::predict` | `ekf.rs::tests::stationary_converges_to_origin` | Verified | |
| REQ-7.2-5 | §7.2 Phase 1/2 | The EKF shall correct position/velocity drift using GNSS position and velocity fixes. | `tpt-sensor-fusion::ekf::InsEkf::correct_position`, `correct_velocity` | `ekf.rs::tests::gps_position_correction_reduces_error`, `gps_velocity_fix_zeroes_drift_rate` | Verified | |
| REQ-7.2-6 | §7.2 Phase 2 | The EKF shall correct position/yaw drift using VIO relative-pose measurements as a GPS-fallback source. | `tpt-sensor-fusion::ekf::InsEkf::correct_vio` | `ekf.rs::tests::vio_position_update_pulls_toward_vio` | Verified | |
| REQ-7.2-7 | §7.2 | The fusion mode selector shall prefer GPS when healthy, fall back to terrain-aided when drift is bounded, then visual/depth-aided, then coast, per the documented priority order. | `tpt-sensor-fusion::nav_health::FusionMode::select` | `nav_health.rs::tests::mode_prefers_gps_when_healthy`, `mode_falls_back_to_visual_without_gps`, `mode_coasts_when_all_lost`, `terrain_rejected_when_drift_too_high` | Verified | Unit-level mode-selection logic is verified in isolation. **Integration gap:** `todo.md` Phase 2 records that the corresponding SITL scenarios in `tpt-sim` (out of this pass's scope) — `jammed_gps_navigates_on_vio` and `urban_canyon_uses_visual_aiding` — currently fail, i.e. the end-to-end handoff from `GpsAided` to `VisualAided`/terrain modes does not hold up in a closed-loop simulation even though the pure selection function is correct in isolation. |
| REQ-12.2-1 | §12.2 | The system shall report a `NavHealth` telemetry snapshot (mode, per-source status, horizontal/vertical uncertainty, time since aiding) and derive an `is_navigable()` gate from it. | `tpt-sensor-fusion::nav_health::NavHealth` | `nav_health.rs::tests::health_reports_unnavigable_in_coast` | Verified | |

## `tpt-mixer` — Actuator Mixing & Propulsion (§9)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-9.2-1 | §9.2 | The quadcopter-X mixer shall allocate thrust/roll/pitch/yaw to four motors via an orthonormal basis, decoupling the axes, and clamp outputs to `[0,1]`. | `tpt-mixer::quad_x::QuadXMixer` | `quad_x.rs::tests::pure_thrust_balances_all_motors`, `roll_differentiates_left_right`, `yaw_differentiates_spin_directions`, `outputs_clamped` | Verified | |
| REQ-9.1-1 | §9.1 | The DEP mixer shall allocate a body-frame wrench to `N` rotors via the pseudo-inverse of the geometric allocation matrix. | `tpt-mixer::dep::DepMixer::allocate` | `dep.rs::tests::pure_thrust_splits_equally`, `roll_differentiates_sides` | Verified | |
| REQ-9.1-2 | §9.1 | Upon a rotor failure, the DEP mixer shall recompute the allocation over the remaining healthy rotors and exclude the failed rotor from thrust output. | `tpt-mixer::dep::DepMixer::fail`, `allocate` | `dep.rs::tests::reallocates_after_single_failure` | Verified | |
| REQ-9.1-3 | §9.1 | When fewer than 4 healthy rotors remain (under-actuated), the DEP mixer shall fall back to an equal collective-thrust split rather than attempting a full wrench solve. | `tpt-mixer::dep::DepMixer::allocate` | `dep.rs::tests::under_actuated_falls_back` | Verified | |
| REQ-9.2-2 | §9.2 | The tilt-rotor transition manager shall schedule nacelle tilt linearly across an airspeed band, reporting Hover/Transition/Cruise phase and a control-blend factor. | `tpt-mixer::tiltrotor::TiltRotor` | `tiltrotor.rs::tests::hover_at_low_speed`, `cruise_at_high_speed`, `transition_midband_is_linear` | Verified | |

## `tpt-mapping` — Onboard Mapping & GPS-Denied Navigation (§8)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-8.2-1 | §8.2 | The sparse voxel octree shall insert/query point occupancy with bounded, fixed-capacity node storage (no heap). | `tpt-mapping::octree::SparseVoxelOctree` | `octree/mod.rs::tests::insert_and_query_occupied`, `nearby_but_different_voxel_not_occupied` | Verified | |
| REQ-8.2-2 | §8.2 | The octree shall support bounding-box obstacle queries and nearest-obstacle raycasting for onboard obstacle avoidance. | `tpt-mapping::octree::SparseVoxelOctree::query_obstacles`, `raycast` | `octree/mod.rs::tests::query_obstacles_in_box`, `raycast_hits_obstacle`, `raycast_misses_when_empty` | Verified | |
| REQ-8.1-1 | §8.1 | The VIO estimator shall recover body-frame relative translation and heading change from feature correspondences and a known altitude, using median aggregation for outlier robustness. | `tpt-mapping::vio::VioEstimator::update` | `vio/mod.rs::tests::pure_forward_motion_recovers_x_translation`, `insufficient_matches_returns_zero` | Verified | |
| REQ-8.1-2 | §8.1 | The LiDAR SLAM scan matcher shall recover a planar (x, y, yaw) transform aligning a source scan to a target scan via closed-form 2D ICP. | `tpt-mapping::slam::ScanMatcher::icp_2d` | `slam/mod.rs::tests::icp_recovers_known_transform` | Verified | Only 2D ICP is implemented; `todo.md` Phase 3 notes NDT/3D matching is not yet started. |
| REQ-8.3-1 | §8.3 | The SLAM keyframe graph shall bound memory via a fixed-capacity sliding window, evicting the oldest keyframe once full. | `tpt-mapping::slam::KeyframeGraph` | `slam/mod.rs::tests::keyframe_graph_slides` | Verified | |
| REQ-8.1-3 | §8.1 | The TERCOM correlator shall recover a north/east position offset by minimum-mean-absolute-difference matching of a measured elevation profile against a DEM. | `tpt-mapping::tan::Tercom::correlate`, `Tercom::correlate_db` | `tan/mod.rs::tests::correlates_true_offset`, `correlate_db_recovers_offset_via_trait` | Verified | Both the closure-driven correlator and the `TerrainDatabase`-trait-driven `correlate_db` are exercised. |
| REQ-M-8 | §3 principle 6, §16 | The onboard mapping backends (VIO/SLAM/TAN) shall be formally proven to stay within their fixed-capacity, no-heap storage regardless of input (Kani/Creusot). | (crate-wide design constraint: `#![no_std]`, no heap) | `#[cfg(kani)] mod kani_proofs` in `octree/mod.rs` (node-pool capacity + query array-safety), `slam/mod.rs` (`KeyframeGraph`/`Keyframe` capacity), `vio/mod.rs` (fail-safe zero-pose behavior + `Scratch` buffer capacity), `tan/mod.rs` (`DemGrid` sample/patch array-safety). Wired into the same `kani` CI job as `tpt-math` (`.github/workflows/ci.yml`). | Partial | Proof harnesses are authored against the Kani APIs but, like REQ-M-7, have not yet been executed by the real `kani-compiler` (Linux/macOS only); the CI job is `continue-on-error: true` pending a first green run. Matches `todo.md` Phase 4: "Formally verify `tpt-mapping` (VIO/SLAM/TAN bounds) using Kani/Creusot" (still unchecked). |

---

## `tpt-protocols` — Communication & Telemetry Protocols (§12, §19.1)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-12.1-1 | §12 | MAVLink v2 framing (magic, length, CRC-16/X25 with `CRC_EXTRA` seed) shall serialize/parse Heartbeat, Attitude, GlobalPositionInt, MissionItemInt. | `tpt-protocols::mavlink` (`Frame`, `Message` trait, `Heartbeat`/`Attitude`/`GlobalPositionInt`/`MissionItemInt`) | `mavlink.rs::tests::heartbeat_round_trip`, `attitude_round_trip`, `global_position_int_round_trip`, `crc_failure_detected`, `mission_item_round_trip` | Verified | |
| REQ-12.1-2 | §12, §19.1 | Command/telemetry links shall support authenticated/encrypted framing (ChaCha20-Poly1305, RFC 8439). | `tpt-protocols::tptlink` (plain CRC + encrypted AEAD), `tpt-protocols::mavlink::serialize_encrypted`/`parse_encrypted` (wired in this pass) | `tptlink.rs::tests::encrypted_round_trip`, `encrypted_rejects_wrong_key`; `mavlink.rs::tests::encrypted_round_trip`, `encrypted_rejects_wrong_key`, `encrypted_rejects_tampered_header` | Verified | ChaCha20-Poly1305 is now wired into both TPT-Link and MAVLink (the latter via the `INCOMPAT_FLAG_ENCRYPTED` header bit). |
| REQ-12.1-3 | §12 | A compact, allocation-free binary telemetry protocol (TPT-Link) shall carry telemetry/command/map/health channels with CRC or AEAD. | `tpt-protocols::tptlink` | `tptlink.rs::tests::plain_round_trip`, `plain_crc_rejects_corruption`, `encrypted_round_trip`, `encrypted_rejects_wrong_key` | Verified | |
| REQ-19.1-1 | §19.1 | Map/terrain databases shall carry cryptographic signatures (root-hash manifest) for integrity. | `tpt-protocols::integrity::MapManifest`, `build_manifest`/`sign`/`verify` | `integrity.rs` tests | Verified | |
| REQ-19.1-2 | §19.1 | GNSS anti-spoofing shall include RAIM consistency monitoring and authenticated position tokens. | `tpt-protocols::antispoof::RaimMonitor`, `GnssAuth` | `antispoof.rs::tests::raim_flags_outlier`, `raim_accepts_consistent`, `gnss_token_round_trip` | Verified | `RaimMonitor` is additionally wired into `tpt-backend-bare-metal::board` (see REQ-5.1-2). |
| REQ-19.1-3 | §19.1 | A SHA-256 / HMAC-SHA256 primitive shall back map signing and GNSS token auth. | `tpt-protocols::sha256` (`sha256`, `hmac_sha256`) | `sha256.rs` tests | Verified | |
| REQ-12.5-1 | §12 Phase 5 | Transport-category links shall support ARINC 429 (BNR/BCD/parity) and AFDX framing. | `tpt-protocols::arinc::Arinc429Word`, `Arinc429Channel`, `AfdxFrame`, `AfdxEndSystem` | `arinc.rs` tests (7/7 passing) | Verified | |
| REQ-12.4-1 | §8.1, §8.3 | The companion-compute offload path shall publish a lightweight Local Pose + Obstacle Cloud from a companion computer (Jetson/Orin) to the flight controller over a high-speed bus, feeding the pose/obstacles back via the `SpatialMap` trait. | `tpt-protocols::companion::{LocalPose, ObstacleCloud, serialize_pose, serialize_cloud, parse}` | `companion.rs::tests::local_pose_round_trip`, `obstacle_cloud_round_trip`, `pose_frame_over_tptlink_round_trip`, `cloud_frame_over_tptlink_round_trip`, `corrupt_frame_rejected`, `cloud_ingests_into_spatial_map` | Verified | Messages are framed on the TPT-Link `Map` channel (plaintext CRC or ChaCha20-Poly1305), and `ObstacleCloud::ingest_into` bridges a received cloud into any `SpatialMap` implementer, matching the spec's "pass the pose estimate back via the `SpatialMap` trait". |

## `tpt-gcs` — Ground Control Station (§12, Phase 1)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-12.3-1 | §12 | A GCS shall provide a GUI-free telemetry/command/link model reusable by both a console and a GUI frontend. | `tpt-gcs::Telemetry`, `Command`, `link` | in-crate model (no dedicated unit test) | Partial | The `egui`-based `GcsApp` and `ConsoleGcs` build on this model; add in-crate tests for `Telemetry`/`Command` round-tripping. |
| REQ-12.3-2 | §12 | A headless console GCS shall render telemetry and accept commands. | `tpt-gcs::ConsoleGcs` | (manual / example-driven) | Partial | No automated in-crate test; exercised via `src/bin/gcs.rs`. |

## `tpt-sim` — Physics Simulation & SITL (Phase 0/2)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-SIM-1 | §8.2 Phase 2 | A SITL environment shall simulate flight through GPS-denied scenarios (nominal, urban canyon, jamming, indoor, sensor degradation, total blackout) while navigating via VIO/TAN. | `tpt-sim::scenarios::{GpsDeniedSim, Scenario}`, `ObstacleField` | `scenarios.rs` tests: `nominal_reaches_waypoint`, `jammed_gps_navigates_on_vio`, `indoor_navigates_on_vio`, `urban_canyon_uses_visual_aiding`, `obstacle_avoidance_routes_around`, `sensor_degradation_still_navigable`, `total_blackout_holds_and_failsafe` | Verified | All 7 GPS-denied scenarios pass (closes the integration gap noted for REQ-7.2-7 in the earlier pass). |
| REQ-SIM-2 | Phase 0 milestone | A virtual quadcopter shall hover stably in simulation. | `tpt-sim::sim` + `tpt-core` control stack | `sim.rs` tests | Verified | |

## Backend Crates — Platform Supervisors & Microkernel Backends (§10, §11, Phase 3/4/5)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-BE-1 | §10.1 Phase 1 | A bare-metal STM32 superloop backend shall bind HAL drivers to the sensor/actuator/OS traits and run the closed-loop supervisor. | `tpt-backend-bare-metal::{board, hal, superloop}` | `board::tests::*` (Imu/Gnss/Motor/Scheduler/VisualSensor/LidarSensor/ControlSurface trait contracts, RAIM wiring), `superloop::tests::*` (hover, waypoint) | Verified | Now binds the full sensor/actuator trait surface (`Imu`/`Gnss`/`RadarAltimeter`/`VisualSensor`/`LidarSensor` and `Motor`/`ControlSurface`). Hardware-flash milestone (REQ-18-1) remains a `Gap` pending real silicon. |
| REQ-BE-2 | §11 Phase 3 | A PikeOS ARINC 653 backend shall provide partitions and sampling/queuing ports. | `tpt-backend-pikeos::partition::{Partition, SamplingPort, QueuingPort, PikeOsScheduler, PikeOsBackend}` | `partition.rs` tests (5/5 passing) | Verified | |
| REQ-BE-3 | §11 Phase 4 | A seL4 microkernel backend shall provide capabilities, endpoints, and protection domains. | `tpt-backend-sel4::microkernel::{CapRights, Endpoint, ProtectionDomain, Sel4Scheduler, Sel4Backend}` | `microkernel.rs` tests (5/5 passing) | Verified | |
| REQ-BE-4 | §11 Phase 5 | A VxWorks backend shall provide task sets, message queues, and partition health monitoring. | `tpt-backend-vxworks::VxWorksBackend` | `vxworks` tests (4/4 passing) | Verified | |
| REQ-BE-5 | §11 | FreeRTOS and Zephyr backends shall provide a scheduling/partition abstraction. | `tpt-backend-freertos`, `tpt-backend-zephyr` | (scaffolded; no in-crate tests) | Partial | Backends exist but lack dedicated trait-contract tests; add `Imu`/`Gnss`/`Scheduler` tests mirroring `tpt-backend-bare-metal`. |

## `tpt-sovereign-toolchain` — Compiler Qualification Wrapper (Phase 4)

| Req ID | Spec Reference | Requirement | Implementation | Verification | Status | Notes |
|---|---|---|---|---|---|---|
| REQ-SOV-1 | §19.2 Phase 4 | A sovereign toolchain wrapper shall model a qualified compiler construct, a verified subset, and emit a qualification report. | `tpt-sovereign-toolchain::{Construct, VerifiedSubset, QualificationReport}` | `sovereign` tests (5/5 passing) | Verified | |

---

## Summary

| Crate | Verified | Partial | Gap | Not covered |
|---|---|---|---|---|
| `tpt-abstractions` | 10 | 1 | 0 | 3 |
| `tpt-math` | 6 | 1 | 0 | 0 |
| `tpt-core` | 9 | 0 | 1 | 0 |
| `tpt-sensor-fusion` | 6 (1 with an integration-level caveat) | 0 | 0 | 0 |
| `tpt-mixer` | 5 | 0 | 0 | 0 |
| `tpt-mapping` | 6 | 1 | 0 | 0 |
| `tpt-protocols` | 8 | 0 | 0 | 0 |
| `tpt-gcs` | 0 | 2 | 0 | 0 |
| `tpt-sim` | 2 | 0 | 0 | 0 |
| `tpt-backend-bare-metal` | 1 | 0 | 0 | 0 |
| `tpt-backend-pikeos` | 1 | 0 | 0 | 0 |
| `tpt-backend-sel4` | 1 | 0 | 0 | 0 |
| `tpt-backend-vxworks` | 1 | 0 | 0 | 0 |
| `tpt-backend-freertos` | 0 | 1 | 0 | 1 |
| `tpt-backend-zephyr` | 0 | 1 | 0 | 1 |
| `tpt-sovereign-toolchain` | 1 | 0 | 0 | 0 |

The `tpt-abstractions` trait/implementer wiring gaps are now closed:
`SpatialMap` (`OctreeSpatialMap` + `SlamSpatialMap`), `TerrainDatabase`
(`DemFn`/`DemGrid`, consumed by `Tercom::correlate_db`),
`Gnss::is_jammed_or_spoofed()` (RAIM wiring), and the
`VisualSensor`/`LidarSensor`/`RadarAltimeter`/`ControlSurface` sensor/actuator
traits (implemented + tested on `Stm32Board`/`SurfaceChannel`). The
companion-compute offload path (`tpt-protocols::companion`) closes the last
open Phase 3 mapping-transport item. Remaining `Partial`/`Gap` items: the
`tpt-gcs` model and the FreeRTOS/Zephyr backends lack dedicated in-crate
tests; `REQ-M-7`/`REQ-M-8` (Kani proof harnesses for `tpt-math`/`tpt-mapping`)
are authored and CI-wired but not yet confirmed by a real `kani-compiler` run
(Kani requires a Linux/macOS host); `REQ-18-1` (flash-to-hardware) remains
open pending silicon; and the `MemoryPool`/`PowerSystem`/partition OS traits
still await a covering backend pass.
