# Architecture: Dissimilar Navigation Sources (VIO + TAN as GNSS backups)

`spec.txt` §16.2 (Phase 5) — implemented in `tpt-sensor-fusion::dissimilar`.

## Why

GNSS alone is not trustworthy: jamming and spoofing are first-class threats
(`spec.txt` §19.1). To argue that navigation-loss probability stays below the
catastrophic threshold for DAL-A, TPT cross-checks GPS against **dissimilar**
sources built on independent physical principles:

- **VIO** (Visual-Inertial Odometry) — optical, camera-based.
- **TAN** (Terrain-Aided Navigation) — radar-altimeter vs. a stored Digital
  Elevation Model (radiometric / geophysical).

Because VIO and TAN do not share GPS's failure modes, a disagreement between
GNSS and a healthy dissimilar source is a strong spoof / jam indicator.

## Components

- `NavSourceKind` — `Ins`, `Gnss`, `Vio`, `Tan`.
- `NavSample` — a source's local NED position, 1σ uncertainty, availability.
- `DissimilarNavMonitor` — ingests GPS/VIO/TAN samples plus a bounded INS
  drift estimate and produces a `DissimilarVerdict`:
  - `gps_distrusted` — set when GNSS disagrees with a dissimilar source beyond
    `disagreement_sigma` sigmas (scaled by combined uncertainty).
  - `mode` — recommended `FusionMode`: yields to `TerrainAided` or
    `VisualAided` when GPS is distrusted, else `GpsAided`; coasts when all
    sources are absent.
- `verdict_to_status` — maps the verdict into the `SourceStatus` flags already
  carried by `NavHealth`, so the existing telemetry model broadcasts the
  dissimilar-source assessment.

## Safety properties (tested)

- GPS spoofed far from a healthy TAN fix → `gps_distrusted`, mode
  `TerrainAided`.
- GPS spoofed far from a healthy VIO fix → `gps_distrusted`, mode
  `VisualAided`.
- Small (sub-σ) GPS/TAN disagreement → not flagged (normal noise).
- All sources absent → `Coast`.

## Relationship to the fusion FSM

`FusionStateMachine` (§7.2) still drives the EKF correction mode. The
`DissimilarNavMonitor` is the *integrity* cross-check layered on top: it can
downgrade a `Healthy` GPS to `Lost` before the FSM would otherwise trust it,
which is exactly the innovation-monitoring function required by §16.2.
