# tpt-web — Browser Dashboard (Rust wasm + leptos)

`spec.txt` §15.4 — crate `tpt-web`.

## Layout

The crate is split so the flight-relevant parts carry no wasm / UI dependency:

- `lib.rs` — `WebTelemetry`, a compact, serializable view of the vehicle that
  the GCS already renders (`tpt_gcs::Telemetry`) augmented with the
  navigation-health assessment (`tpt_sensor_fusion::NavHealth`). It provides a
  dependency-free JSON bridge (`to_json` / `from_json`) so a WebSocket link can
  stream frames to the browser without pulling in `serde`.
- `ui.rs` — the `leptos` front-end (`Dashboard` component). Compiled only with
  `--features web` (after `cargo add leptos`). See `examples/dashboard.rs`.

## Why this split

Following the project's Kani convention (`kani` is not a `Cargo.toml`
dependency), `leptos` is intentionally **not** declared in `tpt-web`'s
`Cargo.toml`. Enabling the UI is a two-step, opt-in action:

```sh
cargo add leptos --features web
cargo build --example dashboard --features web --target wasm32-unknown-unknown -p tpt-web
```

This keeps the default (host) build and the main CI fast and free of the heavy
wasm UI dependency tree, while the host-buildable core (JSON bridge + model +
unit tests) is always checked.

## `WebTelemetry` JSON shape

```json
{"roll":0,"pitch":0,"yaw":0,"pn":0,"pe":0,"pd":0,
 "vn":0,"ve":0,"vd":0,"batt":1,"mode":"Disarmed","nav":"GpsAided",
 "gps":1,"vio":1,"ter":1,"unc":0.4}
```

`from_json` is tolerant of field order and accepts `1`/`0` or
`true`/`false` for the health booleans. Round-tripping is unit-tested.
