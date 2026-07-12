//! # tpt-protocols
//!
//! Communication and telemetry protocols for TPT Flight Control (§12):
//! - **MAVLink v2** — drone / UAS profiles (Phase 1). See [`mavlink`].
//! - **TPT-Link** — zero-copy binary telemetry with ChaCha20-Poly1305 (Phase 3). See [`tptlink`].
//! - **Crypto** — SHA-256, HMAC, ChaCha20-Poly1305 (Phase 4). See [`sha256`], [`chacha`].
//! - **Map integrity** — signed terrain/map manifests (§19.1). See [`integrity`].
//! - **GNSS anti-spoofing** — RAIM + authenticated tokens (§19.1). See [`antispoof`].
//! - **ARINC 429 / AFDX** — transport-category integration (Phase 5, placeholder).
//! - **Companion offload** — Local Pose + Obstacle Cloud over TPT-Link (§8.1/§8.3). See [`companion`].

#![no_std]

pub mod antispoof;
pub mod arinc;
pub mod chacha;
pub mod companion;
pub mod integrity;
pub mod mavlink;
pub mod sha256;
pub mod tptlink;
