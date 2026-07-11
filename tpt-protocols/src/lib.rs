//! # tpt-protocols
//!
//! Communication and telemetry protocols for TPT Flight Control (§12):
//! - **MAVLink v2** — drone / UAS profiles (Phase 1). See [`mavlink`].
//! - **TPT-Link** — zero-copy binary telemetry for eVTOL (Phase 3).
//! - **ARINC 429 / AFDX** — transport-category integration (Phase 5).

#![no_std]

pub mod mavlink;

pub mod tpt_link {
    //! TPT-Link zero-copy binary protocol (placeholder, implemented Phase 3).
}

pub mod arinc {
    //! ARINC 429 / AFDX (placeholder, implemented Phase 5).
}
