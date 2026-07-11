//! # tpt-protocols
//!
//! Communication and telemetry protocols for TPT Flight Control (§12):
//! - **MAVLink v2** — drone / UAS profiles (Phase 1).
//! - **TPT-Link** — zero-copy binary telemetry for eVTOL (Phase 3).
//! - **ARINC 429 / AFDX** — transport-category integration (Phase 5).
//!
//! > **Status:** scaffolded in Phase -1. MAVLink arrives in Phase 1,
//! > TPT-Link in Phase 3, ARINC in Phase 5.

#![no_std]

pub mod mavlink {
    //! MAVLink v2 framing (placeholder).
}

pub mod tpt_link {
    //! TPT-Link zero-copy binary protocol (placeholder).
}

pub mod arinc {
    //! ARINC 429 / AFDX (placeholder).
}
