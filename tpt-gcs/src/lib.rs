//! # tpt-gcs
//!
//! Desktop Ground Control Station for TPT Flight Control, built with
//! `egui` / `iced` (§15.4). Provides telemetry display, mission planning,
//! and live tuning for the `tpt-drone` / `tpt-uas` profiles.
//!
//! The crate is organized so the flight-critical, compilable-everywhere parts
//! (telemetry model, command model, protocol bridge) carry no GUI dependency,
//! while the interactive window lives behind the `gui` feature. A
//! dependency-free console runner (`console`) provides the same connect /
//! display / command loop over plain std UDP, which is what `src/bin/gcs.rs`
//! runs and what the test-suite exercises.
//!
//! **Built in Phase 1.**

pub mod command;
pub mod console;
pub mod dashboard;
pub mod link;
pub mod telemetry;

#[cfg(feature = "gui")]
pub mod ui;

#[cfg(feature = "gui")]
pub use ui::GcsApp;

pub use command::Command;
pub use console::ConsoleGcs;
pub use telemetry::Telemetry;
