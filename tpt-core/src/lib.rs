//! # tpt-core
//!
//! Flight control core for TPT Flight Control: control laws, the flight state
//! machine, the non-bypassable flight-envelope protection layer, and the
//! time-triggered rate-group scheduler (`spec.txt` §4, §6, §7).
//!
//! This crate is `#![no_std]` and performs no heap allocation in its hot
//! paths, per the design principles (§3.2, §3.4). It is the single place
//! where vehicle-agnostic control logic lives; backends supply the sensors,
//! actuators, and OS traits defined in [`tpt_abstractions`].
//!
//! ## Modules
//! - [`pid`] — PID controller with conditional-integration anti-windup.
//! - [`envelope`] — non-bypassable flight envelope protection.
//! - [`scheduler`] — 1000/200/50/10/1 Hz time-triggered scheduler.
//! - [`fsm`] — flight mode state machine.
//! - [`control`] — cascaded attitude controller built on [`pid`].
//! - [`state`] — shared vehicle state / setpoint types.
//!
//! **Milestone (Phase 0):** a virtual quadcopter hovers in `tpt-sim` using
//! these primitives together with [`tpt_sensor_fusion`] and [`tpt_mixer`].

#![no_std]
#![forbid(unsafe_code)]

pub mod control;
pub mod envelope;
pub mod fsm;
pub mod guidance;
pub mod nav;
pub mod pid;
pub mod scheduler;
pub mod state;

pub use control::AttitudeController;
pub use envelope::{EnvelopeConfig, EnvelopeProtector};
pub use fsm::{FlightEvent, FlightMode, FlightStateMachine};
pub use guidance::PositionController;
pub use nav::GpsInsNavigator;
pub use pid::Pid;
pub use scheduler::TimeTriggeredScheduler;
pub use state::{AttitudeSetpoint, PositionTarget, VehicleState, VelocitySetpoint};
