//! # tpt-backend-bare-metal
//!
//! Bare-metal superloop backend targeting entry-class STM32 MCUs (F4/F7/H7)
//! for the `tpt-drone` / `tpt-micro` profiles (`spec.txt` §10.1, §18).
//!
//! This crate provides:
//! - [`hal`] — STM32 peripheral drivers (GPIO, USART, TIM, SysTick, RCC) with a
//!   dual host/MMIO [`RegisterInterface`], so the exact same driver code runs
//!   on the PC (for unit tests) and on `thumbv7em-none-eabihf` (real silicon).
//! - [`board`] — binds the HAL to the [`tpt_abstractions`] sensor/actuator/OS
//!   traits ([`Imu`], [`Gnss`], [`RadarAltimeter`], [`Scheduler`]) and carries
//!   the runtime actuator/sensor state.
//! - [`superloop`] — the time-triggered [`Supervisor`] that wires the platform
//!   traits to the flight core, AHRS and quad-X mixer.
//!
//! The crate is `#![no_std]` and performs no heap allocation in its hot paths.

#![no_std]

pub mod board;
pub mod hal;
pub mod superloop;

pub use board::{MotorChannel, Stm32Board, bring_up};
pub use hal::{GpioBank, Peripheral, RegisterInterface, Regs, init_clocks_and_io, write_telemetry};
pub use superloop::{GRAVITY, Supervisor};
