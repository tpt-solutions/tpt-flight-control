//! # reference-hardware
//!
//! Open-hardware flight computer reference designs (KiCad) for TPT Flight
//! Control (`spec.txt` §17, open-source governance). Hardware sources live in
//! the `reference-hardware/` directory tree; this crate documents the
//! supported boards and exposes board descriptors consumed by backends.
//!
//! > **Status:** scaffolded in Phase -1.

/// A supported reference flight computer board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Board {
    /// Entry-class STM32F4/H7 development board (tpt-micro / tpt-drone).
    TptNucleus,
    /// Mid-range S32K3 / TMS570 UAS board (tpt-uas).
    TptSentinel,
    /// High-end Zynq eVTOL compute (tpt-evtol).
    TptAether,
}

impl Board {
    /// Returns the MCU family string for the board.
    pub const fn mcu_family(&self) -> &'static str {
        match self {
            Board::TptNucleus => "stm32",
            Board::TptSentinel => "s32k3/tms570",
            Board::TptAether => "zynq",
        }
    }
}
