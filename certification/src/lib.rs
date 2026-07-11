//! # certification
//!
//! DO-178C / ARP 4754A / ARP 4761 certification artifact packaging, tracked
//! separately from the open-source core (`spec.txt` §16, §20). This crate is
//! a placeholder for the commercial certification data package; it contains no
//! flight-critical logic.
//!
//! > **Status:** scaffolded in Phase -1.

/// Design Assurance Level per DO-178C.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dal {
    A,
    B,
    C,
    D,
}

impl Dal {
    /// Returns the target catastrophic failure probability bound per flight
    /// hour for this DAL (DO-178C Table A-1).
    pub const fn failure_bound_per_flight_hour(&self) -> f64 {
        match self {
            Dal::A => 1e-9,
            Dal::B => 1e-7,
            Dal::C => 1e-5,
            Dal::D => 1e-3,
        }
    }
}
