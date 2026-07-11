//! Terrain-Aided Navigation (TAN) / TERCOM. Implemented in Phase 3 (`spec.txt` §8.1).
//!
//! Correlates radar-altimeter readings against a stored Digital Elevation
//! Model ([`tpt_abstractions::TerrainDatabase`]) to bound position drift
//! without GNSS.

/// Placeholder TERCOM correlator.
pub struct Tercom {
    matched: bool,
}

impl Tercom {
    pub const fn new() -> Self {
        Self { matched: false }
    }

    /// Whether a terrain correlation fix has been established.
    pub const fn has_fix(&self) -> bool {
        self.matched
    }
}
