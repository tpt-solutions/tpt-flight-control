//! LiDAR / Vision SLAM keyframe management. Implemented in Phase 3 (`spec.txt` §8).
//!
//! Builds a high-fidelity 3D map from LiDAR point clouds using ICP / NDT scan
//! matching, exposed through [`tpt_abstractions::SpatialMap`].

/// Placeholder SLAM keyframe manager.
pub struct SlamBackend {
    keyframes: usize,
}

impl SlamBackend {
    pub const fn new() -> Self {
        Self { keyframes: 0 }
    }

    /// Number of keyframes currently held (0 until implemented).
    pub const fn keyframe_count(&self) -> usize {
        self.keyframes
    }
}
