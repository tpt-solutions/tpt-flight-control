//! Visual-Inertial Odometry (VIO) backend. Implemented in Phase 2 (`spec.txt` §8.1).
//!
//! Fuses camera frames (via [`tpt_abstractions::VisualSensor`]) with IMU data
//! to estimate relative pose for GNSS fallback.

/// VIO estimate confidence in `[0.0, 1.0]`.
pub type Confidence = f64;

/// Placeholder VIO estimator. The real implementation performs feature
/// tracking (FAST/ORB) and pose graph optimization over a sliding window.
pub struct VioEstimator {
    confidence: Confidence,
}

impl VioEstimator {
    pub const fn new() -> Self {
        Self { confidence: 0.0 }
    }

    /// Current VIO confidence (0 until the backend is implemented).
    pub const fn confidence(&self) -> Confidence {
        self.confidence
    }
}
