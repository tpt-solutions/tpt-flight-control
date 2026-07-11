//! Sparse Voxel Octree (SVO) obstacle map. Implemented in Phase 2 (`spec.txt` §8.2).
//!
//! Memory-efficient 3D occupancy representation for LiDAR SLAM and obstacle
//! avoidance, designed for `no_std` / embedded memory constraints.

/// Placeholder sparse voxel octree.
pub struct SparseVoxelOctree {
    occupied: usize,
}

impl SparseVoxelOctree {
    pub const fn new() -> Self {
        Self { occupied: 0 }
    }

    /// Number of occupied voxels currently stored (0 until implemented).
    pub const fn occupied_voxels(&self) -> usize {
        self.occupied
    }
}
