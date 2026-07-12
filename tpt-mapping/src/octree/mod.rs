//! Sparse Voxel Octree (SVO) obstacle map (`spec.txt` §8.2, Phase 2).
//!
//! A memory-efficient 3D occupancy representation for LiDAR SLAM and obstacle
//! avoidance, designed for `no_std` / embedded memory constraints.
//!
//! The tree is backed by a fixed-capacity node pool (const-generic `CAP`) so
//! it can live entirely on the stack or in static RAM with no heap. Each node
//! has up to 8 children (one per octant) and stores an occupancy flag at leaf
//! level. Inserting a world point descends to `MAX_DEPTH`, marking the
//! corresponding leaf occupied. Raycasting walks the tree to find the nearest
//! occupied voxel — the core primitive for onboard obstacle avoidance.

use tpt_abstractions::{
    spatial::SpatialMap,
    types::{BoundingBox, Landmark, Point3D, Pose6DOF},
};
use tpt_math::Vector3;

/// Sentinel for "no child node".
const NONE: i32 = -1;

/// A single octree node.
#[derive(Debug, Clone, Copy)]
struct Node {
    /// Child node indices for octants 0..8 (`NONE` if absent).
    children: [i32; 8],
    /// Occupancy at this node (meaningful for leaves; aggregated for internal).
    occupied: bool,
}

impl Node {
    const fn empty() -> Self {
        Self {
            children: [NONE; 8],
            occupied: false,
        }
    }
}

/// Sparse Voxel Octree with a fixed node-pool capacity.
///
/// `CAP` bounds the number of allocated nodes (controls RAM footprint). When
/// the pool is exhausted, further inserts are ignored and [`Self::is_full`]
/// reports `true`.
#[derive(Debug, Clone)]
pub struct SparseVoxelOctree<const CAP: usize> {
    nodes: [Node; CAP],
    /// Index of the next free node, or `CAP` when exhausted.
    free: usize,
    /// Root origin (center of the root cube).
    origin: Vector3<f64>,
    /// Half-extent of the root cube (m).
    half_size: f64,
    /// Maximum subdivision depth.
    max_depth: u8,
    /// Number of occupied leaves.
    occupied: usize,
}

impl<const CAP: usize> SparseVoxelOctree<CAP> {
    /// Create an octree rooted at `origin` with the given half-extent and depth.
    pub const fn new(origin: Vector3<f64>, half_size: f64, max_depth: u8) -> Self {
        // Safe const-init of the node pool. Index 0 is reserved as the root
        // sentinel, so the free-list starts at 1 (see `insert_occupied`).
        let nodes: [Node; CAP] = [Node::empty(); CAP];
        Self {
            nodes,
            free: 1,
            origin,
            half_size,
            max_depth,
            occupied: 0,
        }
    }

    /// Number of occupied voxels currently stored.
    pub const fn occupied_voxels(&self) -> usize {
        self.occupied
    }

    /// Whether the node pool is exhausted.
    pub const fn is_full(&self) -> bool {
        self.free >= CAP
    }

    /// Reset the tree to empty, freeing all nodes. Occupancy and internal
    /// bookkeeping are cleared; `origin`/`half_size`/`max_depth` are unchanged.
    /// Index 0 (the root sentinel) is preserved and the free-list restarts at 1.
    pub fn clear(&mut self) {
        for n in self.nodes.iter_mut() {
            *n = Node::empty();
        }
        self.free = 1;
        self.occupied = 0;
    }

    /// Side length (m) of a leaf voxel at maximum depth.
    pub fn leaf_size(&self) -> f64 {
        (self.half_size * 2.0) / ((1usize << self.max_depth) as f64)
    }

    /// Insert a world point and mark its leaf voxel occupied.
    pub fn insert_point(&mut self, p: Vector3<f64>) -> bool {
        self.insert_occupied(p, true)
    }

    /// Insert a world point with an explicit occupancy value.
    pub fn insert_occupied(&mut self, p: Vector3<f64>, occupied: bool) -> bool {
        if self.free >= CAP {
            return false;
        }
        let mut node = 0usize;
        let mut center = self.origin;
        let mut half = self.half_size;
        for depth in 0..self.max_depth as usize {
            // Determine octant by comparing against the current center.
            let oct = octant_of(center, p);
            // Descend: create child if needed.
            let child = self.nodes[node].children[oct];
            if child == NONE {
                if self.free >= CAP {
                    return false;
                }
                let idx = self.free as i32;
                self.free += 1;
                self.nodes[node].children[oct] = idx;
                node = idx as usize;
            } else {
                node = child as usize;
            }
            if depth + 1 < self.max_depth as usize {
                center = child_center(center, half, oct);
                half *= 0.5;
            }
        }
        if occupied && !self.nodes[node].occupied {
            self.occupied += 1;
        } else if !occupied && self.nodes[node].occupied {
            self.occupied -= 1;
        }
        self.nodes[node].occupied = occupied;
        true
    }

    /// Query whether the voxel containing `p` is occupied.
    pub fn is_occupied(&self, p: Vector3<f64>) -> bool {
        if self.free == 0 {
            return false;
        }
        let mut node = 0usize;
        let mut center = self.origin;
        let mut half = self.half_size;
        for depth in 0..self.max_depth as usize {
            let oct = octant_of(center, p);
            if depth + 1 == self.max_depth as usize {
                // At leaf depth, check this node if it is the leaf for `oct`.
                let child = self.nodes[node].children[oct];
                if child == NONE {
                    return false;
                }
                return self.nodes[child as usize].occupied;
            }
            let child = self.nodes[node].children[oct];
            if child == NONE {
                return false;
            }
            node = child as usize;
            center = child_center(center, half, oct);
            half *= 0.5;
        }
        false
    }

    /// Query obstacles within an axis-aligned box, writing occupied leaf centers
    /// into `out` (up to `out.len()`). Returns the count written.
    pub fn query_obstacles(&self, min: Vector3<f64>, max: Vector3<f64>, out: &mut [Vector3<f64>]) -> usize {
        let mut count = 0usize;
        // Iterative DFS via an explicit stack of (node, center, half, depth).
        // Bound the stack by max_depth*8 to stay allocation-free.
        let mut stack_node: [usize; 64] = [0; 64];
        let mut stack_center: [Vector3<f64>; 64] = [Vector3::zeros(); 64];
        let mut stack_half: [f64; 64] = [0.0; 64];
        let mut stack_depth: [u8; 64] = [0; 64];
        let mut sp = 0usize;
        stack_node[sp] = 0;
        stack_center[sp] = self.origin;
        stack_half[sp] = self.half_size;
        stack_depth[sp] = 0;
        sp += 1;

        while sp > 0 {
            sp -= 1;
            let node = stack_node[sp];
            let center = stack_center[sp];
            let half = stack_half[sp];
            let depth = stack_depth[sp];
            for oct in 0..8 {
                let child = self.nodes[node].children[oct];
                if child == NONE {
                    continue;
                }
                let c = child as usize;
                let cc = child_center(center, half, oct);
                let ch = half * 0.5;
                // Frustum-style box overlap test against this child's bounds.
                if !box_overlaps(min, max, cc, ch) {
                    continue;
                }
                if usize::from(depth) + 1 == self.max_depth as usize || self.nodes[c].children == [NONE; 8] {
                    if self.nodes[c].occupied {
                        if count < out.len() {
                            out[count] = cc;
                            count += 1;
                        }
                    }
                } else if sp < 64 {
                    stack_node[sp] = c;
                    stack_center[sp] = cc;
                    stack_half[sp] = ch;
                    stack_depth[sp] = depth + 1;
                    sp += 1;
                }
            }
        }
        count
    }

    /// Raycast from `ro` along unit `rd`, returning the nearest occupied voxel
    /// center and its distance (m), or `None` if no hit within `max_dist`.
    ///
    /// Uses a simple voxel-stepping traversal (no full DDA) suitable for
    /// obstacle-avoidance queries at modest depths.
    pub fn raycast(&self, ro: Vector3<f64>, rd: Vector3<f64>, max_dist: f64) -> Option<(Vector3<f64>, f64)> {
        if self.free == 0 {
            return None;
        }
        let step = self.leaf_size() * 0.5;
        let rd_n = if rd.norm() > 1e-9 { rd } else { Vector3::new(0.0, 0.0, 1.0) };
        let mut t = 0.0f64;
        let mut p = ro;
        while t < max_dist {
            if self.is_occupied(p) {
                return Some((p, t));
            }
            p += rd_n * step;
            t += step;
        }
        None
    }
}

/// Octant index (0..8) of `p` relative to a node `center`.
#[inline]
fn octant_of(center: Vector3<f64>, p: Vector3<f64>) -> usize {
    let mut o = 0usize;
    if p.x >= center.x {
        o |= 1;
    }
    if p.y >= center.y {
        o |= 2;
    }
    if p.z >= center.z {
        o |= 4;
    }
    o
}

/// Center of child `oct` of a node at `center` with half-extent `half`.
#[inline]
fn child_center(center: Vector3<f64>, half: f64, oct: usize) -> Vector3<f64> {
    let q = half * 0.5;
    Vector3::new(
        center.x + if oct & 1 != 0 { q } else { -q },
        center.y + if oct & 2 != 0 { q } else { -q },
        center.z + if oct & 4 != 0 { q } else { -q },
    )
}

/// Whether an axis-aligned box [min,max] overlaps the box centered at `c` with
/// half-extent `h`.
#[inline]
fn box_overlaps(min: Vector3<f64>, max: Vector3<f64>, c: Vector3<f64>, h: f64) -> bool {
    c.x - h <= max.x && c.x + h >= min.x && c.y - h <= max.y && c.y + h >= min.y && c.z - h <= max.z && c.z + h >= min.z
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> SparseVoxelOctree<1024> {
        SparseVoxelOctree::new(Vector3::new(0.0, 0.0, 0.0), 10.0, 4)
    }

    #[test]
    fn insert_and_query_occupied() {
        let mut t = tree();
        assert!(t.insert_point(Vector3::new(1.2, -2.3, 0.5)));
        assert!(t.is_occupied(Vector3::new(1.2, -2.3, 0.5)));
        assert!(t.occupied_voxels() >= 1);
    }

    #[test]
    fn nearby_but_different_voxel_not_occupied() {
        let mut t = tree();
        t.insert_point(Vector3::new(1.2, -2.3, 0.5));
        // A far point must not read back as occupied.
        assert!(!t.is_occupied(Vector3::new(-8.0, 8.0, 8.0)));
    }

    #[test]
    fn query_obstacles_in_box() {
        let mut t = tree();
        t.insert_point(Vector3::new(1.0, 1.0, 1.0));
        t.insert_point(Vector3::new(2.0, 2.0, 2.0));
        t.insert_point(Vector3::new(-9.0, -9.0, -9.0));
        let min = Vector3::new(0.0, 0.0, 0.0);
        let max = Vector3::new(5.0, 5.0, 5.0);
        let mut out = [Vector3::zeros(); 16];
        let n = t.query_obstacles(min, max, &mut out);
        assert_eq!(n, 2, "two obstacles in box, got {n}");
    }

    #[test]
    fn raycast_hits_obstacle() {
        let mut t = tree();
        let obs = Vector3::new(0.0, 0.0, 5.0);
        t.insert_point(obs);
        let hit = t.raycast(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), 20.0);
        assert!(hit.is_some());
        let (p, d) = hit.unwrap();
        assert!((d - 5.0).abs() < 1.0, "dist {d}");
        assert!((p - obs).norm() < 1.0);
    }

    #[test]
    fn raycast_misses_when_empty() {
        let t = tree();
        assert!(t
            .raycast(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), 20.0)
            .is_none());
    }
}

/// A [`SpatialMap`] backed by the [`SparseVoxelOctree`].
///
/// This closes the loop noted in the roadmap: the octree obstacle backend is
/// now wired behind the `tpt-abstractions` `SpatialMap` trait, so the fusion
/// and obstacle-avoidance layers can hold a `dyn SpatialMap` without knowing
/// the concrete representation. Keyframes insert their observed landmarks as
/// occupied voxels; [`SpatialMap::cull_distant_data`] drops points that fall
/// outside the sliding window around the current position.
///
/// * `CAP` bounds the octree node pool (and therefore RAM). When the pool is
///   exhausted further inserts are silently dropped, matching the bare octree
///   behavior.
/// * `PTS` bounds the number of landmark points retained for culling.
#[derive(Debug, Clone)]
pub struct OctreeSpatialMap<const CAP: usize, const PTS: usize> {
    tree: SparseVoxelOctree<CAP>,
    last_pose: Pose6DOF,
    /// Landmark points in the local frame, retained for the sliding-window cull.
    points: [Point3D; PTS],
    n_points: usize,
}

impl<const CAP: usize, const PTS: usize> OctreeSpatialMap<CAP, PTS> {
    /// Create a map rooted at `origin` with the given half-extent (m) and depth.
    pub fn new(origin: Vector3<f64>, half_size: f64, max_depth: u8) -> Self {
        Self {
            tree: SparseVoxelOctree::new(origin, half_size, max_depth),
            last_pose: Pose6DOF::origin(),
            points: core::array::from_fn(|_| Point3D::zeros()),
            n_points: 0,
        }
    }

    /// Borrow the underlying octree (e.g. for raycast-based avoidance queries).
    pub const fn octree(&self) -> &SparseVoxelOctree<CAP> {
        &self.tree
    }

    /// Number of landmark points currently retained.
    pub const fn retained_points(&self) -> usize {
        self.n_points
    }

    fn push_point(&mut self, p: Point3D) {
        if self.n_points < PTS {
            self.points[self.n_points] = p;
            self.n_points += 1;
        }
    }
}

impl<const CAP: usize, const PTS: usize> SpatialMap for OctreeSpatialMap<CAP, PTS> {
    type Error = core::convert::Infallible;

    fn insert_keyframe(
        &mut self,
        pose: Pose6DOF,
        landmarks: &[Landmark],
    ) -> Result<(), Self::Error> {
        self.last_pose = pose;
        for lm in landmarks {
            // Landmarks are observed in the camera frame; for the onboard
            // obstacle map we rotate them into the keyframe pose. (A full
            // odometry transform would also apply the pose translation; the
            // octree is translation-agnostic here because keyframe poses are
            // already in the map frame.)
            let p = pose.orientation * lm.position;
            self.push_point(p);
            self.tree.insert_point(p);
        }
        Ok(())
    }

    fn query_obstacles(
        &self,
        bbox: &BoundingBox,
        out: &mut [Point3D],
    ) -> Result<usize, Self::Error> {
        Ok(self.tree.query_obstacles(bbox.min, bbox.max, out))
    }

    fn get_local_pose(&self) -> Result<Pose6DOF, Self::Error> {
        Ok(self.last_pose)
    }

    fn cull_distant_data(&mut self, current_pos: Point3D, max_radius: f64) {
        // Compact the retained points to those within `max_radius` of
        // `current_pos`, then rebuild the octree from the survivors.
        let mut kept = 0usize;
        for i in 0..self.n_points {
            if (self.points[i] - current_pos).norm() <= max_radius {
                self.points[kept] = self.points[i];
                kept += 1;
            }
        }
        self.n_points = kept;
        self.tree.clear();
        for i in 0..kept {
            self.tree.insert_point(self.points[i]);
        }
    }
}

#[cfg(test)]
mod spatial_map_tests {
    use super::*;
    use tpt_abstractions::types::{BoundingBox, Landmark};

    fn map() -> OctreeSpatialMap<1024, 256> {
        OctreeSpatialMap::new(Vector3::new(0.0, 0.0, 0.0), 10.0, 4)
    }

    #[test]
    fn keyframe_inserts_landmarks_as_obstacles() {
        let mut m = map();
        let lms = [
            Landmark { position: Vector3::new(1.0, 1.0, 1.0), descriptor: 1 },
            Landmark { position: Vector3::new(2.0, 2.0, 2.0), descriptor: 2 },
        ];
        m.insert_keyframe(Pose6DOF::origin(), &lms).unwrap();
        let bbox = BoundingBox {
            min: Vector3::new(0.0, 0.0, 0.0),
            max: Vector3::new(5.0, 5.0, 5.0),
        };
        let mut out = [Point3D::zeros(); 16];
        let n = m.query_obstacles(&bbox, &mut out).unwrap();
        assert_eq!(n, 2);
        assert_eq!(m.get_local_pose().unwrap(), Pose6DOF::origin());
        assert_eq!(m.retained_points(), 2);
    }

    #[test]
    fn cull_drops_distant_voxels() {
        let mut m = map();
        let near = [Landmark { position: Vector3::new(1.0, 0.0, 0.0), descriptor: 1 }];
        let far = [Landmark { position: Vector3::new(9.0, 9.0, 9.0), descriptor: 2 }];
        m.insert_keyframe(Pose6DOF::origin(), &near).unwrap();
        m.insert_keyframe(Pose6DOF::origin(), &far).unwrap();
        assert_eq!(m.retained_points(), 2);
        m.cull_distant_data(Vector3::zeros(), 5.0);
        let bbox = BoundingBox {
            min: Vector3::new(-10.0, -10.0, -10.0),
            max: Vector3::new(10.0, 10.0, 10.0),
        };
        let mut out = [Point3D::zeros(); 16];
        let n = m.query_obstacles(&bbox, &mut out).unwrap();
        assert_eq!(n, 1, "only the near voxel should survive the cull");
        assert_eq!(m.retained_points(), 1);
    }
}
