// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Port of `VoxelGrid` from dimos/mapping/voxels.py: a set of occupied voxel
// keys accumulated from world-frame point clouds.
//
// Deliberately naive-first (matching the Python implementation's structure for
// an apples-to-apples benchmark): `carve_columns` scans every voxel in the map
// per frame, exactly like `_carve_and_insert` fetching all active keys. An
// XY-indexed carve is a follow-up, output-identical optimization.

use ahash::AHashSet;

/// Occupied 5cm-cube keys. Key = floor(point / voxel_size) per axis, computed
/// in f32 like the Python tensor path (float32 positions / float voxel_size).
#[derive(Default)]
pub struct VoxelGrid {
    voxels: AHashSet<[i32; 3]>,
}

impl VoxelGrid {
    /// Voxelize one world-frame cloud and insert it, optionally carving the
    /// touched XY columns first (dynamic-obstacle removal).
    pub fn add_frame(&mut self, points: &[(f32, f32, f32)], voxel_size: f32, carve_columns: bool) {
        if points.is_empty() {
            return;
        }
        let keys: Vec<[i32; 3]> = points
            .iter()
            .map(|&(x, y, z)| {
                [
                    (x / voxel_size).floor() as i32,
                    (y / voxel_size).floor() as i32,
                    (z / voxel_size).floor() as i32,
                ]
            })
            .collect();

        if carve_columns {
            // Naive port of `_carve_and_insert`: full scan of the existing map
            // against the new frame's XY column set. O(map size) per frame.
            let new_xy: AHashSet<[i32; 2]> = keys.iter().map(|k| [k[0], k[1]]).collect();
            self.voxels.retain(|k| !new_xy.contains(&[k[0], k[1]]));
        }

        self.voxels.extend(keys);
    }

    /// Center point of every occupied voxel (the Python
    /// `get_global_pointcloud`: key * voxel_size + voxel_size / 2).
    pub fn global_points(&self, voxel_size: f32) -> Vec<(f32, f32, f32)> {
        let half = voxel_size * 0.5;
        self.voxels
            .iter()
            .map(|k| {
                (
                    k[0] as f32 * voxel_size + half,
                    k[1] as f32 * voxel_size + half,
                    k[2] as f32 * voxel_size + half,
                )
            })
            .collect()
    }

    /// Occupied keys in lexicographic order (golden-test comparison against
    /// the Python hashmap's key set).
    pub fn sorted_keys(&self) -> Vec<[i32; 3]> {
        let mut keys: Vec<[i32; 3]> = self.voxels.iter().copied().collect();
        keys.sort_unstable();
        keys
    }

    pub fn len(&self) -> usize {
        self.voxels.len()
    }

    pub fn is_empty(&self) -> bool {
        self.voxels.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dedupes_points_in_same_voxel() {
        let mut g = VoxelGrid::default();
        g.add_frame(&[(0.01, 0.01, 0.01), (0.02, 0.03, 0.04)], 0.05, false);
        assert_eq!(g.len(), 1);
    }

    #[test]
    fn negative_coords_floor_not_truncate() {
        let mut g = VoxelGrid::default();
        // -0.01 / 0.05 = -0.2 -> floor = -1 (truncation would give 0).
        g.add_frame(&[(-0.01, -0.01, -0.01)], 0.05, false);
        let pts = g.global_points(0.05);
        assert_eq!(pts, vec![(-0.025, -0.025, -0.025)]);
    }

    #[test]
    fn carve_removes_whole_column() {
        let mut g = VoxelGrid::default();
        // A "person" occupying a column of voxels at xy cell (0, 0).
        g.add_frame(&[(0.01, 0.01, 0.3), (0.01, 0.01, 0.6)], 0.05, false);
        // Elsewhere, untouched.
        g.add_frame(&[(1.0, 1.0, 0.1)], 0.05, false);
        assert_eq!(g.len(), 3);
        // New frame sees only the floor in that column: old column voxels go.
        g.add_frame(&[(0.01, 0.01, 0.01)], 0.05, true);
        let mut pts = g.global_points(0.05);
        pts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(pts.len(), 2);
        assert_eq!(pts[0], (0.025, 0.025, 0.025));
    }

    #[test]
    fn no_carve_keeps_column() {
        let mut g = VoxelGrid::default();
        g.add_frame(&[(0.01, 0.01, 0.3)], 0.05, false);
        g.add_frame(&[(0.01, 0.01, 0.01)], 0.05, false);
        assert_eq!(g.len(), 2);
    }

    #[test]
    fn empty_frame_is_noop() {
        let mut g = VoxelGrid::default();
        g.add_frame(&[], 0.05, true);
        assert!(g.is_empty());
    }
}
