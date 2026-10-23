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

//! Point clouds and the k-d tree every stage shares.

use crate::mat3::{self, Mat3, Vec3};

/// Body/global-frame cloud: xyz as f32 (nothing in the PGO core reads intensity).
pub type PointCloud = Vec<[f32; 3]>;

/// Transform a cloud by a rotation + translation: p' = rotation*p +
/// translation (rotation applied in f64, stored back to f32).
pub fn transform_cloud(cloud: &[[f32; 3]], rotation: &Mat3, translation: &Vec3) -> PointCloud {
    cloud
        .iter()
        .map(|p| {
            let v = [p[0] as f64, p[1] as f64, p[2] as f64];
            let out = mat3::add(&mat3::mat_vec(rotation, &v), translation);
            [out[0] as f32, out[1] as f32, out[2] as f32]
        })
        .collect()
}

/// Voxel-grid downsample with cubic leaf size: hash points into cells on
/// `floor(p * (1/leaf))` (f32 math), emit the centroid of each occupied
/// cell, ordered by (z, y, x) cell index.
pub fn voxel_downsample(cloud: &[[f32; 3]], leaf: f64) -> PointCloud {
    if leaf <= 0.0 || cloud.is_empty() {
        return cloud.to_vec();
    }
    let inv_leaf = (1.0 / leaf) as f32;
    let mut cells: std::collections::HashMap<(i64, i64, i64), ([f32; 3], u32)> =
        std::collections::HashMap::new();
    for p in cloud {
        if !p[0].is_finite() || !p[1].is_finite() || !p[2].is_finite() {
            continue;
        }
        let key = (
            (p[0] * inv_leaf).floor() as i64,
            (p[1] * inv_leaf).floor() as i64,
            (p[2] * inv_leaf).floor() as i64,
        );
        let entry = cells.entry(key).or_insert(([0.0; 3], 0));
        entry.0[0] += p[0];
        entry.0[1] += p[1];
        entry.0[2] += p[2];
        entry.1 += 1;
    }
    let mut keys: Vec<(i64, i64, i64)> = cells.keys().copied().collect();
    // Sort leaves by linear index i + j*div_x + k*div_x*div_y -> (z, y, x).
    keys.sort_by_key(|&(x, y, z)| (z, y, x));
    keys.iter()
        .map(|key| {
            let (sum, count) = cells[key];
            let n = count as f32;
            [sum[0] / n, sum[1] / n, sum[2] / n]
        })
        .collect()
}

/// Exact nearest-neighbour / k-NN over f32 points (median-split k-d tree, branch-and-bound queries).
struct KdNode {
    /// Index into the original point array.
    point: u32,
    axis: u8,
    left: i32,
    right: i32,
}

/// Exact 3-D kd-tree (median split, branch-and-bound queries). Distances are
/// reported as squared f32.
pub struct KdTree {
    points: Vec<[f32; 3]>,
    nodes: Vec<KdNode>,
    root: i32,
}

impl KdTree {
    pub fn build(points: &[[f32; 3]]) -> KdTree {
        let mut tree = KdTree {
            points: points.to_vec(),
            nodes: Vec::with_capacity(points.len()),
            root: -1,
        };
        let mut indices: Vec<u32> = (0..points.len() as u32).collect();
        tree.root = tree.build_recursive(&mut indices, 0);
        tree
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    fn build_recursive(&mut self, indices: &mut [u32], depth: usize) -> i32 {
        if indices.is_empty() {
            return -1;
        }
        let axis = (depth % 3) as u8;
        let mid = indices.len() / 2;
        indices.select_nth_unstable_by(mid, |&a, &b| {
            self.points[a as usize][axis as usize]
                .partial_cmp(&self.points[b as usize][axis as usize])
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let point = indices[mid];
        let node_index = self.nodes.len() as i32;
        self.nodes.push(KdNode {
            point,
            axis,
            left: -1,
            right: -1,
        });
        let (left_slice, rest) = indices.split_at_mut(mid);
        let right_slice = &mut rest[1..];
        let left = self.build_recursive(left_slice, depth + 1);
        let right = self.build_recursive(right_slice, depth + 1);
        self.nodes[node_index as usize].left = left;
        self.nodes[node_index as usize].right = right;
        node_index
    }

    /// Nearest neighbour: `(point_index, squared_distance)`.
    pub fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)> {
        if self.root < 0 {
            return None;
        }
        let mut best = (usize::MAX, f32::MAX);
        self.nearest_recursive(self.root, query, &mut best);
        if best.0 == usize::MAX {
            None
        } else {
            Some(best)
        }
    }

    fn nearest_recursive(&self, node_index: i32, query: &[f32; 3], best: &mut (usize, f32)) {
        let node = &self.nodes[node_index as usize];
        let p = &self.points[node.point as usize];
        let d = sq_dist(p, query);
        if d < best.1 {
            *best = (node.point as usize, d);
        }
        let axis = node.axis as usize;
        let delta = query[axis] - p[axis];
        let (near, far) = if delta < 0.0 {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };
        if near >= 0 {
            self.nearest_recursive(near, query, best);
        }
        if far >= 0 && delta * delta < best.1 {
            self.nearest_recursive(far, query, best);
        }
    }

    /// k nearest neighbours, ascending by squared distance (includes the
    /// query point itself when it is in the cloud). Distance ties on the k-th
    /// slot keep the earlier find (strict `dist < worstDist()` insertion).
    pub fn knn(&self, query: &[f32; 3], k: usize) -> Vec<(usize, f32)> {
        if self.root < 0 || k == 0 {
            return Vec::new();
        }
        // The k best (sq_dist, index), kept sorted ascending.
        let mut best: Vec<(f32, usize)> = Vec::with_capacity(k + 1);
        self.knn_recursive(self.root, query, k, &mut best);
        best.iter().map(|&(d, i)| (i, d)).collect()
    }

    fn knn_recursive(
        &self,
        node_index: i32,
        query: &[f32; 3],
        k: usize,
        best: &mut Vec<(f32, usize)>,
    ) {
        let node = &self.nodes[node_index as usize];
        let p = &self.points[node.point as usize];
        let d = sq_dist(p, query);
        if best.len() < k || d < best[best.len() - 1].0 {
            // Insert in order; equal distances go after existing entries.
            let pos = best.partition_point(|&(bd, _)| bd <= d);
            best.insert(pos, (d, node.point as usize));
            best.truncate(k);
        }
        let axis = node.axis as usize;
        let delta = query[axis] - p[axis];
        let (near, far) = if delta < 0.0 {
            (node.left, node.right)
        } else {
            (node.right, node.left)
        };
        if near >= 0 {
            self.knn_recursive(near, query, k, best);
        }
        let worst = if best.len() < k {
            f32::MAX
        } else {
            best[best.len() - 1].0
        };
        if far >= 0 && delta * delta <= worst {
            self.knn_recursive(far, query, k, best);
        }
    }
}

#[inline]
fn sq_dist(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    dx * dx + dy * dy + dz * dz
}
