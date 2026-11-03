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
    /// The same points in leaf buckets stored contiguously, for `nearest` (most of ICP's time): a point-per-node tree
    /// jumps to a scattered point on every visited node.
    buckets: BucketTree,
}

/// Points reordered so each leaf's (at most `BUCKET` of them) sit together; `original[i]` is `points[i]`'s index in
/// the cloud the tree was built from.
#[derive(Default)]
struct BucketTree {
    points: Vec<[f32; 3]>,
    original: Vec<u32>,
    nodes: Vec<BucketNode>,
}

/// An inner node splits on `axis` at `split` (children `left`, `right`); a leaf holds points[start..end].
struct BucketNode {
    axis: u8,
    split: f32,
    left: u32,
    right: u32,
    start: u32,
    end: u32,
}

const BUCKET: usize = 16;

impl BucketTree {
    fn build(points: &[[f32; 3]]) -> BucketTree {
        let mut indices: Vec<u32> = (0..points.len() as u32).collect();
        let mut tree = BucketTree::default();
        if !points.is_empty() {
            tree.build_node(points, &mut indices, 0);
        }
        tree.points = indices.iter().map(|&i| points[i as usize]).collect();
        tree.original = indices;
        tree
    }
    /// Build the node over indices[..], which start at `offset` in the final order; returns its index.
    fn build_node(&mut self, points: &[[f32; 3]], indices: &mut [u32], offset: usize) -> u32 {
        let node = self.nodes.len() as u32;
        self.nodes.push(BucketNode { axis: 0, split: 0.0, left: 0, right: 0, start: offset as u32, end: (offset + indices.len()) as u32 });
        if indices.len() <= BUCKET {
            return node;
        }
        // split the widest axis at its median
        let mut low = [f32::MAX; 3];
        let mut high = [f32::MIN; 3];
        for &i in indices.iter() {
            for k in 0..3 {
                low[k] = low[k].min(points[i as usize][k]);
                high[k] = high[k].max(points[i as usize][k]);
            }
        }
        let axis = (0..3).max_by(|&a, &b| (high[a] - low[a]).total_cmp(&(high[b] - low[b]))).unwrap_or(0);
        let mid = indices.len() / 2;
        indices.select_nth_unstable_by(mid, |&a, &b| points[a as usize][axis].total_cmp(&points[b as usize][axis]));
        let split = points[indices[mid] as usize][axis];
        let (left_half, right_half) = indices.split_at_mut(mid);
        let left = self.build_node(points, left_half, offset);
        let right = self.build_node(points, right_half, offset + mid);
        let n = &mut self.nodes[node as usize];
        (n.axis, n.split, n.left, n.right) = (axis as u8, split, left, right);
        node
    }
    /// Nearest point: (index in the built-from cloud, squared distance).
    fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)> {
        if self.nodes.is_empty() {
            return None;
        }
        let mut best = (u32::MAX, f32::MAX);
        // nodes still to visit, each with the squared distance from the query to its side of the parent's split
        let mut stack: [(u32, f32); 64] = [(0, 0.0); 64];
        let mut depth = 1;
        while depth > 0 {
            depth -= 1;
            let (index, bound) = stack[depth];
            if bound >= best.1 {
                continue;
            }
            let node = &self.nodes[index as usize];
            if node.left == 0 && node.right == 0 {
                for i in node.start..node.end {
                    let d = sq_dist(&self.points[i as usize], query);
                    if d < best.1 {
                        best = (i, d);
                    }
                }
                continue;
            }
            let delta = query[node.axis as usize] - node.split;
            let (near, far) = if delta < 0.0 { (node.left, node.right) } else { (node.right, node.left) };
            // the far side first, so the near side is searched next and tightens the bound
            stack[depth] = (far, delta * delta);
            stack[depth + 1] = (near, bound);
            depth += 2;
        }
        (best.0 != u32::MAX).then(|| (self.original[best.0 as usize] as usize, best.1))
    }
}

impl KdTree {
    pub fn build(points: &[[f32; 3]]) -> KdTree {
        let mut tree = KdTree {
            points: points.to_vec(),
            nodes: Vec::with_capacity(points.len()),
            root: -1,
            buckets: BucketTree::build(points),
        };
        let mut indices: Vec<u32> = (0..points.len() as u32).collect();
        tree.root = tree.build_recursive(&mut indices, 0);
        tree
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

    pub fn len(&self) -> usize {
        self.points.len()
    }
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Nearest neighbour: `(point_index, squared_distance)`.
    pub fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)> {
        self.buckets.nearest(query)
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

/// A nearest-neighbour index over a point array: one `KdTree`, or a `KdForest` that grows without rebuilding.
pub trait NearestIndex: Sync {
    /// Nearest neighbour: `(point_index, squared_distance)`.
    fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)>;
    /// k nearest neighbours, ascending by squared distance.
    fn knn(&self, query: &[f32; 3], k: usize) -> Vec<(usize, f32)>;
}

impl NearestIndex for KdTree {
    fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)> {
        KdTree::nearest(self, query)
    }
    fn knn(&self, query: &[f32; 3], k: usize) -> Vec<(usize, f32)> {
        KdTree::knn(self, query, k)
    }
}

/// A growing cloud and its index: appended points go into a new small k-d tree, and the two newest trees are merged
/// whenever the older is at most twice the newer (logarithmic method), so adding n points costs O(n log^2 n) in all and
/// a query searches O(log n) trees. Indices are positions in `points()`, in the order they were added.
#[derive(Default)]
pub struct KdForest {
    points: Vec<[f32; 3]>,
    /// (first point index, tree over points[first..first + size])
    trees: Vec<(usize, KdTree)>,
}

impl KdForest {
    pub fn points(&self) -> &[[f32; 3]] {
        &self.points
    }
    pub fn push(&mut self, new: &[[f32; 3]]) {
        if new.is_empty() {
            return;
        }
        let first = self.points.len();
        self.points.extend_from_slice(new);
        self.trees.push((first, KdTree::build(new)));
        while self.trees.len() >= 2 {
            let n = self.trees.len();
            let (older, newer) = (self.trees[n - 2].1.len(), self.trees[n - 1].1.len());
            if older > 2 * newer {
                break;
            }
            self.trees.pop();
            let (start, _) = self.trees.pop().unwrap();
            self.trees
                .push((start, KdTree::build(&self.points[start..])));
        }
    }
}

impl NearestIndex for KdForest {
    fn nearest(&self, query: &[f32; 3]) -> Option<(usize, f32)> {
        self.trees
            .iter()
            .filter_map(|(first, tree)| tree.nearest(query).map(|(i, d)| (first + i, d)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }
    fn knn(&self, query: &[f32; 3], k: usize) -> Vec<(usize, f32)> {
        let mut all: Vec<(usize, f32)> = self
            .trees
            .iter()
            .flat_map(|(first, tree)| {
                tree.knn(query, k)
                    .into_iter()
                    .map(move |(i, d)| (first + i, d))
            })
            .collect();
        all.sort_by(|a, b| a.1.total_cmp(&b.1));
        all.truncate(k);
        all
    }
}

#[inline]
fn sq_dist(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    dx * dx + dy * dy + dz * dz
}

#[cfg(test)]
mod bucket_tree_tests {
    use super::KdTree;

    /// The bucket tree finds the brute-force nearest distance, on a voxel grid's equal coordinates and far queries.
    #[test]
    fn nearest_matches_brute_force() {
        let mut seed = 12345u64;
        let mut next = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 33) as f32 / (1u64 << 31) as f32
        };
        let cloud: Vec<[f32; 3]> = (0..5000)
            .map(|_| [(next() * 40.0).round() * 0.1, (next() * 40.0).round() * 0.1, (next() * 10.0).round() * 0.1])
            .collect();
        let tree = KdTree::build(&cloud);
        for _ in 0..2000 {
            let q = [next() * 30.0 - 13.0, next() * 30.0 - 13.0, next() * 6.0 - 1.0];
            let (i, d) = tree.nearest(&q).unwrap();
            let brute = cloud.iter().map(|p| super::sq_dist(p, &q)).fold(f32::MAX, f32::min);
            assert_eq!(d, brute);
            assert_eq!(super::sq_dist(&cloud[i], &q), d);
        }
    }
}
