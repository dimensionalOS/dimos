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

//! Point-cloud machinery mirroring the PCL pieces simple_pgo.cpp leans on:
//!
//! - `voxel_downsample`: `pcl::VoxelGrid` semantics — points are binned on
//!   `floor(p / leaf)` (float math, like PCL) and each occupied voxel emits
//!   the CENTROID of its points (not the voxel center). Output is ordered by
//!   (z, y, x) cell index, matching PCL's leaf-index sort.
//! - `transform_cloud`: `pcl::transformPointCloud` with a double rotation +
//!   translation, storing back to f32 points.
//! - `KdTree`: exact nearest-neighbour / k-NN over f32 points (PCL uses
//!   FLANN's exact single-tree index; this is a hand-rolled median-split
//!   kd-tree with branch-and-bound, no external crates).
//! - `icp_point_to_point`: `pcl::IterativeClosestPoint<PointXYZI, PointXYZI>`
//!   (point-to-point, `TransformationEstimationSVD` = Eigen f32 Umeyama with
//!   a two-sided Jacobi SVD) with PCL's `DefaultConvergenceCriteria`
//!   semantics and the exact parameters simple_pgo.cpp sets: max 50
//!   iterations, max correspondence distance 10 m, transformation epsilon
//!   1e-6, euclidean fitness epsilon 1e-6. The whole ICP core runs on f32
//!   4x4 transforms with PCL/Eigen's per-coefficient evaluation order so the
//!   iterates match the C++ reference bit for bit; only the returned r/t are
//!   widened to f64 (mirroring `getFinalTransformation().cast<double>()`).
//!   Fitness score is PCL `getFitnessScore()`: mean *squared* NN distance
//!   over all source points (default max_range = DBL_MAX).
//! - `cloud_degeneracy`: the Zhang-2016/X-ICP normal-scatter observability
//!   measure from the anonymous namespace at the top of simple_pgo.cpp
//!   (PCL NormalEstimation with kSearch=10, then normalized eigenvalues of
//!   the sum of normal outer products).

// Index-based loops mirror the reference Eigen/PCL numeric kernels (diagonal
// access, symmetric r[i][j], dual-row column writes); keep the index form.
#![allow(clippy::needless_range_loop)]

use crate::mat3::{self, Mat3, Vec3};

/// Body/global-frame cloud: xyz as f32, like `pcl::PointXYZI` minus the
/// intensity (nothing in the PGO core reads intensity).
pub type PointCloud = Vec<[f32; 3]>;

/// `pcl::transformPointCloud(cloud, out, t, Quaterniond(r))`: p' = r*p + t.
pub fn transform_cloud(cloud: &[[f32; 3]], r: &Mat3, t: &Vec3) -> PointCloud {
    cloud
        .iter()
        .map(|p| {
            let v = [p[0] as f64, p[1] as f64, p[2] as f64];
            let out = mat3::add(&mat3::mat_vec(r, &v), t);
            [out[0] as f32, out[1] as f32, out[2] as f32]
        })
        .collect()
}

/// `pcl::VoxelGrid` with cubic leaf size: hash points into cells on
/// `floor(p * (1/leaf))` (f32 math, as PCL does), emit the centroid of each
/// occupied cell, ordered by (z, y, x) cell index like PCL's sorted leaf
/// layout.
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
    // PCL sorts leaves by linear index i + j*div_x + k*div_x*div_y -> (z, y, x).
    keys.sort_by_key(|&(x, y, z)| (z, y, x));
    keys.iter()
        .map(|key| {
            let (sum, count) = cells[key];
            let n = count as f32;
            [sum[0] / n, sum[1] / n, sum[2] / n]
        })
        .collect()
}

// ---------------------------------------------------------------------------
// kd-tree
// ---------------------------------------------------------------------------

struct KdNode {
    /// Index into the original point array.
    point: u32,
    axis: u8,
    left: i32,
    right: i32,
}

/// Exact 3-D kd-tree (median split, branch-and-bound queries). Distances are
/// squared f32, matching PCL/FLANN's reporting.
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
    /// query point itself when it is in the cloud, like PCL's kSearch).
    /// Distance ties on the k-th slot keep the earlier find, like FLANN's
    /// KNNResultSet (strict `dist < worstDist()` insertion).
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

// ---------------------------------------------------------------------------
// ICP
// ---------------------------------------------------------------------------

/// Result of `icp_point_to_point`, mirroring what simple_pgo.cpp reads off
/// `m_icp`: `hasConverged()`, `getFinalTransformation()` (r, t) and
/// `getFitnessScore()`.
#[derive(Debug, Clone)]
pub struct IcpResult {
    pub converged: bool,
    pub r: Mat3,
    pub t: Vec3,
    /// PCL `getFitnessScore()`: mean squared NN distance over all source
    /// points (f64::MAX when the target is empty).
    pub fitness: f64,
}

/// Parameters as simple_pgo.cpp configures `m_icp` (constructor, lines
/// 65-69): 50 iterations, 10 m correspondence gate, both epsilons 1e-6.
/// `rotation_threshold` and `mse_threshold_absolute` are the PCL
/// `DefaultConvergenceCriteria` defaults the C++ never overrides.
pub struct IcpParams {
    pub max_iterations: usize,
    pub max_correspondence_distance: f64,
    /// PCL `setTransformationEpsilon`, applied to the squared translation
    /// norm of the incremental transform (`translation_threshold_`).
    pub transformation_epsilon: f64,
    /// PCL `rotation_threshold_` default: converged when the incremental
    /// rotation's cos(angle) >= this (0.99999, ~0.256 deg).
    pub rotation_threshold: f64,
    /// PCL `mse_threshold_absolute_` default: converged when
    /// |cur_mse - prev_mse| < this.
    pub mse_threshold_absolute: f64,
    /// PCL `setEuclideanFitnessEpsilon` -> `mse_threshold_relative_`.
    pub euclidean_fitness_epsilon: f64,
}

impl Default for IcpParams {
    fn default() -> IcpParams {
        IcpParams {
            max_iterations: 50,
            max_correspondence_distance: 10.0,
            transformation_epsilon: 1e-6,
            rotation_threshold: 0.99999,
            mse_threshold_absolute: 1e-12,
            euclidean_fitness_epsilon: 1e-6,
        }
    }
}

// --- f32 4x4 transform plumbing, mirroring PCL/Eigen evaluation order ------

/// Row-major f32 4x4, standing in for `Eigen::Matrix4f`.
type Mat4f = [[f32; 4]; 4];

fn mat4_identity() -> Mat4f {
    let mut m = [[0.0f32; 4]; 4];
    for (i, row) in m.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    m
}

/// `final_transformation_ = transformation_ * final_transformation_`:
/// Eigen fixed-size 4x4 f32 product, per-coefficient left-associated
/// `((a0*b0 + a1*b1) + a2*b2) + a3*b3`.
fn mat4_mul(a: &Mat4f, b: &Mat4f) -> Mat4f {
    let mut out = [[0.0f32; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] =
                ((a[i][0] * b[0][j] + a[i][1] * b[1][j]) + a[i][2] * b[2][j]) + a[i][3] * b[3][j];
        }
    }
    out
}

/// `IterativeClosestPoint::transformCloud` (icp.hpp): per point,
/// `pt_t = tr * [x y z 1]` as an Eigen lazy product in f32 — evaluation
/// order `((x*c0 + y*c1) + z*c2) + c3` (w = 1, so `1*c3` is exactly `c3`).
/// Non-finite points are left untouched, like PCL's in-place skip.
fn icp_transform_cloud(cloud: &[[f32; 3]], m: &Mat4f) -> PointCloud {
    cloud
        .iter()
        .map(|p| {
            let [x, y, z] = *p;
            if !x.is_finite() || !y.is_finite() || !z.is_finite() {
                return *p;
            }
            [
                ((x * m[0][0] + y * m[0][1]) + z * m[0][2]) + m[0][3],
                ((x * m[1][0] + y * m[1][1]) + z * m[1][2]) + m[1][3],
                ((x * m[2][0] + y * m[2][1]) + z * m[2][2]) + m[2][3],
            ]
        })
        .collect()
}

/// `pcl::transformPointCloud` as used by `getFitnessScore`: PCL's SSE
/// `Transformer<float>::se3` folds the translation innermost —
/// `x*c0 + (y*c1 + (z*c2 + c3))` — deliberately DIFFERENT from the
/// ICP-internal `transformCloud` order above.
fn se3_transform_cloud(cloud: &[[f32; 3]], m: &Mat4f) -> PointCloud {
    cloud
        .iter()
        .map(|p| {
            let [x, y, z] = *p;
            [
                x * m[0][0] + (y * m[0][1] + (z * m[0][2] + m[0][3])),
                x * m[1][0] + (y * m[1][1] + (z * m[1][2] + m[1][3])),
                x * m[2][0] + (y * m[2][1] + (z * m[2][2] + m[2][3])),
            ]
        })
        .collect()
}

/// Point-to-point ICP mirroring `pcl::IterativeClosestPoint::align(out,
/// guess)` + `DefaultConvergenceCriteria`, numerically step-for-step with
/// PCL 1.14 `IterativeClosestPoint<PointXYZI, PointXYZI>` (Scalar = float):
///
/// - the working cloud, per-iteration delta and accumulated final transform
///   are all f32 (Matrix4f in PCL); the guess seeds the final transform and
///   pre-transforms the working cloud unless it is exactly identity;
/// - correspondences: for each (transformed) source point, the single NN in
///   the target, kept when the squared distance <= max_dist^2;
/// - rigid estimation: `TransformationEstimationSVD` = Eigen f32 Umeyama
///   (no scaling) with a two-sided Jacobi SVD of the 3x3 covariance;
/// - convergence, checked after each iteration in PCL's order: max
///   iterations reached (counts as converged, `failure_after_max_iter` is
///   false by default); incremental transform small (`cos_angle >=
///   rotation_threshold` AND `|t|^2 <= transformation_epsilon`, with
///   `max_iterations_similar_transforms = 0` so the first hit converges);
///   absolute then relative MSE change below `mse_threshold_absolute` /
///   `euclidean_fitness_epsilon`;
/// - aborts unconverged when fewer than 3 correspondences remain.
///
/// The returned r/t are the f32 final matrix entries widened to f64, like
/// `getFinalTransformation().cast<double>()` in simple_pgo.cpp. The f64
/// guess is narrowed to f32 on entry, so an f32-exact guess (as built by
/// the loop search) round-trips losslessly.
pub fn icp_point_to_point(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    guess_r: &Mat3,
    guess_t: &Vec3,
    params: &IcpParams,
) -> IcpResult {
    let mut result = IcpResult {
        converged: false,
        r: *guess_r,
        t: *guess_t,
        fitness: f64::MAX,
    };
    if target.is_empty() || source.is_empty() {
        return result;
    }
    let tree = KdTree::build(target);
    let max_dist_sqr = params.max_correspondence_distance * params.max_correspondence_distance;

    let mut guess = mat4_identity();
    for i in 0..3 {
        for j in 0..3 {
            guess[i][j] = guess_r[i][j] as f32;
        }
        guess[i][3] = guess_t[i] as f32;
    }

    // `align`: final_transformation_ = guess; pre-transform the working
    // cloud only when the guess is not exactly identity.
    let mut final_m = guess;
    let mut transformed = if guess == mat4_identity() {
        source.to_vec()
    } else {
        icp_transform_cloud(source, &guess)
    };

    let mut prev_mse = f64::MAX;
    let mut iterations = 0usize;

    let converged = loop {
        // Correspondence estimation (source -> single NN in target). The
        // queries are independent and read-only on the tree, so they run in
        // parallel; results are collected back in source order, keeping every
        // downstream f32 accumulation bit-identical to the serial loop.
        use rayon::prelude::*;
        let nn: Vec<Option<(usize, f32)>> =
            transformed.par_iter().map(|p| tree.nearest(p)).collect();
        let mut src_matched: Vec<[f32; 3]> = Vec::with_capacity(transformed.len());
        let mut tgt_matched: Vec<[f32; 3]> = Vec::with_capacity(transformed.len());
        let mut sq_dists: Vec<f32> = Vec::with_capacity(transformed.len());
        for (p, hit) in transformed.iter().zip(&nn) {
            if let Some((idx, d)) = *hit {
                if d as f64 > max_dist_sqr {
                    continue;
                }
                src_matched.push(*p);
                tgt_matched.push(target[idx]);
                sq_dists.push(d);
            }
        }
        if src_matched.len() < 3 {
            // PCL: CONVERGENCE_CRITERIA_NO_CORRESPONDENCES -> converged_ = false.
            break false;
        }

        // Rigid transformation estimation on the matched pairs.
        let delta = umeyama_rigid_f32(&src_matched, &tgt_matched);
        transformed = icp_transform_cloud(&transformed, &delta);
        final_m = mat4_mul(&delta, &final_m);
        iterations += 1;

        // --- DefaultConvergenceCriteria::hasConverged(), in PCL's order ----
        // 1. Iteration budget (counts as converged by default).
        if iterations >= params.max_iterations {
            break true;
        }
        // 2. Incremental transform similarity: the traces/sums are f32
        //    arithmetic on the Matrix4f coefficients (as in PCL), promoted
        //    to f64 only for the final comparison.
        let cos_angle = 0.5 * f64::from((delta[0][0] + delta[1][1] + delta[2][2]) - 1.0f32);
        let translation_sqr = f64::from(
            (delta[0][3] * delta[0][3] + delta[1][3] * delta[1][3]) + delta[2][3] * delta[2][3],
        );
        if cos_angle >= params.rotation_threshold
            && translation_sqr <= params.transformation_epsilon
        {
            break true;
        }
        // 3. MSE change on this iteration's correspondences: absolute
        //    first, then relative (PCL checks both; either converges).
        let cur_mse = sq_dists.iter().map(|&d| d as f64).sum::<f64>() / sq_dists.len() as f64;
        if (cur_mse - prev_mse).abs() < params.mse_threshold_absolute {
            break true;
        }
        if (cur_mse - prev_mse).abs() / prev_mse < params.euclidean_fitness_epsilon {
            break true;
        }
        prev_mse = cur_mse;
    };

    result.converged = converged;
    for i in 0..3 {
        for j in 0..3 {
            result.r[i][j] = f64::from(final_m[i][j]);
        }
        result.t[i] = f64::from(final_m[i][3]);
    }
    result.fitness = fitness_score(source, &tree, &final_m);
    result
}

/// PCL `Registration::getFitnessScore()` with the default `max_range =
/// DBL_MAX`: transform the ORIGINAL source by the f32 final transform
/// (`pcl::transformPointCloud`, se3 order), take each point's squared NN
/// distance in the target (f32), sum in f64, return the mean.
fn fitness_score(source: &[[f32; 3]], target_tree: &KdTree, final_m: &Mat4f) -> f64 {
    use rayon::prelude::*;
    let transformed = se3_transform_cloud(source, final_m);
    // Parallel NN, in-order collect: the f64 sum below runs serially in
    // source order, so the result is bit-identical to the serial loop.
    let nn: Vec<Option<(usize, f32)>> = transformed
        .par_iter()
        .map(|p| target_tree.nearest(p))
        .collect();
    let mut sum = 0.0f64;
    let mut count = 0usize;
    for hit in &nn {
        if let Some((_, d)) = *hit {
            sum += d as f64;
            count += 1;
        }
    }
    if count > 0 {
        sum / count as f64
    } else {
        f64::MAX
    }
}

// ---------------------------------------------------------------------------
// Eigen f32 Umeyama (TransformationEstimationSVD's backend)
// ---------------------------------------------------------------------------

/// `Eigen::JacobiRotation<float>`: the planar rotation
/// [[c, s], [-s, c]] used by the two-sided Jacobi SVD.
#[derive(Clone, Copy)]
struct JacobiRot {
    c: f32,
    s: f32,
}

impl JacobiRot {
    fn transpose(self) -> JacobiRot {
        JacobiRot {
            c: self.c,
            s: -self.s,
        }
    }

    /// `JacobiRotation::operator*` for real scalars.
    fn mul(self, other: JacobiRot) -> JacobiRot {
        JacobiRot {
            c: self.c * other.c - self.s * other.s,
            s: self.c * other.s + self.s * other.c,
        }
    }

    /// `JacobiRotation::makeJacobi(x, y, z)`: diagonalize the symmetric 2x2
    /// [[x, y], [y, z]] (Eigen 3.4, real path).
    fn make_jacobi(x: f32, y: f32, z: f32) -> JacobiRot {
        let deno = 2.0f32 * y.abs();
        if deno < f32::MIN_POSITIVE {
            return JacobiRot { c: 1.0, s: 0.0 };
        }
        let tau = (x - z) / deno;
        let w = (tau * tau + 1.0f32).sqrt();
        let t = if tau > 0.0f32 {
            1.0f32 / (tau + w)
        } else {
            1.0f32 / (tau - w)
        };
        let sign_t = if t > 0.0f32 { 1.0f32 } else { -1.0f32 };
        let n = 1.0f32 / (t * t + 1.0f32).sqrt();
        JacobiRot {
            c: n,
            s: -sign_t * (y / y.abs()) * t.abs() * n,
        }
    }
}

/// `apply_rotation_in_the_plane` on rows p/q of a square matrix, i.e.
/// Eigen's `m.applyOnTheLeft(p, q, j)` (scalar path — fixed size 3 is below
/// the packet width, so Eigen takes it too).
fn apply_rotation_rows<const N: usize>(m: &mut [[f32; N]; N], p: usize, q: usize, j: JacobiRot) {
    for i in 0..N {
        let x = m[p][i];
        let y = m[q][i];
        m[p][i] = j.c * x + j.s * y;
        m[q][i] = -j.s * x + j.c * y;
    }
}

/// Eigen's `m.applyOnTheRight(p, q, j)`: the kernel runs on columns p/q
/// with `j.transpose()`.
fn apply_rotation_cols<const N: usize>(m: &mut [[f32; N]; N], p: usize, q: usize, j: JacobiRot) {
    let jt = j.transpose();
    for row in m.iter_mut() {
        let x = row[p];
        let y = row[q];
        row[p] = jt.c * x + jt.s * y;
        row[q] = -jt.s * x + jt.c * y;
    }
}

/// `Eigen::internal::real_2x2_jacobi_svd` on the (p,q) 2x2 block.
fn real_2x2_jacobi_svd(block: [[f32; 2]; 2]) -> (JacobiRot, JacobiRot) {
    let mut m = block;
    let t = m[0][0] + m[1][1];
    let d = m[1][0] - m[0][1];
    let rot1 = if d.abs() < f32::MIN_POSITIVE {
        JacobiRot { c: 1.0, s: 0.0 }
    } else {
        // If d != 0, t/d cannot overflow (Eigen's reasoning).
        let u = t / d;
        let tmp = (1.0f32 + u * u).sqrt();
        JacobiRot {
            c: u / tmp,
            s: 1.0f32 / tmp,
        }
    };
    apply_rotation_rows(&mut m, 0, 1, rot1);
    let j_right = JacobiRot::make_jacobi(m[0][0], m[0][1], m[1][1]);
    let j_left = rot1.mul(j_right.transpose());
    (j_left, j_right)
}

/// `Eigen::JacobiSVD<Matrix3f>(m, ComputeFullU | ComputeFullV)`: two-sided
/// Jacobi SVD, ported statement-for-statement from Eigen 3.4 (square path,
/// no QR preconditioning). Returns (U, V) with singular values sorted
/// descending (the values themselves are not needed by Umeyama).
fn jacobi_svd_3x3(a: &[[f32; 3]; 3]) -> ([[f32; 3]; 3], [[f32; 3]; 3]) {
    let precision = 2.0f32 * f32::EPSILON;
    let consider_as_zero = f32::MIN_POSITIVE;

    // Scale by the largest |coefficient| to reduce over/under-flow.
    let mut scale = 0.0f32;
    for row in a {
        for &value in row {
            scale = scale.max(value.abs());
        }
    }
    if scale == 0.0f32 {
        scale = 1.0f32;
    }
    let mut work = *a;
    for row in &mut work {
        for value in row.iter_mut() {
            *value /= scale;
        }
    }
    let mut u = [[0.0f32; 3]; 3];
    let mut v = [[0.0f32; 3]; 3];
    for i in 0..3 {
        u[i][i] = 1.0;
        v[i][i] = 1.0;
    }

    let mut max_diag = work[0][0].abs().max(work[1][1].abs()).max(work[2][2].abs());
    let mut finished = false;
    while !finished {
        finished = true;
        // Sweep index pairs (p,q) = (1,0), (2,0), (2,1), like Eigen's
        // `for p in 1..size { for q in 0..p }`.
        for p in 1..3 {
            for q in 0..p {
                let threshold = consider_as_zero.max(precision * max_diag);
                if work[p][q].abs() > threshold || work[q][p].abs() > threshold {
                    finished = false;
                    let block = [[work[p][p], work[p][q]], [work[q][p], work[q][q]]];
                    let (j_left, j_right) = real_2x2_jacobi_svd(block);
                    apply_rotation_rows(&mut work, p, q, j_left);
                    apply_rotation_cols(&mut u, p, q, j_left.transpose());
                    apply_rotation_cols(&mut work, p, q, j_right);
                    apply_rotation_cols(&mut v, p, q, j_right);
                    max_diag = max_diag.max(work[p][p].abs().max(work[q][q].abs()));
                }
            }
        }
    }

    // Make the diagonal positive (flip the U column for negative entries).
    let mut sing = [0.0f32; 3];
    for i in 0..3 {
        let diag = work[i][i];
        sing[i] = diag.abs();
        if diag < 0.0f32 {
            for row in &mut u {
                row[i] = -row[i];
            }
        }
    }
    for value in &mut sing {
        *value *= scale;
    }

    // Sort singular values descending, permuting U/V columns along.
    for i in 0..3 {
        let mut pos = 0usize;
        let mut best = sing[i];
        for (k, &value) in sing[i..].iter().enumerate().skip(1) {
            if value > best {
                best = value;
                pos = k;
            }
        }
        if best == 0.0f32 {
            break;
        }
        if pos != 0 {
            let pos = pos + i;
            sing.swap(i, pos);
            for row in 0..3 {
                u[row].swap(i, pos);
                v[row].swap(i, pos);
            }
        }
    }
    (u, v)
}

/// Eigen's 3x3 determinant (`bruteforce_det3_helper` expansion) in f32.
fn det3(m: &[[f32; 3]; 3]) -> f32 {
    let helper =
        |a: usize, b: usize, c: usize| -> f32 { m[0][a] * (m[1][b] * m[2][c] - m[1][c] * m[2][b]) };
    (helper(0, 1, 2) - helper(1, 0, 2)) + helper(2, 0, 1)
}

/// Eigen's L1 data-cache size as `manage_caching_sizes` sees it: CPUID
/// deterministic cache enumeration (leaf 4 on Intel, 0x80000005 on AMD),
/// falling back to Eigen's 32 KiB x86 default. Cached per process, like
/// Eigen's function-local static. NOTE: on hybrid CPUs (P/E cores) the
/// CPUID answer depends on which core the first query runs on — the same
/// caveat applies to the C++ side.
fn eigen_l1_cache_size() -> usize {
    static L1: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *L1.get_or_init(|| {
        let queried = query_l1_cache_size();
        if queried > 0 {
            queried as usize
        } else {
            32 * 1024
        }
    })
}

#[cfg(target_arch = "x86_64")]
fn query_l1_cache_size() -> i64 {
    use std::arch::x86_64::{__cpuid, __cpuid_count};
    let vendor = __cpuid(0);
    let is_amd = (vendor.ebx, vendor.edx, vendor.ecx) == (0x6874_7541, 0x6974_6e65, 0x444d_4163)
        || (vendor.ebx, vendor.edx, vendor.ecx) == (0x6944_4d41, 0x7465_6273, 0x2172_6574);
    if is_amd {
        // Eigen queryCacheSizes_amd: leaf 0x80000005, ECX[31:24] = L1d KiB.
        let max_ext = __cpuid(0x8000_0000).eax;
        if max_ext >= 0x8000_0006 {
            let leaf = __cpuid(0x8000_0005);
            return i64::from(leaf.ecx >> 24) * 1024;
        }
        return 0;
    }
    // Eigen queryCacheSizes_intel_direct (leaf 4 enumeration), also the
    // default for unknown vendors when max_std_funcs >= 4.
    if vendor.eax < 4 {
        return 0;
    }
    let mut l1 = 0i64;
    for cache_id in 0..16 {
        let abcd = __cpuid_count(4, cache_id);
        let cache_type = abcd.eax & 0x0f;
        if cache_type == 0 {
            break;
        }
        if cache_type == 1 || cache_type == 3 {
            // Data or unified cache.
            let cache_level = (abcd.eax & 0xe0) >> 5;
            let ways = i64::from((abcd.ebx & 0xffc0_0000) >> 22);
            let partitions = i64::from((abcd.ebx & 0x003f_f000) >> 12);
            let line_size = i64::from(abcd.ebx & 0x0000_0fff);
            let sets = i64::from(abcd.ecx);
            if cache_level == 1 {
                l1 = (ways + 1) * (partitions + 1) * (line_size + 1) * (sets + 1);
            }
        }
    }
    l1
}

#[cfg(not(target_arch = "x86_64"))]
fn query_l1_cache_size() -> i64 {
    -1
}

/// The GEMM depth blocking Eigen applies to `dst_demean * src_demean^T`
/// (3xK times Kx3): `evaluateProductBlockingSizesHeuristic<float, float, 1>`
/// with m = n = 3, single thread, SSE gebp traits (mr = 8, nr = 4). The
/// covariance accumulates per depth panel of this size, with alpha applied
/// per panel — which changes the f32 rounding once K exceeds max_kc.
fn eigen_gemm_kc(depth: usize) -> usize {
    // if (max(k, max(m,n)) < 48) return; -- with m = n = 3.
    if depth < 48 {
        return depth;
    }
    let k_peeling = 8usize;
    let k_div = 8 * 4 + 4 * 4; // mr*sizeof(f32) + nr*sizeof(f32)
    let k_sub = 8 * 4 * 4; // mr*nr*sizeof(f32)
    let l1 = eigen_l1_cache_size();
    let max_kc = ((l1.saturating_sub(k_sub) / k_div) & !(k_peeling - 1)).max(1);
    if depth > max_kc {
        if depth.is_multiple_of(max_kc) {
            max_kc
        } else {
            max_kc
                - k_peeling * ((max_kc - 1 - (depth % max_kc)) / (k_peeling * (depth / max_kc + 1)))
        }
    } else {
        depth
    }
}

/// `Eigen::umeyama(src, dst, false)` for f32 3xN point sets (what PCL's
/// `TransformationEstimationSVD` calls on the matched correspondences):
/// demean, sigma = 1/n * dst_demean * src_demean^T, Jacobi SVD, det-sign
/// fix on the last singular direction, r = U * diag(1,1,s2) * V^T,
/// t = dst_mean - r * src_mean. All f32; the means accumulate sequentially
/// (Eigen redux over a strided row is not vectorized) and sigma follows
/// Eigen's GEMM depth panels (verified bit-exact against Eigen 3.4).
fn umeyama_rigid_f32(src: &[[f32; 3]], dst: &[[f32; 3]]) -> Mat4f {
    let n = src.len();
    let one_over_n = 1.0f32 / n as f32;

    let mut src_sum = [0.0f32; 3];
    let mut dst_sum = [0.0f32; 3];
    for (s, d) in src.iter().zip(dst.iter()) {
        for i in 0..3 {
            src_sum[i] += s[i];
            dst_sum[i] += d[i];
        }
    }
    let mut src_mean = [0.0f32; 3];
    let mut dst_mean = [0.0f32; 3];
    for i in 0..3 {
        src_mean[i] = src_sum[i] * one_over_n;
        dst_mean[i] = dst_sum[i] * one_over_n;
    }

    // sigma = one_over_n * dst_demean * src_demean^T, accumulated the way
    // Eigen's gebp kernel does for a 3x3 result: sequentially within each
    // depth panel of kc points, then res += alpha * panel_sum per panel.
    let kc = eigen_gemm_kc(n);
    let mut sigma = [[0.0f32; 3]; 3];
    let mut panel_start = 0usize;
    while panel_start < n {
        let panel_end = (panel_start + kc).min(n);
        let mut acc = [[0.0f32; 3]; 3];
        for k in panel_start..panel_end {
            let s = &src[k];
            let d = &dst[k];
            let sd = [s[0] - src_mean[0], s[1] - src_mean[1], s[2] - src_mean[2]];
            let dd = [d[0] - dst_mean[0], d[1] - dst_mean[1], d[2] - dst_mean[2]];
            for i in 0..3 {
                for j in 0..3 {
                    acc[i][j] += dd[i] * sd[j];
                }
            }
        }
        for i in 0..3 {
            for j in 0..3 {
                sigma[i][j] += one_over_n * acc[i][j];
            }
        }
        panel_start = panel_end;
    }

    let (u, v) = jacobi_svd_3x3(&sigma);
    let s2 = if det3(&u) * det3(&v) < 0.0f32 {
        -1.0f32
    } else {
        1.0f32
    };

    // r = U * diag(1, 1, s2) * V^T: a fixed-size 3x3 Eigen product, whose
    // per-coefficient sum is the unrolled binary-tree redux t0 + (t1 + t2).
    let mut out = mat4_identity();
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = u[i][0] * v[j][0] + (u[i][1] * v[j][1] + (u[i][2] * s2) * v[j][2]);
        }
    }
    // t = dst_mean - r * src_mean: Eigen evaluates `topLeftCorner(m,m) *
    // src_mean` with runtime-sized m, so the redux is NOT unrolled here —
    // plain sequential (t0 + t1) + t2.
    for i in 0..3 {
        out[i][3] = dst_mean[i]
            - ((out[i][0] * src_mean[0] + out[i][1] * src_mean[1]) + out[i][2] * src_mean[2]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::umeyama_rigid_f32;

    /// Bit-exact golden test against `Eigen::umeyama(src, dst, false)` for
    /// `Matrix<float, 3, Dynamic>` (Eigen 3.4.1, g++ -O2, x86-64 baseline).
    /// The whole pipeline (demeaning, covariance, Jacobi SVD, sign fix) uses
    /// only IEEE-754-exact operations (+ - * / sqrt), so these bits are
    /// platform-portable. Regenerate with the C++ snippet in the commit
    /// message if the scenario changes.
    #[test]
    fn umeyama_matches_eigen_bit_for_bit() {
        let mut state = 42u64;
        let mut next = move || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((state >> 40) as f32) / (1u32 << 24) as f32
        };
        let n = 257;
        let mut src: Vec<[f32; 3]> = Vec::with_capacity(n);
        let mut dst: Vec<[f32; 3]> = Vec::with_capacity(n);
        let (c, s) = (0.9689124f32, 0.24740396f32); // cos/sin 0.25
        for _ in 0..n {
            let x = next() * 10.0f32 - 5.0f32;
            let y = next() * 10.0f32 - 5.0f32;
            let z = next() * 2.0f32;
            src.push([x, y, z]);
            dst.push([
                c * x - s * y + 0.35f32 + 0.01f32 * next(),
                s * x + c * y - 0.25f32 + 0.01f32 * next(),
                z + 0.15f32 + 0.01f32 * next(),
            ]);
        }
        let rt = umeyama_rigid_f32(&src, &dst);
        let expected: [[u32; 4]; 4] = [
            [0x3f780aad, 0xbe7d56e1, 0xb8546600, 0x3eb5a036],
            [0x3e7d56e2, 0x3f780aad, 0x3927ce51, 0xbe7b1994],
            [0x371ef000, 0xb92fb9e0, 0x3f7ffffe, 0x3e1ec4e8],
            [0x00000000, 0x00000000, 0x00000000, 0x3f800000],
        ];
        for i in 0..4 {
            for j in 0..4 {
                assert_eq!(
                    rt[i][j].to_bits(),
                    expected[i][j],
                    "entry ({i},{j}): got {:e} (0x{:08x}), Eigen has 0x{:08x}",
                    rt[i][j],
                    rt[i][j].to_bits(),
                    expected[i][j]
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Normal estimation + degeneracy
// ---------------------------------------------------------------------------

/// `Eigen::NumTraits<float>::epsilon()`, the branch threshold PCL's f32
/// eigen-solver uses everywhere.
const F32_EPS: f32 = f32::EPSILON;

// The C library's float transcendentals, NOT Rust std's: `f32::atan2` et al.
// use Rust's own libm port, which can be 1 ulp away from glibc >= 2.40's
// correctly-rounded implementations. The C++ gsc_pgo binary resolves these
// from the glibc it links (nix glibc 2.42), so bit-parity requires calling
// the very same symbols.
unsafe extern "C" {
    fn atan2f(y: f32, x: f32) -> f32;
    fn sinf(x: f32) -> f32;
    fn cosf(x: f32) -> f32;
}

/// `pcl::computeMeanAndCovarianceMatrix` (common/impl/centroid.hpp, indices
/// overload, dense cloud path) with `Scalar = float`: SINGLE-PASS f32
/// accumulation of the 9 moments, shifted by K = the first finite indexed
/// point (the query point itself, since FLANN returns it first at distance
/// 0). The accumulation order over `indices` and every intermediate f32
/// rounding matter — this reproduces PCL's cancellation warts bit for bit.
fn pcl_mean_and_covariance_f32(cloud: &[[f32; 3]], indices: &[(usize, f32)]) -> [[f32; 3]; 3] {
    let mut k = [0.0f32; 3];
    for &(idx, _) in indices {
        let p = &cloud[idx];
        if p[0].is_finite() && p[1].is_finite() && p[2].is_finite() {
            k = *p;
            break;
        }
    }
    let mut accu = [0.0f32; 9];
    for &(idx, _) in indices {
        let x = cloud[idx][0] - k[0];
        let y = cloud[idx][1] - k[1];
        let z = cloud[idx][2] - k[2];
        accu[0] += x * x;
        accu[1] += x * y;
        accu[2] += x * z;
        accu[3] += y * y;
        accu[4] += y * z;
        accu[5] += z * z;
        accu[6] += x;
        accu[7] += y;
        accu[8] += z;
    }
    let point_count = indices.len() as f32;
    for a in &mut accu {
        *a /= point_count;
    }
    let mut cov = [[0.0f32; 3]; 3];
    cov[0][0] = accu[0] - accu[6] * accu[6];
    cov[0][1] = accu[1] - accu[6] * accu[7];
    cov[0][2] = accu[2] - accu[6] * accu[8];
    cov[1][1] = accu[3] - accu[7] * accu[7];
    cov[1][2] = accu[4] - accu[7] * accu[8];
    cov[2][2] = accu[5] - accu[8] * accu[8];
    cov[1][0] = cov[0][1];
    cov[2][0] = cov[0][2];
    cov[2][1] = cov[1][2];
    cov
}

/// `pcl::computeRoots2` (common/impl/eigen.hpp): quadratic fallback when one
/// root is (near) zero. Note the PCL quirk: `b*b - 4.0*c` promotes to f64
/// because of the `4.0` literal, then narrows back to f32.
fn pcl_compute_roots2_f32(b: f32, c: f32) -> [f32; 3] {
    let mut d = ((b * b) as f64 - 4.0 * c as f64) as f32;
    if d < 0.0 {
        d = 0.0; // no real roots! THIS SHOULD NOT HAPPEN!
    }
    let sd = d.sqrt();
    [0.0, 0.5 * (b - sd), 0.5 * (b + sd)]
}

/// `pcl::computeRoots` (common/impl/eigen.hpp) in f32: closed-form roots of
/// the characteristic cubic of a symmetric 3x3, ported branch-for-branch
/// (including the clamps on `a_over_3`/`q`, the trig root formulas and the
/// final sort network). `sin`/`cos`/`atan2` on f32 hit the same libm as the
/// C++ build, so results are bit-identical on this platform.
fn pcl_compute_roots_f32(m: &[[f32; 3]; 3]) -> [f32; 3] {
    let c0 = m[0][0] * m[1][1] * m[2][2] + 2.0 * m[0][1] * m[0][2] * m[1][2]
        - m[0][0] * m[1][2] * m[1][2]
        - m[1][1] * m[0][2] * m[0][2]
        - m[2][2] * m[0][1] * m[0][1];
    let c1 = m[0][0] * m[1][1] - m[0][1] * m[0][1] + m[0][0] * m[2][2] - m[0][2] * m[0][2]
        + m[1][1] * m[2][2]
        - m[1][2] * m[1][2];
    let c2 = m[0][0] + m[1][1] + m[2][2];

    if c0.abs() < F32_EPS {
        // one root is 0 -> quadratic equation
        return pcl_compute_roots2_f32(c2, c1);
    }
    let s_inv3 = 1.0f32 / 3.0f32;
    let s_sqrt3 = 3.0f32.sqrt();
    let c2_over_3 = c2 * s_inv3;
    let mut a_over_3 = (c1 - c2 * c2_over_3) * s_inv3;
    if a_over_3 > 0.0 {
        a_over_3 = 0.0;
    }
    let half_b = 0.5 * (c0 + c2_over_3 * (2.0 * c2_over_3 * c2_over_3 - c1));
    let mut q = half_b * half_b + a_over_3 * a_over_3 * a_over_3;
    if q > 0.0 {
        q = 0.0;
    }
    let rho = (-a_over_3).sqrt();
    let theta = unsafe { atan2f((-q).sqrt(), half_b) } * s_inv3;
    let cos_theta = unsafe { cosf(theta) };
    let sin_theta = unsafe { sinf(theta) };
    let mut roots = [
        c2_over_3 + 2.0 * rho * cos_theta,
        c2_over_3 - rho * (cos_theta + s_sqrt3 * sin_theta),
        c2_over_3 - rho * (cos_theta - s_sqrt3 * sin_theta),
    ];
    // Sort in increasing order.
    if roots[0] >= roots[1] {
        roots.swap(0, 1);
    }
    if roots[1] >= roots[2] {
        roots.swap(1, 2);
        if roots[0] >= roots[1] {
            roots.swap(0, 1);
        }
    }
    if roots[0] <= 0.0 {
        // eigenval of a symmetric PSD matrix can not be negative! Set it to 0
        return pcl_compute_roots2_f32(c2, c1);
    }
    roots
}

/// Eigen `Vector3f::cross` coefficient order.
fn cross_f32(a: &[f32; 3], b: &[f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Eigen `Vector3f::norm()`: sqrt of the unrolled-redux squared norm, which
/// sums as `x^2 + (y^2 + z^2)` (NOT left-to-right — Eigen's fixed-size redux
/// is a binary tree). Matters for the last-ulp of near-degenerate normals.
fn norm3_f32(v: &[f32; 3]) -> f32 {
    (v[0] * v[0] + (v[1] * v[1] + v[2] * v[2])).sqrt()
}

/// `pcl::detail::getLargest3x3Eigenvector` (common/impl/eigen.hpp): pick the
/// largest-magnitude cross product of row pairs of (scaledMat - eval*I) and
/// scale it by its norm. Ties keep the first row, like Eigen's `maxCoeff`
/// visitor (strict `>`). The result is `row / norm` — NOT re-normalized.
fn pcl_largest_3x3_eigenvector(scaled: &[[f32; 3]; 3]) -> [f32; 3] {
    let rows = [
        cross_f32(&scaled[0], &scaled[1]),
        cross_f32(&scaled[0], &scaled[2]),
        cross_f32(&scaled[1], &scaled[2]),
    ];
    let len = [
        norm3_f32(&rows[0]),
        norm3_f32(&rows[1]),
        norm3_f32(&rows[2]),
    ];
    let mut index = 0usize;
    for i in 1..3 {
        if len[i] > len[index] {
            index = i;
        }
    }
    let length = len[index];
    [
        rows[index][0] / length,
        rows[index][1] / length,
        rows[index][2] / length,
    ]
}

/// Eigen `Vector3f::unitOrthogonal()` (Geometry/OrthoMethods.h), used by
/// eigen33's equal-eigenvalue branch. `is_much_smaller` is Eigen's
/// `isMuchSmallerThan` with float `dummy_precision()` = 1e-5.
fn pcl_unit_orthogonal_f32(v: &[f32; 3]) -> [f32; 3] {
    let prec = 1e-5f32;
    let is_much_smaller = |x: f32, y: f32| x.abs() <= y.abs() * prec;
    if !is_much_smaller(v[0], v[2]) || !is_much_smaller(v[1], v[2]) {
        let invnm = 1.0 / (v[0] * v[0] + v[1] * v[1]).sqrt();
        [-v[1] * invnm, v[0] * invnm, 0.0]
    } else {
        let invnm = 1.0 / (v[1] * v[1] + v[2] * v[2]).sqrt();
        [0.0, -v[2] * invnm, v[1] * invnm]
    }
}

/// `pcl::eigen33(mat, eigenvalue, eigenvector)` (common/impl/eigen.hpp) in
/// f32: scale the matrix by its max-abs coefficient, take the closed-form
/// smallest root, and recover the eigenvector from row cross products, with
/// the two equal-eigenvalue fallback branches. Returns the (possibly
/// non-finite) smallest eigenvector, exactly as PCL leaves it.
fn pcl_eigen33_smallest_f32(mat: &[[f32; 3]; 3]) -> [f32; 3] {
    // Scale the matrix so its entries are in [-1,1].
    let mut scale = 0.0f32;
    for row in mat {
        for &v in row {
            scale = scale.max(v.abs());
        }
    }
    if scale <= f32::MIN_POSITIVE {
        scale = 1.0;
    }
    let mut scaled = *mat;
    for row in &mut scaled {
        for v in row.iter_mut() {
            *v /= scale;
        }
    }
    let evals = pcl_compute_roots_f32(&scaled);
    if evals[1] - evals[0] > F32_EPS {
        // usual case: first and second are not equal
        for i in 0..3 {
            scaled[i][i] -= evals[0];
        }
        pcl_largest_3x3_eigenvector(&scaled)
    } else if evals[2] - evals[0] > F32_EPS {
        // first and second equal: any unit vector orthogonal to the third
        for i in 0..3 {
            scaled[i][i] -= evals[2];
        }
        pcl_unit_orthogonal_f32(&pcl_largest_3x3_eigenvector(&scaled))
    } else {
        // all three equal: just use an arbitrary unit vector
        [1.0, 0.0, 0.0]
    }
}

/// `pcl::NormalEstimation<PointXYZI, Normal>` with `setKSearch(k)` and the
/// default FLANN kd-tree, ported to match PCL 1.15.1 bit for bit: exact
/// sorted k-NN (query point first at distance 0), single-pass f32
/// covariance (`computeMeanAndCovarianceMatrix`), closed-form f32 smallest
/// eigenvector (`solvePlaneParameters` -> `eigen33`), then
/// `flipNormalTowardsViewpoint` with the default viewpoint (0,0,0). Normals
/// are NaN when the neighbourhood is too small, like PCL's dense-cloud NaN
/// marker.
pub fn estimate_normals(cloud: &[[f32; 3]], k: usize) -> Vec<[f32; 3]> {
    use rayon::prelude::*;
    let tree = KdTree::build(cloud);
    // Each point's normal is independent (read-only tree queries), so they
    // run in parallel; the in-order collect keeps the caller's serial f64
    // scatter accumulation bit-identical to the serial loop.
    cloud
        .par_iter()
        .map(|p| {
            let neighbors = tree.knn(p, k);
            if neighbors.len() < 3 {
                return [f32::NAN, f32::NAN, f32::NAN];
            }
            let cov = pcl_mean_and_covariance_f32(cloud, &neighbors);
            let mut n = pcl_eigen33_smallest_f32(&cov);
            // flipNormalTowardsViewpoint(point, 0, 0, 0, nx, ny, nz)
            let cos_theta = (0.0 - p[0]) * n[0] + (0.0 - p[1]) * n[1] + (0.0 - p[2]) * n[2];
            if cos_theta < 0.0 {
                n[0] *= -1.0;
                n[1] *= -1.0;
                n[2] *= -1.0;
            }
            n
        })
        .collect()
}

/// Geometric degeneracy of a scan, the exact port of the anonymous-namespace
/// `cloud_degeneracy` at the top of simple_pgo.cpp: estimate per-point
/// normals (PCL NormalEstimation, kSearch = 10 -> smallest eigenvector of
/// the neighbourhood covariance), accumulate the normal scatter M = sum n
/// n^T in f64 over the finite f32 normals (no re-normalization — PCL's
/// eigen33 vector is taken as-is), and report the two smaller normalized
/// eigenvalues (e_min <= e_mid, as fractions of the trace). Returns (-1, -1)
/// when the cloud has fewer than 20 points or fewer than 20 valid normals.
pub fn cloud_degeneracy(cloud: &[[f32; 3]]) -> (f32, f32) {
    if cloud.len() < 20 {
        return (-1.0, -1.0);
    }
    let mut scatter = [[0.0f64; 3]; 3];
    let mut valid = 0i64;
    for nrm in estimate_normals(cloud, 10) {
        if !nrm[0].is_finite() || !nrm[1].is_finite() || !nrm[2].is_finite() {
            continue;
        }
        let n = [nrm[0] as f64, nrm[1] as f64, nrm[2] as f64];
        for i in 0..3 {
            for j in 0..3 {
                scatter[i][j] += n[i] * n[j];
            }
        }
        valid += 1;
    }
    if valid < 20 {
        return (-1.0, -1.0);
    }
    let (values, _) = mat3::jacobi_eigen(scatter);
    let trace = values[0] + values[1] + values[2];
    if trace <= 0.0 {
        return (-1.0, -1.0);
    }
    ((values[0] / trace) as f32, (values[1] / trace) as f32)
}
