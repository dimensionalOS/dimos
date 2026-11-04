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

//! Stage 3, connection fit quality: how well a connection's two places line up without the graph (ICP of the
//! source scan onto the target submap, its fitness and likelihood ratio).

use rayon::prelude::*;

use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, KdTree, NearestIndex};

/// (moved source point, nearest target point, squared distance, target index).
type Correspondence = ([f32; 3], [f32; 3], f32, usize);

pub struct IcpParams {
    pub max_iterations: usize,
    /// Meters; farther pairs are not correspondences.
    pub max_correspondence_distance: f64,
    /// m²; a source point within this of the target overlaps it.
    pub inlier_squared_distance: f64,
    /// Minimise distance along the target normal instead of point-to-point, letting points slide along surfaces.
    pub point_to_plane: bool,
}

/// Why an ICP stopped iterating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcpStop {
    Converged,
    /// The fit could no longer get under the gate's fitness (`icp_with_early_stop`).
    EarlyStop,
    /// It ran `max_iterations` (reported as converged).
    MaxIterations,
    /// Fewer than three correspondences, or no step could be solved for.
    Diverged,
}

impl IcpStop {
    pub fn name(self) -> &'static str {
        match self {
            IcpStop::Converged => "converged",
            IcpStop::EarlyStop => "early_stop",
            IcpStop::MaxIterations => "max_iterations",
            IcpStop::Diverged => "diverged",
        }
    }
    pub fn code(self) -> u8 {
        self as u8
    }
    pub fn from_code(code: u8) -> Self {
        [
            IcpStop::Converged,
            IcpStop::EarlyStop,
            IcpStop::MaxIterations,
            IcpStop::Diverged,
        ]
        .get(code as usize)
        .copied()
        .unwrap_or(IcpStop::Diverged)
    }
}

#[derive(Debug, Clone)]
pub struct IcpResult {
    pub converged: bool,
    pub iterations: u32,
    pub stop: IcpStop,
    /// Target points ICP paired against (thinned in `icp_thinned`; 0 from an old cache entry).
    pub target_points: usize,
    /// Moves the source onto the target.
    pub rotation: Mat3,
    pub translation: Vec3,
    /// Overlapping points' mean squared NN distance / `overlap` (m², lower is better; f64::MAX when nothing overlaps).
    pub fitness: f64,
    /// Fraction of source points overlapping the target (0..1).
    pub overlap: f64,
}

/// Align `source` onto `target` (indexed by `tree`) from the guess; not converged if < 3 correspondences remain.
pub fn icp(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    tree: &impl NearestIndex,
    rotation: &Mat3,
    translation: &Vec3,
    params: &IcpParams,
) -> IcpResult {
    icp_with_early_stop(source, target, tree, rotation, translation, params, None)
}

/// `icp` that gives up when, extrapolating its improvement since settling, fitness cannot reach `fail_fitness` (m²).
pub fn icp_with_early_stop(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    tree: &impl NearestIndex,
    rotation: &Mat3,
    translation: &Vec3,
    params: &IcpParams,
    fail_fitness: Option<f64>,
) -> IcpResult {
    let (mut rotation, mut translation) = (*rotation, *translation);
    let mut fitness_history: Vec<f64> = Vec::new();
    let mut settled_from: Option<usize> = None;
    let mut converged = false;
    let (mut iterations, mut stop) = (0u32, IcpStop::Diverged);
    let limit = (params.max_correspondence_distance * params.max_correspondence_distance) as f32;
    let mut previous_mse = f64::MAX;
    // target surface normals, found the first time a pair uses the point
    let normals: Vec<std::sync::OnceLock<Option<[f64; 3]>>> = if params.point_to_plane {
        (0..target.len())
            .map(|_| std::sync::OnceLock::new())
            .collect()
    } else {
        Vec::new()
    };
    let centroid = source
        .iter()
        .fold([0.0f64; 3], |c, p| {
            [c[0] + p[0] as f64, c[1] + p[1] as f64, c[2] + p[2] as f64]
        })
        .map(|v| v / source.len().max(1) as f64);
    let spread = (source
        .iter()
        .map(|p| {
            (0..3)
                .map(|k| (p[k] as f64 - centroid[k]).powi(2))
                .sum::<f64>()
        })
        .sum::<f64>()
        / source.len().max(1) as f64)
        .sqrt();
    if !source.is_empty() && !target.is_empty() {
        for iteration in 1..=params.max_iterations {
            iterations = iteration as u32;
            let moved = transform_cloud(source, &rotation, &translation);
            let pairs: Vec<Correspondence> = moved
                .par_iter()
                .filter_map(|p| {
                    tree.nearest(p)
                        .filter(|&(_, d)| d <= limit)
                        .map(|(i, d)| (*p, target[i], d, i))
                })
                .collect();
            if pairs.len() < 3 {
                break;
            }
            let (step_rotation, step_translation) = if params.point_to_plane {
                match plane_fit(&pairs, target, tree, &normals) {
                    Some(step) => step,
                    None => break,
                }
            } else {
                rigid_fit(&pairs)
            };
            let placed = mat3::add(&mat3::mat_vec(&rotation, &centroid), &translation);
            rotation = mat3::mat_mul(&step_rotation, &rotation);
            translation = mat3::add(
                &mat3::mat_vec(&step_rotation, &translation),
                &step_translation,
            );
            let cos = (step_rotation[0][0] + step_rotation[1][1] + step_rotation[2][2] - 1.0) / 2.0;
            // how far this step moved the source points: its centroid's move plus the turn at their RMS radius
            let step_motion = mat3::norm(&mat3::sub(
                &mat3::add(&mat3::mat_vec(&step_rotation, &placed), &step_translation),
                &placed,
            )) + cos.clamp(-1.0, 1.0).acos() * spread;
            let mse = pairs.iter().map(|p| p.2 as f64).sum::<f64>() / pairs.len() as f64;
            if let Some(fail) = fail_fitness {
                let inliers: Vec<f64> = pairs
                    .iter()
                    .map(|p| p.2 as f64)
                    .filter(|&d| d <= params.inlier_squared_distance)
                    .collect();
                let overlap = inliers.len() as f64 / source.len() as f64;
                fitness_history.push(if inliers.is_empty() {
                    f64::MAX
                } else {
                    inliers.iter().sum::<f64>() / inliers.len() as f64 / overlap
                });
                // the fit settles once a step moves the points under the inlier distance, and travels again past it
                if step_motion >= params.inlier_squared_distance.sqrt() {
                    settled_from = None;
                } else if settled_from.is_none() {
                    settled_from = Some(fitness_history.len() - 1);
                }
                let latest = fitness_history[fitness_history.len() - 1];
                if let Some(start) = settled_from.filter(|&s| s + 1 < fitness_history.len()) {
                    // improving every remaining iteration at its mean rate since it settled
                    let rate = (fitness_history[start] - latest)
                        / (fitness_history.len() - 1 - start) as f64;
                    let remaining = (params.max_iterations - iteration) as f64;
                    if fitness_history[start] < f64::MAX
                        && latest < f64::MAX
                        && latest - rate.max(0.0) * remaining > fail
                    {
                        stop = IcpStop::EarlyStop;
                        break;
                    }
                }
            }
            if (cos >= 0.99999 && mat3::norm(&step_translation) <= 1e-3)
                || (previous_mse - mse).abs() <= 1e-6 * previous_mse
            {
                converged = true;
                stop = IcpStop::Converged;
                break;
            }
            if iteration == params.max_iterations {
                converged = true;
                stop = IcpStop::MaxIterations;
                break;
            }
            previous_mse = mse;
        }
    }
    let (fitness, overlap) = fit_quality(
        &transform_cloud(source, &rotation, &translation),
        tree,
        params.inlier_squared_distance,
    );
    IcpResult {
        converged,
        iterations,
        stop,
        target_points: target.len(),
        rotation,
        translation,
        fitness,
        overlap,
    }
}

/// `icp_with_early_stop` from `init` against `target` voxelized at `resolution` (0 = full), fitness re-measured on the full `tree`.
pub fn icp_thinned(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    tree: &impl NearestIndex,
    init: (&Mat3, &Vec3),
    params: &IcpParams,
    fail_fitness: Option<f64>,
    resolution: f64,
) -> IcpResult {
    let (rotation, translation) = init;
    if resolution <= 0.0 {
        return icp_with_early_stop(
            source,
            target,
            tree,
            rotation,
            translation,
            params,
            fail_fitness,
        );
    }
    let thin = crate::pointcloud::voxel_downsample(target, resolution);
    let thin_tree = crate::pointcloud::KdTree::build(&thin);
    let mut result = icp_with_early_stop(
        source,
        &thin,
        &thin_tree,
        rotation,
        translation,
        params,
        fail_fitness,
    );
    (result.fitness, result.overlap) = fit_quality(
        &transform_cloud(source, &result.rotation, &result.translation),
        tree,
        params.inlier_squared_distance,
    );
    result
}

impl IcpResult {
    /// Overlapping points' mean squared distance (m²), used as a closure's per-axis variance (as LIO-SAM does).
    pub fn inlier_mean_square(&self) -> f64 {
        self.fitness * self.overlap
    }
}

/// (fitness, overlap) of `points` against the tree's cloud (see `IcpResult`); non-overlapping points are not scored.
pub fn fit_quality(
    points: &[[f32; 3]],
    tree: &impl NearestIndex,
    inlier_squared_distance: f64,
) -> (f64, f64) {
    let inliers: Vec<f64> = points
        .par_iter()
        .filter_map(|p| tree.nearest(p))
        .map(|(_, d)| d as f64)
        .filter(|&d| d <= inlier_squared_distance)
        .collect();
    if inliers.is_empty() {
        return (f64::MAX, 0.0);
    }
    let overlap = inliers.len() as f64 / points.len() as f64;
    (
        inliers.iter().sum::<f64>() / inliers.len() as f64 / overlap,
        overlap,
    )
}

/// (median local thickness over 10-NN, median NN spacing) in meters, sampled on ~500 points.
pub fn noise_and_spacing(cloud: &[[f32; 3]], tree: &impl NearestIndex) -> (f64, f64) {
    let step = (cloud.len() / 500).max(1);
    let (mut thickness, mut gaps) = (Vec::new(), Vec::new());
    for p in cloud.iter().step_by(step) {
        let neighbours = tree.knn(p, 10);
        if let Some(&(_, d)) = neighbours.get(1) {
            gaps.push((d as f64).sqrt());
        }
        if neighbours.len() >= 3 {
            thickness.push(least_spread(cloud, &neighbours).max(0.0).sqrt());
        }
    }
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(0.0)
    };
    (median(thickness), median(gaps))
}

/// The variance of `neighbours` along their least-spread direction.
fn least_spread(cloud: &[[f32; 3]], neighbours: &[(usize, f32)]) -> f64 {
    let n = neighbours.len() as f64;
    let mut mean = [0.0; 3];
    for &(i, _) in neighbours {
        for k in 0..3 {
            mean[k] += cloud[i][k] as f64 / n;
        }
    }
    let mut covariance = [[0.0; 3]; 3];
    for &(i, _) in neighbours {
        let d = [0, 1, 2].map(|k| cloud[i][k] as f64 - mean[k]);
        for a in 0..3 {
            for b in 0..3 {
                covariance[a][b] += d[a] * d[b] / n;
            }
        }
    }
    mat3::jacobi_eigen(covariance).0[0]
}

/// The least-spread direction (unit surface normal) of `neighbours`.
fn least_spread_direction(cloud: &[[f32; 3]], neighbours: &[(usize, f32)]) -> [f64; 3] {
    let n = neighbours.len() as f64;
    let mut mean = [0.0; 3];
    for &(i, _) in neighbours {
        for k in 0..3 {
            mean[k] += cloud[i][k] as f64 / n;
        }
    }
    let mut covariance = [[0.0; 3]; 3];
    for &(i, _) in neighbours {
        let d = [0, 1, 2].map(|k| cloud[i][k] as f64 - mean[k]);
        for a in 0..3 {
            for b in 0..3 {
                covariance[a][b] += d[a] * d[b] / n;
            }
        }
    }
    let vectors = mat3::jacobi_eigen(covariance).1;
    [vectors[0][0], vectors[1][0], vectors[2][0]]
}

/// Sensor-frame spherical range image of a scan (nearest return per bin, ~one bin per point; 0 = no return).
pub struct RangeBins {
    step: f64,
    azimuth_bins: usize,
    elevation_bins: usize,
    ranges: Vec<f32>,
}

impl RangeBins {
    pub fn build(body: &[[f32; 3]]) -> RangeBins {
        let step = (4.0 * std::f64::consts::PI / body.len().max(1) as f64).sqrt();
        let azimuth_bins = (2.0 * std::f64::consts::PI / step).ceil().max(1.0) as usize;
        let elevation_bins = (std::f64::consts::PI / step).ceil().max(1.0) as usize;
        let mut bins = RangeBins {
            step,
            azimuth_bins,
            elevation_bins,
            ranges: vec![0.0; azimuth_bins * elevation_bins],
        };
        for q in body {
            let q = q.map(|v| v as f64);
            if let Some((bin, range)) = bins.bin(&q) {
                let r = &mut bins.ranges[bin];
                if *r == 0.0 || (range as f32) < *r {
                    *r = range as f32;
                }
            }
        }
        bins
    }
    fn bin(&self, q: &[f64; 3]) -> Option<(usize, f64)> {
        let range = mat3::norm(q);
        if range <= f64::EPSILON {
            return None;
        }
        let azimuth = q[1].atan2(q[0]) + std::f64::consts::PI;
        let elevation = (q[2] / range).clamp(-1.0, 1.0).acos();
        let a = ((azimuth / self.step) as usize).min(self.azimuth_bins - 1);
        let e = ((elevation / self.step) as usize).min(self.elevation_bins - 1);
        Some((e * self.azimuth_bins + a, range))
    }
}

/// A scan's `RangeBins` placed at its pose (world).
pub struct RangeImage {
    world_to_sensor: Mat3,
    origin: Vec3,
    bins: std::sync::Arc<RangeBins>,
}

impl RangeImage {
    pub fn at(bins: std::sync::Arc<RangeBins>, rotation: &Mat3, origin: &Vec3) -> RangeImage {
        RangeImage {
            world_to_sensor: mat3::transpose(rotation),
            origin: *origin,
            bins,
        }
    }
    fn bin(&self, q: &[f64; 3]) -> Option<(usize, f64)> {
        self.bins.bin(q)
    }
    /// Whether this scan's return in `p`'s direction is farther than `p` by more than bin width + `width`.
    pub fn sees_through(&self, p: &[f32; 3], width: f64) -> bool {
        let q = mat3::mat_vec(
            &self.world_to_sensor,
            &mat3::sub(&p.map(|v| v as f64), &self.origin),
        );
        let Some((bin, range)) = self.bin(&q) else {
            return false;
        };
        let surface = self.bins.ranges[bin] as f64;
        surface > 0.0 && range < surface - (surface * self.bins.step + width)
    }
}

/// (mean log ratio/point, half-normal vs uniform-to-`reach`; points; width); `visible`: only inliers + seen-through points.
pub fn likelihood_ratio(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    target_tree: &impl NearestIndex,
    reach: f64,
    visible: Option<(f64, &[RangeImage])>,
) -> (f64, usize, f64) {
    if source.len() < 3 || target.len() < 3 {
        return (f64::NEG_INFINITY, 0, f64::NAN);
    }
    let source_tree = KdTree::build(source);
    let (source_noise, source_gap) = noise_and_spacing(source, &source_tree);
    let (target_noise, target_gap) = noise_and_spacing(target, target_tree);
    let width = source_noise
        .hypot(target_noise)
        .hypot(source_gap.max(target_gap) / 2.0)
        .max(f64::EPSILON);
    let log_good_peak = (2.0 / (width * (2.0 * std::f64::consts::PI).sqrt())).ln();
    let log_bad = (1.0 / reach).ln();
    let ratios: Vec<f64> = source
        .par_iter()
        .filter_map(|p| {
            let squared = target_tree.nearest(p)?.1 as f64;
            let d = match visible {
                Some((inlier_squared_distance, views)) => {
                    if squared > inlier_squared_distance
                        && !views.iter().any(|v| v.sees_through(p, width))
                    {
                        return None;
                    }
                    squared.sqrt().min(reach)
                }
                None => Some(squared.sqrt()).filter(|&d| d < reach)?,
            };
            Some(log_good_peak - d * d / (2.0 * width * width) - log_bad)
        })
        .collect();
    if ratios.is_empty() {
        return (f64::NEG_INFINITY, 0, width);
    }
    (
        ratios.iter().sum::<f64>() / ratios.len() as f64,
        ratios.len(),
        width,
    )
}

/// Linearised point-to-plane step; unconstrained directions stay unmoved; None with < 6 pairs on a surface.
fn plane_fit(
    pairs: &[Correspondence],
    target: &[[f32; 3]],
    tree: &impl NearestIndex,
    normals: &[std::sync::OnceLock<Option<[f64; 3]>>],
) -> Option<(Mat3, Vec3)> {
    let normal_of = |i: usize| {
        *normals[i].get_or_init(|| {
            let neighbours = tree.knn(&target[i], 10);
            (neighbours.len() >= 3).then(|| least_spread_direction(target, &neighbours))
        })
    };
    // compute normals in parallel first; the sum below stays in order
    pairs.par_iter().for_each(|&(_, _, _, i)| {
        normal_of(i);
    });
    let mut information = [[0.0; 6]; 6];
    let mut gradient = [0.0; 6];
    let mut used = 0;
    for (p, q, _, i) in pairs {
        let Some(n) = normal_of(*i) else {
            continue;
        };
        let (p, q) = (p.map(|v| v as f64), q.map(|v| v as f64));
        let row = [
            p[1] * n[2] - p[2] * n[1],
            p[2] * n[0] - p[0] * n[2],
            p[0] * n[1] - p[1] * n[0],
            n[0],
            n[1],
            n[2],
        ];
        let gap = (0..3).map(|k| (p[k] - q[k]) * n[k]).sum::<f64>();
        for a in 0..6 {
            gradient[a] -= row[a] * gap;
            for b in 0..6 {
                information[a][b] += row[a] * row[b];
            }
        }
        used += 1;
    }
    if used < 6 {
        return None;
    }
    let (values, vectors) = mat3::jacobi_eigen(information);
    let floor = values[5].abs() * 6.0 * f64::EPSILON;
    let mut step = [0.0; 6];
    for k in 0..6 {
        if values[k] > floor {
            let along = (0..6).map(|a| vectors[a][k] * gradient[a]).sum::<f64>() / values[k];
            for a in 0..6 {
                step[a] += vectors[a][k] * along;
            }
        }
    }
    let turn = [step[0], step[1], step[2]];
    let angle = mat3::norm(&turn);
    let rotation = if angle > 0.0 {
        let s = (angle / 2.0).sin() / angle;
        mat3::mat_from_quat(&[(angle / 2.0).cos(), turn[0] * s, turn[1] * s, turn[2] * s])
    } else {
        mat3::identity()
    };
    Some((rotation, [step[3], step[4], step[5]]))
}

/// Best rigid map of each pair's first point onto its second (Horn's quaternion method).
fn rigid_fit(pairs: &[Correspondence]) -> (Mat3, Vec3) {
    let n = pairs.len() as f64;
    let mean = |pick: fn(&Correspondence) -> [f32; 3]| {
        let mut m = [0.0; 3];
        for p in pairs {
            for (k, v) in pick(p).iter().enumerate() {
                m[k] += *v as f64 / n;
            }
        }
        m
    };
    let (from_mean, to_mean) = (mean(|p| p.0), mean(|p| p.1));
    let mut s = [[0.0; 3]; 3];
    for (a, b, _, _) in pairs {
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] += (a[i] as f64 - from_mean[i]) * (b[j] as f64 - to_mean[j]);
            }
        }
    }
    let n4 = [
        [
            s[0][0] + s[1][1] + s[2][2],
            s[1][2] - s[2][1],
            s[2][0] - s[0][2],
            s[0][1] - s[1][0],
        ],
        [
            s[1][2] - s[2][1],
            s[0][0] - s[1][1] - s[2][2],
            s[0][1] + s[1][0],
            s[2][0] + s[0][2],
        ],
        [
            s[2][0] - s[0][2],
            s[0][1] + s[1][0],
            s[1][1] - s[0][0] - s[2][2],
            s[1][2] + s[2][1],
        ],
        [
            s[0][1] - s[1][0],
            s[2][0] + s[0][2],
            s[1][2] + s[2][1],
            s[2][2] - s[0][0] - s[1][1],
        ],
    ];
    let (_, vectors) = mat3::jacobi_eigen(n4);
    let rotation =
        mat3::mat_from_quat(&[vectors[0][3], vectors[1][3], vectors[2][3], vectors[3][3]]);
    (
        rotation,
        mat3::sub(&to_mean, &mat3::mat_vec(&rotation, &from_mean)),
    )
}

/// `.icp` dump layout version, the file's last 4 bytes; a file without it is version 1.
pub const ICP_DUMP_VERSION: u32 = 2;

/// Write an ICP's input and result (little-endian, layout in the body below) to `<dir>/<source>_<target><suffix>.icp`.
#[allow(clippy::too_many_arguments)]
pub fn dump_icp(
    dir: &str,
    source_id: usize,
    target_id: usize,
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    init: (&Mat3, &Vec3),
    icp: &IcpResult,
    suffix: &str,
) {
    let mut bytes = Vec::with_capacity(8 + 12 * (source.len() + target.len()) + 105);
    bytes.extend_from_slice(&(source.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&(target.len() as u32).to_le_bytes());
    for v in source.iter().chain(target).flatten() {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    for v in icp
        .rotation
        .iter()
        .flatten()
        .chain(&icp.translation)
        .chain([&icp.fitness])
    {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    bytes.push(icp.converged as u8);
    for v in init.0.iter().flatten().chain(init.1) {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    bytes.extend_from_slice(&icp.iterations.to_le_bytes());
    bytes.push(icp.stop.code());
    bytes.extend_from_slice(&ICP_DUMP_VERSION.to_le_bytes());
    let path = std::path::Path::new(dir).join(format!("{source_id}_{target_id}{suffix}.icp"));
    if let Err(e) = std::fs::create_dir_all(dir).and_then(|_| std::fs::write(&path, bytes)) {
        eprintln!(
            "closure_icp_dump_dir: could not write {}: {e}",
            path.display()
        );
    }
}
