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

//! Stage 3, connection fit quality: how well a discovered connection's two places line up, without the
//! graph. Point-to-point ICP of the source keyframe's cloud onto the target's submap, and its fitness
//! (mean squared distance of the points that overlap the target, over the fraction that do).

use rayon::prelude::*;

use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, KdTree, NearestIndex};

/// A source point moved by the current guess, its nearest target point, their squared distance, and the target point's index.
type Correspondence = ([f32; 3], [f32; 3], f32, usize);

pub struct IcpParams {
    pub max_iterations: usize,
    /// Pairs farther apart than this (m) are not correspondences.
    pub max_correspondence_distance: f64,
    /// A source point overlaps the target when its nearest neighbour is within this squared distance (m²).
    pub inlier_squared_distance: f64,
    /// Each step minimises the pairs' distances along the target's surface normal (point-to-plane) instead of their
    /// full distances (point-to-point): two scans never sample the same spots, so a point may slide along the surface.
    pub point_to_plane: bool,
}

/// Why an ICP stopped iterating.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IcpStop {
    /// An iteration barely moved it, or the correspondences' mean squared distance stopped changing.
    Converged,
    /// The early stop gave up: the fit could no longer get under the gate's fitness (`icp_with_early_stop`).
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
    /// Iterations run, and why it stopped.
    pub iterations: u32,
    pub stop: IcpStop,
    /// The target points ICP paired against (the thinned target in `icp_thinned`; 0 from a cache entry made before it).
    pub target_points: usize,
    /// The transform that moves the source onto the target.
    pub rotation: Mat3,
    pub translation: Vec3,
    /// Mean squared nearest-neighbour distance of the overlapping source points after the transform, over
    /// `overlap`: a thin overlap scores as a weak fit (f64::MAX when nothing overlaps).
    pub fitness: f64,
    /// Fraction of the source points that overlap the target after the transform.
    pub overlap: f64,
}

/// Align `source` onto `target` (whose k-d tree is `tree`), starting from the guess. Converged when an
/// iteration barely moves it (under 0.26 degrees and 1 mm) or the correspondences' mean squared
/// distance stops changing (1e-6 relative), or after `max_iterations`; not converged when fewer than
/// three correspondences remain.
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

/// `icp`, but given `fail_fitness` (the gate's largest passing fitness, m²) it gives up, unconverged, once the fit's
/// fitness (as `IcpResult` measures it, over this iteration's pairs) can no longer get under it: even improving every
/// remaining iteration at its mean rate since it settled, it would still end above `fail_fitness`. A fit settles once a
/// step moves the source points under the inlier distance: one seeded metres off scores worse while it slides into place
/// (judged while travelling, by its best step so far, huge's long loops seeded 5-50 m off gave up at iteration 2-4).
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
    // the source's centroid and RMS radius about it, for how far a step moves its points
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

/// `icp_with_early_stop` against `target` voxelized at `resolution` (a coarser target is a cheaper search and fewer
/// normals; at half the inlier distance every inlier still has a target point within it), the result's fitness and
/// overlap then measured against the full target (`tree`) as the gate reads them. `resolution` 0 = the full target.
#[allow(clippy::too_many_arguments)]
pub fn icp_thinned(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    tree: &impl NearestIndex,
    rotation: &Mat3,
    translation: &Vec3,
    params: &IcpParams,
    fail_fitness: Option<f64>,
    resolution: f64,
) -> IcpResult {
    if resolution <= 0.0 {
        return icp_with_early_stop(source, target, tree, rotation, translation, params, fail_fitness);
    }
    let thin = crate::pointcloud::voxel_downsample(target, resolution);
    let thin_tree = crate::pointcloud::KdTree::build(&thin);
    let mut result =
        icp_with_early_stop(source, &thin, &thin_tree, rotation, translation, params, fail_fitness);
    (result.fitness, result.overlap) = fit_quality(
        &transform_cloud(source, &result.rotation, &result.translation),
        tree,
        params.inlier_squared_distance,
    );
    result
}

impl IcpResult {
    /// The overlapping points' mean squared distance to the target (the fitness without its division by the overlap):
    /// a closure's variance on each axis, as LIO-SAM takes PCL's fitness score. On 200 Mid-360 closures with GT it
    /// matched the real error (median real / claimed 0.94) and ranked right from wrong better than Censi or resampling.
    pub fn inlier_mean_square(&self) -> f64 {
        self.fitness * self.overlap
    }
}

/// (fitness, overlap) of `points` against the tree's cloud; see `IcpResult`. Points with nothing nearby
/// are places the target never saw, not misalignment, so only the overlapping ones are scored.
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

/// How sharply `points` (already moved by an alignment) sit on the tree's cloud: the share within `close` (m) of it,
/// times how much the overlap (`fit_quality`) falls when the alignment is nudged `nudge_m` along x and y both ways and
/// turned `nudge_deg` both ways about the points' centre. A wrong place that fits by chance (a floor, a long wall) fits
/// about as well nudged; a right one does not.
pub fn sharpness(
    points: &[[f32; 3]],
    tree: &impl NearestIndex,
    close: f64,
    inlier_squared_distance: f64,
    nudge_m: f64,
    nudge_deg: f64,
) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    let close_squared = (close * close) as f32;
    let near = points
        .par_iter()
        .filter(|p| tree.nearest(p).is_some_and(|(_, d)| d <= close_squared))
        .count() as f64
        / points.len() as f64;
    let overlap = |moved: &[[f32; 3]]| fit_quality(moved, tree, inlier_squared_distance).1;
    let at = overlap(points);
    if at <= 0.0 {
        return 0.0;
    }
    let n = points.len() as f64;
    let centre = points.iter().fold([0.0; 3], |c, p| {
        [
            c[0] + p[0] as f64 / n,
            c[1] + p[1] as f64 / n,
            c[2] + p[2] as f64 / n,
        ]
    });
    let shift = |d: Vec3| transform_cloud(points, &mat3::identity(), &d);
    let turn = |degrees: f64| {
        let rotation = mat3::rot_z(degrees.to_radians());
        transform_cloud(
            points,
            &rotation,
            &mat3::sub(&centre, &mat3::mat_vec(&rotation, &centre)),
        )
    };
    let nudged = [
        overlap(&shift([nudge_m, 0.0, 0.0])),
        overlap(&shift([-nudge_m, 0.0, 0.0])),
        overlap(&shift([0.0, nudge_m, 0.0])),
        overlap(&shift([0.0, -nudge_m, 0.0])),
        overlap(&turn(nudge_deg)),
        overlap(&turn(-nudge_deg)),
    ];
    let drop = 1.0 - nudged.iter().sum::<f64>() / nudged.len() as f64 / at;
    near * drop.max(0.0)
}

pub struct OverlapScoreParams {
    /// Sensor noise (m) of one point.
    pub sigma: f64,
    /// The farthest a misaligned point's residual is spread over (m).
    pub reach: f64,
    /// A point is shared when the other cloud has a point within this many sigmas of it.
    pub overlap_noise: f64,
    /// Evidence a perfect fit adds per doubling of the overlap points.
    pub per_doubling: f64,
    /// Overlap points at which a perfect fit is 50/50.
    pub coin_flip: f64,
    /// Score only the points within the shared radius of a shared point (where both clouds see the same surfaces),
    /// instead of every point in the shared points' bounding box.
    pub local: bool,
}

pub struct OverlapScore {
    /// Points per cloud inside the shared box, its fit quality in [-1, 1], evidence = quality x per_doubling x
    /// log2(points / coin_flip), and score = sigmoid(evidence).
    pub points: f64,
    pub quality: f64,
    pub evidence: f64,
    pub score: f64,
}

/// How well two aligned clouds agree where they overlap: the box around the points each shares with the other, each
/// point in it scored by the log-likelihood ratio of its nearest-neighbour distance under a good fit (half-normal of
/// both clouds' noise and their sample gap) against a misfit (uniform up to `reach`), as tanh(ratio / 2), with
/// evidence growing with the log of how many points overlap.
pub fn overlap_score(a: &[[f32; 3]], b: &[[f32; 3]], params: &OverlapScoreParams) -> OverlapScore {
    let none = OverlapScore {
        points: 0.0,
        quality: 0.0,
        evidence: f64::NEG_INFINITY,
        score: 0.0,
    };
    if a.is_empty() || b.is_empty() {
        return none;
    }
    let within = |tree: &KdTree, p: &[f32; 3], limit: f64| -> Option<f64> {
        tree.nearest(p)
            .map(|(_, d)| (d as f64).sqrt())
            .filter(|&d| d < limit)
    };
    let shared = params.overlap_noise * params.sigma;
    let (tree_a, tree_b) = (KdTree::build(a), KdTree::build(b));
    let near: Vec<[f32; 3]> = a
        .par_iter()
        .filter(|p| within(&tree_b, p, shared).is_some())
        .chain(
            b.par_iter()
                .filter(|p| within(&tree_a, p, shared).is_some()),
        )
        .copied()
        .collect();
    if near.len() < 4 {
        return none;
    }
    let (lo, hi) = near
        .iter()
        .fold(([f32::MAX; 3], [f32::MIN; 3]), |(lo, hi), p| {
            (
                [lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])],
                [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])],
            )
        });
    let near_tree = KdTree::build(&near);
    let in_box = |cloud: &[[f32; 3]]| -> Vec<[f32; 3]> {
        cloud
            .iter()
            .copied()
            .filter(|p| {
                if params.local {
                    within(&near_tree, p, shared).is_some()
                } else {
                    (0..3).all(|k| p[k] >= lo[k] && p[k] <= hi[k])
                }
            })
            .collect()
    };
    let (a_box, b_box) = (in_box(a), in_box(b));
    let (tree_a_box, tree_b_box) = (KdTree::build(&a_box), KdTree::build(&b_box));
    // median nearest-neighbour distance within a cloud, from about 300 of its points
    let spacing = |cloud: &[[f32; 3]], tree: &KdTree| -> f64 {
        let step = (cloud.len() / 300).max(1);
        let mut gaps: Vec<f64> = cloud
            .iter()
            .step_by(step)
            .filter_map(|p| tree.knn(p, 2).get(1).map(|&(_, d)| (d as f64).sqrt()))
            .collect();
        gaps.sort_by(f64::total_cmp);
        gaps.get(gaps.len() / 2).copied().unwrap_or(0.0)
    };
    let gap = spacing(&a_box, &tree_a_box).max(spacing(&b_box, &tree_b_box));
    // aligned, a point's nearest neighbour in the other cloud is off by both clouds' noise plus the gap between samples
    let width = (std::f64::consts::SQRT_2 * params.sigma).hypot(gap / 2.0);
    let log_good = |d: f64| {
        (2.0 / (width * (2.0 * std::f64::consts::PI).sqrt())).ln() - d * d / (2.0 * width * width)
    };
    let log_bad = (1.0 / params.reach).ln();
    let vote = |points: &[[f32; 3]], tree: &KdTree| -> Vec<f64> {
        points
            .par_iter()
            .map(|p| {
                let d = within(tree, p, params.reach).unwrap_or(params.reach);
                ((log_good(d) - log_bad) / 2.0).tanh()
            })
            .collect()
    };
    let votes: Vec<f64> = [vote(&b_box, &tree_a_box), vote(&a_box, &tree_b_box)].concat();
    if votes.is_empty() {
        return none;
    }
    let points = votes.len() as f64 / 2.0;
    let quality = votes.iter().sum::<f64>() / votes.len() as f64;
    // a misfit (quality < 0) is evidence against however few points overlap, so the log never flips its sign
    let doublings = (points.max(1.0) / params.coin_flip).log2();
    let evidence = quality
        * params.per_doubling
        * if quality < 0.0 {
            doublings.abs()
        } else {
            doublings
        };
    OverlapScore {
        points,
        quality,
        evidence,
        score: 1.0 / (1.0 + (-evidence).exp()),
    }
}

/// A cloud's sensor noise and sample spacing, measured from the cloud on about 500 of its points: the noise is the median
/// thickness (sqrt of the smallest covariance eigenvalue) of each point's 10 nearest neighbours, the spacing the median
/// distance to its nearest neighbour.
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

/// What one target scan saw, in its sensor frame: the nearest return in each direction (a spherical range image), at
/// the angular step that gives one bin per point of the scan over the full sphere. It depends on the scan alone, so a
/// keyframe's is built once and shared (`RangeImage::at` places it at the keyframe's current pose).
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
    /// The range image of `body` (the scan in its sensor frame) taken from the pose `rotation`, `origin` (world).
    pub fn build(body: &[[f32; 3]], rotation: &Mat3, origin: &Vec3) -> RangeImage {
        RangeImage::at(
            std::sync::Arc::new(RangeBins::build(body)),
            rotation,
            origin,
        )
    }
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
    /// Whether this scan looked through world point `p`: it saw a return in p's direction farther than p by more than
    /// the bin's own width there plus `width` (so a point on or behind the seen surface, or in a direction it never
    /// returned from, does not count).
    /// Whether this scan could have seen world point `p`: it has a return in p's direction no nearer than p (less the
    /// bin width there plus `width`), so p lies on or in front of what it saw (the beam model's visible region).
    pub fn sees(&self, p: &[f32; 3], width: f64) -> bool {
        let q = mat3::mat_vec(
            &self.world_to_sensor,
            &mat3::sub(&p.map(|v| v as f64), &self.origin),
        );
        let Some((bin, range)) = self.bin(&q) else {
            return false;
        };
        let surface = self.bins.ranges[bin] as f64;
        surface > 0.0 && range <= surface + surface * self.bins.step + width
    }
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

/// `likelihood_ratio` over only the points the target can speak to: those within ICP's inlier distance of it, and those
/// some target scan (`views`) looked straight through (seen-through space where the source has a surface is a misfit).
/// A point no target scan saw, or that sits behind what they saw, says nothing either way.
pub fn likelihood_ratio_visible(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    target_tree: &impl NearestIndex,
    reach: f64,
    inlier_squared_distance: f64,
    views: &[RangeImage],
    beam: bool,
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
            // beam model: a point counts where a target scan could see it; otherwise matched points and seen-through ones
            let counts = if beam {
                views.iter().any(|v| v.sees(p, width))
            } else {
                squared <= inlier_squared_distance || views.iter().any(|v| v.sees_through(p, width))
            };
            if !counts {
                return None;
            }
            let d = squared.sqrt().min(reach);
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

/// How much likelier the aligned `source`'s nearest-neighbour distances to `target` are if the two clouds see the same
/// surfaces than if they do not, as the mean log likelihood ratio per point (nats). Aligned, a point sits a half-normal
/// distance from its neighbour, of width hypot(both clouds' noise, the wider sample spacing / 2), measured from the
/// clouds; not aligned, its distance is anything out to `reach` (the ICP match distance), uniformly. Points farther than
/// `reach` say nothing either way. Returns (mean ratio, matched points, width); the ratio is -inf when nothing matched.
pub fn likelihood_ratio(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    target_tree: &KdTree,
    reach: f64,
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
            let d = (target_tree.nearest(p)?.1 as f64).sqrt();
            (d < reach).then(|| log_good_peak - d * d / (2.0 * width * width) - log_bad)
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

/// The small rotation and translation that best close each pair's gap along the target point's surface normal
/// (point-to-plane, linearised: rows [p x n, n], gap (p - q).n), as a rotation matrix and translation. Directions no
/// pair constrains (eigenvalues of the normal equations at rounding level) are left unmoved. None with too few pairs
/// on a surface.
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
    // the paired target points' normals, found in parallel (each is a 10-neighbour search); the sum below stays in order
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

/// The rotation and translation that best map each pair's first point onto its second (Horn's
/// quaternion method: the top eigenvector of a 4x4 built from the cross-covariance).
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

/// The `.icp` dump layout version, the file's last 4 bytes (`dump_icp`); a file without it is version 1.
pub const ICP_DUMP_VERSION: u32 = 2;

/// Write an ICP's exact input and result to `<dir>/<source>_<target><suffix>.icp`: u32 source count, u32
/// target count, f32 xyz points (source, then target), the rotation (row-major) and translation as f64,
/// f64 fitness, u8 converged (version 1 ends here); then the initial guess's rotation (row-major) and translation as
/// f64, u32 iterations, u8 stop (`IcpStop::code`), and u32 `ICP_DUMP_VERSION` last.
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The overlap page's chair (seat, four legs, back) sampled at `density` points per m² with `sigma` noise, using the
    /// page's generator so the scores below match its checks.
    fn chair(density: f64, sigma: f64, seed: u32) -> Vec<[f32; 3]> {
        let mut s = seed;
        let mut rand = || {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            s as f64 / 4294967296.0
        };
        let mut boxes = vec![[0.0, 0.0, 0.45, 0.45, 0.45, 0.04]];
        for (x, y) in [(-0.2, -0.2), (0.2, -0.2), (-0.2, 0.2), (0.2, 0.2)] {
            boxes.push([x, y, 0.215, 0.04, 0.04, 0.43]);
        }
        boxes.push([0.0, 0.205, 0.72, 0.45, 0.04, 0.5]);
        let mut points = Vec::new();
        for [cx, cy, cz, sx, sy, sz] in boxes {
            for (a, b, axis) in [(sx, sy, 2), (sx, sz, 1), (sy, sz, 0)] {
                let n = (a * b * density).round() as usize;
                for side in [-1.0, 1.0] {
                    for _ in 0..n {
                        let (u, v) = (rand() - 0.5, rand() - 0.5);
                        let p = match axis {
                            2 => [cx + u * sx, cy + v * sy, cz + side * sz / 2.0],
                            1 => [cx + u * sx, cy + side * sy / 2.0, cz + v * sz],
                            _ => [cx + side * sx / 2.0, cy + u * sy, cz + v * sz],
                        };
                        let mut gauss = || {
                            (-2.0 * (rand() + 1e-12).ln()).sqrt()
                                * (2.0 * std::f64::consts::PI * rand()).cos()
                        };
                        let noisy = [
                            p[0] + gauss() * sigma,
                            p[1] + gauss() * sigma,
                            p[2] + gauss() * sigma,
                        ];
                        points.push(noisy.map(|c| c as f32));
                    }
                }
            }
        }
        points
    }

    /// The part of the chair whose x and z fall in these fractions of its extent.
    fn crop(points: &[[f32; 3]], x: [f64; 2], z: [f64; 2]) -> Vec<[f32; 3]> {
        points
            .iter()
            .copied()
            .filter(|p| {
                let fx = (p[0] as f64 + 0.25) / 0.5;
                let fz = p[2] as f64 / 0.97;
                fx >= x[0] && fx <= x[1] && fz >= z[0] && fz <= z[1]
            })
            .collect()
    }

    #[test]
    fn overlap_score_matches_the_page() {
        let params = OverlapScoreParams {
            sigma: 0.01,
            reach: 0.3,
            overlap_noise: 3.0,
            per_doubling: 2.7,
            coin_flip: 350.0,
            local: false,
        };
        let all = [0.0, 1.0];
        let score =
            |density: f64, (ax, az): ([f64; 2], [f64; 2]), (bx, bz): ([f64; 2], [f64; 2])| {
                let a = crop(&chair(density, 0.01, 1), ax, az);
                let b = crop(&chair(density, 0.01, 2), bx, bz);
                100.0 * overlap_score(&a, &b, &params).score
            };
        // the page's numbers: 94.92, 99.77, 100.00, 79.95
        let upper_half = score(2000.0, (all, [0.5, 1.0]), (all, all));
        let whole = score(2000.0, (all, all), (all, all));
        let dense = score(20000.0, (all, all), (all, all));
        let crossed_halves = score(2000.0, ([0.0, 0.5], all), (all, [0.5, 1.0]));
        eprintln!("upper half {upper_half:.2} whole {whole:.2} dense {dense:.4} crossed halves {crossed_halves:.2}");
        assert!(
            (upper_half - 94.92).abs() < 1.0,
            "upper half vs whole {upper_half}"
        );
        assert!((whole - 99.77).abs() < 1.0, "whole vs whole {whole}");
        assert!(dense >= 99.9, "dense whole vs whole {dense}");
        assert!(
            (crossed_halves - 79.95).abs() < 1.0,
            "vertical half vs horizontal half {crossed_halves}"
        );
    }
}
