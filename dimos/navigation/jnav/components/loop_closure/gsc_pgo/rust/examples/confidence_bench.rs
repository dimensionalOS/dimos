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

//! How well each closure confidence metric tells overlapping scans from non-overlapping ones. Pairs of cylinders are
//! cropped from a global map, a scan-sized source and a target twice as wide (like GSC's submap): overlapping (centres
//! closer than 2.2 radii) or not (farther than 4). Shared
//! points are dealt to one crop or the other, never both. Each crop is thinned to a random size and jittered, then
//! turned and moved into its own frame. ICP aligns them from a seed near the truth (overlapping) or a random turn
//! about the other's centre (not), and every metric scores the result. One CSV row per pair goes to stdout.
//! With `hard`, the non-overlapping pairs are look-alikes instead: each far crop's best scan-context match among the
//! far crops, aligned from the scan-context yaw (both signs tried, the better fit kept).
//! usage: confidence_bench <map.xyz (f32 x,y,z)> <name> <pairs> [radius m] [hard]

use dimos_gsc_pgo::connection_fit::{
    icp, likelihood_ratio, likelihood_ratio_visible, overlap_score, sharpness, IcpParams,
    OverlapScoreParams, RangeImage,
};
use dimos_gsc_pgo::mat3::{self, Vec3};
use dimos_gsc_pgo::pointcloud::{transform_cloud, voxel_downsample, KdTree};
use dimos_gsc_pgo::similarity_discovery::{
    self as sc, best_distance, make_descriptor, make_ring_key, ring_key_distances, yaw_from_shift,
};
use dimos_gsc_pgo::uniqueness::degeneracy;
use rayon::prelude::*;
use std::collections::HashMap;

struct SplitMix(u64);
impl SplitMix {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f64 {
        (-2.0 * (self.next() + 1e-12).ln()).sqrt()
            * (2.0 * std::f64::consts::PI * self.next()).cos()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (path, name, pairs) = (&args[1], &args[2], args[3].parse::<usize>().unwrap());
    let radius: f64 = args.get(4).map_or(10.0, |r| r.parse().unwrap());
    let hard = args.get(5).is_some_and(|m| m == "hard");
    let bytes = std::fs::read(path).unwrap();
    let raw: Vec<[f32; 3]> = bytes
        .as_chunks::<12>()
        .0
        .iter()
        .map(|c| [0, 4, 8].map(|o| f32::from_le_bytes(c[o..o + 4].try_into().unwrap())))
        .collect();
    let map = voxel_downsample(&raw, 0.05);
    let cell = radius;
    let key = |p: &[f32; 3]| {
        (
            (p[0] as f64 / cell).floor() as i64,
            (p[1] as f64 / cell).floor() as i64,
        )
    };
    let mut grid: HashMap<(i64, i64), Vec<u32>> = HashMap::new();
    for (i, p) in map.iter().enumerate() {
        grid.entry(key(p)).or_default().push(i as u32);
    }
    // a crop centre is a map point with a full crop around it
    let crop_ids = |c: &[f32; 3], radius: f64| -> Vec<u32> {
        let (cx, cy) = key(c);
        let reach = (radius / cell).ceil() as i64;
        let mut ids = Vec::new();
        for dx in -reach..=reach {
            for dy in -reach..=reach {
                for &i in grid.get(&(cx + dx, cy + dy)).map_or(&[][..], |v| &v[..]) {
                    let p = map[i as usize];
                    if ((p[0] - c[0]) as f64).hypot((p[1] - c[1]) as f64) < radius {
                        ids.push(i);
                    }
                }
            }
        }
        ids
    };
    eprintln!("{name}: {} map points after 5 cm voxels", map.len());
    let sc_config = sc::Config {
        max_range_m: radius,
        ..sc::Config::default()
    };
    let local = |c: &[f32; 3], ids: &[u32]| -> Vec<[f32; 3]> {
        ids.iter()
            .map(|&i| {
                let p = map[i as usize];
                [p[0] - c[0], p[1] - c[1], p[2] - c[2]]
            })
            .collect()
    };
    // look-alikes: from a pool of crops, each one's best scan-context match among the crops far from it
    let look_alikes: Vec<([f32; 3], [f32; 3])> = if hard {
        let mut rng = SplitMix(99);
        let pool: Vec<[f32; 3]> = (0..pairs * 2)
            .map(|_| map[(rng.next() * map.len() as f64) as usize])
            .collect();
        let descriptors: Vec<_> = pool
            .par_iter()
            .map(|c| {
                make_descriptor(
                    &voxel_downsample(&local(c, &crop_ids(c, radius)), 0.2),
                    &sc_config,
                )
            })
            .collect();
        let keys: Vec<_> = descriptors.iter().map(make_ring_key).collect();
        (0..pairs / 2 + 1)
            .into_par_iter()
            .filter_map(|i| {
                // GSC's retrieval: the nearest ring keys among the far crops, then the full descriptor on those
                let mut ranked: Vec<(f32, usize)> = ring_key_distances(&keys[i], &keys)
                    .into_iter()
                    .filter(|&(_, j)| {
                        ((pool[j][0] - pool[i][0]) as f64).hypot((pool[j][1] - pool[i][1]) as f64)
                            > 4.0 * radius
                    })
                    .collect();
                ranked.sort_by(|x, y| x.0.total_cmp(&y.0));
                ranked
                    .into_iter()
                    .take(sc_config.candidate_top_k)
                    .map(|(_, j)| (best_distance(&descriptors[i], &descriptors[j]).0, j))
                    .min_by(|x, y| x.0.total_cmp(&y.0))
                    .map(|(_, j)| (pool[i], pool[j]))
            })
            .collect()
    } else {
        Vec::new()
    };
    println!("map,overlapping,centre_distance,overlap_fraction,points_a,points_b,aligned,icp_converged,fitness,fit_factor,degeneracy_factor,old_fit_x_degeneracy,sharpness_factor,overlap_box_s03,overlap_local_s05,overlap_local_s03,overlap_local_points,overlap_local_quality,sc_distance,old_sc_x_fit_x_degeneracy,rotation_error_deg,translation_error_m,scale,inlier_fraction,inlier_rmse,chamfer,hausdorff,point_to_plane,voxel_iou,likelihood_ratio_mean,likelihood_ratio_total,mixture_evidence,mixture_share,seen_ratio_mean,seen_ratio_per_point,visible_ratio_mean,beam_ratio_mean");
    let rows: Vec<String> = (0..pairs)
        .into_par_iter()
        .filter_map(|trial| {
            let mut rng = SplitMix(trial as u64 * 7919 + 17);
            let overlapping = trial % 2 == 0;
            let look_alike = (hard && !overlapping).then(|| look_alikes.get(trial / 2).copied()).flatten();
            if hard && !overlapping && look_alike.is_none() {
                return None;
            }
            let c1 = look_alike.map_or_else(|| map[(rng.next() * map.len() as f64) as usize], |(c, _)| c);
            let ids1 = crop_ids(&c1, 2.0 * radius);
            if ids1.len() < 2000 {
                return None;
            }
            // the second centre: a map point at the drawn distance (overlapping) or any far one
            let c2 = if let Some((_, c)) = look_alike { c } else { (0..200).find_map(|_| {
                let p = map[(rng.next() * map.len() as f64) as usize];
                let d = ((p[0] - c1[0]) as f64).hypot((p[1] - c1[1]) as f64);
                let ok = if overlapping { d < 2.2 * radius } else { d > 4.0 * radius };
                ok.then_some(p)
            })? };
            let ids2 = crop_ids(&c2, radius);
            if ids2.len() < 2000 {
                return None;
            }
            let in2: std::collections::HashSet<u32> = ids2.iter().copied().collect();
            let shared = ids1.iter().filter(|i| in2.contains(i)).count();
            let overlap_fraction = shared as f64 / ids2.len() as f64;
            // sizes log-uniform in 300..30000 points, shared points dealt to one crop only
            let target_a = (300.0 * 100f64.powf(rng.next())).min(ids1.len() as f64);
            let target_b = (300.0 * 100f64.powf(rng.next())).min(ids2.len() as f64);
            let (keep_a, keep_b) = (target_a / ids1.len() as f64, target_b / ids2.len() as f64);
            let sigma = 0.01;
            let jitter = |p: [f32; 3], rng: &mut SplitMix| -> [f32; 3] {
                p.map(|v| (v as f64 + rng.gauss() * sigma) as f32)
            };
            let mut a = Vec::new();
            for &i in &ids1 {
                let shared_here = in2.contains(&i);
                if shared_here && !a_took(i, trial) {
                    continue;
                }
                if rng.next() < keep_a * if shared_here { 2.0 } else { 1.0 } {
                    let p = map[i as usize];
                    a.push(jitter([p[0] - c1[0], p[1] - c1[1], p[2] - c1[2]], &mut rng));
                }
            }
            let in1: std::collections::HashSet<u32> = ids1.iter().copied().collect();
            let yaw = (rng.next() * 2.0 - 1.0) * std::f64::consts::PI;
            let turn = mat3::rot_z(yaw);
            let mut b = Vec::new();
            for &i in &ids2 {
                let shared_here = in1.contains(&i);
                if shared_here && a_took(i, trial) {
                    continue;
                }
                if rng.next() < keep_b * if shared_here { 2.0 } else { 1.0 } {
                    let p = map[i as usize];
                    let local = mat3::mat_vec(&turn, &[(p[0] - c2[0]) as f64, (p[1] - c2[1]) as f64, (p[2] - c2[2]) as f64]);
                    b.push(jitter(local.map(|v| v as f32), &mut rng));
                }
            }
            if a.len() < 50 || b.len() < 50 {
                return None;
            }
            // truth: b's frame to a's
            let true_rotation = mat3::rot_z(-yaw);
            let true_translation: Vec3 = [(c2[0] - c1[0]) as f64, (c2[1] - c1[1]) as f64, (c2[2] - c1[2]) as f64];
            let (sc_distance, sc_shift) = best_distance(&make_descriptor(&voxel_downsample(&a, 0.2), &sc_config), &make_descriptor(&voxel_downsample(&b, 0.2), &sc_config));
            let sc_yaw = yaw_from_shift(sc_shift, sc_config.n_sectors);
            let (seed_rotation, seed_translation) = if overlapping {
                let dyaw = (rng.next() * 2.0 - 1.0) * 5f64.to_radians();
                let dt = [(rng.next() * 2.0 - 1.0) * 0.5, (rng.next() * 2.0 - 1.0) * 0.5, 0.0];
                (mat3::rot_z(-yaw + dyaw), mat3::add(&true_translation, &dt))
            } else {
                (mat3::rot_z((rng.next() * 2.0 - 1.0) * std::f64::consts::PI), [0.0; 3])
            };
            let tree_a = KdTree::build(&a);
            let params = IcpParams { max_iterations: 50, max_correspondence_distance: 10.0, inlier_squared_distance: 0.15, point_to_plane: true };
            let result = if hard && !overlapping {
                [sc_yaw, -sc_yaw]
                    .map(|y| icp(&b, &a, &tree_a, &mat3::rot_z(y), &[0.0; 3], &params))
                    .into_iter()
                    .min_by(|x, y| x.fitness.total_cmp(&y.fitness))
                    .unwrap()
            } else {
                icp(&b, &a, &tree_a, &seed_rotation, &seed_translation, &params)
            };
            let moved = transform_cloud(&b, &result.rotation, &result.translation);
            let rotation_error = mat3::angular_distance(&result.rotation, &true_rotation).to_degrees();
            let translation_error = mat3::norm(&mat3::sub(&result.translation, &true_translation));
            let aligned = overlapping && rotation_error < 2.0 && translation_error < 0.3;
            let unit = |v: f64| if v.is_finite() { v.clamp(0.0, 1.0) } else { 0.0 };
            let fit_factor = unit(1.0 - result.fitness / 0.15);
            let degeneracy_factor = unit(degeneracy(&b) as f64 / 0.05);
            let s = sharpness(&moved, &tree_a, 0.1, 0.15, 1.0, 5.0);
            let score = |sigma: f64, local: bool| {
                overlap_score(&moved, &a, &OverlapScoreParams { sigma, reach: 0.5, overlap_noise: 3.0, per_doubling: 2.7, coin_flip: 350.0, local })
            };
            let r15 = score(0.05, true);
            let (scale, standard) = standard_metrics(&moved, &a, &tree_a, params.max_correspondence_distance);
            // the library's ratio with the misfit reach at ICP's inlier distance: unmatched points carry no evidence
            let inlier_reach = params.inlier_squared_distance.sqrt();
            let (seen_mean, seen_points, _) = likelihood_ratio(&moved, &a, &tree_a, inlier_reach);
            let seen_per_point = if seen_points > 0 { seen_mean * seen_points as f64 / moved.len() as f64 } else { -1e9 };
            // the visibility-aware ratio, the target crop seen from its own centre (its sensor origin)
            let view = RangeImage::build(&a, &mat3::identity(), &[0.0; 3]);
            let (visible_mean, _, _) = likelihood_ratio_visible(&moved, &a, &tree_a, params.max_correspondence_distance, params.inlier_squared_distance, std::slice::from_ref(&view), false);
            let (beam_mean, _, _) = likelihood_ratio_visible(&moved, &a, &tree_a, params.max_correspondence_distance, params.inlier_squared_distance, std::slice::from_ref(&view), true);
            Some(format!(
                "{name},{},{:.2},{:.3},{},{},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.0},{:.3},{:.4},{:.4},{:.2},{:.3},{:.4},{}",
                overlapping as u8,
                ((c2[0] - c1[0]) as f64).hypot((c2[1] - c1[1]) as f64),
                overlap_fraction,
                a.len(),
                b.len(),
                aligned as u8,
                result.converged as u8,
                result.fitness.min(9.0),
                fit_factor,
                degeneracy_factor,
                fit_factor * degeneracy_factor,
                unit(s / (s + 0.06)),
                score(0.03, false).score,
                r15.score,
                score(0.03, true).score,
                r15.points,
                r15.quality,
                sc_distance,
                unit(1.0 - sc_distance as f64 / 0.4) * fit_factor * degeneracy_factor,
                rotation_error,
                translation_error,
                scale,
                standard.iter().map(|v| format!("{v:.5}")).collect::<Vec<_>>().join(","),
            ) + &format!(",{:.5},{:.5},{:.5}", if seen_mean.is_finite() { seen_mean } else { -1e9 }, seen_per_point, if visible_mean.is_finite() { visible_mean } else { -1e9 }) + &format!(",{:.5}", if beam_mean.is_finite() { beam_mean } else { -1e9 }))
        })
        .collect();
    for row in rows {
        println!("{row}");
    }
}

/// Whether crop A kept shared point `i` (its half of the shared points), the same answer for both crops.
fn a_took(i: u32, trial: usize) -> bool {
    let mut rng = SplitMix((i as u64) << 20 ^ trial as u64);
    rng.next() < 0.5
}

/// Each cloud's sensor noise and sample spacing, measured from the cloud itself on about 500 of its points: the noise is
/// the median thickness (sqrt of the smallest covariance eigenvalue) of each point's 10 nearest neighbours, the spacing
/// the median distance to its nearest neighbour.
fn noise_and_spacing(cloud: &[[f32; 3]], tree: &KdTree) -> (f64, f64) {
    let step = (cloud.len() / 500).max(1);
    let mut thickness = Vec::new();
    let mut gaps = Vec::new();
    for p in cloud.iter().step_by(step) {
        let neighbours = tree.knn(p, 10);
        if let Some(&(_, d)) = neighbours.get(1) {
            gaps.push((d as f64).sqrt());
        }
        if neighbours.len() >= 3 {
            thickness.push(normal_of(cloud, &neighbours).1.max(0.0).sqrt());
        }
    }
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(0.0)
    };
    (median(thickness), median(gaps))
}

/// The least-spread direction of `neighbours` and its variance along it.
fn normal_of(cloud: &[[f32; 3]], neighbours: &[(usize, f32)]) -> ([f64; 3], f64) {
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
    let (values, vectors) = mat3::jacobi_eigen(covariance);
    ([vectors[0][0], vectors[1][0], vectors[2][0]], values[0])
}

/// The textbook fit measures of `source` (already aligned) against `target`, signed so higher means "same place". One
/// scale for all, from the clouds: the distance an aligned point is expected to sit from its neighbour, hypot(noise,
/// spacing / 2) with the two clouds' noises combined; the inlier cutoff is its two-sided 95% point (1.96 scales). The
/// likelihood ratio's misfit model is uniform out to the ICP match distance, `reach`.
fn standard_metrics(
    source: &[[f32; 3]],
    target: &[[f32; 3]],
    target_tree: &KdTree,
    reach: f64,
) -> (f64, Vec<f64>) {
    let source_tree = KdTree::build(source);
    let (source_noise, source_gap) = noise_and_spacing(source, &source_tree);
    let (target_noise, target_gap) = noise_and_spacing(target, target_tree);
    let scale = source_noise
        .hypot(target_noise)
        .hypot(source_gap.max(target_gap) / 2.0)
        .max(1e-4);
    let cutoff = 1.96 * scale;
    let nearest = |p: &[f32; 3], tree: &KdTree| {
        tree.nearest(p)
            .map_or((usize::MAX, f64::INFINITY), |(i, d)| (i, (d as f64).sqrt()))
    };
    let forward: Vec<(usize, f64)> = source.par_iter().map(|p| nearest(p, target_tree)).collect();
    let backward: Vec<f64> = target
        .par_iter()
        .map(|p| nearest(p, &source_tree).1)
        .collect();
    let n = source.len() as f64;
    let inliers: Vec<usize> = (0..source.len())
        .filter(|&i| forward[i].1 < cutoff)
        .collect();
    let inlier_fraction = inliers.len() as f64 / n;
    let inlier_rmse = if inliers.is_empty() {
        f64::INFINITY
    } else {
        (inliers.iter().map(|&i| forward[i].1.powi(2)).sum::<f64>() / inliers.len() as f64).sqrt()
    };
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len().max(1) as f64;
    let forward_d: Vec<f64> = forward.iter().map(|f| f.1).collect();
    let chamfer = (mean(&forward_d) + mean(&backward)) / 2.0;
    let hausdorff = forward_d
        .iter()
        .chain(&backward)
        .fold(0.0f64, |m, &d| m.max(d));
    // point-to-plane: each inlier's offset along its target neighbour's surface normal
    let plane: Vec<f64> = inliers
        .par_iter()
        .map(|&i| {
            let j = forward[i].0;
            let (normal, _) = normal_of(target, &target_tree.knn(&target[j], 10));
            let d = [0, 1, 2].map(|k| source[i][k] as f64 - target[j][k] as f64);
            (d[0] * normal[0] + d[1] * normal[1] + d[2] * normal[2]).abs()
        })
        .collect();
    let plane_rmse = if plane.is_empty() {
        f64::INFINITY
    } else {
        (plane.iter().map(|d| d * d).sum::<f64>() / plane.len() as f64).sqrt()
    };
    // occupied voxels (cutoff wide) shared over occupied by either
    let voxels = |cloud: &[[f32; 3]]| -> std::collections::HashSet<[i64; 3]> {
        cloud
            .iter()
            .map(|p| p.map(|v| (v as f64 / cutoff).floor() as i64))
            .collect()
    };
    let (vs, vt) = (voxels(source), voxels(target));
    let iou = vs.intersection(&vt).count() as f64 / vs.union(&vt).count().max(1) as f64;
    // likelihood ratio per matched point: aligned = half-normal of width `scale`, misfit = uniform out to `reach`
    let ratios: Vec<f64> = forward_d
        .iter()
        .filter(|&&d| d < reach)
        .map(|&d| {
            ((2.0 / (scale * (2.0 * std::f64::consts::PI).sqrt())).ln()
                - d * d / (2.0 * scale * scale))
                - (1.0 / reach).ln()
        })
        .collect();
    let total_ratio = ratios.iter().sum::<f64>();
    // the same test with the overlap unknown: aligned = a share `share` of points fit (half-normal) and the rest are
    // surfaces the other cloud never saw (uniform), the share fit by maximum likelihood (EM), less the BIC cost of
    // fitting it (ln(n) / 2)
    let fits: Vec<f64> = ratios.iter().map(|r| r.exp()).collect();
    let mut share = 0.5;
    for _ in 0..100 {
        let responsibility = fits
            .iter()
            .map(|f| share * f / (share * f + 1.0 - share))
            .sum::<f64>()
            / fits.len().max(1) as f64;
        if (responsibility - share).abs() < 1e-9 {
            break;
        }
        share = responsibility;
    }
    let mixture = fits
        .iter()
        .map(|f| (share * f + 1.0 - share).ln())
        .sum::<f64>()
        - (fits.len().max(1) as f64).ln() / 2.0;
    let bounded = |v: f64| if v.is_finite() { v } else { -1e9 };
    (
        scale,
        vec![
            inlier_fraction,
            bounded(-inlier_rmse / scale),
            -chamfer,
            -hausdorff,
            bounded(-plane_rmse / scale),
            iou,
            bounded(mean(&ratios)),
            bounded(total_ratio),
            bounded(mixture),
            share,
        ],
    )
}
