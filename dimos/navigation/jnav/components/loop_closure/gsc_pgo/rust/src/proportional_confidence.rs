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

//! Stage 4, proportional confidence: the evidence a connection needs grows with its yank (stdevs of predicted spread);
//! confidence must reach 1 - exp(-required evidence), and a shortfall gets extended ICP on more keyframes.

use crate::connection_fit::{self, dump_icp, icp, IcpParams, IcpResult};
use crate::gsc_pgo::{ClosureCandidateDiag, ClosureSearch};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    transform_cloud, voxel_downsample, KdForest, KdTree, NearestIndex, PointCloud,
};
use std::sync::{Arc, Mutex, OnceLock};

/// One keyframe's body-frame scan, lazily voxelized and range-binned; pose-independent, so solves never stale it.
pub struct KeyframeCloud {
    body: Arc<PointCloud>,
    voxelized: OnceLock<PointCloud>,
    bins: OnceLock<Arc<connection_fit::RangeBins>>,
}

impl KeyframeCloud {
    pub(crate) fn voxelized(&self, resolution: f64) -> &PointCloud {
        self.voxelized
            .get_or_init(|| voxel_downsample(&self.body, resolution))
    }
    pub(crate) fn bins(&self) -> Arc<connection_fit::RangeBins> {
        self.bins
            .get_or_init(|| Arc::new(connection_fit::RangeBins::build(&self.body)))
            .clone()
    }
}

/// The keyframes' `KeyframeCloud`s, shared by the searches.
#[derive(Default)]
pub struct KeyframeClouds {
    entries: Mutex<std::collections::HashMap<usize, Arc<KeyframeCloud>>>,
}

impl KeyframeClouds {
    pub(crate) fn get(&self, k: usize, body: &Arc<PointCloud>) -> Arc<KeyframeCloud> {
        self.entries
            .lock()
            .unwrap()
            .entry(k)
            .or_insert_with(|| {
                Arc::new(KeyframeCloud {
                    body: body.clone(),
                    voxelized: OnceLock::new(),
                    bins: OnceLock::new(),
                })
            })
            .clone()
    }
    /// Drop keyframe `k`'s entry (it was simplified away and no search can ask for it again).
    pub(crate) fn forget(&self, k: usize) {
        self.entries.lock().unwrap().remove(&k);
    }
}

/// A target stack of keyframes grown in place: voxelized points deduplicated per world voxel in a growing `KdForest`.
#[derive(Default)]
struct Stack {
    keyframes: std::collections::BTreeSet<usize>,
    voxels: std::collections::HashSet<(i64, i64, i64)>,
    index: KdForest,
}

impl Stack {
    /// Add the keyframes not yet in the stack (each voxel's first point only).
    fn add(&mut self, search: &ClosureSearch, ids: &[usize]) {
        let resolution = search.config.submap_resolution;
        let inverse = (1.0 / resolution) as f32;
        let mut fresh = PointCloud::new();
        for &k in ids {
            if !self.keyframes.insert(k) {
                continue;
            }
            let key_pose = &search.key_poses.get(k);
            let Some(body) = &key_pose.body_cloud else {
                continue;
            };
            let cloud = search.keyframe_clouds.get(k, body);
            for p in transform_cloud(
                cloud.voxelized(resolution),
                &key_pose.rotation_global,
                &key_pose.translation_global,
            ) {
                let key = (
                    (p[0] * inverse).floor() as i64,
                    (p[1] * inverse).floor() as i64,
                    (p[2] * inverse).floor() as i64,
                );
                if p.iter().all(|v| v.is_finite()) && self.voxels.insert(key) {
                    fresh.push(p);
                }
            }
        }
        self.index.push(&fresh);
    }
}

impl ClosureSearch {
    /// Fill in the yank / confidence gate's fields for a candidate ICP has scored.
    pub(crate) fn score_candidate(&self, diag: &mut ClosureCandidateDiag) {
        // yank = Mahalanobis innovation against graph covariance plus the closure's ICP variance
        let p = &diag.innovation_covariance;
        let c = (diag.icp_fitness * diag.icp_overlap).max(1e-6);
        let covariance: Mat3 = std::array::from_fn(|i| {
            std::array::from_fn(|j| p[i][j] + if i == j { c } else { 0.0 })
        });
        let v = diag.innovation;
        diag.yank_stdevs = match mat3::inverse(&covariance) {
            Some(inverse) if v.iter().all(|x| x.is_finite()) => {
                let iv = mat3::mat_vec(&inverse, &v);
                (v[0] * iv[0] + v[1] * iv[1] + v[2] * iv[2]).max(0.0).sqrt()
            }
            _ => f64::INFINITY,
        };
        let unit = |v: f64| {
            if v.is_finite() {
                v.clamp(0.0, 1.0)
            } else {
                0.0
            }
        };
        diag.scan_context_factor = if diag.scan_context_best.is_finite() {
            unit(
                1.0 - diag.scan_context_best as f64
                    / self.config.scan_context_match_threshold.max(f64::EPSILON),
            )
        } else {
            unit(
                1.0 - diag.candidate_distance / self.config.closure_search_radius.max(f64::EPSILON),
            )
        };
        diag.icp_factor = unit(
            1.0 - diag.icp_fitness
                / self
                    .config
                    .closure_icp_max_fitness_m
                    .powi(2)
                    .max(f64::EPSILON),
        );
        diag.degeneracy_factor =
            if self.config.closure_min_degeneracy > 0.0 && diag.degeneracy_min >= 0.0 {
                unit(diag.degeneracy_min as f64 / self.config.closure_min_degeneracy)
            } else {
                1.0
            };
        diag.confidence = if self.config.closure_likelihood_confidence {
            // the ratio is the evidence: confidence = 1 - e^-evidence, none for a ratio that favours a misfit
            1.0 - (-diag.likelihood_ratio.max(0.0)).exp()
        } else {
            diag.scan_context_factor * diag.icp_factor * diag.degeneracy_factor
        };
        diag.required_confidence = 1.0 - (-self.required_evidence(diag)).exp();
    }
    /// `connection_fit::likelihood_ratio` of `source` moved by `icp` against `target`, out to the ICP match distance.
    pub(crate) fn likelihood_ratio(
        &self,
        source: &[[f32; 3]],
        target: &[[f32; 3]],
        tree: &impl NearestIndex,
        icp: &IcpResult,
        target_ids: &[usize],
    ) -> (f64, usize, f64) {
        let moved = transform_cloud(source, &icp.rotation, &icp.translation);
        if !self.config.closure_likelihood_visibility {
            let tree_points = KdTree::build(target);
            return connection_fit::likelihood_ratio(
                &moved,
                target,
                &tree_points,
                self.config.closure_icp_max_correspondence_m,
                None,
            );
        }
        let views: Vec<connection_fit::RangeImage> = target_ids
            .iter()
            .filter_map(|&k| {
                let key_pose = &self.key_poses.get(k);
                let cloud = self.keyframe_clouds.get(k, key_pose.body_cloud.as_ref()?);
                Some(connection_fit::RangeImage::at(
                    cloud.bins(),
                    &key_pose.rotation_global,
                    &key_pose.translation_global,
                ))
            })
            .collect();
        connection_fit::likelihood_ratio(
            &moved,
            target,
            tree,
            self.config.closure_icp_max_correspondence_m,
            Some((self.config.closure_inlier_squared_distance(), &views)),
        )
    }
    /// Evidence (-ln(1 - confidence)) needed before look-alikes: `yank_evidence` + max(0, ln((1 - keep_rate) / keep_rate)).
    pub(crate) fn required_evidence(&self, diag: &ClosureCandidateDiag) -> f64 {
        let width = self.config.closure_yank_gate_width.max(f64::EPSILON);
        let record = if diag.keep_rate.is_finite() {
            ((1.0 - diag.keep_rate) / diag.keep_rate).ln().max(0.0)
        } else {
            0.0
        };
        yank_evidence(
            diag.yank_stdevs / width,
            self.config.closure_yank_drift_range,
        ) + record
    }
    /// Global-frame cloud of keyframes `ids`, voxel-downsampled at `submap_resolution`.
    pub(crate) fn stacked_cloud(&self, ids: impl Iterator<Item = usize>) -> PointCloud {
        let resolution = self.config.submap_resolution;
        let mut cloud = PointCloud::new();
        for k in ids {
            let key_pose = &self.key_poses.get(k);
            if let Some(body_cloud) = &key_pose.body_cloud {
                cloud.extend(transform_cloud(
                    self.keyframe_clouds
                        .get(k, body_cloud)
                        .voxelized(resolution),
                    &key_pose.rotation_global,
                    &key_pose.translation_global,
                ));
            }
        }
        voxel_downsample(&cloud, resolution)
    }
    /// Fit one more source cloud per round on a growing target; median ratio x agreement vs `UncertaintyRange` decides.
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    pub(crate) fn extended_icp(
        &self,
        cur_idx: usize,
        target_idx: usize,
        first: &IcpResult,
        first_ratio: f64,
        diag: &mut ClosureCandidateDiag,
        required: &mut dyn FnMut(f64, &mut ClosureCandidateDiag) -> f64,
        // runs a cloud's computation through the ICP cache; the encoded alignment starts at the given index
        memo: &dyn Fn(&str, usize, &mut dyn FnMut() -> Vec<f64>) -> Vec<f64>,
    ) -> Option<IcpResult> {
        let max_clouds = diag
            .extension_budget
            .min(self.config.closure_extended_icp_max_keyframes);
        if !first.converged {
            return None;
        }
        let accept = |ratio: f64| (ratio >= 1.0).then(|| first.clone());
        let half_range = self.config.target_half_range(1);
        let radius = self.config.closure_search_radius;
        let positions: Vec<Vec3> = self
            .key_poses
            .iter()
            .map(|k| k.translation_global)
            .collect();
        let odometry: Vec<Vec3> = self.key_poses.iter().map(|k| k.translation_local).collect();
        let clouds = extension_clouds(
            &odometry,
            target_idx,
            cur_idx,
            half_range,
            max_clouds,
            self.mean_reach * self.config.extension_cloud_spacing,
        );
        // the target side stays > half_range older than every source cloud, so no scan is on both sides
        let target_end = clouds
            .iter()
            .copied()
            .chain([cur_idx])
            .min()
            .unwrap_or(cur_idx)
            .saturating_sub(half_range);
        let mut target_ids: Vec<usize> = (target_idx.saturating_sub(half_range)
            ..=target_idx + half_range)
            .filter(|&k| k < target_end && k < self.key_poses.len())
            .collect();
        if max_clouds < 2 || target_ids.is_empty() || clouds.is_empty() {
            return accept(first_ratio);
        }
        // refine only (inlier distance, not the single fit's max correspondence), so a half-overlapping cloud can't drift
        let params = IcpParams {
            max_iterations: self.config.closure_icp_max_iterations as usize,
            max_correspondence_distance: self.config.closure_icp_inlier_distance_m,
            inlier_squared_distance: self.config.closure_inlier_squared_distance(),
            point_to_plane: self.config.closure_icp_point_to_plane,
        };
        let record_all = self
            .icp_cache
            .as_ref()
            .is_some_and(|c| c.mode == crate::icp_cache::Mode::Record);
        let submap_size = target_ids.len();
        let mut target = Stack::default();
        target.add(self, &target_ids);
        let range = UncertaintyRange::new(&self.config, clouds.len() + 1, diag.pressure);
        let mut ratios = vec![first_ratio];
        // consecutive alignments' agreement at the source compounds into `multiplier`
        let source_position = positions[cur_idx];
        let mut previous = (first.rotation, first.translation);
        let mut multiplier = 1.0;
        let mut found: Option<Option<IcpResult>> = None;
        diag.extension_clouds_planned = clouds.len();
        for (step, &j) in clouds.iter().enumerate() {
            // round seconds, minus debug dumps
            let round_started = std::time::Instant::now();
            let dumped_before = *self.dump_s.lock().expect("dump seconds");
            let step = step + 1;
            // widen the target submap for this many clouds, then add keyframes near where this cloud lands
            let range_half = self.config.target_half_range(step + 1);
            let widened: Vec<usize> = (target_idx.saturating_sub(range_half)
                ..=target_idx + range_half)
                .filter(|&k| k < target_end && !target_ids.contains(&k))
                .collect();
            target.add(self, &widened);
            target_ids.extend(widened);
            let placed = mat3::add(
                &mat3::mat_vec(&first.rotation, &positions[j]),
                &first.translation,
            );
            let mut near: Vec<(f64, usize)> = (0..target_end)
                .filter(|k| !target_ids.contains(k))
                .map(|k| (mat3::norm(&mat3::sub(&positions[k], &placed)), k))
                .filter(|&(d, _)| d < self.mean_reach)
                .collect();
            near.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let added: Vec<usize> = near.into_iter().map(|(_, k)| k).take(submap_size).collect();
            target.add(self, &added);
            target_ids.extend(added);
            // report this round's estimated cost to the deterministic schedule before its ICP starts
            if let Some(progress) = &self.progress {
                let source_points = self.key_poses.body_cloud(j).as_ref().map_or(0, |body| {
                    self.keyframe_clouds
                        .get(j, body)
                        .voxelized(self.config.submap_resolution)
                        .len()
                });
                let config = &self.config.deterministic_config;
                progress.round_started(
                    config.verification_scans_per_round
                        + config.verification_scans_per_round_point
                            * (source_points + target.index.points().len()) as f64,
                );
            }
            let round = memo(&format!("xc{j}"), 1, &mut || {
                let key_pose = &self.key_poses.get(j);
                let Some(body) = &key_pose.body_cloud else {
                    return vec![f64::NAN; 17];
                };
                let source = transform_cloud(
                    self.keyframe_clouds
                        .get(j, body)
                        .voxelized(self.config.submap_resolution),
                    &key_pose.rotation_global,
                    &key_pose.translation_global,
                );
                let target_points = target.index.points();
                let icp = icp(
                    &source,
                    target_points,
                    &target.index,
                    &first.rotation,
                    &first.translation,
                    &params,
                );
                if !self.config.closure_icp_dump_dir.is_empty() {
                    let t0 = std::time::Instant::now();
                    dump_icp(
                        &self.config.closure_icp_dump_dir,
                        cur_idx,
                        target_idx,
                        &source,
                        target_points,
                        (&first.rotation, &first.translation),
                        &icp,
                        &format!("_c{j}"),
                    );
                    *self.dump_s.lock().expect("dump seconds") += t0.elapsed().as_secs_f64();
                }
                let mut v = vec![source.len() as f64];
                v.extend(crate::icp_cache::encode_icp(&icp));
                v.push(if icp.converged {
                    self.likelihood_ratio(&source, target_points, &target.index, &icp, &target_ids)
                        .0
                } else {
                    f64::NAN
                });
                v
            });
            let icp = crate::icp_cache::decode_icp(&round[1..16]);
            // a cloud that did not fit, or fit > closure_search_radius from the single fit's placement, adds no evidence
            let moved = mat3::norm(&mat3::sub(
                &mat3::add(
                    &mat3::mat_vec(&icp.rotation, &positions[j]),
                    &icp.translation,
                ),
                &placed,
            )) > radius;
            let evidence = if icp.converged && !moved && round[16].is_finite() {
                round[16].max(0.0)
            } else {
                0.0
            };
            let bar = required(evidence, diag);
            ratios.push(if bar > 0.0 {
                evidence / bar
            } else {
                f64::INFINITY
            });
            let median = median_ratio(&ratios);
            let at = |(rotation, translation): (Mat3, Vec3)| {
                mat3::add(&mat3::mat_vec(&rotation, &source_position), &translation)
            };
            let shift = mat3::norm(&mat3::sub(
                &at((icp.rotation, icp.translation)),
                &at(previous),
            ));
            let turn = mat3::angular_distance(&previous.0, &icp.rotation);
            let apart = (shift.powi(2) + (self.mean_reach * turn).powi(2)).sqrt()
                / self.config.closure_icp_inlier_distance_m;
            // capped at 5; a cloud that did not fit counts as 5
            let apart = if apart.is_finite() {
                apart.min(5.0)
            } else {
                5.0
            };
            multiplier *=
                (self.config.compounding_agreement_sensitivity * (1.0 - apart.powi(2))).exp();
            previous = (icp.rotation, icp.translation);
            diag.cloud_agreement.push(apart);
            diag.agreement_multiplier = multiplier;
            diag.extension_clouds_used = step;
            diag.cost_extension_rounds_s.push(
                round_started.elapsed().as_secs_f64()
                    - (*self.dump_s.lock().expect("dump seconds") - dumped_before),
            );
            let score = median * multiplier;
            diag.cloud_ratios = ratios.clone();
            diag.extended_keyframes = ratios.len();
            diag.extended_source_points = round[0] as usize;
            diag.extended_target_points = target.index.points().len();
            diag.extension_rounds_source_points.push(round[0] as usize);
            diag.extension_rounds_target_points
                .push(target.index.points().len());
            diag.extended_fitness = icp.fitness;
            diag.extended_likelihood_ratio = round[16];
            diag.extended_confidence = 1.0 - (-median * bar.max(0.0)).exp();
            let (low, high) = range.at(step);
            let verdict = if score >= high {
                Some(Some(first.clone()))
            } else if score <= low {
                Some(None)
            } else {
                None
            };
            if let Some(verdict) = verdict {
                if !record_all {
                    return verdict;
                }
                found.get_or_insert(verdict);
            }
        }
        match found {
            Some(verdict) => verdict,
            None => accept(median_ratio(&ratios) * multiplier),
        }
    }
}

/// Median of the per-cloud ratios, NaNs dropped (0 if none).
pub(crate) fn median_ratio(ratios: &[f64]) -> f64 {
    let mut sorted: Vec<f64> = ratios.iter().copied().filter(|r| !r.is_nan()).collect();
    if sorted.is_empty() {
        return 0.0;
    }
    sorted.sort_by(f64::total_cmp);
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else if sorted[n / 2 - 1].is_infinite() {
        sorted[n / 2 - 1]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0
    }
}

/// Extension source clouds walking back from `cur_idx`, `spacing` odometry apart, > 2 half_range after target, <= max - 1.
pub(crate) fn extension_clouds(
    odometry: &[Vec3],
    target_idx: usize,
    cur_idx: usize,
    half_range: usize,
    max_clouds: usize,
    spacing: f64,
) -> Vec<usize> {
    let floor = target_idx + 2 * half_range + 1;
    let mut chosen = Vec::new();
    let mut since = 0.0;
    let mut k = cur_idx;
    while k > floor && chosen.len() + 1 < max_clouds {
        k -= 1;
        since += mat3::norm(&mat3::sub(&odometry[k + 1], &odometry[k]));
        if since >= spacing {
            chosen.push(k);
            since = 0.0;
        }
    }
    chosen
}

/// Evidence/bar ratio band: >= high accepts, <= low rejects; closes linearly to 1, stretched by 1/(1-pressure)^rate.
pub(crate) struct UncertaintyRange {
    low: f64,
    high: f64,
    steps: f64,
}

impl UncertaintyRange {
    pub(crate) fn new(
        config: &crate::gsc_pgo::Config,
        clouds: usize,
        pressure: f64,
    ) -> UncertaintyRange {
        let rounds = (clouds.max(2) - 1) as f64;
        let pressure = if pressure.is_finite() {
            pressure.clamp(0.0, 0.999)
        } else {
            0.0
        };
        UncertaintyRange {
            low: config.barely_fail_uncertainty_range,
            high: config.barely_pass_uncertainty_range,
            steps: rounds * (1.0 / (1.0 - pressure)).powf(config.decisive_pressure_rate.max(0.0)),
        }
    }
    /// (low, high) after `step` extension steps (0: the single scan).
    pub(crate) fn at(&self, step: usize) -> (f64, f64) {
        let left = (1.0 - step as f64 / self.steps).max(0.0);
        (
            1.0 - (1.0 - self.low) * left,
            1.0 + (self.high - 1.0) * left,
        )
    }
}

#[cfg(test)]
mod uncertainty_range_tests {
    use super::UncertaintyRange;

    /// The range starts at the configured ratios, closes by the last step at no pressure, and takes twice as long at 50%.
    #[test]
    fn pressure_keeps_the_uncertainty_range_open_longer() {
        let config = crate::gsc_pgo::Config::default();
        // 5 clouds: the single scan and 4 more, so 4 rounds
        let calm = UncertaintyRange::new(&config, 5, 0.0);
        // 1 - (1 - low) is low only to rounding
        let (low, high) = calm.at(0);
        assert!((low - config.barely_fail_uncertainty_range).abs() < 1e-12);
        assert!((high - config.barely_pass_uncertainty_range).abs() < 1e-12);
        assert_eq!(calm.at(4), (1.0, 1.0));
        let pressed = UncertaintyRange::new(&config, 5, 0.5);
        let half = 1.0 + (config.barely_pass_uncertainty_range - 1.0) / 2.0;
        assert!((pressed.at(4).1 - half).abs() < 1e-12);
        assert_eq!(pressed.at(8), (1.0, 1.0));
        for step in 0..8 {
            assert!(pressed.at(step).0 <= calm.at(step).0 && pressed.at(step).1 >= calm.at(step).1);
        }
    }
}

#[cfg(test)]
mod extension_tests {
    use super::{extension_clouds, median_ratio};
    use crate::connection_fit::{icp, IcpParams};
    use crate::mat3;
    use crate::pointcloud::{KdForest, KdTree, NearestIndex, PointCloud};

    /// Source clouds never share a keyframe with the target side, stay half_range apart, and are spaced along odometry.
    #[test]
    fn the_sides_never_share_a_keyframe() {
        let half_range = 5;
        // one metre of odometry per keyframe
        let odometry: Vec<[f64; 3]> = (0..60).map(|k| [k as f64, 0.0, 0.0]).collect();
        for (target_idx, cur_idx) in [(20, 22), (20, 32), (20, 59), (0, 5), (40, 59)] {
            let clouds = extension_clouds(&odometry, target_idx, cur_idx, half_range, 5, 3.0);
            let oldest_source = clouds.iter().copied().chain([cur_idx]).min().unwrap();
            let target_end = oldest_source.saturating_sub(half_range);
            let submap: Vec<usize> = (target_idx.saturating_sub(half_range)
                ..=target_idx + half_range)
                .filter(|&k| k < target_end)
                .collect();
            for &s in clouds.iter().chain([&cur_idx]) {
                assert!(
                    !submap.contains(&s),
                    "{target_idx}->{cur_idx}: {s} on both sides"
                );
                assert!(
                    s >= target_end + half_range,
                    "{target_idx}->{cur_idx}: {s} within the gap"
                );
            }
            for pair in std::iter::once(cur_idx)
                .chain(clouds.iter().copied())
                .collect::<Vec<_>>()
                .windows(2)
            {
                assert!(
                    pair[0] - pair[1] >= 3,
                    "{target_idx}->{cur_idx}: clouds {pair:?} closer than the spacing"
                );
            }
            assert!(clouds.len() <= 4);
        }
    }

    /// The median: one cloud alone cannot carry a candidate, and one bad cloud cannot sink it.
    #[test]
    fn the_median_of_the_cloud_ratios_scores_a_candidate() {
        assert_eq!(median_ratio(&[1.2]), 1.2);
        assert_eq!(median_ratio(&[0.2, 1.5]), 0.85);
        assert_eq!(median_ratio(&[0.2, 1.5, 1.1]), 1.1);
        assert_eq!(median_ratio(&[5.0, 0.1, 0.0, 0.2]), 0.15000000000000002);
        assert_eq!(median_ratio(&[f64::INFINITY, f64::INFINITY]), f64::INFINITY);
        assert_eq!(median_ratio(&[f64::NAN, 0.9]), 0.9);
        assert_eq!(median_ratio(&[]), 0.0);
    }

    /// A grown `KdForest` answers, and ICPs, exactly like one tree over the same points.
    #[test]
    fn a_grown_index_matches_a_rebuilt_one() {
        let mut points = PointCloud::new();
        let mut seed = 12345u64;
        let mut next = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 33) as f64 / (1u64 << 31) as f64 * 4.0 - 2.0) as f32
        };
        for _ in 0..3000 {
            points.push([next(), next(), next() * 0.5]);
        }
        let mut forest = KdForest::default();
        for chunk in points.chunks(173) {
            forest.push(chunk);
        }
        let tree = KdTree::build(&points);
        for q in points.iter().step_by(37) {
            let probe = [q[0] + 0.01, q[1] - 0.02, q[2]];
            assert_eq!(
                forest.nearest(&probe).unwrap().1,
                tree.nearest(&probe).unwrap().1
            );
            let a: Vec<f32> = forest.knn(&probe, 10).iter().map(|x| x.1).collect();
            let b: Vec<f32> = tree.knn(&probe, 10).iter().map(|x| x.1).collect();
            assert_eq!(a, b);
        }
        let source: PointCloud = points
            .iter()
            .step_by(3)
            .map(|p| [p[0] + 0.05, p[1] - 0.03, p[2]])
            .collect();
        let params = IcpParams {
            max_iterations: 30,
            max_correspondence_distance: 1.0,
            inlier_squared_distance: 0.04,
            point_to_plane: false,
        };
        let a = icp(
            &source,
            forest.points(),
            &forest,
            &mat3::identity(),
            &[0.0; 3],
            &params,
        );
        let b = icp(
            &source,
            &points,
            &tree,
            &mat3::identity(),
            &[0.0; 3],
            &params,
        );
        assert!(mat3::norm(&mat3::sub(&a.translation, &b.translation)) < 1e-9);
        assert!(
            mat3::norm(&mat3::sub(&a.translation, &[-0.05, 0.03, 0.0])) < 0.01,
            "{:?}",
            a.translation
        );
    }
}

/// Evidence cost of a `z`-stdev yank with drift scale log-uniform in [1, drift_range]: z^2/2 at 1, ~3 ln z when wide.
pub(crate) fn yank_evidence(z: f64, drift_range: f64) -> f64 {
    let range = drift_range.max(1.0);
    if range <= 1.0 || !z.is_finite() {
        return z * z / 2.0;
    }
    const STEPS: usize = 64;
    let top = range.ln();
    let likelihood = |z: f64| {
        (0..=STEPS)
            .map(|i| {
                let ln_s = top * i as f64 / STEPS as f64;
                let weight = if i == 0 || i == STEPS { 0.5 } else { 1.0 };
                weight * (-3.0 * ln_s - z * z / (2.0 * (2.0 * ln_s).exp())).exp()
            })
            .sum::<f64>()
    };
    (likelihood(0.0) / likelihood(z).max(f64::MIN_POSITIVE)).ln()
}

#[cfg(test)]
mod yank_evidence_tests {
    use super::yank_evidence;

    /// drift_range 1 prices a yank at z^2 / 2; 100x prices a 79-sigma jump at ~a dozen and never exceeds z^2 / 2.
    #[test]
    fn a_wide_drift_range_caps_large_yanks() {
        assert!((yank_evidence(3.0, 1.0) - 4.5).abs() < 1e-12);
        let big = yank_evidence(79.0, 100.0);
        assert!(big > 10.0 && big < 14.0, "{big}");
        for z in [0.5, 1.0, 3.0, 10.0, 100.0] {
            assert!(yank_evidence(z, 100.0) <= z * z / 2.0 + 1e-9);
        }
    }
}
