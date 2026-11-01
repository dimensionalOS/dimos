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

//! Stage 4, proportional confidence: how much evidence a connection needs grows with how hard it pulls
//! the graph. The yank (how far the closure would move its source keyframe from where the graph has it) is measured in
//! standard deviations of its predicted spread (the graph's covariance of the two ends plus the closure's ICP variance)
//! into z; the connection's confidence (scan context x ICP fitness x degeneracy) must reach
//! 1 - exp(-required evidence) (yank, the solve's keep record, look-alike places), and one that falls short gets a second
//! look with extended ICP on more keyframes, as many as the loop-overlap pressure allows.

use crate::connection_fit::{self, dump_icp, icp, IcpParams, IcpResult};
use crate::gsc_pgo::{ClosureCandidateDiag, ClosureSearch};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    above_floor, transform_cloud, voxel_downsample, KdForest, KdTree, NearestIndex, PointCloud,
};
use std::sync::{Arc, Mutex, OnceLock};

/// One keyframe's scan as every closure search uses it, built once: voxelized in its own frame (placed by its pose when
/// stacked) and its range image (`connection_fit::RangeBins`). Neither depends on the pose, so solves never stale them.
pub struct KeyframeCloud {
    body: Arc<PointCloud>,
    voxelized: OnceLock<PointCloud>,
    bins: OnceLock<Arc<connection_fit::RangeBins>>,
    reach: OnceLock<f64>,
}

impl KeyframeCloud {
    pub(crate) fn voxelized(&self, resolution: f64) -> &PointCloud {
        self.voxelized
            .get_or_init(|| voxel_downsample(&self.body, resolution))
    }
    /// How far the scan sees across the ground: its points' 95th-percentile planar range.
    pub(crate) fn reach(&self) -> f64 {
        *self.reach.get_or_init(|| {
            let mut ranges: Vec<f64> = self
                .body
                .iter()
                .map(|p| (p[0] as f64).hypot(p[1] as f64))
                .collect();
            if ranges.is_empty() {
                return 0.0;
            }
            let k = (ranges.len() * 95 / 100).min(ranges.len() - 1);
            *ranges.select_nth_unstable_by(k, f64::total_cmp).1
        })
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
                    reach: OnceLock::new(),
                })
            })
            .clone()
    }
}

/// A stack of keyframes grown in place: their voxelized points, merged per world voxel (centroid), and for a target
/// the index searched against (a `KdForest`, so a step adds only the new keyframes' points).
#[derive(Default)]
struct Stack {
    keyframes: std::collections::BTreeSet<usize>,
    voxels: std::collections::HashMap<(i64, i64, i64), usize>,
    sums: Vec<([f64; 3], u32)>,
    /// target only: the floor (5th-percentile height of the first step) and the indexed points above it
    floor: Option<f32>,
    index: KdForest,
}

impl Stack {
    /// Add the keyframes not yet in the stack; returns the target points added (above the floor).
    fn add(&mut self, search: &ClosureSearch, ids: &[usize], target: bool) -> usize {
        let resolution = search.config.submap_resolution;
        let inverse = (1.0 / resolution) as f32;
        let mut placed = PointCloud::new();
        for &k in ids {
            if !self.keyframes.insert(k) {
                continue;
            }
            let key_pose = &search.key_poses[k];
            let Some(body) = &key_pose.body_cloud else {
                continue;
            };
            let cloud = search.keyframe_clouds.get(k, body);
            placed.extend(transform_cloud(
                cloud.voxelized(resolution),
                &key_pose.rotation_global,
                &key_pose.translation_global,
            ));
        }
        let mut fresh = PointCloud::new();
        for p in placed {
            if !p.iter().all(|v| v.is_finite()) {
                continue;
            }
            let key = (
                (p[0] * inverse).floor() as i64,
                (p[1] * inverse).floor() as i64,
                (p[2] * inverse).floor() as i64,
            );
            match self.voxels.get(&key) {
                Some(&i) if !target => {
                    let (sum, count) = &mut self.sums[i];
                    (0..3).for_each(|a| sum[a] += p[a] as f64);
                    *count += 1;
                }
                Some(_) => {}
                None => {
                    self.voxels.insert(key, self.sums.len());
                    self.sums.push((p.map(|v| v as f64), 1));
                    fresh.push(p);
                }
            }
        }
        if !target {
            return 0;
        }
        let clearance = search.config.closure_ground_clearance_m;
        if self.floor.is_none() && clearance > 0.0 && !fresh.is_empty() {
            let mut heights: Vec<f32> = fresh.iter().map(|p| p[2]).collect();
            let k = heights.len() / 20;
            self.floor = Some(*heights.select_nth_unstable_by(k, f32::total_cmp).1);
        }
        if let Some(floor) = self.floor {
            fresh.retain(|p| (p[2] - floor) as f64 > clearance);
        }
        self.index.push(&fresh);
        fresh.len()
    }
    /// The source stack's points (voxel centroids).
    fn points(&self) -> PointCloud {
        self.sums
            .iter()
            .map(|(sum, count)| sum.map(|v| (v / *count as f64) as f32))
            .collect()
    }
}

impl ClosureSearch {
    /// Fill in the yank / confidence gate's inputs and outputs for a candidate ICP has scored, and
    /// return its yank for the scale.
    pub(crate) fn score_candidate(&self, diag: &mut ClosureCandidateDiag) {
        // the innovation against its predicted spread: the graph's covariance of the two ends' relative position plus the
        // closure's variance, its ICP inliers' mean square (`IcpResult::inlier_mean_square`)
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
        let half = self.config.closure_sharpness_half;
        diag.icp_factor = if half > 0.0 {
            unit(diag.sharpness / (diag.sharpness + half))
        } else {
            unit(
                1.0 - diag.icp_fitness
                    / self
                        .config
                        .closure_icp_max_fitness_m
                        .powi(2)
                        .max(f64::EPSILON),
            )
        };
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
    /// `connection_fit::sharpness` of `source` moved by `icp` onto the tree's cloud, by the configured nudge.
    pub(crate) fn sharpness(
        &self,
        source: &PointCloud,
        tree: &impl NearestIndex,
        icp: &IcpResult,
    ) -> f64 {
        connection_fit::sharpness(
            &transform_cloud(source, &icp.rotation, &icp.translation),
            tree,
            self.config.submap_resolution,
            self.config.closure_inlier_squared_distance(),
            self.config.closure_sharpness_nudge_m,
            self.config.closure_sharpness_nudge_deg,
        )
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
            );
        }
        let views: Vec<connection_fit::RangeImage> = target_ids
            .iter()
            .filter_map(|&k| {
                let key_pose = &self.key_poses[k];
                let cloud = self.keyframe_clouds.get(k, key_pose.body_cloud.as_ref()?);
                Some(connection_fit::RangeImage::at(
                    cloud.bins(),
                    &key_pose.rotation_global,
                    &key_pose.translation_global,
                ))
            })
            .collect();
        connection_fit::likelihood_ratio_visible(
            &moved,
            target,
            tree,
            self.config.closure_icp_max_correspondence_m,
            self.config.closure_inlier_squared_distance(),
            &views,
            false,
        )
    }
    /// Evidence (-ln(1 - confidence)) a candidate must reach before look-alike places: its yank's (`yank_evidence`), plus,
    /// when the solve has been rejecting more closures than it keeps, the log-odds against a passed closure being kept,
    /// ln((1 - keep_rate) / keep_rate) (never below 0, so a good record never lowers the bar).
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
    /// Global-frame cloud of keyframes `ids` (each keyframe's shared voxelized scan moved by its global pose, in order),
    /// voxel-downsampled at the submap resolution.
    pub(crate) fn stacked_cloud(&self, ids: impl Iterator<Item = usize>) -> PointCloud {
        let resolution = self.config.submap_resolution;
        let mut cloud = PointCloud::new();
        for k in ids {
            let key_pose = &self.key_poses[k];
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
    /// Extended ICP: stacks of 2, 4, 8, ... keyframes per side up to `diag.extension_budget`, chosen by how close they
    /// sit to the match under the single-scan fit (`extension_pools`, any time order, the sides kept apart), grown in
    /// place (each step adds only its new keyframes' points to the stacks and the target's index) and warm-started from
    /// the previous step. Returns the extended ICP of the first step whose evidence reaches `required(evidence)`, or
    /// None; it stops once a step's evidence stops growing or its fit leaves the single fit's place (moves any stacked
    /// source point farther than the search radius from where the single fit put it).
    pub(crate) fn extended_icp(
        &self,
        cur_idx: usize,
        target_idx: usize,
        source_points: usize,
        first: &IcpResult,
        first_ratio: f64,
        diag: &mut ClosureCandidateDiag,
        required: &mut dyn FnMut(f64, &mut ClosureCandidateDiag) -> f64,
        // an alignment's cloud computation through the cache, its encoded alignment starting at the given index
        memo: &dyn Fn(&str, usize, &mut dyn FnMut() -> Vec<f64>) -> Vec<f64>,
    ) -> Option<IcpResult> {
        let max_keyframes = diag
            .extension_budget
            .min(self.config.closure_extended_icp_max_keyframes);
        if !first.converged {
            return None;
        }
        let single = (first.overlap * source_points as f64).round() as usize;
        diag.single_inliers = single;
        if max_keyframes < 2 || single == 0 {
            return (first_ratio >= 1.0).then(|| first.clone());
        }
        let params = IcpParams {
            max_iterations: self.config.closure_icp_max_iterations as usize,
            max_correspondence_distance: self.config.closure_icp_max_correspondence_m,
            inlier_squared_distance: self.config.closure_inlier_squared_distance(),
            point_to_plane: self.config.closure_icp_point_to_plane,
        };
        let half_range = self.config.closure_submap_half_range.max(0) as usize;
        let radius = self.config.closure_search_radius;
        let positions: Vec<Vec3> = self
            .key_poses
            .iter()
            .map(|k| k.translation_global)
            .collect();
        let placed_cur = mat3::add(
            &mat3::mat_vec(&first.rotation, &positions[cur_idx]),
            &first.translation,
        );
        // a keyframe can add to a stack wherever its scan overlaps the match's: two scans of reach r cover discs that
        // meet while their centres are within 2r
        let reach = 2.0
            * self.key_poses[cur_idx]
                .body_cloud
                .as_ref()
                .map_or(radius, |body| {
                    self.keyframe_clouds.get(cur_idx, body).reach()
                })
                .max(radius);
        let (source_pool, target_pool, base_len) = extension_pools(
            &positions,
            target_idx,
            cur_idx,
            half_range,
            max_keyframes,
            reach,
            &placed_cur,
        );
        // recording an ICP cache, every step up to the budget is computed (so a replay that extends further, or
        // differently, still finds it), but the result and the diagnostics stay those of the first step that passed
        let record_all = self
            .icp_cache
            .as_ref()
            .is_some_and(|c| c.mode == crate::icp_cache::Mode::Record);
        let sharp = self.config.closure_sharpness_half > 0.0;
        let likelihood = self.config.closure_likelihood_confidence;
        // what is decided when the extension cannot go on: the newest converged step (the single fit before any) on
        // which side of its bar it fell
        let mut last = (first.clone(), diag.clone(), first_ratio);
        let decide = |last: (IcpResult, ClosureCandidateDiag, f64),
                      diag: &mut ClosureCandidateDiag| {
            let (icp, kept, ratio) = last;
            *diag = kept;
            (ratio >= 1.0).then_some(icp)
        };
        if target_pool.is_empty() {
            return decide(last, diag);
        }
        let range = UncertaintyRange::new(&self.config, max_keyframes, diag.pressure);
        let mut source = Stack::default();
        let mut target = Stack::default();
        let (mut seed_rotation, mut seed_translation) = (first.rotation, first.translation);
        let mut found: Option<Option<(IcpResult, ClosureCandidateDiag)>> = None;
        let mut previous_evidence = f64::NEG_INFINITY;
        let mut keyframes = 2;
        let mut step = 0;
        while keyframes <= max_keyframes {
            step += 1;
            let source_ids: Vec<usize> = std::iter::once(cur_idx)
                .chain(source_pool.iter().copied())
                .take(keyframes)
                .collect();
            let target_ids: Vec<usize> = target_pool
                .iter()
                .copied()
                .take(base_len + keyframes)
                .collect();
            if keyframes > 2
                && source_ids.len() + target_ids.len()
                    <= source.keyframes.len() + target.keyframes.len()
            {
                break;
            }
            let round = memo(&format!("xo{keyframes}"), 2, &mut || {
                source.add(self, &source_ids, false);
                target.add(self, &target_ids, true);
                // the stacked source is thinned to about the single scan's size, so a step costs about what the first
                // ICP did; the target keeps full resolution so the inlier test means the same thing in every step
                let clearance = self.config.closure_ground_clearance_m;
                let stacked = above_floor(&source.points(), clearance);
                let thinning = (stacked.len() as f64 / source_points.max(1) as f64).max(1.0);
                let thinned = if thinning > 1.0 {
                    voxel_downsample(&stacked, self.config.submap_resolution * thinning.sqrt())
                } else {
                    stacked.clone()
                };
                let target_points = target.index.points();
                let icp = icp(
                    &thinned,
                    target_points,
                    &target.index,
                    &seed_rotation,
                    &seed_translation,
                    &params,
                );
                if !self.config.closure_icp_dump_dir.is_empty() {
                    let t0 = std::time::Instant::now();
                    dump_icp(
                        &self.config.closure_icp_dump_dir,
                        cur_idx,
                        target_idx,
                        &thinned,
                        target_points,
                        &icp,
                        &format!("_x{keyframes}"),
                    );
                    self.debug_io_s
                        .set(self.debug_io_s.get() + t0.elapsed().as_secs_f64());
                }
                let score = |on: bool, f: &dyn Fn() -> f64| {
                    if on && icp.converged {
                        f()
                    } else {
                        f64::NAN
                    }
                };
                let mut v = vec![stacked.len() as f64, target_points.len() as f64];
                v.extend(crate::icp_cache::encode_icp(&icp));
                v.push(score(sharp, &|| {
                    self.sharpness(&thinned, &target.index, &icp)
                }));
                v.push(score(likelihood, &|| {
                    self.likelihood_ratio(&thinned, target_points, &target.index, &icp, &target_ids)
                        .0
                }));
                v
            });
            let (stacked_len, target_len) = (round[0] as usize, round[1] as usize);
            let icp = crate::icp_cache::decode_icp(&round[2..17]);
            // inliers of the full stack, estimated from the thinned one's inlier fraction
            let extended = if icp.converged {
                (icp.overlap * stacked_len as f64).round() as usize
            } else {
                0
            };
            let held_up = ((extended as f64 / stacked_len.max(1) as f64)
                / (single as f64 / source_points as f64))
                .min(1.0);
            let per_scan = diag.confidence * held_up;
            if sharp && icp.converged {
                diag.extended_sharpness = round[17];
            }
            if likelihood && icp.converged {
                diag.extended_likelihood_ratio = round[18];
            }
            // evidence = -ln(1 - confidence); the stack adds evidence as far as it covers surface the single scan did
            // not: its per-point ratio times its size over the single scan's (both voxelized at the submap resolution, so
            // a keyframe seeing the same ground again adds nothing). Scored by sharpness, the stack earns its own
            // confidence; otherwise its evidence grows with its inlier count
            let evidence = if likelihood {
                diag.extended_likelihood_ratio.max(0.0) * stacked_len as f64
                    / source_points.max(1) as f64
            } else if sharp {
                let half = self.config.closure_sharpness_half;
                -(1.0
                    - diag.scan_context_factor
                        * diag.degeneracy_factor
                        * (diag.extended_sharpness / (diag.extended_sharpness + half))
                            .clamp(0.0, 1.0))
                .ln()
            } else {
                -(1.0 - per_scan).ln() * extended as f64 / single as f64
            };
            let required = required(evidence, diag);
            let confidence = 1.0 - (-evidence).exp();
            diag.extended_keyframes = source_ids.len();
            diag.extended_source_points = stacked_len;
            diag.extended_target_points = target_len;
            diag.extended_inliers = extended;
            diag.extended_fitness = icp.fitness;
            diag.extended_confidence = if confidence.is_finite() {
                confidence.clamp(0.0, 1.0)
            } else {
                0.0
            };
            if !icp.converged {
                break;
            }
            // a step that moves the stack off the single fit's place found a different place, not more of this one
            let moved = source_ids.iter().any(|&k| {
                let p = &positions[k];
                let a = mat3::add(&mat3::mat_vec(&icp.rotation, p), &icp.translation);
                let b = mat3::add(&mat3::mat_vec(&first.rotation, p), &first.translation);
                mat3::norm(&mat3::sub(&a, &b)) > radius
            }) && self.config.closure_extension_stop_on_swing;
            let ratio = if required > 0.0 {
                evidence / required
            } else {
                f64::INFINITY
            };
            let flat =
                self.config.closure_extension_stop_when_flat && evidence <= previous_evidence;
            let (low, high) = range.at(step);
            let verdict = if moved {
                Some(None)
            } else if ratio >= high {
                Some(Some((icp.clone(), diag.clone())))
            } else if ratio <= low {
                Some(None)
            } else if flat {
                // no new information: decide by which side of the bar it sits
                Some((ratio >= 1.0).then(|| (icp.clone(), diag.clone())))
            } else {
                None
            };
            last = (icp.clone(), diag.clone(), ratio);
            if let Some(verdict) = verdict {
                if !record_all {
                    return verdict.map(|(icp, kept)| {
                        *diag = kept;
                        icp
                    });
                }
                found.get_or_insert(verdict);
            }
            previous_evidence = evidence;
            if !sharp {
                (seed_rotation, seed_translation) = (icp.rotation, icp.translation);
            }
            keyframes *= 2;
        }
        match found {
            Some(verdict) => verdict.map(|(icp, kept)| {
                *diag = kept;
                icp
            }),
            None => decide(last, diag),
        }
    }
}

/// The uncertainty range: ratios of a candidate's evidence to its bar (both in yank units). At or above `high` it accepts,
/// at or below `low` it rejects, and in between the stack grows. Both ratios move linearly to 1 over the extension's
/// steps, stretched by 1 / (1 - pressure)^`decisive_pressure_rate` (50% pressure: twice as long at the default rate 1),
/// so the budget's end forces a decision at ratio >= 1 while a high pressure keeps the uncertainty range wide longer.
pub(crate) struct UncertaintyRange {
    low: f64,
    high: f64,
    steps: f64,
}

impl UncertaintyRange {
    pub(crate) fn new(
        config: &crate::gsc_pgo::Config,
        max_keyframes: usize,
        pressure: f64,
    ) -> UncertaintyRange {
        let doublings = (max_keyframes.max(2) as f64).log2().floor().max(1.0);
        let pressure = if pressure.is_finite() {
            pressure.clamp(0.0, 0.999)
        } else {
            0.0
        };
        UncertaintyRange {
            low: config.barely_fail_uncertainty_range,
            high: config.barely_pass_uncertainty_range,
            steps: doublings
                * (1.0 / (1.0 - pressure)).powf(config.decisive_pressure_rate.max(0.0)),
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

/// The keyframes an extension may stack, each list nearest first (the target's own submap ahead of the rest; the third
/// value is its size). Source: the newer side, keyframes from `split` to `cur_idx` within `reach` (how far a scan
/// overlaps another) of the current keyframe. Target: the older side, any keyframe up to `split - 1 - half_range`
/// (either side of `target_idx` in time) within `reach` of where the single fit puts the current keyframe. The sides
/// never share a keyframe, and `half_range` keyframes separate them so neighbouring scans (which share points) cannot
/// land on both; `split` leaves room for the target's own submap (`target_idx` +- `half_range`) and at most twice the
/// budget of newer keyframes.
pub(crate) fn extension_pools(
    positions: &[Vec3],
    target_idx: usize,
    cur_idx: usize,
    half_range: usize,
    max_keyframes: usize,
    reach: f64,
    placed_cur: &Vec3,
) -> (Vec<usize>, Vec<usize>, usize) {
    let split = (target_idx + 2 * half_range + 1)
        .max(cur_idx.saturating_sub(2 * max_keyframes))
        .min(cur_idx);
    let near = |ids: std::ops::Range<usize>, to: &Vec3| -> Vec<usize> {
        let mut found: Vec<(f64, usize)> = ids
            .map(|k| (mat3::norm(&mat3::sub(&positions[k], to)), k))
            .filter(|&(d, _)| d < reach)
            .collect();
        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        found.into_iter().map(|(_, k)| k).collect()
    };
    let source = near(split..cur_idx, &positions[cur_idx]);
    let target_end = split.saturating_sub(half_range);
    // the target's own submap first (as far as it is old enough), then the rest nearest first
    let base: Vec<usize> = (target_idx.saturating_sub(half_range)..=target_idx + half_range)
        .filter(|&j| j < target_end)
        .collect();
    let target = base
        .iter()
        .copied()
        .chain(
            near(0..target_end, placed_cur)
                .into_iter()
                .filter(|j| !base.contains(j)),
        )
        .collect();
    (source, target, base.len())
}

#[cfg(test)]
mod uncertainty_range_tests {
    use super::UncertaintyRange;

    /// The uncertainty range starts at the configured ratios, is shut (both at 1) by the budget's last step at no pressure, and at
    /// 50% pressure takes twice as many steps to close.
    #[test]
    fn pressure_keeps_the_uncertainty_range_open_longer() {
        let config = crate::gsc_pgo::Config::default();
        let calm = UncertaintyRange::new(&config, 16, 0.0);
        assert_eq!(
            calm.at(0),
            (
                config.barely_fail_uncertainty_range,
                config.barely_pass_uncertainty_range
            )
        );
        assert_eq!(calm.at(4), (1.0, 1.0));
        let pressed = UncertaintyRange::new(&config, 16, 0.5);
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
    use super::extension_pools;
    use crate::connection_fit::{icp, IcpParams};
    use crate::mat3;
    use crate::pointcloud::{KdForest, KdTree, NearestIndex, PointCloud};

    /// A short-gap revisit (the current keyframe a few after the target): the source and target sides share no keyframe and
    /// stay half_range apart, however close in space their scans are.
    #[test]
    fn the_sides_never_share_a_keyframe() {
        // a robot standing still: every keyframe at the origin, so every keyframe is near every other
        let positions = vec![[0.0; 3]; 60];
        for (target_idx, cur_idx) in [(20, 22), (20, 30), (20, 59), (0, 5), (40, 59)] {
            let half_range = 5;
            let (source, target, _) = extension_pools(
                &positions, target_idx, cur_idx, half_range, 16, 3.0, &[0.0; 3],
            );
            let Some(&target_top) = target.iter().max() else {
                continue;
            };
            for &s in source.iter().chain([&cur_idx]) {
                assert!(
                    !target.contains(&s),
                    "{target_idx}->{cur_idx}: {s} on both sides"
                );
                assert!(
                    s > target_top + half_range,
                    "{target_idx}->{cur_idx}: source {s} within {half_range} of target {target_top}"
                );
            }
            assert!(source.iter().all(|&s| s < cur_idx));
        }
    }

    /// The growing index answers exactly as one tree over the same points, and ICP against it lands where ICP against a
    /// tree built from scratch does.
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

/// The evidence a yank of `z` standard deviations (at the stated odometry error estimate) costs when the drift's true scale is
/// unknown: -ln of the jump's likelihood averaged over a drift standard deviation from 1 to `drift_range` times the stated
/// (log-uniform), against a jump of 0. A 3-dof jump at scale s is N(z / s) with density s^-3; at `drift_range` 1 this is
/// z^2 / 2, the stated odometry error estimate alone. Larger ranges let a big jump cost about 3 ln z instead of z^2 / 2, so the match
/// evidence and GNC decide it rather than the stated odometry error estimate.
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

    /// Trusting the stated odometry error estimate exactly prices a yank as z^2 / 2; an unknown drift up to 100x prices a 79-sigma jump
    /// (huge's missed return to the start) at a dozen yank units, and leaves no yank more expensive than z^2 / 2.
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
