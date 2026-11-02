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
//! the graph. The yank (how far the closure would move its source keyframe from where the graph has it) is scaled against the yanks of the candidates scored so far
//! into z; the connection's confidence (scan context x ICP fitness x degeneracy) must reach
//! 1 - exp(-required evidence) (yank, the solve's keep record, look-alike places), and one that falls short gets a second
//! look with extended ICP on more keyframes, as many as the loop-overlap pressure allows.

use crate::connection_fit::{
    self, dump_icp, icp, IcpParams, IcpResult, OverlapScore, OverlapScoreParams,
};
use crate::gsc_pgo::{ClosureCandidateDiag, ClosureSearch};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{above_floor, transform_cloud, voxel_downsample, KdTree, PointCloud};

impl ClosureSearch {
    /// Fill in the yank / confidence gate's inputs and outputs for a candidate ICP has scored, and
    /// return its yank for the scale.
    pub(crate) fn score_candidate(&self, diag: &mut ClosureCandidateDiag) -> (f64, f64) {
        let robust_scale = |values: Vec<f64>| -> f64 {
            let mut values = values;
            values.sort_by(f64::total_cmp);
            (1.4826 * values[values.len() / 2]).max(f64::EPSILON)
        };
        // odometry drift is a random walk, so the yank a correct closure needs grows with sqrt(keyframes between its ends)
        let drift = (diag.source_id.abs_diff(diag.target_id as usize) as f64)
            .max(1.0)
            .sqrt();
        diag.yank_drift_scale = drift;
        let predicted = self.config.closure_yank_gate_principled.then(|| {
            let p = &diag.innovation_covariance;
            let c = diag.closure_noise_scale * diag.icp_fitness.max(1e-6);
            let covariance: Mat3 = std::array::from_fn(|i| {
                std::array::from_fn(|j| p[i][j] + if i == j { c } else { 0.0 })
            });
            covariance
        });
        if let Some(covariance) = predicted {
            // the innovation against its predicted spread (the graph's, both noise scales applied, plus the closure's)
            let v = diag.innovation;
            diag.yank_stdevs = match mat3::inverse(&covariance) {
                Some(inverse) if v.iter().all(|x| x.is_finite()) => {
                    let iv = mat3::mat_vec(&inverse, &v);
                    (v[0] * iv[0] + v[1] * iv[1] + v[2] * iv[2]).max(0.0).sqrt()
                }
                _ => f64::INFINITY,
            };
        } else if self.scored_yanks.is_empty() {
            // no yank scale yet: nothing vouches for this yank, so no confidence clears it (it only seeds the scale)
            diag.yank_stdevs = f64::INFINITY;
        } else {
            diag.sigma_rotation_deg = robust_scale(self.scored_yanks.iter().map(|y| y.0).collect());
            diag.sigma_translation_m =
                robust_scale(self.scored_yanks.iter().map(|y| y.1).collect());
            diag.yank_stdevs = (diag.yank_rotation_deg / drift / diag.sigma_rotation_deg)
                .hypot(diag.yank_translation / drift / diag.sigma_translation_m);
        }
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
        } else if self.config.closure_overlap_score {
            unit(diag.overlap_score)
        } else {
            diag.scan_context_factor * diag.icp_factor * diag.degeneracy_factor
        };
        diag.required_confidence = 1.0 - (-self.required_evidence(diag)).exp();
        (
            diag.yank_rotation_deg / drift,
            diag.yank_translation / drift,
        )
    }
    /// `connection_fit::sharpness` of `source` moved by `icp` onto the tree's cloud, by the configured nudge.
    pub(crate) fn sharpness(&self, source: &PointCloud, tree: &KdTree, icp: &IcpResult) -> f64 {
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
        source: &PointCloud,
        target: &PointCloud,
        tree: &KdTree,
        icp: &IcpResult,
        target_ids: &[usize],
    ) -> (f64, usize, f64) {
        let moved = transform_cloud(source, &icp.rotation, &icp.translation);
        if !self.config.closure_likelihood_visibility {
            return connection_fit::likelihood_ratio(
                &moved,
                target,
                tree,
                self.config.closure_icp_max_correspondence_m,
            );
        }
        let views: Vec<connection_fit::RangeImage> = target_ids
            .iter()
            .filter_map(|&k| {
                let key_pose = &self.key_poses[k];
                Some(connection_fit::RangeImage::build(
                    key_pose.body_cloud.as_ref()?,
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
            self.config.closure_likelihood_beam,
        )
    }
    /// `connection_fit::overlap_score` of `source` moved by `icp` against `target`, by the `closure_overlap_*` knobs.
    pub(crate) fn overlap_score(
        &self,
        source: &PointCloud,
        target: &PointCloud,
        icp: &IcpResult,
    ) -> OverlapScore {
        connection_fit::overlap_score(
            &transform_cloud(source, &icp.rotation, &icp.translation),
            target,
            &OverlapScoreParams {
                sigma: self.config.closure_overlap_sigma_m,
                reach: self.config.closure_overlap_reach_m,
                overlap_noise: self.config.closure_overlap_noise_sigmas,
                per_doubling: self.config.closure_overlap_per_doubling,
                coin_flip: self.config.closure_overlap_coin_flip,
                local: self.config.closure_overlap_local,
            },
        )
    }
    /// Evidence (-ln(1 - confidence)) a candidate must reach before look-alike places: z^2 / 2k^2 for its yank, plus,
    /// when the solve has been rejecting more closures than it keeps, the log-odds against a passed closure being kept,
    /// ln((1 - keep_rate) / keep_rate) (never below 0, so a good record never lowers the bar).
    pub(crate) fn required_evidence(&self, diag: &ClosureCandidateDiag) -> f64 {
        let width = self.config.closure_yank_gate_width.max(f64::EPSILON);
        let record = if diag.keep_rate.is_finite() {
            ((1.0 - diag.keep_rate) / diag.keep_rate).ln().max(0.0)
        } else {
            0.0
        };
        diag.yank_stdevs * diag.yank_stdevs / (2.0 * width * width) + record
    }
    /// Global-frame cloud of keyframes `ids` (body clouds moved by their global poses, in order), voxel-downsampled at the
    /// submap resolution.
    pub(crate) fn stacked_cloud(&self, ids: impl Iterator<Item = usize>) -> PointCloud {
        let mut cloud = PointCloud::new();
        for k in ids {
            let key_pose = &self.key_poses[k];
            if let Some(body_cloud) = &key_pose.body_cloud {
                cloud.extend(transform_cloud(
                    body_cloud,
                    &key_pose.rotation_global,
                    &key_pose.translation_global,
                ));
            }
        }
        voxel_downsample(&cloud, self.config.submap_resolution)
    }
    /// Extended ICP on 2, 4, 8, ... keyframes, up to `diag.extension_budget` (closure_extended_icp_max_keyframes x
    /// pressure). Returns the extended ICP of the first round whose evidence reaches `required(evidence)` (which may
    /// count more look-alike places now that the evidence could cover them), or None.
    pub(crate) fn extended_icp(
        &self,
        cur_idx: usize,
        target_idx: usize,
        source_points: usize,
        first: &IcpResult,
        diag: &mut ClosureCandidateDiag,
        required: &mut dyn FnMut(f64, &mut ClosureCandidateDiag) -> f64,
        // an alignment's cloud computation through the cache, its encoded alignment starting at the given index
        memo: &dyn Fn(&str, usize, &mut dyn FnMut() -> Vec<f64>) -> Vec<f64>,
    ) -> Option<IcpResult> {
        let max_keyframes = diag
            .extension_budget
            .min(self.config.closure_extended_icp_max_keyframes);
        if max_keyframes < 2 || !first.converged {
            return None;
        }
        let max_distance = self.config.closure_icp_max_correspondence_m;
        let single = (first.overlap * source_points as f64).round() as usize;
        diag.single_inliers = single;
        if single == 0 {
            return None;
        }
        let params = IcpParams {
            max_iterations: self.config.closure_icp_max_iterations as usize,
            max_correspondence_distance: max_distance,
            inlier_squared_distance: self.config.closure_inlier_squared_distance(),
            point_to_plane: self.config.closure_icp_point_to_plane,
        };
        let half_range = self.config.closure_submap_half_range.max(0) as usize;
        let radius = self.config.closure_search_radius;
        // each round starts from the previous round's alignment, which saw more of both places than the first ICP
        let (mut seed_rotation, mut seed_translation) = (first.rotation, first.translation);
        // recording an ICP cache, every round up to the budget is computed (so a replay that extends further, or
        // differently, still finds it), but the result and the diagnostics stay those of the first round that passed
        let record_all = self
            .icp_cache
            .as_ref()
            .is_some_and(|c| c.mode == crate::icp_cache::Mode::Record);
        let mut found: Option<(IcpResult, ClosureCandidateDiag)> = None;
        let mut keyframes = 2;
        while keyframes <= max_keyframes {
            // never stack source keyframes into the stretch the target side is drawn from
            let window = half_range + keyframes;
            let source_first = cur_idx
                .saturating_sub(keyframes)
                .max(target_idx + window + 1);
            if source_first >= cur_idx {
                break;
            }
            // where the seed puts each stacked source keyframe
            let placed: Vec<Vec3> = (source_first..=cur_idx)
                .map(|k| {
                    mat3::add(
                        &mat3::mat_vec(&seed_rotation, &self.key_poses[k].translation_global),
                        &seed_translation,
                    )
                })
                .collect();
            let target_ids =
                target_idx.saturating_sub(window)..=(target_idx + window).min(source_first - 1);
            let near = |j: usize| {
                j.abs_diff(target_idx) <= half_range
                    || placed.iter().any(|p| {
                        mat3::norm(&mat3::sub(p, &self.key_poses[j].translation_global)) < radius
                    })
            };
            // the round's clouds and every score of them, through the ICP cache: the stack's and the target's sizes,
            // the ICP, then its sharpness, likelihood ratio and overlap score where configured (NaN otherwise)
            let target_ids: Vec<usize> = target_ids.filter(|&j| near(j)).collect();
            let sharp = self.config.closure_sharpness_half > 0.0;
            let likelihood = self.config.closure_likelihood_confidence;
            let overlap = self.config.closure_overlap_score && !likelihood;
            let round = memo(&format!("xr{keyframes}"), 2, &mut || {
                // the stacked source is thinned to about the single scan's size, so a round costs about what the first
                // ICP did; the target keeps full resolution so the inlier test below means the same thing in every round
                let clearance = self.config.closure_ground_clearance_m;
                let stacked = above_floor(&self.stacked_cloud(source_first..=cur_idx), clearance);
                let thinning = (stacked.len() as f64 / source_points.max(1) as f64).max(1.0);
                let source = if thinning > 1.0 {
                    voxel_downsample(&stacked, self.config.submap_resolution * thinning.sqrt())
                } else {
                    stacked.clone()
                };
                let target =
                    above_floor(&self.stacked_cloud(target_ids.iter().copied()), clearance);
                let tree = KdTree::build(&target);
                let icp = icp(
                    &source,
                    &target,
                    &tree,
                    &seed_rotation,
                    &seed_translation,
                    &params,
                );
                if !self.config.closure_icp_dump_dir.is_empty() {
                    dump_icp(
                        &self.config.closure_icp_dump_dir,
                        cur_idx,
                        target_idx,
                        &source,
                        &target,
                        &icp,
                        &format!("_x{keyframes}"),
                    );
                }
                let score = |on: bool, f: &dyn Fn() -> f64| {
                    if on && icp.converged {
                        f()
                    } else {
                        f64::NAN
                    }
                };
                let mut v = vec![stacked.len() as f64, target.len() as f64];
                v.extend(crate::icp_cache::encode_icp(&icp));
                v.push(score(sharp, &|| self.sharpness(&source, &tree, &icp)));
                v.push(score(likelihood, &|| {
                    self.likelihood_ratio(&source, &target, &tree, &icp, &target_ids)
                        .0
                }));
                v.push(score(overlap, &|| {
                    self.overlap_score(&source, &target, &icp).score
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
            if overlap && icp.converged {
                diag.extended_overlap_score = round[19];
            }
            // evidence = -ln(1 - confidence); compared with the required evidence without rounding to 1. Scored by
            // sharpness, the stack earns its own confidence (more scans count only as far as they line up better);
            // otherwise its evidence grows with its inlier count
            // the stack adds evidence as far as it covers surface the single scan did not: its per-point ratio times its
            // size over the single scan's (both voxelized at the submap resolution, so a keyframe seeing the same ground
            // again adds nothing)
            let evidence = if likelihood {
                diag.extended_likelihood_ratio.max(0.0) * stacked_len as f64
                    / source_points.max(1) as f64
            } else if overlap {
                -(1.0 - diag.extended_overlap_score.clamp(0.0, 1.0)).ln()
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
            diag.extended_keyframes = cur_idx - source_first + 1;
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
            if evidence >= required && found.is_none() {
                if !record_all {
                    return Some(icp);
                }
                found = Some((icp.clone(), diag.clone()));
            }
            if !sharp {
                (seed_rotation, seed_translation) = (icp.rotation, icp.translation);
            }
            keyframes *= 2;
        }
        found.map(|(icp, kept)| {
            *diag = kept;
            icp
        })
    }
}
