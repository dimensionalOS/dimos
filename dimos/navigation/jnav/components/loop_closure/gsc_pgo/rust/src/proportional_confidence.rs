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
//! the graph. The yank (how far the loop would move its source keyframe from where the graph has it, or
//! its strain with `loop_yank_from_strain`) is scaled against the yanks of the candidates scored so far
//! into z; the connection's confidence (scan context x ICP fitness x degeneracy) must reach
//! 1 - exp(-z^2 / 2k^2), and one that falls short gets a second look with extended ICP on more keyframes.

use crate::connection_fit::{dump_icp, icp, IcpParams, IcpResult};
use crate::gsc_pgo::{LoopCandidateDiag, LoopSearch};
use crate::mat3::{self, Vec3};
use crate::pointcloud::{transform_cloud, voxel_downsample, KdTree, PointCloud};

impl LoopSearch {
    /// Fill in the yank / confidence gate's inputs and outputs for a candidate ICP has scored, and
    /// return its yank for the scale.
    pub(crate) fn score_candidate(&self, diag: &mut LoopCandidateDiag) -> (f64, f64) {
        let robust_scale = |values: Vec<f64>| -> f64 {
            let mut values = values;
            values.sort_by(f64::total_cmp);
            (1.4826 * values[values.len() / 2]).max(f64::EPSILON)
        };
        // odometry drift is a random walk, so the yank a correct loop needs grows with sqrt(keyframes between its ends)
        let drift = if self.config.loop_yank_per_sqrt_gap {
            (diag.source_id as f64 - diag.target_id as f64)
                .abs()
                .max(1.0)
                .sqrt()
        } else {
            1.0
        };
        diag.yank_drift_scale = drift;
        if self.config.loop_yank_from_strain {
            diag.yank_z = if self.scored_yanks.is_empty() {
                0.0
            } else {
                (diag.strain / robust_scale(self.scored_yanks.iter().map(|y| y.0).collect())).sqrt()
            };
        } else if self.scored_yanks.is_empty() {
            diag.yank_z = 0.0;
        } else {
            diag.sigma_rotation_deg = robust_scale(self.scored_yanks.iter().map(|y| y.0).collect());
            diag.sigma_translation_m =
                robust_scale(self.scored_yanks.iter().map(|y| y.1).collect());
            diag.yank_z = (diag.yank_rotation_deg / drift / diag.sigma_rotation_deg)
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
            unit(1.0 - diag.candidate_distance / self.config.loop_search_radius.max(f64::EPSILON))
        };
        diag.icp_factor =
            unit(1.0 - diag.icp_fitness / self.config.loop_score_thresh.max(f64::EPSILON));
        diag.degeneracy_factor =
            if self.config.loop_min_degeneracy > 0.0 && diag.degeneracy_min >= 0.0 {
                unit(diag.degeneracy_min as f64 / self.config.loop_min_degeneracy)
            } else {
                1.0
            };
        diag.confidence = diag.scan_context_factor * diag.icp_factor * diag.degeneracy_factor;
        let width = self.config.loop_yank_gate_width.max(f64::EPSILON);
        diag.required_confidence = 1.0 - (-diag.yank_z * diag.yank_z / (2.0 * width * width)).exp();
        if self.config.loop_yank_from_strain {
            return (diag.strain, diag.strain);
        }
        (
            diag.yank_rotation_deg / drift,
            diag.yank_translation / drift,
        )
    }
    /// Global-frame cloud of keyframes `ids`, voxel-downsampled at the submap resolution.
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
        if self.config.submap_resolution > 0.0 {
            cloud = voxel_downsample(&cloud, self.config.submap_resolution);
        }
        cloud
    }
    /// See `Config::loop_extended_icp_max_keyframes`. Returns the extended ICP of the first round
    /// whose confidence clears `diag.required_confidence`, or None.
    pub(crate) fn extended_icp(
        &self,
        cur_idx: usize,
        loop_idx: usize,
        source_cloud: &PointCloud,
        target_tree: &KdTree,
        first: &IcpResult,
        diag: &mut LoopCandidateDiag,
    ) -> Option<IcpResult> {
        let max_keyframes = self.config.loop_extended_icp_max_keyframes;
        if max_keyframes < 2 || !first.converged {
            return None;
        }
        let max_distance = self.config.loop_icp_max_correspondence_m;
        // a point overlaps when it is within the RMS distance the ICP score gate allows (fitness is a mean squared distance)
        let inliers = |source: &PointCloud, tree: &KdTree, icp: &IcpResult| {
            let limit = self.config.loop_score_thresh as f32;
            transform_cloud(source, &icp.rotation, &icp.translation)
                .iter()
                .filter(|p| matches!(tree.nearest(p), Some((_, d)) if d <= limit))
                .count()
        };
        let single = inliers(source_cloud, target_tree, first);
        diag.single_inliers = single;
        if single == 0 {
            return None;
        }
        let params = IcpParams {
            max_iterations: self.config.loop_icp_max_iterations as usize,
            max_correspondence_distance: max_distance,
        };
        let half_range = self.config.loop_submap_half_range.max(0) as usize;
        let radius = self.config.loop_search_radius;
        // each round starts from the previous round's alignment, which saw more of both places than the first ICP
        let (mut seed_rotation, mut seed_translation) = (first.rotation, first.translation);
        let mut keyframes = 2;
        while keyframes <= max_keyframes {
            // never stack source keyframes into the stretch the target side is drawn from
            let window = half_range + keyframes;
            let source_first = cur_idx.saturating_sub(keyframes).max(loop_idx + window + 1);
            if source_first >= cur_idx {
                return None;
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
                loop_idx.saturating_sub(window)..=(loop_idx + window).min(source_first - 1);
            let near = |j: usize| {
                j.abs_diff(loop_idx) <= half_range
                    || placed.iter().any(|p| {
                        mat3::norm(&mat3::sub(p, &self.key_poses[j].translation_global)) < radius
                    })
            };
            // the stacked source is thinned to about the single scan's size, so a round costs about what the first ICP did;
            // the target keeps full resolution so the inlier test below means the same thing in every round
            let stacked = self.stacked_cloud(source_first..=cur_idx);
            let thinning = (stacked.len() as f64 / source_cloud.len().max(1) as f64).max(1.0);
            let source = if thinning > 1.0 {
                voxel_downsample(&stacked, self.config.submap_resolution * thinning.sqrt())
            } else {
                stacked.clone()
            };
            let target = self.stacked_cloud(target_ids.filter(|&j| near(j)));
            let tree = KdTree::build(&target);
            let icp = icp(
                &source,
                &target,
                &tree,
                &seed_rotation,
                &seed_translation,
                &params,
            );
            if !self.config.loop_icp_dump_dir.is_empty() {
                dump_icp(
                    &self.config.loop_icp_dump_dir,
                    cur_idx,
                    loop_idx,
                    &source,
                    &target,
                    &icp,
                    &format!("_x{keyframes}"),
                );
            }
            // inliers of the full stack, estimated from the thinned one's inlier fraction
            let extended = if icp.converged {
                (inliers(&source, &tree, &icp) as f64 * stacked.len() as f64
                    / source.len().max(1) as f64)
                    .round() as usize
            } else {
                0
            };
            let held_up = ((extended as f64 / stacked.len().max(1) as f64)
                / (single as f64 / source_cloud.len() as f64))
                .min(1.0);
            let per_scan = diag.confidence * held_up;
            // evidence = -ln(1 - confidence); the gate compares it to z^2 / 2k^2 without rounding to 1
            let evidence = -(1.0 - per_scan).ln() * extended as f64 / single as f64;
            let width = self.config.loop_yank_gate_width.max(f64::EPSILON);
            let required = diag.yank_z * diag.yank_z / (2.0 * width * width);
            let confidence = 1.0 - (-evidence).exp();
            diag.extended_keyframes = cur_idx - source_first + 1;
            diag.extended_source_points = stacked.len();
            diag.extended_target_points = target.len();
            diag.extended_inliers = extended;
            diag.extended_fitness = icp.fitness;
            diag.extended_confidence = if confidence.is_finite() {
                confidence.clamp(0.0, 1.0)
            } else {
                0.0
            };
            if !icp.converged {
                return None;
            }
            if evidence >= required {
                return Some(icp);
            }
            (seed_rotation, seed_translation) = (icp.rotation, icp.translation);
            keyframes *= 2;
        }
        None
    }
}
