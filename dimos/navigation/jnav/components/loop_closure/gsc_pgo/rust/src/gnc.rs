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

//! Batch GNC (graduated non-convexity) re-solve and its background worker.
//!
//! `GscPgo` owns the live iSAM2 smoother; this module holds the full-graph batch
//! GNC solve it runs alongside it. The solve is a pure function of a plain-data
//! `GncJob` (no gtsam handles), so the same `solve_gnc` runs either inline on the
//! main thread (offline harness) or on the `GncWorker` background thread (live
//! path), where it never blocks the pipeline.

use std::sync::mpsc::{Receiver, Sender};

use crate::gsc_pgo::{
    CommittedLoop, Config, FREE_VARIANCE, LOOP_GNC_INLIER_THRESHOLD, LOOP_GNC_MIN_VARIANCE,
};
use crate::gtsam::{FactorGraph, Isam2, NoiseModel, Pose3, Values};
use crate::mat3::{self, Mat3, Vec3};

/// One keyframe's contribution to the odometry backbone: the local
/// (dead-reckoned) pose that defines the between-factors, and the initial
/// estimate to seed the optimizer (local for GNC, global for the counterfactual
/// probe).
pub(crate) struct BackboneKeyframe {
    pub(crate) rotation_local: Mat3,
    pub(crate) translation_local: Vec3,
    pub(crate) initial_rotation: Mat3,
    pub(crate) initial_translation: Vec3,
}

/// The odometry-backbone noise variances pulled out of `Config` so the backbone
/// can be rebuilt on a worker thread without borrowing `GscPgo`.
#[derive(Clone, Copy)]
pub(crate) struct BackboneNoise {
    anchor_roll_pitch_var: f64,
    odom_rot_roll_pitch_var: f64,
    odom_rot_yaw_var: f64,
    odom_trans_xy_var: f64,
    odom_trans_z_var: f64,
    per_keyframe_roll_pitch_prior: bool,
    per_keyframe_roll_pitch_var: f64,
}

impl BackboneNoise {
    pub(crate) fn from_config(config: &Config) -> BackboneNoise {
        BackboneNoise {
            anchor_roll_pitch_var: config.anchor_roll_pitch_var,
            odom_rot_roll_pitch_var: config.odom_rot_roll_pitch_var,
            odom_rot_yaw_var: config.odom_rot_yaw_var,
            odom_trans_xy_var: config.odom_trans_xy_var,
            odom_trans_z_var: config.odom_trans_z_var,
            per_keyframe_roll_pitch_prior: config.per_keyframe_roll_pitch_prior,
            per_keyframe_roll_pitch_var: config.per_keyframe_roll_pitch_var,
        }
    }
}

/// Rebuild the odometry backbone (anchor prior + odom between-factors, plus the
/// optional per-keyframe roll/pitch prior) into a fresh graph/values. The
/// between-factors always come from the local dead-reckoned poses; the initial
/// estimate is whatever `BackboneKeyframe::initial_*` carries (local for GNC —
/// its TLS loss is non-convex and must start from the odom backbone, not the
/// under-closed Huber estimate — or global for the counterfactual probe).
pub(crate) fn build_odometry_backbone(
    graph: &mut FactorGraph,
    values: &mut Values,
    keyframes: &[BackboneKeyframe],
    noise: &BackboneNoise,
) {
    for (idx, keyframe) in keyframes.iter().enumerate() {
        let previous = idx
            .checked_sub(1)
            .map(|previous_idx| &keyframes[previous_idx]);
        add_backbone_keyframe(graph, values, previous, keyframe, idx as u64, noise);
    }
}

/// One keyframe's share of the backbone at `key`: its initial value, plus the
/// anchor prior (no `previous`) or the odom between-factor from `previous` and
/// the optional per-keyframe roll/pitch prior. Also extends a swapped-in iSAM2
/// with keyframes added after its GNC snapshot.
pub(crate) fn add_backbone_keyframe(
    graph: &mut FactorGraph,
    values: &mut Values,
    previous: Option<&BackboneKeyframe>,
    keyframe: &BackboneKeyframe,
    key: u64,
    noise: &BackboneNoise,
) {
    const ANCHOR_YAW_VAR: f64 = 1e-12;
    const ANCHOR_TRANS_VAR: f64 = 1e-12;
    let initial_pose = Pose3 {
        rotation: keyframe.initial_rotation,
        translation: keyframe.initial_translation,
    };
    values
        .insert_pose3(key, &initial_pose)
        .expect("gtsam: backbone init value");
    let Some(previous) = previous else {
        let prior_var = [
            noise.anchor_roll_pitch_var,
            noise.anchor_roll_pitch_var,
            ANCHOR_YAW_VAR,
            ANCHOR_TRANS_VAR,
            ANCHOR_TRANS_VAR,
            ANCHOR_TRANS_VAR,
        ];
        let anchor_noise = NoiseModel::diagonal_variances(&prior_var);
        graph
            .add_prior_pose3(key, &initial_pose, &anchor_noise)
            .expect("gtsam: backbone anchor prior");
        return;
    };
    let previous_rotation_transpose = mat3::transpose(&previous.rotation_local);
    let rotation_between = mat3::mat_mul(&previous_rotation_transpose, &keyframe.rotation_local);
    let translation_between = mat3::mat_vec(
        &previous_rotation_transpose,
        &mat3::sub(&keyframe.translation_local, &previous.translation_local),
    );
    let odom_noise = NoiseModel::diagonal_variances(&[
        noise.odom_rot_roll_pitch_var,
        noise.odom_rot_roll_pitch_var,
        noise.odom_rot_yaw_var,
        noise.odom_trans_xy_var,
        noise.odom_trans_xy_var,
        noise.odom_trans_z_var,
    ]);
    graph
        .add_between_pose3(
            key - 1,
            key,
            &Pose3 {
                rotation: rotation_between,
                translation: translation_between,
            },
            &odom_noise,
        )
        .expect("gtsam: backbone odom between factor");
    if noise.per_keyframe_roll_pitch_prior {
        let roll_pitch_var = [
            noise.per_keyframe_roll_pitch_var,
            noise.per_keyframe_roll_pitch_var,
            FREE_VARIANCE,
            FREE_VARIANCE,
            FREE_VARIANCE,
            FREE_VARIANCE,
        ];
        let roll_pitch_noise = NoiseModel::diagonal_variances(&roll_pitch_var);
        graph
            .add_prior_pose3(key, &initial_pose, &roll_pitch_noise)
            .expect("gtsam: backbone per-keyframe prior");
    }
}

/// The loop-factor noise knobs of a rebuilt iSAM2, pulled out of `Config` so the
/// rebuild can run on the worker thread.
#[derive(Clone, Copy)]
pub(crate) struct RebuildLoopNoise {
    loop_gnc_var_scale: f64,
    loop_robust_kernel: bool,
    loop_robust_huber_k: f64,
}

impl RebuildLoopNoise {
    pub(crate) fn from_config(config: &Config) -> RebuildLoopNoise {
        RebuildLoopNoise {
            loop_gnc_var_scale: config.loop_gnc_var_scale,
            loop_robust_kernel: config.loop_robust_kernel,
            loop_robust_huber_k: config.loop_robust_huber_k,
        }
    }
}

/// Stage one committed loop as a rebuild between-factor: variance from its ICP
/// score (GNC's scale), same robust kernel the live path uses.
pub(crate) fn add_rebuild_loop_factor(
    graph: &mut FactorGraph,
    loop_edge: &CommittedLoop,
    loop_noise: &RebuildLoopNoise,
) {
    let scale = if loop_edge.strong {
        1.0
    } else {
        loop_noise.loop_gnc_var_scale
    };
    let variance = loop_edge.score.max(LOOP_GNC_MIN_VARIANCE) * scale;
    let gaussian = NoiseModel::diagonal_variances(&[variance; 6]);
    let robust;
    let noise: &NoiseModel = if loop_noise.loop_robust_kernel && !loop_edge.strong {
        robust = NoiseModel::robust_huber(loop_noise.loop_robust_huber_k, &gaussian);
        &robust
    } else {
        &gaussian
    };
    graph
        .add_between_pose3(
            loop_edge.target_id as u64,
            loop_edge.source_id as u64,
            &Pose3 {
                rotation: loop_edge.rotation_offset,
                translation: loop_edge.translation_offset,
            },
            noise,
        )
        .expect("gtsam: rebuild loop between factor");
}

/// A fresh iSAM2 holding the odometry backbone plus the kept loops, and the
/// bookkeeping to swap it in for the incremental one.
pub(crate) struct RebuiltIsam2 {
    pub(crate) isam2: Isam2,
    /// (committed loop index, iSAM2 factor index) of every loop it holds.
    pub(crate) loop_factor_indices: Vec<(usize, u64)>,
    /// Keyframes and committed loops it was built from (a prefix of each).
    pub(crate) keyframe_count: usize,
    pub(crate) loop_count: usize,
    /// False when gtsam threw with the loops in and it fell back to the backbone.
    pub(crate) loops_included: bool,
}

/// Build an iSAM2 from the odometry backbone plus every loop whose GNC weight
/// still marks it an inlier, seeded at the keyframes' `initial_*`. If GTSAM
/// throws even on the fresh graph (mid-rejection it can hold hundreds of
/// conflicting false closures), fall back to the backbone alone.
pub(crate) fn build_isam2_from_backbone(
    keyframes: &[BackboneKeyframe],
    loops: &[CommittedLoop],
    backbone_noise: &BackboneNoise,
    loop_noise: &RebuildLoopNoise,
) -> RebuiltIsam2 {
    for include_loops in [true, false] {
        let mut graph = FactorGraph::new();
        let mut values = Values::new();
        build_odometry_backbone(&mut graph, &mut values, keyframes, backbone_noise);
        let mut kept_loop_positions: Vec<(usize, usize)> = Vec::new();
        if include_loops {
            for (edge_index, loop_edge) in loops.iter().enumerate() {
                if loop_edge.gnc_weight < LOOP_GNC_INLIER_THRESHOLD {
                    continue;
                }
                kept_loop_positions.push((edge_index, graph.len()));
                add_rebuild_loop_factor(&mut graph, loop_edge, loop_noise);
            }
        }
        let mut isam2 = Isam2::new();
        let new_factor_indices = match isam2.update(&graph, &values, &[]) {
            Ok(indices) => indices,
            Err(gtsam_error) if include_loops => {
                eprintln!(
                    "gtsam: isam2 rebuild with loops failed ({gtsam_error:?}); \
                     retrying backbone-only"
                );
                continue;
            }
            Err(gtsam_error) => panic!("gtsam: backbone-only isam2 rebuild: {gtsam_error:?}"),
        };
        let loop_factor_indices = kept_loop_positions
            .into_iter()
            .filter(|(_, graph_position)| *graph_position < new_factor_indices.len())
            .map(|(edge_index, graph_position)| (edge_index, new_factor_indices[graph_position]))
            .collect();
        return RebuiltIsam2 {
            isam2,
            loop_factor_indices,
            keyframe_count: keyframes.len(),
            loop_count: loops.len(),
            loops_included: include_loops,
        };
    }
    unreachable!("backbone-only rebuild either returns or panics")
}

/// A self-contained snapshot of everything the batch GNC re-solve needs, built
/// on the main thread and handed to the background worker. Holds only plain data
/// (no gtsam handles), so it is `Send`.
pub(crate) struct GncJob {
    pub(crate) keyframes: Vec<BackboneKeyframe>,
    pub(crate) loops: Vec<CommittedLoop>,
    pub(crate) backbone_noise: BackboneNoise,
    pub(crate) rebuild_loop_noise: RebuildLoopNoise,
    pub(crate) loop_gnc_var_scale: f64,
    pub(crate) loop_gnc_inlier_probability: f64,
    /// Monotonic dispatch counter, echoed in the result so the pipeline can
    /// tell whether a newer solve is still in flight (worker coalesces jobs).
    pub(crate) sequence: u64,
}

/// The result of one batch GNC re-solve: the corrected global pose of every
/// keyframe that was in the job, each committed loop's final GNC weight, and
/// (from `solve_gnc_and_rebuild`) an iSAM2 already rebuilt from them.
pub(crate) struct GncResult {
    pub(crate) keyframe_globals: Vec<(Mat3, Vec3)>,
    pub(crate) loop_weights: Vec<f64>,
    pub(crate) sequence: u64,
    pub(crate) rebuilt: Option<RebuiltIsam2>,
}

/// Run one batch GNC re-solve. Pure function of the job — no shared state — so it
/// runs identically on the main thread (offline harness) or the worker thread
/// (live path).
pub(crate) fn solve_gnc(job: &GncJob) -> GncResult {
    let mut graph = FactorGraph::new();
    let mut values = Values::new();
    build_odometry_backbone(&mut graph, &mut values, &job.keyframes, &job.backbone_noise);
    let mut known_inlier_indices: Vec<u64> = (0..graph.len() as u64).collect();
    let backbone_factor_count = known_inlier_indices.len();
    for (edge_idx, loop_edge) in job.loops.iter().enumerate() {
        if loop_edge.strong {
            known_inlier_indices.push((backbone_factor_count + edge_idx) as u64);
        }
        let scale = if loop_edge.strong {
            1.0
        } else {
            job.loop_gnc_var_scale
        };
        let variance = loop_edge.score.max(LOOP_GNC_MIN_VARIANCE) * scale;
        let noise = NoiseModel::diagonal_variances(&[variance; 6]);
        graph
            .add_between_pose3(
                loop_edge.target_id as u64,
                loop_edge.source_id as u64,
                &Pose3 {
                    rotation: loop_edge.rotation_offset,
                    translation: loop_edge.translation_offset,
                },
                &noise,
            )
            .expect("gtsam: gnc loop between factor");
    }
    let (estimate_values, factor_weights) = graph
        .gnc_optimize(
            &values,
            &known_inlier_indices,
            job.loop_gnc_inlier_probability,
        )
        .expect("gtsam: gnc optimize");
    let keyframe_globals = (0..job.keyframes.len())
        .map(|idx| {
            let pose = estimate_values
                .pose3(idx as u64)
                .expect("gtsam: keyframe missing from gnc estimate");
            (pose.rotation, pose.translation)
        })
        .collect();
    let loop_weights = (0..job.loops.len())
        .map(|edge_idx| {
            factor_weights
                .get(backbone_factor_count + edge_idx)
                .copied()
                .unwrap_or(1.0)
        })
        .collect();
    GncResult {
        keyframe_globals,
        loop_weights,
        sequence: job.sequence,
        rebuilt: None,
    }
}

/// What the background worker runs: the GNC solve, then the iSAM2 rebuild from
/// its poses and kept loops, so adopting it on the pipeline thread is a swap.
pub(crate) fn solve_gnc_and_rebuild(job: &GncJob) -> GncResult {
    let mut result = solve_gnc(job);
    let keyframes: Vec<BackboneKeyframe> = job
        .keyframes
        .iter()
        .zip(&result.keyframe_globals)
        .map(
            |(keyframe, (rotation_global, translation_global))| BackboneKeyframe {
                rotation_local: keyframe.rotation_local,
                translation_local: keyframe.translation_local,
                initial_rotation: *rotation_global,
                initial_translation: *translation_global,
            },
        )
        .collect();
    let loops: Vec<CommittedLoop> = job
        .loops
        .iter()
        .zip(&result.loop_weights)
        .map(|(loop_edge, weight)| CommittedLoop {
            gnc_weight: *weight,
            ..*loop_edge
        })
        .collect();
    result.rebuilt = Some(build_isam2_from_backbone(
        &keyframes,
        &loops,
        &job.backbone_noise,
        &job.rebuild_loop_noise,
    ));
    result
}

/// Handles for the background GNC worker thread. The main thread sends a fresh
/// `GncJob` after every closure; the worker coalesces (only the newest queued
/// job is solved) and sends back a `GncResult` with its rebuilt iSAM2 when
/// done, so the pipeline thread never blocks on the solve or the rebuild.
pub(crate) struct GncWorker {
    pub(crate) job_sender: Sender<GncJob>,
    pub(crate) result_receiver: Receiver<GncResult>,
}

/// The worker loop: block for a job, drain to the newest queued job (dropping
/// superseded ones so a burst of closures collapses to a single solve), solve,
/// and return the result. Exits when the main thread drops the job sender.
pub(crate) fn gnc_worker_loop(job_receiver: Receiver<GncJob>, result_sender: Sender<GncResult>) {
    while let Ok(job) = job_receiver.recv() {
        let mut latest_job = job;
        while let Ok(newer_job) = job_receiver.try_recv() {
            latest_job = newer_job;
        }
        let solve_t0 = std::time::Instant::now();
        let result = solve_gnc_and_rebuild(&latest_job);
        eprintln!(
            "gnc-solve + isam2 rebuild (background): {} keyframes, {} loop factors, {:.1} ms",
            latest_job.keyframes.len(),
            latest_job.loops.len(),
            solve_t0.elapsed().as_secs_f64() * 1000.0,
        );
        if result_sender.send(result).is_err() {
            return;
        }
    }
}
