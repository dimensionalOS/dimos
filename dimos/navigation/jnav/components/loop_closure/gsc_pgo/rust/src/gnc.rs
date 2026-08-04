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

use crate::gsc_pgo::{CommittedLoop, Config, FREE_VARIANCE, LOOP_GNC_MIN_VARIANCE};
use crate::gtsam::{FactorGraph, NoiseModel, Pose3, Values};
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
    const ANCHOR_YAW_VAR: f64 = 1e-12;
    const ANCHOR_TRANS_VAR: f64 = 1e-12;
    for (idx, keyframe) in keyframes.iter().enumerate() {
        let initial_pose = Pose3 {
            rotation: keyframe.initial_rotation,
            translation: keyframe.initial_translation,
        };
        values
            .insert_pose3(idx as u64, &initial_pose)
            .expect("gtsam: backbone init value");
        if idx == 0 {
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
                .add_prior_pose3(0, &initial_pose, &anchor_noise)
                .expect("gtsam: backbone anchor prior");
            continue;
        }
        let previous = &keyframes[idx - 1];
        let previous_rotation_transpose = mat3::transpose(&previous.rotation_local);
        let rotation_between =
            mat3::mat_mul(&previous_rotation_transpose, &keyframe.rotation_local);
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
                (idx - 1) as u64,
                idx as u64,
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
                .add_prior_pose3(idx as u64, &initial_pose, &roll_pitch_noise)
                .expect("gtsam: backbone per-keyframe prior");
        }
    }
}

/// A self-contained snapshot of everything the batch GNC re-solve needs, built
/// on the main thread and handed to the background worker. Holds only plain data
/// (no gtsam handles), so it is `Send`.
pub(crate) struct GncJob {
    pub(crate) keyframes: Vec<BackboneKeyframe>,
    pub(crate) loops: Vec<CommittedLoop>,
    pub(crate) backbone_noise: BackboneNoise,
    pub(crate) loop_gnc_var_scale: f64,
    pub(crate) loop_gnc_inlier_probability: f64,
    /// Monotonic dispatch counter, echoed in the result so the pipeline can
    /// tell whether a newer solve is still in flight (worker coalesces jobs).
    pub(crate) sequence: u64,
}

/// The result of one batch GNC re-solve: the corrected global pose of every
/// keyframe that was in the job, and each committed loop's final GNC weight.
pub(crate) struct GncResult {
    pub(crate) keyframe_globals: Vec<(Mat3, Vec3)>,
    pub(crate) loop_weights: Vec<f64>,
    pub(crate) sequence: u64,
}

/// Run one batch GNC re-solve. Pure function of the job — no shared state — so it
/// runs identically on the main thread (offline harness) or the worker thread
/// (live path).
pub(crate) fn solve_gnc(job: &GncJob) -> GncResult {
    let mut graph = FactorGraph::new();
    let mut values = Values::new();
    build_odometry_backbone(&mut graph, &mut values, &job.keyframes, &job.backbone_noise);
    let known_inlier_indices: Vec<u64> = (0..graph.len() as u64).collect();
    let backbone_factor_count = known_inlier_indices.len();
    for loop_edge in &job.loops {
        let variance = loop_edge.score.max(LOOP_GNC_MIN_VARIANCE) * job.loop_gnc_var_scale;
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
    }
}

/// Handles for the background GNC worker thread. The main thread sends a fresh
/// `GncJob` after every closure; the worker coalesces (only the newest queued
/// job is solved) and sends back a `GncResult` when done, so the pipeline thread
/// never blocks on the growing full-graph batch solve.
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
        let result = solve_gnc(&latest_job);
        eprintln!(
            "gnc-solve (background): {} keyframes, {} loop factors, {:.1} ms",
            latest_job.keyframes.len(),
            latest_job.loops.len(),
            solve_t0.elapsed().as_secs_f64() * 1000.0,
        );
        if result_sender.send(result).is_err() {
            return;
        }
    }
}
