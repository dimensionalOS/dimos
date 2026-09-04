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

//! Offline PGO eval: replay a recorded db straight through `GscPgo` as fast as
//! the machine allows, then dump an eval.py-shaped JSON (raw odom trajectory,
//! optimized keyframe graph, closures, keyframes, runtime, realtime factor).
//!
//! This deliberately bypasses the module/transport/replay scaffolding: it opens
//! the SQLite recording directly, decodes the LCM blobs, and drives the same
//! worker-loop logic as `main.rs` (pair each scan with the latest odom pose →
//! add_key_pose → on keyframe: search_for_loop_pairs + smooth_and_update).
//! Trajectory-warp / map / tag scoring is left to a thin Python postprocess that
//! reuses `trajectory_metrics`.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use std::collections::BTreeMap;

use dimos_gsc_pgo::gsc_pgo::{
    CloudWithPose, CommittedLoop, Config as PgoConfig, GscPgo, LoopCandidateDiag, PoseWithTime,
};
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::pointcloud;
use rusqlite::Connection;
use serde::{Deserialize, Serialize};

mod memory2;
use memory2::{read_odometry, read_scans, OdomRow};

/// Eval config. Defaults mirror `module.py`'s `PGOConfig` so a bare run
/// reproduces the module default; a `--config` JSON overlays any subset.
// Unknown keys are ignored on purpose: the resolved config is dumped from the
// Python PGOConfig, which carries module-lifecycle fields (executable, cwd,
// global_map_*, ...) this eval never uses. PGOConfig itself rejects typos.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct EvalConfig {
    map_frame: String,
    child_frame_id: String,
    body_frame: String,

    keyframe_min_rotation_degrees: f64,
    keyframe_min_distance_meters: f64,

    loop_search_radius: f64,
    loop_time_thresh: f64,
    loop_score_thresh: f64,
    loop_submap_half_range: i32,
    submap_resolution: f64,
    min_loop_detect_duration: f64,
    min_descriptor_std: f64,
    loop_min_occupancy: i32,
    loop_min_degeneracy: f64,
    loop_max_lowe_ratio: f64,
    loop_candidate_max_distance_m: f64,
    loop_max_yank_rotation_deg: f64,
    loop_yank_gate_max_distance_m: f64,
    loop_min_id_gap: u64,

    loop_instant_accept_distance_m: f64,
    loop_buffer_agreement_trans_m: f64,
    loop_buffer_agreement_rot_deg: f64,
    loop_buffer_min_agree: u64,

    subtract_odom_from_cloud: bool,

    use_scan_context: bool,
    scan_context_num_rings: i32,
    scan_context_num_sectors: i32,
    scan_context_max_range_m: f64,
    scan_context_top_k: i32,
    scan_context_match_threshold: f64,
    scan_context_lidar_height_m: f64,

    loop_robust_kernel: bool,
    loop_robust_huber_k: f64,
    loop_gnc_final: bool,
    loop_gnc_var_scale: f64,
    loop_gnc_inlier_probability: f64,

    use_location_constraints: bool,

    anchor_roll_pitch_var: f64,
    per_keyframe_roll_pitch_prior: bool,
    per_keyframe_roll_pitch_var: f64,

    odom_rot_roll_pitch_var: f64,
    odom_rot_yaw_var: f64,
    odom_trans_xy_var: f64,
    odom_trans_z_var: f64,

    max_scan_queue: i32,

    loop_conservativeness: i32,
}

impl Default for EvalConfig {
    fn default() -> Self {
        Self {
            map_frame: "map".into(),
            child_frame_id: "odom".into(),
            body_frame: "base_link".into(),
            keyframe_min_rotation_degrees: 10.0,
            keyframe_min_distance_meters: 0.5,
            loop_search_radius: 3.0,
            loop_time_thresh: 5.0,
            loop_score_thresh: 0.15,
            loop_submap_half_range: 5,
            submap_resolution: 0.1,
            min_loop_detect_duration: 2.0,
            min_descriptor_std: 0.0,
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_max_lowe_ratio: 0.0,
            loop_candidate_max_distance_m: 0.0,
            loop_max_yank_rotation_deg: 0.0,
            loop_yank_gate_max_distance_m: 0.0,
            loop_min_id_gap: 0,
            loop_instant_accept_distance_m: 0.0,
            loop_buffer_agreement_trans_m: 1.0,
            loop_buffer_agreement_rot_deg: 10.0,
            loop_buffer_min_agree: 2,
            subtract_odom_from_cloud: false,
            use_scan_context: true,
            scan_context_num_rings: 20,
            scan_context_num_sectors: 60,
            scan_context_max_range_m: 0.0,
            scan_context_top_k: 10,
            scan_context_match_threshold: 0.4,
            scan_context_lidar_height_m: 2.0,
            loop_robust_kernel: true,
            loop_robust_huber_k: 1.345,
            loop_gnc_final: true,
            loop_gnc_var_scale: 10.0,
            loop_gnc_inlier_probability: 0.99,
            use_location_constraints: false,
            anchor_roll_pitch_var: 1e-12,
            per_keyframe_roll_pitch_prior: false,
            per_keyframe_roll_pitch_var: 1e-4,
            odom_rot_roll_pitch_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            max_scan_queue: 100,
            loop_conservativeness: -1,
        }
    }
}

impl EvalConfig {
    fn to_pgo(&self) -> PgoConfig {
        let mut config = PgoConfig {
            keyframe_min_rotation_degrees: self.keyframe_min_rotation_degrees,
            keyframe_min_distance_meters: self.keyframe_min_distance_meters,
            loop_search_radius: self.loop_search_radius,
            loop_time_thresh: self.loop_time_thresh,
            loop_score_thresh: self.loop_score_thresh,
            loop_submap_half_range: self.loop_submap_half_range,
            submap_resolution: self.submap_resolution,
            min_loop_detect_duration: self.min_loop_detect_duration,
            loop_candidate_max_distance_m: self.loop_candidate_max_distance_m,
            min_descriptor_std: self.min_descriptor_std,
            loop_min_occupancy: self.loop_min_occupancy,
            loop_min_degeneracy: self.loop_min_degeneracy,
            loop_max_lowe_ratio: self.loop_max_lowe_ratio,
            loop_max_yank_rotation_deg: self.loop_max_yank_rotation_deg,
            loop_yank_gate_max_distance_m: self.loop_yank_gate_max_distance_m,
            loop_min_id_gap: self.loop_min_id_gap,
            loop_instant_accept_distance_m: self.loop_instant_accept_distance_m,
            loop_buffer_agreement_trans_m: self.loop_buffer_agreement_trans_m,
            loop_buffer_agreement_rot_deg: self.loop_buffer_agreement_rot_deg,
            loop_buffer_min_agree: self.loop_buffer_min_agree,
            loop_robust_kernel: self.loop_robust_kernel,
            loop_robust_huber_k: self.loop_robust_huber_k,
            loop_gnc_final: self.loop_gnc_final,
            loop_gnc_var_scale: self.loop_gnc_var_scale,
            loop_gnc_inlier_probability: self.loop_gnc_inlier_probability,
            use_location_constraints: self.use_location_constraints,
            odom_rot_roll_pitch_var: self.odom_rot_roll_pitch_var,
            odom_rot_yaw_var: self.odom_rot_yaw_var,
            odom_trans_xy_var: self.odom_trans_xy_var,
            odom_trans_z_var: self.odom_trans_z_var,
            anchor_roll_pitch_var: self.anchor_roll_pitch_var,
            per_keyframe_roll_pitch_prior: self.per_keyframe_roll_pitch_prior,
            per_keyframe_roll_pitch_var: self.per_keyframe_roll_pitch_var,
            use_scan_context: self.use_scan_context,
            scan_context_num_rings: self.scan_context_num_rings,
            scan_context_num_sectors: self.scan_context_num_sectors,
            scan_context_max_range_m: self.scan_context_max_range_m,
            scan_context_top_k: self.scan_context_top_k,
            scan_context_match_threshold: self.scan_context_match_threshold,
            scan_context_lidar_height_m: self.scan_context_lidar_height_m,
        };
        config.apply_conservativeness(self.loop_conservativeness);
        config
    }
}

/// Serializable mirror of `LoopCandidateDiag` (the core struct carries no serde
/// derive so the PGO library stays dependency-free). NaN metrics serialize as
/// JSON null, marking a value the rejecting path never computed.
#[derive(Serialize)]
struct LoopDiag {
    outcome: String,
    source_id: usize,
    source_time: f64,
    source_xyz: [f64; 3],
    target_id: i64,
    target_time: f64,
    target_xyz: [f64; 3],
    from_scan_context: bool,
    descriptor_structure: f32,
    descriptor_occupancy: i32,
    scan_context_best: f32,
    scan_context_second: f32,
    lowe_ratio: f32,
    candidate_distance: f64,
    degeneracy_min: f32,
    icp_converged: bool,
    icp_fitness: f64,
    icp_translation_norm: f64,
    yank_translation: f64,
    yank_rotation_deg: f64,
}

/// A committed loop's ICP-refined relative pose (target frame -> source), so an
/// offline PCM pass can compose it with odometry. `rotation` is row-major 3x3.
#[derive(Serialize)]
struct CommittedLoopEdge {
    source_id: usize,
    target_id: usize,
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
    score: f64,
    /// Final GNC weight (~1 kept / green, ~0 rejected / red). 1.0 when the batch
    /// GNC pass is disabled.
    gnc_weight: f64,
}

impl From<&CommittedLoop> for CommittedLoopEdge {
    fn from(edge: &CommittedLoop) -> CommittedLoopEdge {
        CommittedLoopEdge {
            source_id: edge.source_id,
            target_id: edge.target_id,
            rotation: edge.rotation_offset,
            translation: edge.translation_offset,
            score: edge.score,
            gnc_weight: edge.gnc_weight,
        }
    }
}

impl From<&LoopCandidateDiag> for LoopDiag {
    fn from(diag: &LoopCandidateDiag) -> LoopDiag {
        LoopDiag {
            outcome: diag.outcome.to_string(),
            source_id: diag.source_id,
            source_time: diag.source_time,
            source_xyz: diag.source_xyz,
            target_id: diag.target_id,
            target_time: diag.target_time,
            target_xyz: diag.target_xyz,
            from_scan_context: diag.from_scan_context,
            descriptor_structure: diag.descriptor_structure,
            descriptor_occupancy: diag.descriptor_occupancy,
            scan_context_best: diag.scan_context_best,
            scan_context_second: diag.scan_context_second,
            lowe_ratio: diag.lowe_ratio,
            candidate_distance: diag.candidate_distance,
            degeneracy_min: diag.degeneracy_min,
            icp_converged: diag.icp_converged,
            icp_fitness: diag.icp_fitness,
            icp_translation_norm: diag.icp_translation_norm,
            yank_translation: diag.yank_translation,
            yank_rotation_deg: diag.yank_rotation_deg,
        }
    }
}

#[derive(Serialize)]
struct EvalOutput {
    db: String,
    odom_stream: String,
    lidar_stream: String,
    odom_frame: String,
    lidar_frame: String,
    scan_stride: usize,
    odom_stride: usize,
    config: serde_json::Value,
    /// Raw odom trajectory: [ts, x, y, z, qx, qy, qz, qw] per row.
    odom_rows: Vec<[f64; 8]>,
    /// Optimized keyframe graph: [ts, x, y, z, qx, qy, qz, qw] per node.
    graph: Vec<[f64; 8]>,
    closures: usize,
    keyframes: usize,
    scans_processed: usize,
    odoms_used: usize,
    data_duration_s: f64,
    runtime_s: f64,
    realtime_factor: f64,
    /// Count of loop-search outcomes by gate name (accepted + each reject gate).
    gate_counts: BTreeMap<String, usize>,
    /// One record per loop-search evaluation (accepted + rejected), with the
    /// metric at each gate and the source/target locations.
    loop_diagnostics: Vec<LoopDiag>,
    /// Total nonlinear error of the final factor graph (½·Σ whitened residual²,
    /// robust kernels applied) — how much all the constraints conflict.
    graph_error: f64,
    /// Every committed loop's refined relative pose (target->source), in commit
    /// order — the raw constraints an offline consistency pass needs.
    committed_loop_edges: Vec<CommittedLoopEdge>,
}

struct Args {
    db: PathBuf,
    odom_stream: String,
    lidar_stream: String,
    config_path: Option<PathBuf>,
    output: PathBuf,
    scan_stride: usize,
    odom_stride: usize,
    /// body←lidar static as "x,y,z,qx,qy,qz,qw" — applied to every scan's
    /// points, for lidar streams not already expressed in the odom body frame.
    lidar_tf: Option<[f64; 7]>,
}

fn parse_lidar_tf(text: &str) -> Result<[f64; 7], String> {
    let values: Vec<f64> = text
        .split(',')
        .map(|part| part.trim().parse::<f64>().map_err(|e| format!("{e}")))
        .collect::<Result<_, _>>()?;
    values
        .try_into()
        .map_err(|_| "--lidar-tf needs 7 comma-separated values: x,y,z,qx,qy,qz,qw".to_string())
}

fn parse_args() -> Result<Args, String> {
    let mut db = None;
    let mut odom_stream = None;
    let mut lidar_stream = None;
    let mut config_path = None;
    let mut output = None;
    let mut scan_stride = 1usize;
    let mut odom_stride = 1usize;
    let mut lidar_tf = None;

    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--db" | "--db-path" => db = Some(PathBuf::from(value()?)),
            "--odom-stream" => odom_stream = Some(value()?),
            "--lidar-stream" => lidar_stream = Some(value()?),
            "--config" | "--pgo-config-json" => config_path = Some(PathBuf::from(value()?)),
            "--output" | "-o" => output = Some(PathBuf::from(value()?)),
            "--scan-stride" => scan_stride = value()?.parse().map_err(|e| format!("{e}"))?,
            "--odom-stride" => odom_stride = value()?.parse().map_err(|e| format!("{e}"))?,
            "--lidar-tf" => lidar_tf = Some(parse_lidar_tf(&value()?)?),
            other => return Err(format!("unknown flag {other}")),
        }
    }

    Ok(Args {
        db: db.ok_or("--db is required")?,
        odom_stream: odom_stream.ok_or("--odom-stream is required")?,
        lidar_stream: lidar_stream.ok_or("--lidar-stream is required")?,
        config_path,
        output: output.ok_or("--output is required")?,
        scan_stride: scan_stride.max(1),
        odom_stride: odom_stride.max(1),
        lidar_tf,
    })
}

/// The single frame shared by every row, or an error if the frames disagree.
fn unique_frame(frames: impl Iterator<Item = String>, what: &str) -> Result<String, String> {
    let mut seen: Vec<String> = Vec::new();
    for frame in frames {
        if !seen.contains(&frame) {
            seen.push(frame);
        }
    }
    match seen.as_slice() {
        [only] => Ok(only.clone()),
        [] => Err(format!("{what} has no rows")),
        many => Err(format!("{what} mixes frames {many:?}; expected one")),
    }
}

/// GscPgo transforms each keyframe's body cloud by its (body→world) pose, so the
/// lidar must already be in the odom's body/child frame. We apply no tf, so
/// verify it here. `PoseStamped` odom carries no child frame — then we can only
/// warn, since there's nothing to compare against.
fn check_same_frame(lidar_frame: &str, odom_body_frame: &str) -> Result<(), String> {
    if odom_body_frame.is_empty() {
        eprintln!(
            "pgo-eval: WARNING: odom payload has no child frame (PoseStamped); \
             cannot verify the lidar frame {lidar_frame:?} matches the odom body frame. \
             Assuming they match (no tf transform is applied)."
        );
        return Ok(());
    }
    if lidar_frame != odom_body_frame {
        return Err(format!(
            "lidar frame {lidar_frame:?} != odom body frame {odom_body_frame:?}; \
             GscPgo needs both in the same frame and no tf transform is applied. \
             Feed a lidar stream already expressed in {odom_body_frame:?}."
        ));
    }
    Ok(())
}

/// Latest odom sample at or before `ts` (the module pairs each scan with the
/// most recent odometry message). `cursor` advances monotonically as scans are
/// processed in time order.
fn advance_latest<'a>(odoms: &'a [OdomRow], ts: f64, cursor: &mut usize) -> Option<&'a OdomRow> {
    while *cursor + 1 < odoms.len() && odoms[*cursor + 1].ts <= ts {
        *cursor += 1;
    }
    if odoms.is_empty() || odoms[*cursor].ts > ts {
        return None;
    }
    Some(&odoms[*cursor])
}

fn run() -> Result<(), String> {
    let args = parse_args()?;

    let config: EvalConfig = match &args.config_path {
        Some(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path:?}: {e}"))?;
            serde_json::from_str(&text).map_err(|e| format!("config {path:?}: {e}"))?
        }
        None => EvalConfig::default(),
    };

    let connection =
        Connection::open_with_flags(&args.db, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
            .map_err(|e| format!("open {:?}: {e}", args.db))?;

    let odoms = read_odometry(&connection, &args.odom_stream, args.odom_stride)?;
    let scans = read_scans(&connection, &args.lidar_stream, args.scan_stride)?;

    let odom_frame = unique_frame(odoms.iter().map(|row| row.frame_id.clone()), "odom stream")?;
    let odom_body_frame = unique_frame(
        odoms.iter().map(|row| row.child_frame_id.clone()),
        "odom child",
    )?;
    let lidar_frame = unique_frame(scans.iter().map(|row| row.frame_id.clone()), "lidar stream")?;
    if args.lidar_tf.is_none() {
        check_same_frame(&lidar_frame, &odom_body_frame)?;
    }
    let lidar_tf = args
        .lidar_tf
        .map(|[x, y, z, qx, qy, qz, qw]| (mat3::mat_from_quat(&[qw, qx, qy, qz]), [x, y, z]));

    let odom_rows: Vec<[f64; 8]> = odoms
        .iter()
        .map(|row| {
            let [qx, qy, qz, qw] = row.quaternion_xyzw;
            [
                row.ts,
                row.translation[0],
                row.translation[1],
                row.translation[2],
                qx,
                qy,
                qz,
                qw,
            ]
        })
        .collect();

    let mut pgo = GscPgo::new(config.to_pgo());
    let mut cursor = 0usize;
    let mut closures = 0usize;
    let mut scans_processed = 0usize;

    let started = Instant::now();
    for scan in &scans {
        let Some(latest) = advance_latest(&odoms, scan.ts, &mut cursor) else {
            continue;
        };
        scans_processed += 1;

        let mut cloud = scan.points.clone();
        if let Some((rotation, translation)) = &lidar_tf {
            cloud = pointcloud::transform_cloud(&cloud, rotation, translation);
        }
        if config.subtract_odom_from_cloud && !cloud.is_empty() {
            let rotation_inverse = mat3::transpose(&latest.rotation);
            let negated = [
                -latest.translation[0],
                -latest.translation[1],
                -latest.translation[2],
            ];
            let translation_body = mat3::mat_vec(&rotation_inverse, &negated);
            cloud = pointcloud::transform_cloud(&cloud, &rotation_inverse, &translation_body);
        }

        let mut pose = PoseWithTime::new(latest.rotation, latest.translation);
        let seconds = latest.ts as i32;
        let nanos = ((latest.ts - seconds as f64) * 1e9) as u32;
        pose.set_time(seconds, nanos);

        let cloud_with_pose = CloudWithPose {
            cloud: Some(Arc::new(cloud)),
            pose,
            frame_id: latest.frame_id.clone(),
        };

        if !pgo.add_key_pose(&cloud_with_pose) {
            continue;
        }
        pgo.search_for_loop_pairs();
        if pgo.has_loop() {
            closures += 1;
        }
        pgo.smooth_and_update();
    }
    pgo.finalize_gnc();
    let runtime_s = started.elapsed().as_secs_f64();

    let graph: Vec<[f64; 8]> = pgo
        .key_poses()
        .iter()
        .map(|key_pose| {
            let quaternion = mat3::quat_from_mat(&key_pose.rotation_global);
            [
                key_pose.time,
                key_pose.translation_global[0],
                key_pose.translation_global[1],
                key_pose.translation_global[2],
                quaternion[1],
                quaternion[2],
                quaternion[3],
                quaternion[0],
            ]
        })
        .collect();

    let data_duration_s = match (scans.first(), scans.last()) {
        (Some(first), Some(last)) => last.ts - first.ts,
        _ => 0.0,
    };
    let realtime_factor = if runtime_s > 0.0 {
        data_duration_s / runtime_s
    } else {
        f64::INFINITY
    };

    let mut gate_counts: BTreeMap<String, usize> = BTreeMap::new();
    for diag in pgo.loop_diagnostics() {
        *gate_counts.entry(diag.outcome.to_string()).or_insert(0) += 1;
    }
    let loop_diagnostics: Vec<LoopDiag> =
        pgo.loop_diagnostics().iter().map(LoopDiag::from).collect();
    let graph_error = pgo.graph_error();
    let committed_loop_edges: Vec<CommittedLoopEdge> = pgo
        .committed_loops()
        .iter()
        .map(CommittedLoopEdge::from)
        .collect();

    let output = EvalOutput {
        db: args.db.display().to_string(),
        odom_stream: args.odom_stream.clone(),
        lidar_stream: args.lidar_stream.clone(),
        odom_frame,
        lidar_frame,
        scan_stride: args.scan_stride,
        odom_stride: args.odom_stride,
        config: serde_json::to_value(&config).unwrap_or(serde_json::Value::Null),
        keyframes: graph.len(),
        odom_rows,
        graph,
        closures,
        scans_processed,
        odoms_used: odoms.len(),
        data_duration_s,
        runtime_s,
        realtime_factor,
        gate_counts,
        loop_diagnostics,
        graph_error,
        committed_loop_edges,
    };

    let json = serde_json::to_string_pretty(&output).map_err(|e| e.to_string())?;
    std::fs::write(&args.output, json).map_err(|e| format!("write {:?}: {e}", args.output))?;

    eprintln!(
        "pgo-eval: {} scans, {} keyframes, {} closures, {:.1}s data in {:.2}s wall ({:.1}x realtime) -> {:?}",
        scans_processed,
        output.keyframes,
        output.closures,
        output.data_duration_s,
        output.runtime_s,
        output.realtime_factor,
        args.output,
    );
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("pgo-eval: {error}");
        std::process::exit(1);
    }
}
