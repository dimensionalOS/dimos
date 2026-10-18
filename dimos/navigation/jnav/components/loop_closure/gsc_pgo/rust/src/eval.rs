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

//! Live PGO eval: replay a recorded db through `GscPgo` the way the live module
//! runs it, then dump the result as JSON (raw odom trajectory, optimized keyframe
//! graph, closures, per-scan timing and latency).
//!
//! Scans arrive at their recorded times (read and decoded on their own thread).
//! The PGO thread follows `main.rs`: pair each scan with the latest odom pose,
//! add_key_pose, and on a keyframe search for loops, fold in a finished GNC
//! classification, smooth_and_update, and hand a closure to the background GNC
//! worker; while idle it adopts a finished GNC solve. Scans more than
//! `max_scan_queue` behind are dropped, as the module's queue drops them.

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
use memory2::{for_each_scan, read_odometry, OdomRow, ScanRow};

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
    keyframe_voxel_size: f64,
    min_loop_detect_duration: f64,
    loop_search_min_interval: f64,
    min_descriptor_std: f64,
    loop_min_occupancy: i32,
    loop_min_degeneracy: f64,
    loop_max_lowe_ratio: f64,
    loop_candidate_max_distance_m: f64,
    loop_max_yank_rotation_deg: f64,
    loop_yank_gate_max_distance_m: f64,
    loop_min_id_gap: u64,

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
    loop_icp_max_iterations: u64,
    loop_icp_max_correspondence_m: f64,
    loop_icp_inlier_fitness: bool,
    loop_icp_min_points: u64,
    loop_icp_seed_at_current_pose: bool,
    loop_rotation_variance: f64,
    loop_translation_variance_floor: f64,
    loop_refinement_passes: u64,
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
            keyframe_voxel_size: 0.0,
            min_loop_detect_duration: 2.0,
            loop_search_min_interval: 1.0,
            min_descriptor_std: 0.0,
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_max_lowe_ratio: 0.0,
            loop_candidate_max_distance_m: 0.0,
            loop_max_yank_rotation_deg: 0.0,
            loop_yank_gate_max_distance_m: 0.0,
            loop_min_id_gap: 0,
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
            loop_icp_max_iterations: 50,
            loop_icp_max_correspondence_m: 10.0,
            loop_icp_inlier_fitness: false,
            loop_icp_min_points: 0,
            loop_icp_seed_at_current_pose: false,
            loop_rotation_variance: 0.0,
            loop_translation_variance_floor: 0.0,
            loop_refinement_passes: 5,
            loop_gnc_final: true,
            loop_gnc_var_scale: 10.0,
            loop_gnc_inlier_probability: 0.95,
            use_location_constraints: false,
            anchor_roll_pitch_var: 1e-12,
            per_keyframe_roll_pitch_prior: false,
            per_keyframe_roll_pitch_var: 1e-4,
            odom_rot_roll_pitch_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            max_scan_queue: 5,
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
            keyframe_voxel_size: self.keyframe_voxel_size,
            min_loop_detect_duration: self.min_loop_detect_duration,
            loop_search_min_interval: self.loop_search_min_interval,
            loop_candidate_max_distance_m: self.loop_candidate_max_distance_m,
            min_descriptor_std: self.min_descriptor_std,
            loop_min_occupancy: self.loop_min_occupancy,
            loop_min_degeneracy: self.loop_min_degeneracy,
            loop_max_lowe_ratio: self.loop_max_lowe_ratio,
            loop_max_yank_rotation_deg: self.loop_max_yank_rotation_deg,
            loop_yank_gate_max_distance_m: self.loop_yank_gate_max_distance_m,
            loop_min_id_gap: self.loop_min_id_gap,
            loop_robust_kernel: self.loop_robust_kernel,
            loop_robust_huber_k: self.loop_robust_huber_k,
            loop_icp_max_iterations: self.loop_icp_max_iterations,
            loop_icp_max_correspondence_m: self.loop_icp_max_correspondence_m,
            loop_icp_inlier_fitness: self.loop_icp_inlier_fitness,
            loop_icp_min_points: self.loop_icp_min_points,
            loop_icp_seed_at_current_pose: self.loop_icp_seed_at_current_pose,
            loop_rotation_variance: self.loop_rotation_variance,
            loop_translation_variance_floor: self.loop_translation_variance_floor,
            loop_refinement_passes: self.loop_refinement_passes,
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
    /// Per processed scan: [ts, wall ms spent in the PGO thread for it, keyframes so far].
    frame_timing: Vec<[f64; 3]>,
    /// Scans dropped because they were more than `max_scan_queue` scans behind.
    scans_dropped: usize,
    /// Per processed scan: [ts, ms from the scan's arrival (its recorded time) until the PGO
    /// finished with it: the wait in the queue plus the work].
    latency: Vec<[f64; 2]>,
    /// Every `RSS_SAMPLE_EVERY` processed scans: [ts, process resident memory in MB] (from `ps`).
    rss_samples: Vec<[f64; 2]>,
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

const RSS_SAMPLE_EVERY: usize = 300;

/// This process's resident memory in MB, via `ps` (no extra crate; Mac and Linux).
fn resident_megabytes() -> Option<f64> {
    let output = std::process::Command::new("ps")
        .args(["-o", "rss=", "-p", &std::process::id().to_string()])
        .output()
        .ok()?;
    let kilobytes: f64 = String::from_utf8(output.stdout).ok()?.trim().parse().ok()?;
    Some(kilobytes / 1024.0)
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

    let odom_frame = unique_frame(odoms.iter().map(|row| row.frame_id.clone()), "odom stream")?;
    let odom_body_frame = unique_frame(
        odoms.iter().map(|row| row.child_frame_id.clone()),
        "odom child",
    )?;
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
    let mut frame_timing: Vec<[f64; 3]> = Vec::new();
    let mut rss_samples: Vec<[f64; 2]> = Vec::new();
    let mut lidar_frame: Option<String> = None;
    let mut scan_span: Option<(f64, f64)> = None;
    let mut pgo_s = 0.0f64;
    // wall clock and recording time of the first scan: a scan arrives at wall0 + (ts - ts0)
    let mut replay_clock: Option<(Instant, f64)> = None;
    let mut scans_dropped = 0usize;
    let mut latency: Vec<[f64; 2]> = Vec::new();

    // Scans are read and decoded on their own thread, as the live module receives them on its
    // transport's: the PGO thread below never pays for reading the recording.
    let (scan_sender, scan_receiver) = std::sync::mpsc::sync_channel::<ScanRow>(256);
    let (lidar_stream, scan_stride) = (args.lidar_stream.clone(), args.scan_stride);
    let reader = std::thread::spawn(move || {
        for_each_scan(&connection, &lidar_stream, scan_stride, |scan| {
            scan_sender.send(scan).map_err(|e| e.to_string())
        })
    });
    let mut visit = |scan: ScanRow| -> Result<(), String> {
        match &lidar_frame {
            None => {
                if args.lidar_tf.is_none() {
                    check_same_frame(&scan.frame_id, &odom_body_frame)?;
                }
                lidar_frame = Some(scan.frame_id.clone());
            }
            Some(frame) if *frame != scan.frame_id => {
                return Err(format!(
                    "lidar stream mixes frames [{frame:?}, {:?}]; expected one",
                    scan.frame_id
                ));
            }
            Some(_) => {}
        }
        scan_span = Some((scan_span.map_or(scan.ts, |(first, _)| first), scan.ts));
        let Some(latest) = advance_latest(&odoms, scan.ts, &mut cursor) else {
            return Ok(());
        };
        let (wall0, ts0) = *replay_clock.get_or_insert((Instant::now(), scan.ts));
        let arrival = wall0 + std::time::Duration::from_secs_f64((scan.ts - ts0).max(0.0));
        // idle until the scan arrives; the live module adopts a finished GNC solve while idle
        while Instant::now() < arrival {
            pgo.poll_and_adopt_gnc_solution();
            // short slices: macOS stretches a background process's sleep up to ~7x
            let remaining = arrival.saturating_duration_since(Instant::now());
            std::thread::sleep((remaining / 8).clamp(
                std::time::Duration::from_micros(500),
                std::time::Duration::from_millis(2),
            ));
        }
        // the live queue holds `max_scan_queue` scans and drops the oldest past that
        let (first_ts, _) = scan_span.unwrap_or((scan.ts, scan.ts));
        let scans_seen = scans_processed + scans_dropped;
        let period = if scans_seen > 0 {
            (scan.ts - first_ts) / scans_seen as f64
        } else {
            0.0
        };
        if period > 0.0 && arrival.elapsed().as_secs_f64() > config.max_scan_queue as f64 * period {
            scans_dropped += 1;
            return Ok(());
        }
        scans_processed += 1;
        let frame_started = Instant::now();

        let mut cloud = scan.points;
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

        let cloud = Arc::new(cloud);
        let cloud_with_pose = CloudWithPose {
            cloud: Some(cloud.clone()),
            pose,
            frame_id: latest.frame_id.clone(),
        };

        let mut closed = false;
        if pgo.add_key_pose(&cloud_with_pose) {
            // as main.rs: skip the loop search while newer scans are already waiting
            if period > 0.0 && arrival.elapsed().as_secs_f64() > period {
                pgo.skip_loop_search_behind();
            } else {
                pgo.search_for_loop_pairs();
            }
            if pgo.has_loop() {
                closures += 1;
                closed = true;
            }
            // the live module's order: fold in a finished GNC classification, update iSAM2, and
            // hand a new closure to the background worker
            pgo.poll_and_apply_gnc_classification();
            pgo.smooth_and_update();
            if closed {
                pgo.dispatch_gnc_async();
            }
        }
        let frame_s = frame_started.elapsed().as_secs_f64();
        latency.push([scan.ts, arrival.elapsed().as_secs_f64() * 1e3]);
        pgo_s += frame_s;
        frame_timing.push([scan.ts, frame_s * 1e3, pgo.key_poses().len() as f64]);
        if scans_processed % RSS_SAMPLE_EVERY == 1 {
            if let Some(megabytes) = resident_megabytes() {
                rss_samples.push([scan.ts, megabytes]);
            }
        }
        Ok(())
    };
    for scan in scan_receiver {
        visit(scan)?;
    }
    reader
        .join()
        .map_err(|_| "the scan reader panicked".to_string())??;
    let finalize_started = Instant::now();
    // the module keeps running after the last scan, so the solve in flight still lands
    while pgo.gnc_in_flight() {
        if !pgo.poll_and_adopt_gnc_solution() {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
    }
    let runtime_s = pgo_s + finalize_started.elapsed().as_secs_f64();
    let lidar_frame = lidar_frame.unwrap_or_default();

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

    let data_duration_s = scan_span.map_or(0.0, |(first, last)| last - first);
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
        frame_timing,
        scans_dropped,
        latency,
        rss_samples,
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
