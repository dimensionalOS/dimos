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
//! add_key_pose → on keyframe: search_for_closure_pairs + update).
//! Trajectory-warp / map / tag scoring is left to a thin Python postprocess that
//! reuses `trajectory_metrics`.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use dimos_gsc_pgo::gsc_pgo::{
    ClosureCandidateDiag, CloudWithPose, CommittedClosure, Config as PgoConfig, GscPgo,
    PoseWithTime,
};
use dimos_gsc_pgo::loop_overlap::LoopOverlap;
use dimos_gsc_pgo::mat3;
use dimos_gsc_pgo::pointcloud;
use dimos_gsc_pgo::records::{
    append_pose_changes, closure_edge_records, write_keyframes, HeardPoses,
};
use rusqlite::Connection;
use serde::{Deserialize, Serialize};

mod memory2;
use memory2::{for_each_scan, read_odometry, OdomRow, Rigid};

/// Eval config: the PGO core's config (defaults mirror `module.py`'s `PGOConfig`, so a bare run reproduces
/// the module default) plus how the scans arrive; a `--config` JSON overlays any subset.
// Unknown keys are ignored on purpose: the resolved config is dumped from the Python PGOConfig, which
// carries module-lifecycle fields (executable, cwd, global_map_*, ...) this eval never uses.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default)]
struct EvalConfig {
    #[serde(flatten)]
    pgo: PgoConfig,
    /// `--live`: once more than this many scans wait, all but the newest are dropped (the module's max_scan_queue; <= 0 = unbounded).
    max_scan_queue: i32,
    /// Multiplies every stated odometry_error_estimate_* sd term; < 1 states the odometry as better than it is (over-trust test: 0.1).
    odometry_error_estimate_scale: f64,
    /// The frame the odometry's poses are really of, where its child_frame_id misnames it (office555's Point-LIO names
    /// base_link but reports the lidar's pose: "livox_frame"); empty = the child frame.
    odom_pose_frame: String,
    /// Threads closure search and solves may use (the module's `worker_threads`): the rayon pool pgo-eval builds, so a
    /// replay runs as fast as the module would. 0 (the default) = every core: no GSC run times itself handicapped.
    worker_threads: usize,
}

impl Default for EvalConfig {
    fn default() -> EvalConfig {
        EvalConfig {
            pgo: PgoConfig::default(),
            max_scan_queue: 5,
            odometry_error_estimate_scale: 1.0,
            odom_pose_frame: String::new(),
            worker_threads: 0,
        }
    }
}

/// A committed closure's ICP-refined relative pose (target frame -> source), so an
/// offline PCM pass can compose it with odometry. `rotation` is row-major 3x3.
#[derive(Serialize)]
struct CommittedClosureEdge {
    source_id: usize,
    target_id: usize,
    rotation: [[f64; 3]; 3],
    translation: [f64; 3],
    score: f64,
    /// 1 kept, 0 rejected by the newest applied solve; NaN until a solve has judged it.
    gnc_weight: f64,
}

impl From<&CommittedClosure> for CommittedClosureEdge {
    fn from(edge: &CommittedClosure) -> CommittedClosureEdge {
        CommittedClosureEdge {
            source_id: edge.source_id,
            target_id: edge.target_id,
            rotation: edge.rotation_offset,
            translation: edge.translation_offset,
            score: edge.score,
            gnc_weight: edge.gnc_weight,
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
    /// Seconds `finish` took after the last scan (the run's closing solves; a live module has no such step).
    finish_s: f64,
    /// Data seconds per replay second up to the last scan, `finish` left out.
    realtime_factor: f64,
    /// `--live` replay (else lockstep), the scans it dropped from a full queue, and each processed scan's ms from arriving
    /// to done (queue wait + processing).
    live: bool,
    scans_dropped: usize,
    scan_latency_ms: Vec<f32>,
    /// Per keyframe, ms the scan thread spends on it (add_key_pose + closure search), in order.
    keyframe_scan_thread_ms: Vec<f32>,
    /// Per keyframe, ms in `update` (adopting a finished solve, waiting for it when due, and starting the next).
    keyframe_gnc_ms: Vec<f32>,
    /// Per scan, ms of the scan thread's own work (add_key_pose, and on a keyframe its closure search and solve steps),
    /// the lockstep replay's waits on due jobs left out.
    scan_thread_ms: Vec<f32>,
    /// Per pressure computation (inline, or as a job: `pressure_let_enabled`): keyframes, committed closures, seconds.
    pressure_calls: Vec<(usize, usize, f64)>,
    /// Per adopted solve, the graph change heat cost's part on the solve's thread: keyframes, seconds; and the solves
    /// whose cost the scan thread recomputed (a solve adopted while they ran).
    change_cost_calls: Vec<(usize, f64)>,
    change_cost_recomputed: usize,
    /// The LET coefficients this run measured (`GscPgo::runtime_profile`): per LET the 99th percentile of measured /
    /// estimated scans, its jobs, and its coefficients scaled by it (a config block ready to paste; diagnostics only).
    runtime_profile: Vec<RuntimeProfileRow>,
    /// Cumulative wall seconds per GSC stage (GscPgo::stage_seconds).
    stage_seconds: BTreeMap<String, f64>,
    /// How much faster than real each part of the module runs (`Speed`).
    speed: Speed,
    /// Each loop-overlap pressure recompute (every pressure_check_frequency-th scan): the scan's time, the pressure and the
    /// scan count (the module's debug_log numbers).
    loop_overlap: Vec<(f64, LoopOverlap, usize)>,
    /// Count of closure-search outcomes by gate name (accepted + each reject gate).
    gate_counts: BTreeMap<String, usize>,
    /// One record per closure-search evaluation (accepted + rejected), with the
    /// metric at each gate and the source/target locations.
    closure_diagnostics: Vec<ClosureCandidateDiag>,
    /// Total nonlinear error of the final factor graph (½·Σ whitened residual²,
    /// robust kernels applied) — how much all the constraints conflict.
    graph_error: f64,
    /// Every committed closure's refined relative pose (target->source), in commit
    /// order — the raw constraints an offline consistency pass needs.
    committed_closure_edges: Vec<CommittedClosureEdge>,
    /// With an ICP cache replay: the cloud computations answered from it, and those it lacked (computed live).
    icp_cache_hits: usize,
    icp_cache_misses: usize,
    /// The odometry error estimate the run measured, as next run's config inputs (`GscPgo::odometry_error_estimate_for_next_run`): per
    /// source xy, z, yaw, tilt, [per sqrt m, per sqrt rad], and whether a kept closure measured it.
    odometry_drift: [[f64; 2]; 4],
    odometry_drift_measured: [bool; 4],
}

struct Args {
    db: PathBuf,
    odom_stream: String,
    lidar_stream: String,
    config_path: Option<PathBuf>,
    output: PathBuf,
    /// Write the keyframes (`<path>`, as the module's dump_path) and every
    /// keyframe pose change as the module publishes it (`<path>.closures`).
    event_dump: Option<PathBuf>,
    /// `--icp-cache-record <path>`: write every closure search's cloud computations there (`icp_cache`).
    /// `--icp-cache-replay <path>`: answer them from such a file, computing (and counting) any it lacks.
    icp_cache: Option<(dimos_gsc_pgo::icp_cache::Mode, PathBuf)>,
    /// Replay as the module runs live instead of lockstep: scans arrive at their recorded times, wait in a queue of
    /// max_scan_queue (all but the newest dropped when it overflows), and the closure search and solves run in the background.
    live: bool,
    /// World-frame position fixes to feed as the scans reach their times (`GscPgo::add_coordinate_factor`), a CSV with one
    /// fix per line: time, frame id, x, y, z, the 3x3 covariance row-major (m²), the sensor's offset in the body frame
    /// (x, y, z). Lockstep replay only.
    world_positions: Option<PathBuf>,
    /// Landmark sightings to feed the same way (`GscPgo::add_landmark_factor`), a CSV with one per line: time,
    /// landmark id, x, y, z in the body frame, then either the 3x3 position covariance row-major (m², a point) or a
    /// rotation vector (rad) and the 6x6 covariance row-major in `LandmarkSighting`'s order (a pose; a variance that is
    /// not a positive finite number is unknown). Lockstep replay only.
    landmark_sightings: Option<PathBuf>,
    /// `--until-s <seconds>`: replay only the lidar stream's first this many seconds (a quick speed check on a crop).
    until_s: f64,
}

/// The sightings of a `--landmark-sightings` file, in time order.
#[allow(clippy::type_complexity)]
fn read_landmark_sightings(
    path: &PathBuf,
) -> Result<Vec<(f64, u64, [[f64; 3]; 3], [f64; 3], [[f64; 6]; 6])>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut sightings = Vec::new();
    for line in text
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
    {
        let v: Vec<f64> = line
            .split(',')
            .map(|x| x.trim().parse::<f64>().map_err(|e| format!("{line}: {e}")))
            .collect::<Result<_, _>>()?;
        let position = [v[2], v[3], v[4]];
        let (rotation, information) = match v.len() {
            14 => {
                let point = dimos_gsc_pgo::landmarks::LandmarkSighting::point(
                    0,
                    0,
                    position,
                    std::array::from_fn(|i| std::array::from_fn(|j| v[5 + 3 * i + j])),
                );
                (point.rotation, point.information)
            }
            44 => (
                rotation_vector([v[5], v[6], v[7]]),
                dimos_gsc_pgo::landmarks::known_inverse(&std::array::from_fn(|i| {
                    std::array::from_fn(|j| v[8 + 6 * i + j])
                })),
            ),
            _ => {
                return Err(format!(
                    "{line}: needs 14 values (time, landmark id, xyz, 3x3 covariance) or 44 (time, landmark id, xyz, \
                     rotation vector, 6x6 covariance)"
                ))
            }
        };
        sightings.push((v[0], v[1] as u64, rotation, position, information));
    }
    sightings.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(sightings)
}

/// Rotation by axis-angle `w` (Rodrigues).
fn rotation_vector(w: [f64; 3]) -> [[f64; 3]; 3] {
    let angle = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
    if angle < 1e-12 {
        return dimos_gsc_pgo::mat3::identity();
    }
    let axis = [w[0] / angle, w[1] / angle, w[2] / angle];
    let half = angle / 2.0;
    dimos_gsc_pgo::mat3::mat_from_quat(&[
        half.cos(),
        axis[0] * half.sin(),
        axis[1] * half.sin(),
        axis[2] * half.sin(),
    ])
}

/// A `runtime_profile` row in a.json: a job kind, its 99th-percentile measured / estimated scans, its count and its
/// coefficients scaled by it.
type RuntimeProfileRow = (String, f64, usize, Vec<(String, f64)>);

/// A `--world-positions` fix: time, frame, sensor lever, position and covariance.
type WorldPositionFix = (f64, u32, [f64; 3], [f64; 3], [[f64; 3]; 3]);

/// The fixes of a `--world-positions` file, in time order.
fn read_world_positions(
    path: &PathBuf,
) -> Result<Vec<WorldPositionFix>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut fixes = Vec::new();
    for line in text
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
    {
        let v: Vec<f64> = line
            .split(',')
            .map(|x| x.trim().parse::<f64>().map_err(|e| format!("{line}: {e}")))
            .collect::<Result<_, _>>()?;
        if v.len() != 17 {
            return Err(format!(
                "{line}: needs 17 values (time, frame, xyz, 3x3 covariance, lever xyz)"
            ));
        }
        fixes.push((
            v[0],
            v[1] as u32,
            [v[14], v[15], v[16]],
            [v[2], v[3], v[4]],
            [
                [v[5], v[6], v[7]],
                [v[8], v[9], v[10]],
                [v[11], v[12], v[13]],
            ],
        ));
    }
    fixes.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(fixes)
}

fn parse_args() -> Result<Args, String> {
    let mut db = None;
    let mut odom_stream = None;
    let mut lidar_stream = None;
    let mut config_path = None;
    let mut output = None;
    let mut event_dump = None;
    let mut live = false;
    let mut world_positions = None;
    let mut landmark_sightings = None;
    let mut icp_cache = None;
    let mut until_s = f64::INFINITY;

    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{flag} needs a value"));
        match flag.as_str() {
            "--db" => db = Some(PathBuf::from(value()?)),
            "--odom-stream" => odom_stream = Some(value()?),
            "--lidar-stream" => lidar_stream = Some(value()?),
            "--config" => config_path = Some(PathBuf::from(value()?)),
            "--output" => output = Some(PathBuf::from(value()?)),
            "--event-dump" => event_dump = Some(PathBuf::from(value()?)),
            "--live" => live = true,
            "--until-s" => until_s = value()?.parse().map_err(|e| format!("{e}"))?,
            "--world-positions" => world_positions = Some(PathBuf::from(value()?)),
            "--landmark-sightings" => landmark_sightings = Some(PathBuf::from(value()?)),
            "--icp-cache-record" => {
                icp_cache = Some((
                    dimos_gsc_pgo::icp_cache::Mode::Record,
                    PathBuf::from(value()?),
                ))
            }
            "--icp-cache-replay" => {
                icp_cache = Some((
                    dimos_gsc_pgo::icp_cache::Mode::Replay,
                    PathBuf::from(value()?),
                ))
            }
            other => return Err(format!("unknown flag {other}")),
        }
    }

    Ok(Args {
        db: db.ok_or("--db is required")?,
        odom_stream: odom_stream.ok_or("--odom-stream is required")?,
        lidar_stream: lidar_stream.ok_or("--lidar-stream is required")?,
        config_path,
        output: output.ok_or("--output is required")?,
        event_dump,
        live,
        world_positions,
        landmark_sightings,
        icp_cache,
        until_s,
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

/// The level robot body frame GscPgo works in: odometry poses and lidar points are re-expressed in it through the
/// recording's tf (closure search assumes level clouds, so a pitched sensor's own frame will not do).
const BODY_FRAME: &str = "base_link";

/// Per odometry (by the odom child frame it moves), its ideal stated odometry error estimate: [per sqrt m, per sqrt rad] for xy, z, yaw,
/// tilt. Mid-360 (FAST-LIO / Point-LIO) is the Config default; Go2 L1 leg odometry is its calibrated drift.
fn stated_odometry_error_estimate(odom_child_frame: &str) -> Option<[[f64; 2]; 4]> {
    match odom_child_frame {
        // livox_frame: the Mid-360 frame name Point-LIO recordings like office555 use
        "mid360_link" | "livox_frame" => {
            let c = PgoConfig::default();
            Some([
                [
                    c.odometry_error_estimate_xy_per_sqrt_m,
                    c.odometry_error_estimate_xy_per_sqrt_rad,
                ],
                [
                    c.odometry_error_estimate_z_per_sqrt_m,
                    c.odometry_error_estimate_z_per_sqrt_rad,
                ],
                [
                    c.odometry_error_estimate_yaw_per_sqrt_m,
                    c.odometry_error_estimate_yaw_per_sqrt_rad,
                ],
                [
                    c.odometry_error_estimate_tilt_per_sqrt_m,
                    c.odometry_error_estimate_tilt_per_sqrt_rad,
                ],
            ])
        }
        // calibrated Go2 L1 drift (Go2 leg odometry, clouds in l1_link)
        "l1_link" => Some([
            [2.948197514740707e-4, 4.787993406292453e-3],
            [2.3613682139832578e-4, 1.419763443298345e-3],
            [1.1336520957451434e-6, 9.817427149152942e-3],
            [3.837937416144e-4, 1.8328789396925035e-3],
        ]),
        _ => None,
    }
}

const ODOMETRY_ERROR_ESTIMATE_KEYS: [[&str; 2]; 4] = [
    [
        "odometry_error_estimate_xy_per_sqrt_m",
        "odometry_error_estimate_xy_per_sqrt_rad",
    ],
    [
        "odometry_error_estimate_z_per_sqrt_m",
        "odometry_error_estimate_z_per_sqrt_rad",
    ],
    [
        "odometry_error_estimate_yaw_per_sqrt_m",
        "odometry_error_estimate_yaw_per_sqrt_rad",
    ],
    [
        "odometry_error_estimate_tilt_per_sqrt_m",
        "odometry_error_estimate_tilt_per_sqrt_rad",
    ],
];

/// The eval config: `--config` JSON over the odometry's stated odometry error estimate (`stated_odometry_error_estimate`) over the defaults, then every
/// drift term times `odometry_error_estimate_scale`.
fn load_config(
    path: Option<&std::path::Path>,
    odom_child_frame: &str,
) -> Result<EvalConfig, String> {
    let mut overlay = match path {
        Some(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path:?}: {e}"))?;
            serde_json::from_str(&text).map_err(|e| format!("config {path:?}: {e}"))?
        }
        None => serde_json::Value::Object(Default::default()),
    };
    let fields = overlay
        .as_object_mut()
        .ok_or("--config must be a JSON object")?;
    let given = ODOMETRY_ERROR_ESTIMATE_KEYS.iter().flatten().any(|key| fields.contains_key(*key));
    match stated_odometry_error_estimate(odom_child_frame) {
        Some(drift) => {
            for (keys, values) in ODOMETRY_ERROR_ESTIMATE_KEYS.iter().zip(drift) {
                for (key, value) in keys.iter().zip(values) {
                    fields.entry(key.to_string()).or_insert(value.into());
                }
            }
        }
        // as the module: no odometry error estimate stated anywhere, so it follows the lidar's range
        None if !given => {
            eprintln!(
                "pgo-eval: no stated odometry error estimate known for odometry child frame {odom_child_frame:?}; \
                 it follows the lidar's range (odometry_error_estimate_from_lidar_range)"
            );
            fields
                .entry("odometry_error_estimate_from_lidar_range".to_string())
                .or_insert(true.into());
        }
        None => {}
    }
    let mut config: EvalConfig =
        serde_json::from_value(overlay).map_err(|e| format!("config: {e}"))?;
    let k = config.odometry_error_estimate_scale;
    let c = &mut config.pgo;
    for term in [
        &mut c.odometry_error_estimate_xy_per_sqrt_m,
        &mut c.odometry_error_estimate_xy_per_sqrt_rad,
        &mut c.odometry_error_estimate_z_per_sqrt_m,
        &mut c.odometry_error_estimate_z_per_sqrt_rad,
        &mut c.odometry_error_estimate_yaw_per_sqrt_m,
        &mut c.odometry_error_estimate_yaw_per_sqrt_rad,
        &mut c.odometry_error_estimate_tilt_per_sqrt_m,
        &mut c.odometry_error_estimate_tilt_per_sqrt_rad,
    ] {
        *term *= k;
    }
    config.pgo.validate().map_err(|e| format!("config: {e}"))?;
    Ok(config)
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
    // every adopted solve's record (`solve_log_path`) goes next to the output unless GSC_SOLVE_LOG names a file
    if std::env::var_os("GSC_SOLVE_LOG").is_none() {
        let path = args.output.with_file_name("solve.jsonl");
        let _ = std::fs::remove_file(&path);
        std::env::set_var("GSC_SOLVE_LOG", path);
    }

    let connection =
        Connection::open_with_flags(&args.db, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
            .map_err(|e| format!("open {:?}: {e}", args.db))?;

    let mut odoms = read_odometry(&connection, &args.odom_stream)?;
    let odom_frame = unique_frame(odoms.iter().map(|row| row.frame_id.clone()), "odom stream")?;
    let odom_child_frame = unique_frame(
        odoms.iter().map(|row| row.child_frame_id.clone()),
        "odom child",
    )?;
    let config = load_config(args.config_path.as_deref(), &odom_child_frame)?;
    let tf_tree = memory2::TfTree::read(&connection)?;
    // the body's pose in the odom child frame: each odom pose (odom -> child) becomes odom -> child * child -> body. A
    // PoseStamped odom names no child, so it is taken as the body already (and its clouds as in the body frame).
    let child_to_body = if odom_child_frame.is_empty() {
        eprintln!(
            "pgo-eval: WARNING: odom payload has no child frame (PoseStamped); \
             taking its poses and the lidar clouds as already in the body frame (no tf applied)"
        );
        Rigid::identity()
    } else {
        let pose_frame = if config.odom_pose_frame.is_empty() {
            &odom_child_frame
        } else {
            &config.odom_pose_frame
        };
        tf_tree
            .lookup(pose_frame, BODY_FRAME)
            .map_err(|e| format!("odometry pose frame {pose_frame:?} -> {BODY_FRAME}: {e}"))?
    };
    for row in &mut odoms {
        if !child_to_body.is_identity() {
            row.translation = mat3::add(
                &row.translation,
                &mat3::mat_vec(&row.rotation, &child_to_body.translation),
            );
            row.rotation = mat3::mat_mul(&row.rotation, &child_to_body.rotation);
            let [qw, qx, qy, qz] = mat3::quat_from_mat(&row.rotation);
            row.quaternion_xyzw = [qx, qy, qz, qw];
        }
    }
    let [qw, qx, qy, qz] = mat3::quat_from_mat(&child_to_body.rotation);
    eprintln!(
        "pgo-eval: odom {odom_child_frame:?} (poses of {:?}) -> {BODY_FRAME} from tf: t={:?} q(xyzw)={:?}",
        if config.odom_pose_frame.is_empty() { &odom_child_frame } else { &config.odom_pose_frame },
        child_to_body.translation,
        [qx, qy, qz, qw]
    );
    // points in the lidar's frame -> body, looked up once its frame is known (the first scan)
    let mut lidar_to_body: Option<Rigid> = None;

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

    // with an ICP cache the clouds are not dumped: the cache stands in for them (and the dumps run to gigabytes)
    let mut pgo_config = config.pgo.clone();
    if args.icp_cache.is_some() {
        pgo_config.closure_icp_dump_dir.clear();
    }
    // the rayon pool ICP and the solver's factorization run on, as the module builds it (`worker_threads`)
    if let Err(e) = rayon::ThreadPoolBuilder::new()
        .num_threads(config.worker_threads)
        .thread_name(|i| format!("gsc-worker-{i}"))
        .build_global()
    {
        eprintln!("pgo-eval: could not cap the worker threads: {e}");
    }
    let mut pgo = GscPgo::new(pgo_config);
    // a lockstep replay waits for a due job that has not finished (live, it lands late)
    pgo.set_wait_for_due_jobs(!args.live);
    // the ICP cache: recorded, or loaded to answer from
    let icp_cache = match &args.icp_cache {
        Some((mode, path)) => {
            let entries: std::collections::HashMap<String, Vec<f64>> =
                if *mode == dimos_gsc_pgo::icp_cache::Mode::Replay {
                    let bytes = std::fs::read(path).map_err(|e| format!("read {path:?}: {e}"))?;
                    dimos_gsc_pgo::icp_cache::from_bytes(&bytes)
                        .map_err(|e| format!("parse {path:?}: {e}"))?
                } else {
                    Default::default()
                };
            let cache =
                std::sync::Arc::new(dimos_gsc_pgo::icp_cache::IcpCache::new(*mode, entries));
            pgo.set_icp_cache(cache.clone());
            Some(cache)
        }
        None => None,
    };
    let mut cursor = 0usize;
    let mut scans_processed = 0usize;
    let mut recorder = Recorder::new(args.event_dump.as_deref())?;
    let started = Instant::now();
    let mut lidar_frame: Option<String> = None;
    let (mut first_ts, mut last_ts) = (f64::NAN, f64::NAN);
    let live_queue = config.max_scan_queue;
    let mut scans_dropped = 0usize;
    let mut scan_latency_ms: Vec<f32> = Vec::new();
    // each scan paired with the newest odometry and brought into the body frame, as the module receives it
    let mut prepare = |scan: memory2::ScanRow| -> Result<Option<(CloudWithPose, f64)>, String> {
        match &lidar_frame {
            Some(frame) if *frame != scan.frame_id => {
                return Err(format!(
                    "lidar stream mixes frames {frame:?} and {:?}; expected one",
                    scan.frame_id
                ));
            }
            Some(_) => {}
            None => {
                let to_body = if odom_child_frame.is_empty() {
                    Rigid::identity()
                } else {
                    tf_tree
                        .lookup(BODY_FRAME, &scan.frame_id)
                        .map_err(|e| format!("lidar frame -> {BODY_FRAME}: {e}"))?
                };
                let [qw, qx, qy, qz] = mat3::quat_from_mat(&to_body.rotation);
                eprintln!(
                    "pgo-eval: lidar points {:?} -> {BODY_FRAME} from tf: t={:?} q(xyzw)={:?}",
                    scan.frame_id,
                    to_body.translation,
                    [qx, qy, qz, qw]
                );
                if matches!(scan.frame_id.as_str(), "odom" | "map" | "world") {
                    eprintln!(
                        "pgo-eval: WARNING: lidar scans are in {:?}; they should be in the sensor frame, not a fixed frame like odom, map or world",
                        scan.frame_id
                    );
                }
                lidar_to_body = Some(to_body);
                lidar_frame = Some(scan.frame_id.clone());
            }
        }
        if first_ts.is_nan() {
            first_ts = scan.ts;
        }
        last_ts = scan.ts;
        let Some(latest) = advance_latest(&odoms, scan.ts, &mut cursor) else {
            return Ok(None);
        };
        scans_processed += 1;

        let mut cloud = scan.points;
        let to_body = lidar_to_body.as_ref().expect("set with the lidar frame");
        if !to_body.is_identity() {
            cloud = pointcloud::transform_cloud(&cloud, &to_body.rotation, &to_body.translation);
        }

        let mut pose = PoseWithTime::new(latest.rotation, latest.translation);
        let seconds = latest.ts as i32;
        let nanos = ((latest.ts - seconds as f64) * 1e9) as u32;
        pose.set_time(seconds, nanos);

        let cloud_with_pose = CloudWithPose {
            cloud: Some(Arc::new(cloud)),
            pose,
        };
        Ok(Some((cloud_with_pose, latest.ts)))
    };
    if args.live {
        // as the module runs: scans arrive at their recorded times into a queue of max_scan_queue (all but the newest dropped when
        // full), the closure search runs on its own thread and only starts while no scans wait, solves adopt when done
        let queue = std::sync::Mutex::new((std::collections::VecDeque::new(), false));
        let arrived = std::sync::Condvar::new();
        let wall_start = Instant::now();
        let mut feed_error = Ok(());
        std::thread::scope(|scope| -> Result<(), String> {
            scope.spawn(|| {
                let connection = match Connection::open_with_flags(
                    &args.db,
                    rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY,
                ) {
                    Ok(connection) => connection,
                    Err(e) => {
                        feed_error = Err(format!("open {:?}: {e}", args.db));
                        queue.lock().expect("queue").1 = true;
                        arrived.notify_all();
                        return;
                    }
                };
                let mut recording_start = None;
                feed_error =
                    for_each_scan(&connection, &args.lidar_stream, args.until_s, |scan| {
                        let Some((cloud_with_pose, ts)) = prepare(scan)? else {
                            return Ok(());
                        };
                        let offset = ts - *recording_start.get_or_insert(ts);
                        if let Some(wait) = std::time::Duration::from_secs_f64(offset.max(0.0))
                            .checked_sub(wall_start.elapsed())
                        {
                            std::thread::sleep(wait);
                        }
                        let mut queued = queue.lock().expect("queue");
                        queued.0.push_back((cloud_with_pose, ts, Instant::now()));
                        if live_queue > 0 && queued.0.len() > live_queue as usize {
                            // overflow keeps only the newest scan, as the module does
                            scans_dropped += queued.0.len() - 1;
                            let newest = queued.0.pop_back();
                            queued.0.clear();
                            queued.0.extend(newest);
                        }
                        drop(queued);
                        arrived.notify_one();
                        Ok(())
                    });
                queue.lock().expect("queue").1 = true;
                arrived.notify_all();
            });
            loop {
                let (cloud_with_pose, ts, arrived_at) = {
                    let mut queued = queue.lock().expect("queue");
                    loop {
                        if let Some(next) = queued.0.pop_front() {
                            break next;
                        }
                        if queued.1 {
                            pgo.land_remaining_closure_search();
                            return Ok(());
                        }
                        queued = arrived.wait(queued).expect("queue");
                    }
                };
                let step_started = Instant::now();
                if recorder.add_scan(&mut pgo, &cloud_with_pose, ts) {
                    // the closure search's jobs that may land do, and this keyframe's match joins the pipeline
                    pgo.step_closure_search(false);
                    recorder.keyframe_done(&mut pgo, step_started, ts, |pgo| {
                        pgo.update();
                    })?;
                }
                scan_latency_ms.push((arrived_at.elapsed().as_secs_f64() * 1e3) as f32);
            }
        })?;
        feed_error?;
    } else {
        // scans stream from the db one at a time, so the eval holds no more of the recording than the module would
        let fixes = match &args.world_positions {
            Some(path) => read_world_positions(path)?,
            None => Vec::new(),
        };
        let mut next_fix = 0;
        let sightings = match &args.landmark_sightings {
            Some(path) => read_landmark_sightings(path)?,
            None => Vec::new(),
        };
        let mut next_sighting = 0;
        for_each_scan(&connection, &args.lidar_stream, args.until_s, |scan| {
            let Some((cloud_with_pose, ts)) = prepare(scan)? else {
                return Ok(());
            };
            // the fixes up to this scan's time, as the module would have received them
            while next_fix < fixes.len() && fixes[next_fix].0 <= ts {
                let (time, frame, lever, position, covariance) = fixes[next_fix];
                pgo.add_coordinate_factor(time, frame, lever, position, covariance);
                next_fix += 1;
            }
            while next_sighting < sightings.len() && sightings[next_sighting].0 <= ts {
                let (time, landmark, rotation, position, information) = sightings[next_sighting];
                pgo.add_landmark_factor(time, landmark, rotation, position, information);
                next_sighting += 1;
            }
            let step_started = Instant::now();
            if !recorder.add_scan(&mut pgo, &cloud_with_pose, ts) {
                return Ok(());
            }
            pgo.search_for_closure_pairs();
            recorder.keyframe_done(&mut pgo, step_started, ts, |pgo| {
                pgo.wait_for_due_solve();
            })?;
            Ok(())
        })?;
    }
    let finish_started = Instant::now();
    let replay_s = (finish_started - started).as_secs_f64();
    pgo.finish();
    let finish_s = finish_started.elapsed().as_secs_f64();
    let overshoots = pgo.overshoot_report();
    let ts = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
    recorder.dump(&pgo, false, ts)?;
    if let Some(path) = &args.event_dump {
        write_keyframes(path, pgo.key_poses()).map_err(|e| format!("write keyframes: {e}"))?;
    }
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

    let Some(lidar_frame) = lidar_frame else {
        return Err(format!(
            "lidar stream {:?} decoded no PointCloud2 rows",
            args.lidar_stream
        ));
    };
    let data_duration_s = last_ts - first_ts;
    let realtime_factor = if replay_s > 0.0 {
        data_duration_s / replay_s
    } else {
        f64::INFINITY
    };

    let mut gate_counts: BTreeMap<String, usize> = BTreeMap::new();
    for diag in pgo.closure_diagnostics() {
        *gate_counts.entry(diag.outcome.to_string()).or_insert(0) += 1;
    }
    let closure_diagnostics = pgo.closure_diagnostics().to_vec();
    let graph_error = pgo.graph_error();
    let committed_closure_edges: Vec<CommittedClosureEdge> = pgo
        .committed_closures()
        .iter()
        .map(|closure| CommittedClosureEdge::from(&closure))
        .collect();

    let (icp_cache_hits, icp_cache_misses) = match &icp_cache {
        Some(cache) => {
            if cache.mode == dimos_gsc_pgo::icp_cache::Mode::Record {
                let (_, path) = args.icp_cache.as_ref().unwrap();
                std::fs::write(path, dimos_gsc_pgo::icp_cache::to_bytes(&cache.entries()))
                    .map_err(|e| format!("write {path:?}: {e}"))?;
            }
            (
                cache.hits.load(std::sync::atomic::Ordering::Relaxed),
                cache.misses.load(std::sync::atomic::Ordering::Relaxed),
            )
        }
        None => (0, 0),
    };
    // lockstep waits for every due solve on the keyframe thread, and for the last ones in `finish`; live they run on
    // their own thread
    let replay_solve_wait_s = if args.live {
        finish_s
    } else {
        recorder
            .keyframe_gnc_ms
            .iter()
            .map(|&ms| ms as f64 / 1e3)
            .sum::<f64>()
            + finish_s
    };
    let keyframe_thread_s = recorder
        .scan_thread_ms
        .iter()
        .map(|&ms| ms as f64 / 1e3)
        .sum::<f64>();
    let speed = Speed::new(
        &pgo,
        data_duration_s,
        keyframe_thread_s,
        runtime_s,
        replay_solve_wait_s,
    );
    let (odometry_drift, odometry_drift_measured) = pgo.odometry_error_estimate_for_next_run();
    let output = EvalOutput {
        odometry_drift,
        odometry_drift_measured,
        icp_cache_hits,
        icp_cache_misses,
        db: args.db.display().to_string(),
        odom_stream: args.odom_stream.clone(),
        lidar_stream: args.lidar_stream.clone(),
        odom_frame,
        lidar_frame,
        config: serde_json::to_value(&config).unwrap_or(serde_json::Value::Null),
        keyframes: graph.len(),
        odom_rows,
        graph,
        closures: recorder.closures,
        scans_processed,
        odoms_used: odoms.len(),
        data_duration_s,
        runtime_s,
        finish_s,
        realtime_factor,
        live: args.live,
        scans_dropped,
        scan_latency_ms,
        keyframe_scan_thread_ms: recorder.keyframe_scan_thread_ms,
        keyframe_gnc_ms: recorder.keyframe_gnc_ms,
        scan_thread_ms: recorder.scan_thread_ms,
        pressure_calls: pgo.pressure_calls().to_vec(),
        change_cost_calls: pgo.change_cost_calls().0.to_vec(),
        change_cost_recomputed: pgo.change_cost_calls().1,
        runtime_profile: pgo
            .runtime_profile()
            .into_iter()
            .map(|(name, ratio, jobs, coefficients)| {
                (
                    name.to_string(),
                    ratio,
                    jobs,
                    coefficients.into_iter().map(|(f, v)| (f.to_string(), v)).collect(),
                )
            })
            .collect(),
        stage_seconds: pgo
            .stage_seconds()
            .iter()
            .map(|(k, v)| (k.to_string(), *v))
            .collect(),
        speed,
        loop_overlap: recorder.loop_overlap,
        gate_counts,
        closure_diagnostics,
        graph_error,
        committed_closure_edges,
    };

    let json = serde_json::to_string_pretty(&output).map_err(|e| e.to_string())?;
    std::fs::write(&args.output, json).map_err(|e| format!("write {:?}: {e}", args.output))?;

    let speed = &output.speed;
    let parts: Vec<String> = speed
        .search_parts
        .iter()
        .map(|(part, seconds)| format!("{part} {seconds:.1}s"))
        .collect();
    eprintln!(
        "pgo-eval: keyframe thread {:.1}x real ({:.1}s measured, waits left out; scan work {:.1}s of it); closure search workers {:.1}s of jobs ({:.1}x real summed over the pools: {})",
        speed.keyframe_thread.x_real,
        speed.keyframe_thread.seconds,
        speed.scan.seconds,
        speed.search_workers.seconds,
        speed.search_workers.x_real,
        parts.join(", "),
    );
    eprintln!(
        "pgo-eval: solver thread {} solves, median {:.2}s, max {:.2}s; lag median {:.0} keyframes / {:.1}s, max {} keyframes / {:.1}s; {} overshot their estimate; this replay waited {:.1}s for solves",
        speed.solves,
        speed.solve_median_s,
        speed.solve_max_s,
        speed.lag_median_keyframes,
        speed.lag_median_s,
        speed.lag_max_keyframes,
        speed.lag_max_s,
        speed.solve_overshoots,
        speed.replay_solve_wait_s,
    );
    eprintln!(
        "pgo-eval: overshoots per LET (jobs): {}",
        overshoots
            .iter()
            .map(|(kind, jobs, over)| format!("{kind} {over} / {jobs}"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    eprintln!(
        "pgo-eval: pressure: {} computations, {} overshot their estimate; graph change cost: {} measured on the solve's thread, {} recomputed on the scan thread",
        pgo.pressure_calls().len(),
        pgo.pressure_overshoots(),
        pgo.change_cost_calls().0.len(),
        pgo.change_cost_calls().1,
    );
    {
        let mut ms: Vec<f32> = output.scan_thread_ms.clone();
        ms.sort_by(f32::total_cmp);
        let at = |q: f64| ms.get(((ms.len() as f64 - 1.0) * q).round() as usize).copied().unwrap_or(f32::NAN);
        eprintln!(
            "pgo-eval: scan thread work per scan (waits left out): median {:.3} ms, p99 {:.3} ms, max {:.1} ms over {} scans",
            at(0.5),
            at(0.99),
            ms.last().copied().unwrap_or(f32::NAN),
            ms.len()
        );
    }
    // the LET coefficients this run measured, a config block ready to paste (each LET's coefficients scaled by its jobs'
    // 99th-percentile measured / estimated scans; diagnostics only)
    {
        let mut block = Vec::new();
        let mut ratios = Vec::new();
        for (name, ratio, jobs, coefficients) in &output.runtime_profile {
            ratios.push(format!("{name} {ratio:.2} ({jobs} jobs)"));
            for (field, value) in coefficients {
                if value.is_finite() {
                    block.push(format!("\"{field}\": {value:.4e}"));
                }
            }
        }
        eprintln!("pgo-eval: LET measured / estimated scans, 99th percentile: {}", ratios.join(", "));
        let _ = block;
        // the exact `deterministic_config` object to paste (diagnostics only)
        eprintln!(
            "pgo-eval: suggested deterministic_config: {}",
            serde_json::to_string(&pgo.suggested_deterministic_config()).unwrap_or_default()
        );
    }
    eprintln!(
        "pgo-eval: closure searches: {} overshot their estimate ({} verifications); the buffers dropped {} scan-context candidates and {} matches waiting for verification; this replay waited {:.1}s for searches",
        pgo.closure_search_overshoots(),
        pgo.verification_overshoots(),
        pgo.buffer_drops().0,
        pgo.buffer_drops().1,
        pgo.closure_search_wait_s(),
    );
    if let Some(range_m) = pgo.lidar_range_estimate_m() {
        eprintln!(
            "pgo-eval: lidar range estimate {range_m:.2} m -> odometry error estimate {:?} (odometry_error_estimate_from_lidar_range)",
            pgo.config_odometry_error_estimate()
        );
    }
    eprintln!(
        "pgo-eval: {} scans, {} keyframes, {} closures, {:.1}s data in {:.2}s replay wall time ({:.1}x realtime; then {:.1}s in finish) -> {:?}",
        scans_processed,
        output.keyframes,
        output.closures,
        output.data_duration_s,
        replay_s,
        output.realtime_factor,
        output.finish_s,
        args.output,
    );
    // where the time went: each stage's wall seconds, and the single ICPs run (candidates that reached ICP) and their cost
    let stages: Vec<String> = output
        .stage_seconds
        .iter()
        .map(|(stage, seconds)| format!("{stage} {seconds:.1}s"))
        .collect();
    // a candidate that ran its single ICP has its fitness set (NaN until then)
    let icps = output
        .closure_diagnostics
        .iter()
        .filter(|d| !d.icp_fitness.is_nan())
        .count();
    let icp_seconds = output.stage_seconds.get("icp").copied().unwrap_or(0.0);
    eprintln!(
        "pgo-eval: stages {}; {} ICPs at {:.0} ms each",
        stages.join(", "),
        icps,
        1000.0 * icp_seconds / icps.max(1) as f64
    );
    Ok(())
}


/// Seconds and how many times faster than real (data seconds / seconds) one part of the work runs.
#[derive(Serialize)]
struct Pace {
    seconds: f64,
    x_real: f64,
}

impl Pace {
    fn of(seconds: f64, data_s: f64) -> Pace {
        Pace {
            seconds,
            x_real: if seconds > 0.0 {
                data_s / seconds
            } else {
                f64::INFINITY
            },
        }
    }
}

/// The module's speed by thread. `keyframe_thread`, the headline, is the keyframe thread's measured work (every scan's,
/// waits left out): the per-scan work (`scan`: drop-reseen, aggregation, voxelizing, the keyframe and its scan-context
/// descriptor, plus the degeneracy check), the scan-context lookup and proposals, and dispatching the jobs, never a
/// solve. `search_workers` is the closure search's job seconds (the fast-match and verification pools, the pressure),
/// summed over the jobs: off the keyframe thread, so over its pools' threads it can pass the wall time. The solver is its own thread live: its solves' seconds, and each solve's lag, the
/// keyframes and data seconds from its dispatch to the first keyframe that arrives after it would have finished live.
/// `replay_solve_wait_s` is what this deterministic replay spent waiting for solves (not a cost live).
#[derive(Serialize)]
struct Speed {
    scan: Pace,
    search_workers: Pace,
    keyframe_thread: Pace,
    /// The search's parts in seconds: scan-context lookup, the rest of proposing, pressure checks, single ICP, extension
    /// and everything else a search does (submaps, gates, likelihoods).
    search_parts: BTreeMap<String, f64>,
    solves: usize,
    solve_median_s: f64,
    solve_max_s: f64,
    lag_median_keyframes: f64,
    lag_max_keyframes: usize,
    lag_median_s: f64,
    lag_max_s: f64,
    /// Solves slower than the data their estimate spanned (`GscPgo::solve_overshoots`).
    solve_overshoots: usize,
    replay_solve_wait_s: f64,
    replay_wall: Pace,
}

/// The median of `values` (NaN when empty).
fn median(mut values: Vec<f64>) -> f64 {
    values.sort_by(f64::total_cmp);
    match values.len() {
        0 => f64::NAN,
        n if n % 2 == 1 => values[n / 2],
        n => (values[n / 2 - 1] + values[n / 2]) / 2.0,
    }
}

impl Speed {
    fn new(
        pgo: &GscPgo,
        data_s: f64,
        keyframe_thread_s: f64,
        replay_wall_s: f64,
        replay_solve_wait_s: f64,
    ) -> Speed {
        let stage: BTreeMap<&str, f64> = pgo.stage_seconds().into_iter().collect();
        let at = |name: &str| stage.get(name).copied().unwrap_or(0.0);
        let scan = at("scan") + at("degeneracy");
        let search = at("propose") + at("pressure") + at("search") - at("degeneracy");
        let search_parts = BTreeMap::from([
            ("lookup".to_string(), at("scan_context")),
            ("proposals".to_string(), at("propose") - at("scan_context")),
            ("pressure".to_string(), at("pressure")),
            ("icp".to_string(), at("icp")),
            ("extension".to_string(), at("extended_icp")),
            (
                "other".to_string(),
                at("search") - at("icp") - at("extended_icp") - at("degeneracy"),
            ),
        ]);
        let times: Vec<f64> = pgo.key_poses().iter().map(|k| k.time).collect();
        let (mut lag_keyframes, mut lag_s) = (Vec::new(), Vec::new());
        for &(keyframes, dispatched_at, seconds) in pgo.solve_records() {
            let done = dispatched_at + seconds;
            let first = times.partition_point(|&t| t < done);
            lag_keyframes.push(first.saturating_sub(keyframes - 1));
            lag_s.push(times.get(first).map_or(seconds, |&t| t - dispatched_at));
        }
        let solve_s: Vec<f64> = pgo.solve_records().iter().map(|r| r.2).collect();
        Speed {
            scan: Pace::of(scan, data_s),
            search_workers: Pace::of(search, data_s),
            keyframe_thread: Pace::of(keyframe_thread_s, data_s),
            search_parts,
            solves: solve_s.len(),
            solve_median_s: median(solve_s.clone()),
            solve_max_s: solve_s.iter().copied().fold(f64::NAN, f64::max),
            lag_median_keyframes: median(lag_keyframes.iter().map(|&k| k as f64).collect()),
            lag_max_keyframes: lag_keyframes.iter().copied().max().unwrap_or(0),
            lag_median_s: median(lag_s.clone()),
            lag_max_s: lag_s.iter().copied().fold(f64::NAN, f64::max),
            solve_overshoots: pgo.solve_overshoots(),
            replay_solve_wait_s,
            replay_wall: Pace::of(replay_wall_s, data_s),
        }
    }
}

/// What both replays record per scan and keyframe: the loop-overlap pressures, per-keyframe timings and closures, and
/// with `--event-dump` every keyframe pose change as the module publishes it (`<path>.closures`) and the closure-edge
/// events (`<path>.edges`).
struct Recorder {
    heard: HeardPoses,
    events: Option<(std::fs::File, std::fs::File)>,
    edges_written: usize,
    closures: usize,
    keyframe_scan_thread_ms: Vec<f32>,
    keyframe_gnc_ms: Vec<f32>,
    loop_overlap: Vec<(f64, LoopOverlap, usize)>,
    /// Per scan, ms of the scan thread's own work (`Output::scan_thread_ms`), and the waits when this scan began.
    scan_thread_ms: Vec<f32>,
    step_waits_s: f64,
}

impl Recorder {
    fn new(event_dump: Option<&std::path::Path>) -> Result<Recorder, String> {
        let create = |suffix: &str, path: &std::path::Path| {
            std::fs::File::create(format!("{}{suffix}", path.display()))
                .map_err(|e| format!("create {suffix} dump: {e}"))
        };
        let events = match event_dump {
            Some(path) => Some((create(".closures", path)?, create(".edges", path)?)),
            None => None,
        };
        Ok(Recorder {
            heard: HeardPoses::default(),
            events,
            edges_written: 0,
            closures: 0,
            keyframe_scan_thread_ms: Vec::new(),
            keyframe_gnc_ms: Vec::new(),
            loop_overlap: Vec::new(),
            scan_thread_ms: Vec::new(),
            step_waits_s: 0.0,
        })
    }

    /// `add_key_pose`, keeping each new pressure; returns whether the scan became a keyframe.
    fn add_scan(&mut self, pgo: &mut GscPgo, cloud_with_pose: &CloudWithPose, ts: f64) -> bool {
        let (started, waits) = (Instant::now(), pgo.thread_waits_s());
        let keyframe = pgo.add_key_pose(cloud_with_pose);
        self.step_waits_s = waits;
        self.scan_thread_ms.push(
            ((started.elapsed().as_secs_f64() - (pgo.thread_waits_s() - waits)) * 1e3) as f32,
        );
        if let Some((scan, overlap)) = pgo.pressure() {
            if self
                .loop_overlap
                .last()
                .is_none_or(|(_, _, sent)| *sent != scan)
            {
                self.loop_overlap.push((ts, overlap, scan));
            }
        }
        if keyframe {
            self.heard.add_new(pgo.key_poses());
        }
        keyframe
    }

    /// After a keyframe's closure search: count its closure and time the scan thread, run `solve` (timed), then dump.
    fn keyframe_done(
        &mut self,
        pgo: &mut GscPgo,
        step_started: Instant,
        ts: f64,
        solve: impl FnOnce(&mut GscPgo),
    ) -> Result<(), String> {
        let had_closure = pgo.has_closure();
        if had_closure {
            self.closures += 1;
        }
        self.keyframe_scan_thread_ms
            .push((step_started.elapsed().as_secs_f64() * 1e3) as f32);
        let gnc_started = Instant::now();
        solve(pgo);
        self.keyframe_gnc_ms
            .push((gnc_started.elapsed().as_secs_f64() * 1e3) as f32);
        // this keyframe's scan: its add_key_pose, closure search and solve steps, the waits left out
        if let Some(last) = self.scan_thread_ms.last_mut() {
            *last = ((step_started.elapsed().as_secs_f64() - (pgo.thread_waits_s() - self.step_waits_s))
                * 1e3) as f32;
        }
        self.dump(pgo, had_closure, ts)
    }

    /// With `--event-dump`, append the pose changes (when a closure closed or a keyframe moved) and the new closure-edge
    /// events.
    fn dump(&mut self, pgo: &GscPgo, closure_closed: bool, ts: f64) -> Result<(), String> {
        let Some((events, edges)) = &mut self.events else {
            return Ok(());
        };
        if let Some(changes) = self.heard.take_changes(pgo.key_poses(), closure_closed) {
            append_pose_changes(events, ts, &changes)
                .map_err(|e| format!("write event dump: {e}"))?;
        }
        let new_edges = &pgo.closure_edge_events()[self.edges_written..];
        if !new_edges.is_empty() {
            edges
                .write_all(&closure_edge_records(new_edges, pgo.key_poses()))
                .map_err(|e| format!("write edge dump: {e}"))?;
            self.edges_written += new_edges.len();
        }
        Ok(())
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("pgo-eval: {error}");
        std::process::exit(1);
    }
}
