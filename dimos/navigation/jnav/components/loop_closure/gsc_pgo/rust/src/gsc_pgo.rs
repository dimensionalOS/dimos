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

//! `GscPgo`: keyframe gating, the loop search and its gates, and the pose graph (each keyframe's
//! odometry plus the loops that passed the gates), re-solved by `pose_graph` on a background thread
//! whenever new loops arrive. Keyframe ids are their indices; rotations and vectors are `mat3`
//! (row-major f64).

use std::sync::Arc;
use std::thread::JoinHandle;

use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    self, cloud_degeneracy, icp_point_to_point_with_tree, transform_cloud, voxel_downsample,
    IcpParams, KdTree, PointCloud,
};
use crate::pose_graph::{self, Edge, Solution, SolveConfig};
use crate::scan_context;

/// A pose paired with a timestamp.
#[derive(Debug, Clone)]
pub struct PoseWithTime {
    pub translation: Vec3,
    pub rotation: Mat3,
    pub sec: i32,
    pub nsec: u32,
    pub second: f64,
}
impl PoseWithTime {
    pub fn new(rotation: Mat3, translation: Vec3) -> PoseWithTime {
        PoseWithTime {
            translation,
            rotation,
            sec: 0,
            nsec: 0,
            second: 0.0,
        }
    }

    /// Set the timestamp from sec/nsec, deriving the fractional `second`.
    pub fn set_time(&mut self, sec: i32, nsec: u32) {
        self.sec = sec;
        self.nsec = nsec;
        self.second = sec as f64 + nsec as f64 / 1e9;
    }
}

/// A point cloud paired with the pose it was captured at.
#[derive(Debug, Clone)]
pub struct CloudWithPose {
    pub cloud: Option<Arc<PointCloud>>,
    pub pose: PoseWithTime,
    pub frame_id: String,
}
/// A keyframe: its local and global poses, time, and body-frame cloud.
#[derive(Debug, Clone)]
pub struct KeyPoseWithCloud {
    pub rotation_local: Mat3,
    pub translation_local: Vec3,
    pub rotation_global: Mat3,
    pub translation_global: Vec3,
    pub time: f64,
    pub body_cloud: Option<Arc<PointCloud>>,
}

/// A loop that passed every gate: its relative pose (source in the target's frame), its ICP
/// fitness, and the newest applied solve's verdict.
#[derive(Clone, Copy)]
pub struct CommittedLoop {
    pub source_id: usize,
    pub target_id: usize,
    pub rotation_offset: Mat3,
    pub translation_offset: Vec3,
    pub score: f64,
    /// 1 kept, 0 rejected by the newest applied solve; NaN until a solve has judged it.
    pub gnc_weight: f64,
}

/// Per-search loop-closure diagnostics
/// one record every time `search_for_loop_pairs` evaluates a keyframe, capturing the metric at each
/// gate and the outcome (accepted, or the name of the gate that rejected it).
/// Lets an offline eval reconstruct WHERE and WHY closures are accepted/rejected
#[derive(Debug, Clone)]
pub struct LoopCandidateDiag {
    /// Outcome: "accepted", or the rejecting gate — one of "throttle_min_duration", "throttle_search_interval", "throttle_backlog", "no_structure",
    /// "low_occupancy", "no_candidate", "lowe_reject", "distance_reject",
    /// "degeneracy_reject", "icp_reject", "yank_reject".
    pub outcome: &'static str,
    pub source_id: usize,
    pub source_time: f64,
    pub source_xyz: Vec3,
    /// Candidate target keyframe, or -1 if no candidate was found.
    pub target_id: i64,
    pub target_time: f64,
    pub target_xyz: Vec3,
    pub from_scan_context: bool,
    /// Scan-context descriptor structure std and occupied-cell count of the
    /// source keyframe (the feature-poverty + occupancy gate inputs).
    pub descriptor_structure: f32,
    pub descriptor_occupancy: i32,
    /// Best / second-best scan-context distances and their Lowe ratio
    /// (best/second). NaN when the candidate came from the position fallback.
    pub scan_context_best: f32,
    pub scan_context_second: f32,
    pub lowe_ratio: f32,
    /// Global distance between source and candidate keyframe (the
    /// candidate-distance gate input).
    pub candidate_distance: f64,
    /// Minimum degeneracy (observability) of the source submap, -1 if not
    /// computed on this path.
    pub degeneracy_min: f32,
    pub icp_converged: bool,
    /// ICP fitness (the score-threshold gate input); NaN if ICP did not run.
    pub icp_fitness: f64,
    /// Magnitude of the ICP translation correction, NaN if ICP did not run.
    pub icp_translation_norm: f64,
    /// How far the accepted constraint pulls the graph away from the current
    /// odom-chain estimate: translation (m) and rotation (deg) between the
    /// ICP-refined relative pose and the current relative pose. A large yank on
    /// a plausible-distance candidate is the signature of a bad closure that
    /// disagrees with odometry. NaN if ICP did not run.
    pub yank_translation: f64,
    pub yank_rotation_deg: f64,
    /// The inputs and outputs of the yank / confidence gate (`loop_yank_gate`), recorded for every
    /// candidate that reaches ICP (NaN otherwise) whether the gate is on or not:
    /// yank_drift_scale = sqrt(keyframe gap) with `loop_yank_per_sqrt_gap`, else 1;
    /// sigma_* = 1.4826 x the median yank / drift scale of the candidates ICP scored before it (NaN before any);
    /// yank_z = hypot(yank_rotation_deg / drift / sigma_rotation_deg, yank_translation / drift / sigma_translation_m)
    /// (0 before any); each factor is its metric normalized by its own gate and clamped to [0, 1];
    /// confidence = their product; required_confidence = 1 - exp(-yank_z^2 / 2k^2), k = `loop_yank_gate_width`.
    pub yank_drift_scale: f64,
    pub sigma_rotation_deg: f64,
    pub sigma_translation_m: f64,
    pub yank_z: f64,
    pub scan_context_factor: f64,
    pub icp_factor: f64,
    pub degeneracy_factor: f64,
    pub confidence: f64,
    pub required_confidence: f64,
    /// Extended ICP (`loop_extended_icp_max_keyframes`), recorded when it ran (0 / NaN otherwise):
    /// the source keyframes stacked into the last round, both clouds' sizes, the source points
    /// within the correspondence distance of the target for the single scan and the extended
    /// clouds, the extended ICP's fitness, and the confidence that evidence gives.
    pub extended_keyframes: usize,
    pub extended_source_points: usize,
    pub extended_target_points: usize,
    pub single_inliers: usize,
    pub extended_inliers: usize,
    pub extended_fitness: f64,
    pub extended_confidence: f64,
    /// Rise in the graph's error if this loop were forced in rigidly (`pose_graph::strain`), with
    /// `loop_record_strain`; NaN otherwise.
    pub strain: f64,
}
impl LoopCandidateDiag {
    fn new(source_id: usize, source_time: f64, source_xyz: Vec3) -> LoopCandidateDiag {
        LoopCandidateDiag {
            outcome: "no_candidate",
            source_id,
            source_time,
            source_xyz,
            target_id: -1,
            target_time: f64::NAN,
            target_xyz: [f64::NAN; 3],
            from_scan_context: false,
            descriptor_structure: f32::NAN,
            descriptor_occupancy: -1,
            scan_context_best: f32::NAN,
            scan_context_second: f32::NAN,
            lowe_ratio: f32::NAN,
            candidate_distance: f64::NAN,
            degeneracy_min: -1.0,
            icp_converged: false,
            icp_fitness: f64::NAN,
            icp_translation_norm: f64::NAN,
            yank_translation: f64::NAN,
            yank_rotation_deg: f64::NAN,
            yank_drift_scale: f64::NAN,
            sigma_rotation_deg: f64::NAN,
            sigma_translation_m: f64::NAN,
            yank_z: f64::NAN,
            scan_context_factor: f64::NAN,
            icp_factor: f64::NAN,
            degeneracy_factor: f64::NAN,
            confidence: f64::NAN,
            required_confidence: f64::NAN,
            extended_keyframes: 0,
            extended_source_points: 0,
            extended_target_points: 0,
            single_inliers: 0,
            extended_inliers: 0,
            extended_fitness: f64::NAN,
            extended_confidence: f64::NAN,
            strain: f64::NAN,
        }
    }
}

/// Where a loop candidate is in its life: proposed by the search, through the gates, GNC's
/// votes, in or out of the graph, and applied to the map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LoopEdgeState {
    /// The search found a candidate keyframe for this one.
    Proposed = 0,
    /// A gate (distance, degeneracy, ICP, yank, ...) turned it down; it never reaches GNC.
    GateRejected = 1,
    /// Passed every gate; waits for GNC to vote it into the graph.
    Committed = 2,
    /// One GNC solve's weight for it.
    Vote = 3,
    /// GNC voted it into the graph.
    Accepted = 4,
    /// GNC voted it out of the graph (never in, or taken back out).
    Rejected = 5,
    /// The graph update that moved the map with it in force.
    Applied = 6,
}

/// One step in a loop candidate's life, for visualizing proposed / accepted /
/// rejected / applied edges as they happen. Purely observational.
#[derive(Debug, Clone, Copy)]
pub struct LoopEdgeEvent {
    /// Keyframe time of the newest keyframe when it happened.
    pub time: f64,
    pub source_id: usize,
    pub target_id: usize,
    pub state: LoopEdgeState,
    /// How likely the edge is right, in [0, 1]: scan-context similarity while only proposed,
    /// ICP fitness against the fitness its yank requires once ICP ran, then GNC's weight times that.
    pub confidence: f64,
    /// How hard the edge pulls the accepted pose graph, in robust standard deviations of the
    /// yanks of loops committed so far (`LoopCandidateDiag::yank_z`); NaN before ICP.
    pub yank_z: f64,
}

/// Bytes per packed loop-edge record (`LoopEdgeEvent::put_record`).
pub const LOOP_EDGE_RECORD_BYTES: usize = 52;

impl LoopEdgeEvent {
    /// Append this event as one packed record: both keyframes where they are now (x y z, then
    /// target x y z, f32), source and target ids, state as u32, confidence f32, yank_z f32, time f64.
    pub fn put_record(&self, data: &mut Vec<u8>, key_poses: &[KeyPoseWithCloud]) {
        let at = |id: usize| {
            key_poses
                .get(id)
                .map_or([f64::NAN; 3], |kp| kp.translation_global)
        };
        for v in at(self.source_id).into_iter().chain(at(self.target_id)) {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
        data.extend_from_slice(&(self.source_id as u32).to_le_bytes());
        data.extend_from_slice(&(self.target_id as u32).to_le_bytes());
        data.extend_from_slice(&(self.state as u32).to_le_bytes());
        data.extend_from_slice(&(self.confidence as f32).to_le_bytes());
        data.extend_from_slice(&(self.yank_z as f32).to_le_bytes());
        data.extend_from_slice(&self.time.to_le_bytes());
    }
}

/// Tunable configuration for the PGO core.
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "native",
    derive(serde::Serialize, serde::Deserialize),
    serde(default)
)]
pub struct Config {
    pub keyframe_min_rotation_degrees: f64,
    pub keyframe_min_distance_meters: f64,
    pub loop_search_radius: f64,
    pub loop_time_thresh: f64,
    pub loop_score_thresh: f64,
    pub loop_submap_half_range: i32,
    pub submap_resolution: f64,
    /// Voxel size each incoming cloud is downsampled to before a keyframe
    /// stores it. 0 keeps clouds as they arrive.
    pub keyframe_voxel_size: f64,
    pub min_loop_detect_duration: f64,
    /// Keyframe-time seconds between loop searches, whatever their outcome
    /// (a rejected ICP costs as much as an accepted one). 0 disables.
    pub loop_search_min_interval: f64,
    /// Structure-spread gate: minimum occupied Scan-Context cells. 0 disables.
    pub loop_min_occupancy: i32,
    /// Observability gate (Zhang 2016 / X-ICP degeneracy factor). 0 disables.
    pub loop_min_degeneracy: f64,
    /// Reject a loop whose match confidence is too low for how hard it yanks the accepted pose
    /// graph: confidence < 1 - exp(-yank_z^2 / 2) (see `LoopCandidateDiag`). A small yank needs
    /// little confidence, a large one needs a near-perfect match. The inputs are recorded either way.
    pub loop_yank_gate: bool,
    /// k in the yank gate's 1 - exp(-yank_z^2 / 2k^2): how many sigmas of yank before a near-perfect match is required.
    pub loop_yank_gate_width: f64,
    /// Measure yank against the drift odometry could have built up between the loop's ends: divide
    /// it by sqrt(keyframe gap) before scaling (on huge_loop this ranks loops that help above loops
    /// that hurt at AUC 0.82 vs 0.70). Off = the plain yank.
    pub loop_yank_per_sqrt_gap: bool,
    /// When set, every candidate ICP scores writes `<dir>/<source>_<target>.icp`: the source cloud
    /// and target submap exactly as ICP got them (global frame, current graph), then ICP's
    /// rotation (row-major) and translation, fitness and convergence. Little-endian: u32 source
    /// count, u32 target count, f32 xyz points, 9 + 3 f64, f64 fitness, u8 converged.
    pub loop_icp_dump_dir: String,
    /// Record each ICP-scored candidate's `strain` (one sparse factorization each, on the search thread).
    pub loop_record_strain: bool,
    /// The yank gate measures a candidate by its `strain` (how hard the graph resists it as a near-rigid
    /// edge), scaled by the median strain of the candidates scored so far, instead of its raw yank.
    pub loop_yank_from_strain: bool,
    /// With `loop_yank_gate`, a candidate whose single-scan confidence is too low for its yank gets
    /// a second look on more data instead of an outright reject: each round seeded with the previous
    /// round's ICP (the first round with the single-scan ICP), the source grows to the last 2, 4, 8, ... keyframes (up to this many), and the target to the
    /// keyframes near the matched one in time that those land within `loop_search_radius` of.
    /// Inliers are source points within sqrt(`loop_score_thresh`) of the target after ICP.
    /// Each round counts as (extended inliers / single-scan inliers) independent checks, each as
    /// good as the single scan's confidence times how well the overlap held up (extended inlier
    /// fraction / single inlier fraction, at most 1): confidence = 1 - (1 - c_round)^checks.
    /// (Mean squared fitness is not used: the stacked clouds see places the other side never did.)
    /// The first round that clears the gate is kept, and its transform becomes the loop. 0 = off.
    pub loop_extended_icp_max_keyframes: usize,
    /// Huber threshold on a kept loop's whitened residual in the final solve; 0 = plain least squares.
    pub loop_robust_huber_k: f64,
    /// Loop-verification ICP: iteration cap and correspondence gate (m).
    pub loop_icp_max_iterations: u64,
    pub loop_icp_max_correspondence_m: f64,
    /// GNC loop-factor variance is `score * loop_gnc_var_scale`. The raw ICP
    /// score gives a variance (~1e-3) tight enough that GNC over-rejects; scaling
    /// it up loosens each loop so GNC keeps the true consensus set.
    pub loop_gnc_var_scale: f64,
    /// Probability that a true inlier's residual falls under GNC's chi-squared
    /// cost threshold. Lowering it tightens the test so GNC rejects more loops;
    /// outside (0, 1) it keeps GNC's own 0.99 default.
    pub loop_gnc_inlier_probability: f64,
    /// A background GNC solve is folded in once the map has grown by this fraction of its size
    /// at dispatch (at least one keyframe), waiting for it if it is not done, so the result
    /// depends on the data, not the machine. A solve's cost grows with the map, so the delay does.
    pub loop_gnc_apply_after_fraction: f64,
    /// true: fold a background GNC solve on the keyframe schedule above, so a replay gives the same
    /// graph on any machine. false: fold it as soon as it finishes (see `poll_gnc`), keyframes or
    /// not, so a robot standing still at a loop still gets the closure; results then depend on timing.
    pub loop_gnc_deterministic: bool,
    // Odometry between-factor noise (anisotropic).
    pub odom_rot_roll_pitch_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,
    // Scan Context settings.
    pub use_scan_context: bool,
    pub scan_context_num_rings: i32,
    pub scan_context_num_sectors: i32,
    pub scan_context_max_range_m: f64,
    pub scan_context_top_k: i32,
    pub scan_context_match_threshold: f64,
    pub scan_context_lidar_height_m: f64,
}

impl Default for Config {
    fn default() -> Config {
        Config {
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
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_yank_gate: true,
            loop_yank_gate_width: 1.0,
            loop_yank_per_sqrt_gap: true,
            loop_icp_dump_dir: String::new(),
            loop_record_strain: false,
            loop_yank_from_strain: false,
            loop_extended_icp_max_keyframes: 16,
            loop_robust_huber_k: 1.345,
            loop_icp_max_iterations: 50,
            loop_icp_max_correspondence_m: 10.0,
            loop_gnc_var_scale: 10.0,
            loop_gnc_inlier_probability: 0.95,
            loop_gnc_apply_after_fraction: 0.05,
            loop_gnc_deterministic: true,
            odom_rot_roll_pitch_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            use_scan_context: true,
            scan_context_num_rings: 20,
            scan_context_num_sectors: 60,
            scan_context_max_range_m: 0.0,
            scan_context_top_k: 10,
            scan_context_match_threshold: 0.4,
            scan_context_lidar_height_m: 2.0,
        }
    }
}

/// A pose-graph solve running on another thread over the first `keyframes` keyframes, adopted once
/// the map reaches `due_at` keyframes or, on request, as soon as it finishes (see `GscPgo::update`).
struct Solving {
    handle: JoinHandle<(Solution, f64)>,
    keyframes: usize,
    due_at: usize,
}

pub struct GscPgo {
    config: Config,
    scan_context_config: scan_context::Config,
    key_poses: Vec<KeyPoseWithCloud>,
    history_pairs: Vec<(usize, usize)>,
    committed_loops: Vec<CommittedLoop>,
    /// Per committed loop: its yank_z and gate confidence, for its loop-edge events.
    loop_yank_z: Vec<f64>,
    loop_fit_confidence: Vec<f64>,
    /// A loop was committed since the last `update`.
    new_loop: bool,
    /// Committed loops a solve has been started for.
    loops_dispatched: usize,
    solving: Option<Solving>,
    graph_error: f64,
    scan_context_descriptors: Vec<scan_context::Descriptor>,
    scan_context_ring_keys: Vec<scan_context::RingKey>,
    rotation_offset: Mat3,
    translation_offset: Vec3,
    timing: TimingStats,
    loop_diagnostics: Vec<LoopCandidateDiag>,
    loop_edge_events: Vec<LoopEdgeEvent>,
    /// Rotation (deg) and translation (m) yank of every candidate ICP has scored (per sqrt keyframe
    /// gap with `loop_yank_per_sqrt_gap`), the scale new yanks are measured against. Every scored
    /// candidate, not just committed ones: a scale taken only from loops the gate let through would
    /// shrink with each rejection and reject the rest.
    scored_yanks: Vec<(f64, f64)>,
    /// Keyframe time of the last loop search that got past the throttles.
    last_loop_search_time: Option<f64>,
}

/// Cumulative per-stage wall time, printed periodically when `debug` is on —
/// purely observational (never read back into the pipeline).
#[derive(Default)]
struct TimingStats {
    scan_context_s: f64,
    submap_s: f64,
    icp_s: f64,
    degeneracy_s: f64,
    extended_icp_s: f64,
    solve_s: f64,
}

/// The ICP half of one loop search, on its own copy of the keyframes so it can run on another thread
/// while scans keep arriving (`GscPgo::begin_loop_search` makes it, `finish_loop_search` applies it).
pub struct LoopSearch {
    key_poses: Vec<KeyPoseWithCloud>,
    config: Config,
    scored_yanks: Vec<(f64, f64)>,
    /// The loops the newest solve kept, for `loop_record_strain`.
    kept_loops: Vec<Edge>,
    diag: LoopCandidateDiag,
    loop_idx: usize,
    init_rotation: Mat3,
    init_translation: Vec3,
}

/// What a `LoopSearch` found: its diagnostics, the loop it accepts (relative rotation, translation,
/// fitness), the yank it scored, and the time each stage took.
pub struct LoopSearchResult {
    diag: LoopCandidateDiag,
    accepted: Option<(Mat3, Vec3, f64)>,
    scored_yank: Option<(f64, f64)>,
    timing: TimingStats,
}

impl LoopSearch {
    /// The ICP half of a loop search: submaps, ICP, its gates, the yank / confidence gate and extended ICP.
    pub fn run(self) -> LoopSearchResult {
        let mut diag = self.diag.clone();
        let (cur_idx, loop_idx) = (diag.source_id, self.loop_idx);
        let (init_rotation, init_translation) = (self.init_rotation, self.init_translation);
        let mut timing = TimingStats::default();
        let mut scored_yank = None;
        let t0 = std::time::Instant::now();
        let target_cloud = sub_map(
            &self.key_poses,
            loop_idx as i32,
            self.config.loop_submap_half_range,
            self.config.submap_resolution,
        );
        let source_cloud = sub_map(
            &self.key_poses,
            cur_idx as i32,
            0,
            self.config.submap_resolution,
        );
        timing.submap_s += t0.elapsed().as_secs_f64();

        let t0 = std::time::Instant::now();
        let target_tree = KdTree::build(&target_cloud);
        let icp = icp_point_to_point_with_tree(
            &source_cloud,
            &target_cloud,
            &target_tree,
            &init_rotation,
            &init_translation,
            &IcpParams {
                max_iterations: self.config.loop_icp_max_iterations as usize,
                max_correspondence_distance: self.config.loop_icp_max_correspondence_m,
                ..IcpParams::default()
            },
        );
        timing.icp_s += t0.elapsed().as_secs_f64();
        if !self.config.loop_icp_dump_dir.is_empty() {
            dump_icp_inputs(
                &self.config.loop_icp_dump_dir,
                cur_idx,
                loop_idx,
                &source_cloud,
                &target_cloud,
                &icp,
                "",
            );
        }
        diag.icp_converged = icp.converged;
        diag.icp_fitness = icp.fitness;
        diag.icp_translation_norm = mat3::norm(&icp.translation);

        // Observability gate: a planar/degenerate source scan leaves the
        // alignment unconstrained in-plane — fitness lies. Computed for every
        // candidate that reaches ICP (not just when the gate is on) so the
        // metric is always available to compare good vs bad closures.
        let t0 = std::time::Instant::now();
        let (degeneracy_min, _) = cloud_degeneracy(&source_cloud);
        timing.degeneracy_s += t0.elapsed().as_secs_f64();
        diag.degeneracy_min = degeneracy_min;

        if self.config.loop_min_degeneracy > 0.0
            && degeneracy_min >= 0.0
            && (degeneracy_min as f64) < self.config.loop_min_degeneracy
        {
            diag.outcome = "degeneracy_reject";
            return LoopSearchResult {
                diag,
                accepted: None,
                scored_yank,
                timing,
            };
        }

        if !icp.converged || icp.fitness > self.config.loop_score_thresh {
            diag.outcome = "icp_reject";
            return LoopSearchResult {
                diag,
                accepted: None,
                scored_yank,
                timing,
            };
        }

        let mut score = icp.fitness;
        let rotation_refined =
            mat3::mat_mul(&icp.rotation, &self.key_poses[cur_idx].rotation_global);
        let translation_refined = mat3::add(
            &mat3::mat_vec(&icp.rotation, &self.key_poses[cur_idx].translation_global),
            &icp.translation,
        );
        let loop_rotation_transpose = mat3::transpose(&self.key_poses[loop_idx].rotation_global);
        let mut rotation_offset = mat3::mat_mul(&loop_rotation_transpose, &rotation_refined);
        let mut translation_offset = mat3::mat_vec(
            &loop_rotation_transpose,
            &mat3::sub(
                &translation_refined,
                &self.key_poses[loop_idx].translation_global,
            ),
        );

        // Graph yank: how far this constraint pulls the source keyframe from the
        // current odom-chain estimate (relative pose loop->cur, in the loop's
        // frame). A large yank on a plausible-distance candidate = a closure that
        // disagrees with odometry = the signature of a false closure.
        let current_relative_rotation = mat3::mat_mul(
            &loop_rotation_transpose,
            &self.key_poses[cur_idx].rotation_global,
        );
        let current_relative_translation = mat3::mat_vec(
            &loop_rotation_transpose,
            &mat3::sub(
                &self.key_poses[cur_idx].translation_global,
                &self.key_poses[loop_idx].translation_global,
            ),
        );
        diag.yank_translation = mat3::norm(&mat3::sub(
            &translation_offset,
            &current_relative_translation,
        ));
        diag.yank_rotation_deg =
            mat3::angular_distance(&rotation_offset, &current_relative_rotation).to_degrees();

        scored_yank = Some(self.score_candidate(&mut diag));
        if self.config.loop_record_strain || self.config.loop_yank_from_strain {
            let poses = &self.key_poses[..=cur_idx];
            let loops: Vec<Edge> = self
                .kept_loops
                .iter()
                .filter(|l| l.from.max(l.to) <= cur_idx)
                .copied()
                .collect();
            diag.strain = pose_graph::strain(
                &poses.iter().map(|k| k.rotation_global).collect::<Vec<_>>(),
                &poses
                    .iter()
                    .map(|k| k.translation_global)
                    .collect::<Vec<_>>(),
                &odometry_edges(poses, &self.config),
                &loops,
                &vec![1.0; loops.len()],
                &Edge {
                    from: loop_idx,
                    to: cur_idx,
                    rotation: rotation_offset,
                    translation: translation_offset,
                    // a near-rigid edge: how hard the graph resists being forced to the measurement
                    variances: [1e-6; 6],
                },
            );
        }
        if self.config.loop_yank_gate && diag.confidence < diag.required_confidence {
            let t0 = std::time::Instant::now();
            let extended = self.extended_icp(
                cur_idx,
                loop_idx,
                &source_cloud,
                &target_tree,
                &icp,
                &mut diag,
            );
            timing.extended_icp_s += t0.elapsed().as_secs_f64();
            let Some(extended) = extended else {
                diag.outcome = "yank_confidence_reject";
                return LoopSearchResult {
                    diag,
                    accepted: None,
                    scored_yank,
                    timing,
                };
            };
            // the loop measurement comes from the extended alignment, which saw more of both places
            let rotation_refined =
                mat3::mat_mul(&extended.rotation, &self.key_poses[cur_idx].rotation_global);
            let translation_refined = mat3::add(
                &mat3::mat_vec(
                    &extended.rotation,
                    &self.key_poses[cur_idx].translation_global,
                ),
                &extended.translation,
            );
            rotation_offset = mat3::mat_mul(&loop_rotation_transpose, &rotation_refined);
            translation_offset = mat3::mat_vec(
                &loop_rotation_transpose,
                &mat3::sub(
                    &translation_refined,
                    &self.key_poses[loop_idx].translation_global,
                ),
            );
            score = extended.fitness;
        }

        diag.outcome = "accepted";
        LoopSearchResult {
            diag,
            accepted: Some((rotation_offset, translation_offset, score)),
            scored_yank,
            timing,
        }
    }

    /// Fill in the yank / confidence gate's inputs and outputs for a candidate ICP has scored, and
    /// return its yank for the scale.
    fn score_candidate(&self, diag: &mut LoopCandidateDiag) -> (f64, f64) {
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
    fn stacked_cloud(&self, ids: impl Iterator<Item = usize>) -> PointCloud {
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
    fn extended_icp(
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
            ..IcpParams::default()
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
            let icp = icp_point_to_point_with_tree(
                &source,
                &target,
                &tree,
                &seed_rotation,
                &seed_translation,
                &params,
            );
            if !self.config.loop_icp_dump_dir.is_empty() {
                dump_icp_inputs(
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

impl GscPgo {
    pub fn new(config: Config) -> GscPgo {
        let scan_context_config = scan_context::Config {
            n_rings: config.scan_context_num_rings.max(0) as usize,
            n_sectors: config.scan_context_num_sectors.max(0) as usize,
            max_range_m: config.scan_context_max_range_m,
            candidate_top_k: config.scan_context_top_k.max(0) as usize,
            match_threshold: config.scan_context_match_threshold,
            lidar_height_m: config.scan_context_lidar_height_m,
        };
        GscPgo {
            config,
            scan_context_config,
            key_poses: Vec::new(),
            history_pairs: Vec::new(),
            committed_loops: Vec::new(),
            loop_yank_z: Vec::new(),
            loop_fit_confidence: Vec::new(),
            new_loop: false,
            loops_dispatched: 0,
            solving: None,
            graph_error: 0.0,
            scan_context_descriptors: Vec::new(),
            scan_context_ring_keys: Vec::new(),
            rotation_offset: mat3::identity(),
            translation_offset: [0.0; 3],
            timing: TimingStats::default(),
            loop_diagnostics: Vec::new(),
            loop_edge_events: Vec::new(),
            scored_yanks: Vec::new(),
            last_loop_search_time: None,
        }
    }

    /// Half the sum of squared whitened residuals of the newest applied solve (how much its
    /// constraints conflict).
    pub fn graph_error(&self) -> f64 {
        self.graph_error
    }

    pub fn committed_loops(&self) -> &[CommittedLoop] {
        &self.committed_loops
    }

    /// Whether a loop was committed since the last `update`.
    pub fn has_loop(&self) -> bool {
        self.new_loop
    }

    /// Whether a solve is running or loops are waiting for one.
    pub fn solve_pending(&self) -> bool {
        self.solving.is_some() || self.committed_loops.len() > self.loops_dispatched
    }

    /// Apply a finished solve, and start one when loops have arrived since the last. A solve is
    /// adopted once the map reaches its `due_at` keyframes (waiting for it if needed), or, with
    /// `now`, as soon as it has finished. Returns whether keyframes moved.
    pub fn update(&mut self, now: bool) -> bool {
        self.new_loop = false;
        let keyframes = self.key_poses.len();
        let ready = self.solving.as_ref().is_some_and(|solving| {
            keyframes >= solving.due_at || (now && solving.handle.is_finished())
        });
        let moved = ready && self.apply_solve();
        if self.solving.is_none() && self.committed_loops.len() > self.loops_dispatched {
            self.dispatch_solve();
        }
        moved
    }

    /// Wait for the running solve, and solve again until every loop is in. Returns whether keyframes moved.
    pub fn finish(&mut self) -> bool {
        let mut moved = false;
        loop {
            moved |= self.apply_solve();
            if self.committed_loops.len() == self.loops_dispatched {
                return moved;
            }
            self.dispatch_solve();
        }
    }

    /// Start a solve of every keyframe so far and every committed loop, from the odometry (GNC
    /// must start from the dead-reckoned chain, not an estimate earlier loops already bent).
    fn dispatch_solve(&mut self) {
        let n = self.key_poses.len();
        let rotations: Vec<Mat3> = self.key_poses.iter().map(|k| k.rotation_local).collect();
        let positions: Vec<Vec3> = self.key_poses.iter().map(|k| k.translation_local).collect();
        let config = &self.config;
        let odometry = odometry_edges(&self.key_poses, config);
        let loops = loop_edges(self.committed_loops.iter());
        let solve_config = SolveConfig {
            gnc_variance_scale: config.loop_gnc_var_scale,
            gnc_inlier_probability: config.loop_gnc_inlier_probability,
            huber_k: config.loop_robust_huber_k,
        };
        let handle = std::thread::Builder::new()
            .name("gsc-pose-graph".into())
            .spawn(move || {
                let started = std::time::Instant::now();
                let solution =
                    pose_graph::solve(&rotations, &positions, &odometry, &loops, &solve_config);
                (solution, started.elapsed().as_secs_f64())
            })
            .expect("spawn pose graph thread");
        let delay = (n as f64 * self.config.loop_gnc_apply_after_fraction).ceil() as usize;
        self.loops_dispatched = self.committed_loops.len();
        self.solving = Some(Solving {
            handle,
            keyframes: n,
            due_at: n + delay.max(1),
        });
    }

    /// Wait for the running solve and adopt it: its poses, the correction the keyframes since then
    /// ride on, and its loop verdicts. Returns whether there was one.
    fn apply_solve(&mut self) -> bool {
        let Some(solving) = self.solving.take() else {
            return false;
        };
        let (solution, seconds) = solving.handle.join().expect("pose graph solve panicked");
        self.timing.solve_s += seconds;
        let n = solving.keyframes;
        for (key_pose, (rotation, position)) in self.key_poses[..n]
            .iter_mut()
            .zip(solution.rotations.iter().zip(&solution.positions))
        {
            key_pose.rotation_global = *rotation;
            key_pose.translation_global = *position;
        }
        // keyframes made since the solve started ride on its newest keyframe's correction
        let newest = &self.key_poses[n - 1];
        self.rotation_offset = mat3::mat_mul(
            &newest.rotation_global,
            &mat3::transpose(&newest.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &newest.translation_global,
            &mat3::mat_vec(&self.rotation_offset, &newest.translation_local),
        );
        for key_pose in &mut self.key_poses[n..] {
            key_pose.rotation_global =
                mat3::mat_mul(&self.rotation_offset, &key_pose.rotation_local);
            key_pose.translation_global = mat3::add(
                &mat3::mat_vec(&self.rotation_offset, &key_pose.translation_local),
                &self.translation_offset,
            );
        }
        for (index, &weight) in solution.loop_weights.iter().enumerate() {
            let previous = self.committed_loops[index].gnc_weight;
            self.committed_loops[index].gnc_weight = weight;
            if previous == weight {
                continue;
            }
            let (source, target) = (
                self.committed_loops[index].source_id,
                self.committed_loops[index].target_id,
            );
            let (confidence, yank_z) = (self.loop_fit_confidence[index], self.loop_yank_z[index]);
            if weight > 0.5 {
                self.push_edge(source, target, LoopEdgeState::Accepted, confidence, yank_z);
                self.push_edge(source, target, LoopEdgeState::Applied, confidence, yank_z);
            } else {
                self.push_edge(source, target, LoopEdgeState::Rejected, 0.0, yank_z);
            }
        }
        self.graph_error = solution.error;
        true
    }

    /// All per-search loop-closure diagnostic records accumulated so far.
    /// Cumulative wall seconds per stage: scan context, submaps, ICP, degeneracy, extended ICP, iSAM2 (smooth_and_update).
    pub fn stage_seconds(&self) -> [(&'static str, f64); 6] {
        let t = &self.timing;
        [
            ("scan_context", t.scan_context_s),
            ("submap", t.submap_s),
            ("icp", t.icp_s),
            ("degeneracy", t.degeneracy_s),
            ("extended_icp", t.extended_icp_s),
            ("solve", t.solve_s),
        ]
    }
    pub fn loop_diagnostics(&self) -> &[LoopCandidateDiag] {
        &self.loop_diagnostics
    }
    /// Every loop-edge lifecycle event so far, oldest first.
    pub fn loop_edge_events(&self) -> &[LoopEdgeEvent] {
        &self.loop_edge_events
    }
    fn newest_keyframe_time(&self) -> f64 {
        self.key_poses.last().map_or(f64::NAN, |k| k.time)
    }
    fn push_edge(
        &mut self,
        source_id: usize,
        target_id: usize,
        state: LoopEdgeState,
        confidence: f64,
        yank_z: f64,
    ) {
        self.loop_edge_events.push(LoopEdgeEvent {
            time: self.newest_keyframe_time(),
            source_id,
            target_id,
            state,
            confidence: confidence.clamp(0.0, 1.0),
            yank_z,
        });
    }
    /// Keep a search's diagnostics, and when it found a candidate, its proposal and gate outcome
    /// as loop-edge events.
    fn record_search(&mut self, diag: LoopCandidateDiag) {
        if diag.target_id >= 0 {
            let (source, target) = (diag.source_id, diag.target_id as usize);
            let proposed = if diag.scan_context_best.is_finite() {
                1.0 - diag.scan_context_best as f64
            } else {
                1.0 - diag.candidate_distance / self.config.loop_search_radius.max(f64::EPSILON)
            };
            self.push_edge(source, target, LoopEdgeState::Proposed, proposed, f64::NAN);
            let checked = if diag.confidence.is_finite() {
                diag.confidence
            } else {
                proposed
            };
            let outcome = if diag.outcome == "accepted" {
                LoopEdgeState::Committed
            } else {
                LoopEdgeState::GateRejected
            };
            if outcome == LoopEdgeState::Committed {
                self.loop_fit_confidence.push(checked.clamp(0.0, 1.0));
                self.loop_yank_z.push(diag.yank_z);
            }
            self.push_edge(source, target, outcome, checked, diag.yank_z);
        }
        self.loop_diagnostics.push(diag);
    }
    pub fn config(&self) -> &Config {
        &self.config
    }
    /// Keyframe gating by translation / rotation delta against the last
    /// keyframe's LOCAL pose.
    pub fn is_key_pose(&self, pose: &PoseWithTime) -> bool {
        if self.key_poses.is_empty() {
            return true;
        }
        let last_item = self.key_poses.last().unwrap();
        let delta_trans = mat3::norm(&mat3::sub(&pose.translation, &last_item.translation_local));
        let delta_deg =
            mat3::angular_distance(&pose.rotation, &last_item.rotation_local).to_degrees();
        delta_trans > self.config.keyframe_min_distance_meters
            || delta_deg > self.config.keyframe_min_rotation_degrees
    }
    /// Insert a keyframe if the pose clears the keyframe gate; returns
    /// whether it was added.
    pub fn add_key_pose(&mut self, cloud_with_pose: &CloudWithPose) -> bool {
        if !self.is_key_pose(&cloud_with_pose.pose) {
            return false;
        }
        let cloud = match &cloud_with_pose.cloud {
            Some(cloud) if self.config.keyframe_voxel_size > 0.0 => {
                Some(Arc::new(voxel_downsample(cloud, self.config.keyframe_voxel_size)))
            }
            cloud => cloud.clone(),
        };
        self.insert_keyframe(&cloud_with_pose.pose, cloud);
        true
    }
    pub fn key_poses(&self) -> &[KeyPoseWithCloud] {
        &self.key_poses
    }
    pub fn rotation_offset(&self) -> Mat3 {
        self.rotation_offset
    }
    pub fn translation_offset(&self) -> Vec3 {
        self.translation_offset
    }
    /// Aggregate the body clouds of keyframes [idx-half_range,
    /// idx+half_range] transformed by their GLOBAL poses,
    /// then voxel-downsample at `resolution` (when > 0).
    pub fn get_sub_map(&self, idx: i32, half_range: i32, resolution: f64) -> PointCloud {
        sub_map(&self.key_poses, idx, half_range, resolution)
    }
    /// Radius search on past key-pose global positions (candidates ascending
    /// by distance), returning the first far-enough-in-time hit.
    fn search_by_position(&self) -> i64 {
        let cur_idx = self.key_poses.len() - 1;
        let last_item = self.key_poses.last().unwrap();
        let last = [
            last_item.translation_global[0] as f32,
            last_item.translation_global[1] as f32,
            last_item.translation_global[2] as f32,
        ];
        let radius_sq = (self.config.loop_search_radius * self.config.loop_search_radius) as f32;
        let mut neighbors: Vec<(f32, usize)> = Vec::new();
        for (i, key_pose) in self.key_poses[..cur_idx].iter().enumerate() {
            let dx = key_pose.translation_global[0] as f32 - last[0];
            let dy = key_pose.translation_global[1] as f32 - last[1];
            let dz = key_pose.translation_global[2] as f32 - last[2];
            let sq = dx * dx + dy * dy + dz * dz;
            if sq <= radius_sq {
                neighbors.push((sq, i));
            }
        }
        if neighbors.is_empty() {
            return -1;
        }
        neighbors.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        for &(_, idx) in &neighbors {
            if (last_item.time - self.key_poses[idx].time).abs() > self.config.loop_time_thresh {
                return idx as i64;
            }
        }
        -1
    }
    /// Scan-context candidate search. Returns (loop_idx or -1,
    /// sector_shift, best, second): the accepted candidate under the match
    /// threshold, plus the closest / 2nd-closest cosine distances across the
    /// top-K (the Lowe ratio distinctiveness signal).
    fn search_by_scan_context(&self) -> (i64, i32, f32, f32) {
        let mut out_best = 2.0f32;
        let mut out_second = 2.0f32;
        if self.scan_context_descriptors.is_empty()
            || self.scan_context_descriptors.last().unwrap().is_empty()
        {
            return (-1, 0, out_best, out_second);
        }
        let query = self.scan_context_descriptors.last().unwrap();
        let query_key = self.scan_context_ring_keys.last().unwrap();
        let current_time = self.key_poses.last().unwrap().time;

        // Two-stage retrieval: rank by ring-key L2 distance, then score the
        // top-K via column-shifted cosine distance on the full descriptor.
        // TODO: switch to a kd-tree later for O(log n) instead of O(n) — this
        // ring-key ranking is a linear scan (O(N^2) over a run) and won't scale
        // to massive maps.
        let cur_idx = self.key_poses.len() - 1;
        let mut ranked: Vec<(f32, usize)> = Vec::with_capacity(cur_idx);
        for i in 0..cur_idx {
            if self.scan_context_descriptors[i].is_empty() {
                continue;
            }
            if (current_time - self.key_poses[i].time).abs() <= self.config.loop_time_thresh {
                continue; // too recent — not a true loop candidate
            }
            let key = &self.scan_context_ring_keys[i];
            let mut sq = 0.0f32;
            for (a, b) in key.iter().zip(query_key.iter()) {
                let d = a - b;
                sq += d * d;
            }
            ranked.push((sq.sqrt(), i));
        }
        if ranked.is_empty() {
            return (-1, 0, out_best, out_second);
        }

        let top_k_count = ranked.len().min(self.scan_context_config.candidate_top_k);
        ranked.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let mut best_dist = f32::MAX;
        let mut second_dist = f32::MAX;
        let mut best_dist_filtered = self.scan_context_config.match_threshold as f32;
        let mut best_idx: i64 = -1;
        let mut best_shift = 0i32;
        for &(_, idx) in &ranked[..top_k_count] {
            let (distance, shift) =
                scan_context::best_distance(query, &self.scan_context_descriptors[idx]);
            if distance < best_dist {
                second_dist = best_dist; // demote previous best
                best_dist = distance;
            } else if distance < second_dist {
                second_dist = distance;
            }
            if distance < best_dist_filtered {
                best_dist_filtered = distance;
                best_idx = idx as i64;
                best_shift = shift;
            }
        }

        if best_dist < 2.0 {
            out_best = best_dist;
        }
        if second_dist < 2.0 {
            out_second = second_dist;
        }
        (best_idx, best_shift, out_best, out_second)
    }
    /// Candidate search (scan context with position fallback) + all the
    /// gates, ICP verification, and LoopPair construction.
    /// Record that this keyframe's loop search was skipped because newer scans
    /// are already queued behind it: a search (an ICP on a large cloud can take
    /// seconds) must never be what makes the module fall behind.
    pub fn skip_loop_search_behind(&mut self) {
        self.skip_loop_search("throttle_backlog");
    }
    /// Record that the newest keyframe's loop search was skipped, and why.
    pub fn skip_loop_search(&mut self, reason: &'static str) {
        if let Some(key_pose) = self.key_poses.last() {
            let mut diag = LoopCandidateDiag::new(
                self.key_poses.len() - 1,
                key_pose.time,
                key_pose.translation_global,
            );
            diag.outcome = reason;
            self.record_search(diag);
        }
    }
    /// Search for a loop at the newest keyframe: the cheap gates and the candidate pick happen here, and the
    /// ICP half comes back as a `LoopSearch` to run (on any thread) and hand to `finish_loop_search`.
    pub fn begin_loop_search(&mut self) -> Option<LoopSearch> {
        if self.key_poses.len() < 10 {
            return None;
        }

        let cur_idx = self.key_poses.len() - 1;
        let mut diag = LoopCandidateDiag::new(
            cur_idx,
            self.key_poses[cur_idx].time,
            self.key_poses[cur_idx].translation_global,
        );

        if self.config.min_loop_detect_duration > 0.0 {
            if let Some(&(_, last_source)) = self.history_pairs.last() {
                let current_time = self.key_poses.last().unwrap().time;
                let last_time = self.key_poses[last_source].time;
                if current_time - last_time < self.config.min_loop_detect_duration {
                    diag.outcome = "throttle_min_duration";
                    self.record_search(diag);
                    return None;
                }
            }
        }

        if self.config.loop_search_min_interval > 0.0 {
            let current_time = self.key_poses[cur_idx].time;
            if let Some(last_search) = self.last_loop_search_time {
                if current_time - last_search < self.config.loop_search_min_interval {
                    diag.outcome = "throttle_search_interval";
                    self.record_search(diag);
                    return None;
                }
            }
            self.last_loop_search_time = Some(current_time);
        }

        // Feature-poverty gate: a scan with no spatially-spread structure
        // can't reliably place itself; skip loop search entirely.
        if self.config.use_scan_context
            && !self.scan_context_descriptors.is_empty()
            && !self.scan_context_descriptors.last().unwrap().is_empty()
        {
            let descriptor = self.scan_context_descriptors.last().unwrap();
            diag.descriptor_structure = scan_context::descriptor_structure(descriptor);
            diag.descriptor_occupancy = scan_context::descriptor_occupancy(descriptor) as i32;
            if self.config.loop_min_occupancy > 0
                && diag.descriptor_occupancy < self.config.loop_min_occupancy
            {
                diag.outcome = "low_occupancy";
                self.record_search(diag);
                return None;
            }
        }

        let mut loop_idx: i64 = -1;
        let mut sector_shift = 0i32;
        let mut sc_best = 2.0f32;
        let mut sc_second = 2.0f32;
        if self.config.use_scan_context {
            let t0 = std::time::Instant::now();
            (loop_idx, sector_shift, sc_best, sc_second) = self.search_by_scan_context();
            self.timing.scan_context_s += t0.elapsed().as_secs_f64();
        }
        // sc_best/sc_second are only meaningful for a scan-context candidate;
        // the position fallback leaves them at their sentinel init.
        let from_scan_context = loop_idx >= 0;
        if loop_idx < 0 {
            // Fallback (or sole path if SC disabled): search past positions.
            loop_idx = self.search_by_position();
        }
        if loop_idx < 0 {
            diag.outcome = "no_candidate";
            self.record_search(diag);
            return None;
        }
        let loop_idx = loop_idx as usize;

        diag.from_scan_context = from_scan_context;
        diag.target_id = loop_idx as i64;
        diag.target_time = self.key_poses[loop_idx].time;
        diag.target_xyz = self.key_poses[loop_idx].translation_global;
        diag.candidate_distance = mat3::norm(&mat3::sub(
            &self.key_poses[cur_idx].translation_global,
            &self.key_poses[loop_idx].translation_global,
        ));

        if from_scan_context && sc_second > 1e-6 {
            diag.scan_context_best = sc_best;
            diag.scan_context_second = sc_second;
            diag.lowe_ratio = sc_best / sc_second;
        }

        // Seed ICP from the place-recognition match: the source keyframe is put
        // at the *target's* global position (rotated by the scan-context yaw),
        // i.e. init = T(target_position) * Rz(yaw) * T(-source_position). When
        // odometry has drifted far, source_position and target_position differ by
        // the whole drift; anchoring at source_position instead would leave the
        // two submaps non-overlapping and ICP could never converge. For small
        // drift target_position ~= source_position, so this reduces to the old
        // rotate-in-place seed. Built in f32, then widened to f64 for the
        // icp_point_to_point API — which narrows it back losslessly.
        let mut init_rotation = mat3::identity();
        let mut init_translation = [0.0f64; 3];
        {
            let yaw_f = if self.config.use_scan_context && sector_shift != 0 {
                scan_context::yaw_from_shift(sector_shift, self.scan_context_config.n_sectors)
                    as f32
            } else {
                0.0f32
            };
            // Rz(yaw) from AngleAxisf(yaw_f, UnitZ()): note the (2,2) entry is
            // computed as (1 - cos) + cos, which is not always exactly 1.0f.
            let (s, c) = (yaw_f.sin(), yaw_f.cos());
            let rotation_zz = (1.0f32 - c) + c;
            let rotation = [
                [c, -s, 0.0f32],
                [s, c, 0.0f32],
                [0.0f32, 0.0f32, rotation_zz],
            ];
            let source_position = [
                self.key_poses[cur_idx].translation_global[0] as f32,
                self.key_poses[cur_idx].translation_global[1] as f32,
                self.key_poses[cur_idx].translation_global[2] as f32,
            ];
            let target_position = [
                self.key_poses[loop_idx].translation_global[0] as f32,
                self.key_poses[loop_idx].translation_global[1] as f32,
                self.key_poses[loop_idx].translation_global[2] as f32,
            ];
            // rotation * source_position: fixed-size Eigen Matrix3f *
            // Vector3f, unrolled binary-tree redux t0 + (t1 + t2).
            for i in 0..3 {
                let rotated_source = source_position[0] * rotation[i][0]
                    + (source_position[1] * rotation[i][1] + source_position[2] * rotation[i][2]);
                init_translation[i] = f64::from(target_position[i] - rotated_source);
                init_rotation[i] = [
                    f64::from(rotation[i][0]),
                    f64::from(rotation[i][1]),
                    f64::from(rotation[i][2]),
                ];
            }
        }

        Some(LoopSearch {
            key_poses: self.key_poses.clone(),
            config: self.config.clone(),
            scored_yanks: self.scored_yanks.clone(),
            kept_loops: if self.config.loop_record_strain || self.config.loop_yank_from_strain {
                loop_edges(self.committed_loops.iter().filter(|l| l.gnc_weight > 0.5))
            } else {
                Vec::new()
            },
            diag,
            loop_idx,
            init_rotation,
            init_translation,
        })
    }
    /// Search for a loop at the newest keyframe, all on this thread.
    pub fn search_for_loop_pairs(&mut self) {
        if let Some(search) = self.begin_loop_search() {
            let result = search.run();
            self.finish_loop_search(result);
        }
    }
    /// Apply a `LoopSearch`'s result: record its diagnostics and commit the loop it accepted.
    pub fn finish_loop_search(&mut self, result: LoopSearchResult) {
        let LoopSearchResult {
            diag,
            accepted,
            scored_yank,
            timing,
        } = result;
        self.timing.submap_s += timing.submap_s;
        self.timing.icp_s += timing.icp_s;
        self.timing.degeneracy_s += timing.degeneracy_s;
        self.timing.extended_icp_s += timing.extended_icp_s;
        if let Some(yank) = scored_yank {
            self.scored_yanks.push(yank);
        }
        let Some((rotation_offset, translation_offset, score)) = accepted else {
            self.record_search(diag);
            return;
        };
        let (source, target) = (diag.source_id, diag.target_id as usize);
        self.record_search(diag);
        self.commit_loop(source, target, rotation_offset, translation_offset, score);
    }
    /// Direct access to the ICP used for loop verification — handy for
    /// standalone experiments; the loop path calls `icp_point_to_point`
    /// with these same defaults.
    pub fn icp_params() -> IcpParams {
        IcpParams::default()
    }

    /// Add a keyframe at `pose`, placed by the current correction.
    fn insert_keyframe(&mut self, pose: &PoseWithTime, cloud: Option<Arc<PointCloud>>) {
        self.key_poses.push(KeyPoseWithCloud {
            time: pose.second,
            rotation_local: pose.rotation,
            translation_local: pose.translation,
            body_cloud: cloud.clone(),
            rotation_global: mat3::mat_mul(&self.rotation_offset, &pose.rotation),
            translation_global: mat3::add(
                &mat3::mat_vec(&self.rotation_offset, &pose.translation),
                &self.translation_offset,
            ),
        });

        // Cache the Scan Context descriptor + ring-key (empty when the node
        // has no cloud, e.g. a constraint-triggered node). A non-positive
        // max_range means "auto": resolve it once from the first real cloud's
        // extent and reuse it, so all descriptors share one binning.
        if let Some(cloud) = cloud {
            if self.scan_context_config.max_range_m <= 0.0 {
                let auto_range = scan_context::auto_max_range(&cloud);
                if auto_range > 0.0 {
                    self.scan_context_config.max_range_m = auto_range;
                    eprintln!(
                        "scan-context: auto ring range = {auto_range:.2} m (from cloud extent)"
                    );
                }
            }
            if self.scan_context_config.max_range_m > 0.0 {
                let descriptor = scan_context::make_descriptor(&cloud, &self.scan_context_config);
                self.scan_context_ring_keys
                    .push(scan_context::make_ring_key(&descriptor));
                self.scan_context_descriptors.push(descriptor);
            } else {
                self.scan_context_descriptors
                    .push(scan_context::Descriptor::empty());
                self.scan_context_ring_keys
                    .push(scan_context::RingKey::new());
            }
        } else {
            self.scan_context_descriptors
                .push(scan_context::Descriptor::empty());
            self.scan_context_ring_keys
                .push(scan_context::RingKey::new());
        }
    }

    /// Commit a loop that passed every gate; the next `update` starts a solve with it.
    fn commit_loop(
        &mut self,
        source_id: usize,
        target_id: usize,
        rotation_offset: Mat3,
        translation_offset: Vec3,
        score: f64,
    ) {
        self.history_pairs.push((target_id, source_id));
        self.committed_loops.push(CommittedLoop {
            source_id,
            target_id,
            rotation_offset,
            translation_offset,
            score,
            gnc_weight: f64::NAN,
        });
        self.new_loop = true;
    }
}

/// The odometry between consecutive keyframes, from their dead-reckoned poses, at the configured variances.
fn odometry_edges(key_poses: &[KeyPoseWithCloud], config: &Config) -> Vec<Edge> {
    let (tilt, yaw) = (config.odom_rot_roll_pitch_var, config.odom_rot_yaw_var);
    let (xy, z) = (config.odom_trans_xy_var, config.odom_trans_z_var);
    key_poses
        .windows(2)
        .enumerate()
        .map(|(i, pair)| {
            let back = mat3::transpose(&pair[0].rotation_local);
            Edge {
                from: i,
                to: i + 1,
                rotation: mat3::mat_mul(&back, &pair[1].rotation_local),
                translation: mat3::mat_vec(
                    &back,
                    &mat3::sub(&pair[1].translation_local, &pair[0].translation_local),
                ),
                variances: [tilt, tilt, yaw, xy, xy, z],
            }
        })
        .collect()
}

/// Committed loops as pose-graph edges; a loop's variance is its ICP fitness.
fn loop_edges<'a>(loops: impl Iterator<Item = &'a CommittedLoop>) -> Vec<Edge> {
    loops
        .map(|l| Edge {
            from: l.target_id,
            to: l.source_id,
            rotation: l.rotation_offset,
            translation: l.translation_offset,
            variances: [l.score.max(1e-6); 6],
        })
        .collect()
}

/// Free an iSAM2 on a short-lived thread: tearing down a large Bayes tree would
/// stall the pipeline thread.
/// Write one candidate's ICP inputs and result (see `Config::loop_icp_dump_dir`).
/// Aggregate the body clouds of keyframes [idx-half_range, idx+half_range] transformed by their
/// GLOBAL poses, then voxel-downsample at `resolution` (when > 0).
pub fn sub_map(
    key_poses: &[KeyPoseWithCloud],
    idx: i32,
    half_range: i32,
    resolution: f64,
) -> PointCloud {
    assert!(idx >= 0 && (idx as usize) < key_poses.len());
    let min_idx = 0.max(idx - half_range) as usize;
    let max_idx = (key_poses.len() as i32 - 1).min(idx + half_range) as usize;

    let mut ret: PointCloud = Vec::new();
    for key_pose in &key_poses[min_idx..=max_idx] {
        if let Some(body_cloud) = &key_pose.body_cloud {
            let global_cloud = transform_cloud(
                body_cloud,
                &key_pose.rotation_global,
                &key_pose.translation_global,
            );
            ret.extend_from_slice(&global_cloud);
        }
    }
    if resolution > 0.0 {
        ret = voxel_downsample(&ret, resolution);
    }
    ret
}

fn dump_icp_inputs(
    dir: &str,
    source_id: usize,
    target_id: usize,
    source: &PointCloud,
    target: &PointCloud,
    icp: &IcpResult,
    suffix: &str,
) {
    let mut bytes = Vec::with_capacity(8 + 12 * (source.len() + target.len()) + 105);
    bytes.extend_from_slice(&(source.len() as u32).to_le_bytes());
    bytes.extend_from_slice(&(target.len() as u32).to_le_bytes());
    for p in source.iter().chain(target) {
        for v in p {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    for row in &icp.rotation {
        for v in row {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    for v in &icp.translation {
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    bytes.extend_from_slice(&icp.fitness.to_le_bytes());
    bytes.push(icp.converged as u8);
    let path = std::path::Path::new(dir).join(format!("{source_id}_{target_id}{suffix}.icp"));
    if let Err(e) = std::fs::create_dir_all(dir).and_then(|_| std::fs::write(&path, bytes)) {
        eprintln!("loop_icp_dump_dir: could not write {}: {e}", path.display());
    }
}

pub use pointcloud::IcpResult;
