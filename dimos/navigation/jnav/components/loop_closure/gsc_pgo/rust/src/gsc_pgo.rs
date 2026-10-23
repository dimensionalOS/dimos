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

use crate::big_picture_fit::{self, loop_edges, odometry_edges, Edge, Solving};
use crate::connection_fit::{dump_icp, icp, IcpParams};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, voxel_downsample, KdTree, PointCloud};
use crate::similarity_discovery;
use crate::uniqueness;

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
    /// Outcome: "accepted", "outranked" (a better proposal took the search thread), or the rejecting gate — one of "no_structure",
    /// "low_occupancy", "no_candidate", "lowe_reject", "distance_reject",
    /// "degeneracy_reject", "icp_reject", "uninformative", "yank_reject".
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
    /// Rise in the graph's error if this loop were forced in rigidly (`big_picture_fit::strain`), with
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
    pub loop_score_thresh: f64,
    pub loop_submap_half_range: i32,
    pub submap_resolution: f64,
    /// Voxel size each incoming cloud is downsampled to before a keyframe
    /// stores it. 0 keeps clouds as they arrive.
    pub keyframe_voxel_size: f64,
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
    /// Hold the keyframes no new loop can move by more than one odometry step's noise (see
    /// `big_picture_fit::position_variance`), so a solve covers only the loose part of the map.
    pub loop_cooling: bool,
    /// Replay only (`search_for_loop_pairs`): keyframes a loop search occupies the search thread for; 0 = instant.
    pub loop_search_keyframes: usize,
    // Odometry between-factor noise (anisotropic).
    pub odom_rot_roll_pitch_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,
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
            loop_score_thresh: 0.15,
            loop_submap_half_range: 5,
            submap_resolution: 0.1,
            keyframe_voxel_size: 0.0,
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
            loop_cooling: false,
            loop_search_keyframes: 0,
            odom_rot_roll_pitch_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            scan_context_num_rings: 20,
            scan_context_num_sectors: 60,
            scan_context_max_range_m: 0.0,
            scan_context_top_k: 10,
            scan_context_match_threshold: 0.4,
            scan_context_lidar_height_m: 2.0,
        }
    }
}

pub struct GscPgo {
    pub(crate) config: Config,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) committed_loops: Vec<CommittedLoop>,
    /// Per committed loop: its yank_z and gate confidence, for its loop-edge events.
    pub(crate) loop_yank_z: Vec<f64>,
    pub(crate) loop_fit_confidence: Vec<f64>,
    /// A loop was committed since the last `update`.
    pub(crate) new_loop: bool,
    /// Committed loops a solve has been started for.
    pub(crate) loops_dispatched: usize,
    pub(crate) solving: Option<Solving>,
    pub(crate) graph_error: f64,
    pub(crate) scan_context_descriptors: Vec<similarity_discovery::Descriptor>,
    pub(crate) scan_context_ring_keys: Vec<similarity_discovery::RingKey>,
    pub(crate) rotation_offset: Mat3,
    pub(crate) translation_offset: Vec3,
    pub(crate) timing: TimingStats,
    pub(crate) loop_diagnostics: Vec<LoopCandidateDiag>,
    pub(crate) loop_edge_events: Vec<LoopEdgeEvent>,
    /// Rotation (deg) and translation (m) yank of every candidate ICP has scored (per sqrt keyframe
    /// gap with `loop_yank_per_sqrt_gap`), the scale new yanks are measured against. Every scored
    /// candidate, not just committed ones: a scale taken only from loops the gate let through would
    /// shrink with each rejection and reject the rest.
    pub(crate) scored_yanks: Vec<(f64, f64)>,
    pub(crate) waiting_proposal: Option<LoopProposal>,
    /// Keyframes the newest dispatched solve covered.
    pub(crate) solving_keyframes: usize,
    /// A finished search waiting for its keyframe (`loop_search_keyframes`).
    pub(crate) pending_search: Option<(LoopSearchResult, usize)>,
}

/// Cumulative per-stage wall time, printed periodically when `debug` is on —
/// purely observational (never read back into the pipeline).
#[derive(Default)]
pub(crate) struct TimingStats {
    pub(crate) scan_context_s: f64,
    pub(crate) submap_s: f64,
    pub(crate) icp_s: f64,
    pub(crate) degeneracy_s: f64,
    pub(crate) extended_icp_s: f64,
    pub(crate) solve_s: f64,
}

/// The ICP half of one loop search, on its own copy of the keyframes so it can run on another thread
/// while scans keep arriving (`GscPgo::begin_loop_search` makes it, `finish_loop_search` applies it).
/// A keyframe through stages 1 and 2, waiting for the search thread: its candidate target, the
/// scan-context column shift between them, and the match's cosine distance (lower is better).
pub struct LoopProposal {
    pub(crate) diag: LoopCandidateDiag,
    pub(crate) loop_idx: usize,
    pub(crate) sector_shift: i32,
    pub(crate) similarity: f32,
}

pub struct LoopSearch {
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) config: Config,
    pub(crate) scored_yanks: Vec<(f64, f64)>,
    /// The loops the newest solve kept, for `loop_record_strain`.
    pub(crate) kept_loops: Vec<Edge>,
    pub(crate) diag: LoopCandidateDiag,
    pub(crate) loop_idx: usize,
    pub(crate) init_rotation: Mat3,
    pub(crate) init_translation: Vec3,
}

/// What a `LoopSearch` found: its diagnostics, the loop it accepts (relative rotation, translation,
/// fitness), the yank it scored, and the time each stage took.
pub struct LoopSearchResult {
    diag: LoopCandidateDiag,
    accepted: Option<(Mat3, Vec3, f64)>,
    scored_yank: Option<(f64, f64)>,
    timing: TimingStats,
}

/// A search that ended at the gate `outcome`.
fn rejected(
    mut diag: LoopCandidateDiag,
    outcome: &'static str,
    scored_yank: Option<(f64, f64)>,
    timing: TimingStats,
) -> LoopSearchResult {
    diag.outcome = outcome;
    LoopSearchResult {
        diag,
        accepted: None,
        scored_yank,
        timing,
    }
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
        let icp = icp(
            &source_cloud,
            &target_cloud,
            &target_tree,
            &init_rotation,
            &init_translation,
            &IcpParams {
                max_iterations: self.config.loop_icp_max_iterations as usize,
                max_correspondence_distance: self.config.loop_icp_max_correspondence_m,
            },
        );
        timing.icp_s += t0.elapsed().as_secs_f64();
        if !self.config.loop_icp_dump_dir.is_empty() {
            dump_icp(
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
        let degeneracy_min = uniqueness::degeneracy(&source_cloud);
        timing.degeneracy_s += t0.elapsed().as_secs_f64();
        diag.degeneracy_min = degeneracy_min;

        if self.config.loop_min_degeneracy > 0.0
            && degeneracy_min >= 0.0
            && (degeneracy_min as f64) < self.config.loop_min_degeneracy
        {
            return rejected(diag, "degeneracy_reject", scored_yank, timing);
        }

        if !icp.converged || icp.fitness > self.config.loop_score_thresh {
            return rejected(diag, "icp_reject", scored_yank, timing);
        }
        // the pose graph takes ICP's fitness as the loop's variance: when the odometry between the two ends is
        // already surer of their relative position than that, the loop adds only ICP's noise
        if icp.fitness >= (cur_idx - loop_idx) as f64 * self.config.odom_trans_xy_var {
            return rejected(diag, "uninformative", scored_yank, timing);
        }

        let mut score = icp.fitness;
        let (source, target) = (&self.key_poses[cur_idx], &self.key_poses[loop_idx]);
        let loop_rotation_transpose = mat3::transpose(&target.rotation_global);
        // the loop measurement an alignment gives: the source keyframe moved by it, in the target's frame
        let measured = |rotation: &Mat3, translation: &Vec3| {
            let moved = mat3::add(
                &mat3::mat_vec(rotation, &source.translation_global),
                translation,
            );
            (
                mat3::mat_mul(
                    &loop_rotation_transpose,
                    &mat3::mat_mul(rotation, &source.rotation_global),
                ),
                mat3::mat_vec(
                    &loop_rotation_transpose,
                    &mat3::sub(&moved, &target.translation_global),
                ),
            )
        };
        let (mut rotation_offset, mut translation_offset) =
            measured(&icp.rotation, &icp.translation);

        // Graph yank: how far this constraint pulls the source keyframe from the
        // current odom-chain estimate (relative pose loop->cur, in the loop's
        // frame). A large yank on a plausible-distance candidate = a closure that
        // disagrees with odometry = the signature of a false closure.
        let (current_relative_rotation, current_relative_translation) =
            measured(&mat3::identity(), &[0.0; 3]);
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
            diag.strain = big_picture_fit::strain(
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
                return rejected(diag, "yank_confidence_reject", scored_yank, timing);
            };
            // the loop measurement comes from the extended alignment, which saw more of both places
            (rotation_offset, translation_offset) =
                measured(&extended.rotation, &extended.translation);
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
}

impl GscPgo {
    pub fn new(config: Config) -> GscPgo {
        let scan_context_config = similarity_discovery::Config {
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
            waiting_proposal: None,
            solving_keyframes: 0,
            pending_search: None,
        }
    }

    /// Cumulative wall seconds per stage: scan context, submaps, ICP, degeneracy, extended ICP, pose-graph solves.
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
    pub(crate) fn push_edge(
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
    pub(crate) fn record_search(&mut self, diag: LoopCandidateDiag) {
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
    /// Whether a loop was committed since the last `update`.
    pub fn has_loop(&self) -> bool {
        self.new_loop
    }

    /// Candidate search (scan context with position fallback) + all the
    /// gates, ICP verification, and LoopPair construction.
    /// Record that this keyframe's loop search was skipped because newer scans
    /// are already queued behind it: a search (an ICP on a large cloud can take
    /// seconds) must never be what makes the module fall behind.
    /// Stages 1 and 2 for the newest keyframe, inline on every keyframe: it becomes the waiting
    /// proposal if its scan-context match beats the one already waiting (the loser is recorded as
    /// "outranked"), so a busy search thread still gets the most promising keyframe it missed.
    pub fn propose_loop(&mut self) {
        if self.key_poses.len() < 10 {
            return;
        }
        let cur_idx = self.key_poses.len() - 1;
        let current = &self.key_poses[cur_idx];
        let mut diag = LoopCandidateDiag::new(cur_idx, current.time, current.translation_global);

        // stage 1, universal uniqueness: a scan whose structure barely fills the descriptor can't place itself
        let descriptor = &self.scan_context_descriptors[cur_idx];
        if !descriptor.is_empty() {
            diag.descriptor_structure = similarity_discovery::descriptor_structure(descriptor);
            diag.descriptor_occupancy =
                similarity_discovery::descriptor_occupancy(descriptor) as i32;
            if diag.descriptor_occupancy < self.config.loop_min_occupancy {
                diag.outcome = "low_occupancy";
                self.record_search(diag);
                return;
            }
        }

        // stage 2, similarity discovery: scan context, else the nearest past keyframe by position
        let t0 = std::time::Instant::now();
        let (mut loop_idx, sector_shift, sc_best, sc_second) = self.search_by_scan_context();
        self.timing.scan_context_s += t0.elapsed().as_secs_f64();
        diag.from_scan_context = loop_idx >= 0;
        if loop_idx < 0 {
            loop_idx = self.search_by_position();
        }
        if loop_idx < 0 {
            diag.outcome = "no_candidate";
            self.record_search(diag);
            return;
        }
        let loop_idx = loop_idx as usize;
        diag.target_id = loop_idx as i64;
        diag.target_time = self.key_poses[loop_idx].time;
        diag.target_xyz = self.key_poses[loop_idx].translation_global;
        diag.candidate_distance = mat3::norm(&mat3::sub(&diag.source_xyz, &diag.target_xyz));
        if diag.from_scan_context && sc_second > 1e-6 {
            diag.scan_context_best = sc_best;
            diag.scan_context_second = sc_second;
            diag.lowe_ratio = sc_best / sc_second;
        }
        // a position-only candidate ranks below any scan-context match (cosine distance is at most 2)
        let similarity = if diag.from_scan_context { sc_best } else { 2.0 };
        let proposal = LoopProposal {
            diag,
            loop_idx,
            sector_shift,
            similarity,
        };
        let loser = match self.waiting_proposal.take() {
            Some(waiting) if waiting.similarity <= proposal.similarity => {
                self.waiting_proposal = Some(waiting);
                proposal
            }
            waiting => {
                self.waiting_proposal = Some(proposal);
                match waiting {
                    Some(waiting) => waiting,
                    None => return,
                }
            }
        };
        let mut diag = loser.diag;
        diag.outcome = "outranked";
        self.record_search(diag);
    }
    /// Take the waiting proposal as a `LoopSearch` (stages 3 and 4) to run on any thread and hand to
    /// `finish_loop_search`.
    pub fn begin_loop_search(&mut self) -> Option<LoopSearch> {
        let LoopProposal {
            diag,
            loop_idx,
            sector_shift,
            ..
        } = self.waiting_proposal.take()?;
        let source_position = self.key_poses[diag.source_id].translation_global;
        let target_position = self.key_poses[loop_idx].translation_global;
        // ICP starts with the source keyframe at the target's position, turned by the scan-context yaw:
        // after a long drift the two positions differ by the whole drift, and seeding at the source's
        // own position would leave the two submaps without overlap
        let yaw = if sector_shift != 0 {
            similarity_discovery::yaw_from_shift(sector_shift, self.scan_context_config.n_sectors)
        } else {
            0.0
        };
        let init_rotation = mat3::rot_z(yaw);
        let init_translation = mat3::sub(
            &target_position,
            &mat3::mat_vec(&init_rotation, &source_position),
        );
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
    /// With `loop_search_keyframes`, a search holds the search thread for that many keyframes before its
    /// result lands (proposals queue meanwhile), the way a slower machine would, but the same on every machine.
    pub fn search_for_loop_pairs(&mut self) {
        self.propose_loop();
        if self
            .pending_search
            .as_ref()
            .is_some_and(|(_, due)| self.key_poses.len() < *due)
        {
            return;
        }
        if let Some((result, _)) = self.pending_search.take() {
            self.finish_loop_search(result);
        }
        if let Some(search) = self.begin_loop_search() {
            let result = search.run();
            match self.config.loop_search_keyframes {
                0 => self.finish_loop_search(result),
                k => self.pending_search = Some((result, self.key_poses.len() + k)),
            }
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
                let auto_range = similarity_discovery::auto_max_range(&cloud);
                if auto_range > 0.0 {
                    self.scan_context_config.max_range_m = auto_range;
                    eprintln!(
                        "scan-context: auto ring range = {auto_range:.2} m (from cloud extent)"
                    );
                }
            }
            if self.scan_context_config.max_range_m > 0.0 {
                let descriptor =
                    similarity_discovery::make_descriptor(&cloud, &self.scan_context_config);
                self.scan_context_ring_keys
                    .push(similarity_discovery::make_ring_key(&descriptor));
                self.scan_context_descriptors.push(descriptor);
            } else {
                self.scan_context_descriptors
                    .push(similarity_discovery::Descriptor::empty());
                self.scan_context_ring_keys
                    .push(similarity_discovery::RingKey::new());
            }
        } else {
            self.scan_context_descriptors
                .push(similarity_discovery::Descriptor::empty());
            self.scan_context_ring_keys
                .push(similarity_discovery::RingKey::new());
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
