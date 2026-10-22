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

//! `GscPgo` — keyframe gating, loop-candidate search + gates, factor graph
//! construction, and the iSAM2 update sequence.
//!
//! Key structural details:
//! - keyframe node keys are the plain contiguous indices (0, 1, 2, ...);
//! - location variables are `Symbol('l', i)` keys;
//! - a constraint-triggered node has no cloud (`Option<Arc<PointCloud>>`);
//! - rotations/vectors are `mat3::Mat3` / `mat3::Vec3` (row-major f64).

use std::collections::BTreeMap;
use std::sync::Arc;

use crate::gnc::{
    add_backbone_keyframe, add_rebuild_loop_factor, build_isam2_from_backbone,
    build_odometry_backbone, gnc_worker_loop, BackboneKeyframe, BackboneNoise, GncJob, GncResult,
    GncWorker, RebuildLoopNoise, RebuiltIsam2,
};
use crate::gtsam::{symbol_key, FactorGraph, Isam2, NoiseModel, Pose3, Values};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    self, cloud_degeneracy, icp_point_to_point, transform_cloud, voxel_downsample, IcpParams,
    KdTree, PointCloud,
};
use crate::scan_context;

/// Variance that leaves a Pose3 tangent component effectively free while still
/// contributing a tiny bit of information (keeps the linear system non-singular).
pub(crate) const FREE_VARIANCE: f64 = 1e8;

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

/// A relative-pose measurement from the body frame to a location variable,
/// plus its 6x6 covariance (row-major, GTSAM Pose3 tangent order
/// [rot(3), trans(3)]).
#[derive(Debug, Clone)]
pub struct LocationConstraintObs {
    pub to_id: String,
    pub constraint_instance_id: String,
    pub rotation_body_loc: Mat3,
    pub translation_body_loc: Vec3,
    pub covariance: [f64; 36],
    pub ts: f64,
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

/// A detected loop with the ICP-refined relative pose and the per-constraint
/// noise model built at detection.
pub struct LoopPair {
    pub source_id: usize,
    pub target_id: usize,
    pub rotation_offset: Mat3,
    pub translation_offset: Vec3,
    /// Diagnostic: ICP fitness (lidar) or 0 otherwise.
    pub score: f64,
    pub noise: NoiseModel,
}

/// The relative-pose data of a committed loop, retained after the run so an
/// offline pass can rebuild the graph and probe each edge. The live `LoopPair`
/// is drained into iSAM2 and dropped; this is the plain-data residue.
#[derive(Clone, Copy)]
pub struct CommittedLoop {
    pub source_id: usize,
    pub target_id: usize,
    pub rotation_offset: Mat3,
    pub translation_offset: Vec3,
    pub score: f64,
    /// Weight from the newest folded-in GNC solve (~1 inlier / kept, ~0 rejected). A loop that
    /// would move the map enters the graph only once a GNC solve accepts it, so it starts at 0;
    /// one that already fits the map (or any loop with `loop_gnc_final` off) goes straight in at 1.
    pub gnc_weight: f64,
    /// Live path (Approach B): the iSAM2 factor index of this loop's between
    /// factor, captured the update it was inserted, so a later background GNC
    /// classification can remove it via `removeFactorIndices` if it is judged an
    /// outlier. `None` until the containing update commits.
    pub isam2_factor_index: Option<u64>,
    /// Live path (Approach B): whether this loop's between factor is currently in
    /// the iSAM2 graph. Set false once removed as a GNC-classified outlier.
    pub active_in_isam2: bool,
    /// Consecutive GNC solves whose vote disagreed with whether this loop is in the graph.
    pub gnc_vote_streak: u64,
}

/// gtsam's elimination/Bayes-tree code recurses proportionally to graph size;
/// the 2 MB default thread stack overflows on multi-thousand-keyframe runs.
pub const GTSAM_THREAD_STACK_BYTES: usize = 64 * 1024 * 1024;

/// Floor on the per-loop ICP score before it is used as a GNC factor variance,
/// so a zero-score closure still gets a finite (very tight) noise model.
pub(crate) const LOOP_GNC_MIN_VARIANCE: f64 = 1e-6;

/// Approach B: a background GNC weight below this classifies the loop as an
/// outlier, so its between factor is dropped from iSAM2. GNC's TLS weights are
/// near 0 (reject) or near 1 (keep), so the midpoint is a safe cut.
pub(crate) const LOOP_GNC_INLIER_THRESHOLD: f64 = 0.5;

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

/// The key -> PoseStamped frame bookkeeping (`m_node_poses`): GTSAM nodes
/// carry no frame; this records the odometry frame each node was created in.
#[derive(Debug, Clone)]
pub struct PoseStamped {
    pub frame_id: String,
    pub sec: i32,
    pub nsec: u32,
    pub position: Vec3,
    /// Quaternion [x, y, z, w] (geometry_msgs order).
    pub orientation: [f64; 4],
}

/// Tunable configuration for the PGO core.
#[derive(Debug, Clone)]
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
    /// Sanity gate: skip ICP if candidate keyframe is farther than this from
    /// the current keyframe in global pose. 0 disables the check.
    pub loop_candidate_max_distance_m: f64,
    /// Feature-poverty gate on the descriptor vertical-structure std.
    /// 0 disables. Superseded in practice by occupancy + degeneracy.
    pub min_descriptor_std: f64,
    /// Structure-spread gate: minimum occupied Scan-Context cells. 0 disables.
    pub loop_min_occupancy: i32,
    /// Observability gate (Zhang 2016 / X-ICP degeneracy factor). 0 disables.
    pub loop_min_degeneracy: f64,
    /// Aliasing gate (Lowe ratio = best/second-best Scan-Context distance).
    /// In self-similar spaces the top two matches tie (ratio -> 1), so the
    /// candidate can't be trusted to pick the right keyframe. Reject when the
    /// ratio exceeds this. 0 disables. Only applies to scan-context candidates.
    pub loop_max_lowe_ratio: f64,
    /// False-closure gate on the graph yank (ICP-refined relative rotation vs the
    /// current odom-chain estimate). A pair the graph places within
    /// `loop_yank_gate_max_distance_m` is nearly coincident, so a large ICP
    /// rotation disagreement is physically impossible = a structural-alias false
    /// match. Reject when yank rotation exceeds this many degrees. 0 disables.
    pub loop_max_yank_rotation_deg: f64,
    /// Only apply `loop_max_yank_rotation_deg` when the candidate distance is
    /// below this (near-coincident pairs, where odom rotation is trustworthy).
    /// 0 disables the whole yank-rotation gate.
    pub loop_yank_gate_max_distance_m: f64,
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
    /// With `loop_yank_gate`, a candidate whose single-scan confidence is too low for its yank gets
    /// a second look on more data instead of an outright reject: seeded with the first ICP, the
    /// source grows to the last 2, 4, 8, ... keyframes (up to this many), and the target to the
    /// keyframes near the matched one in time that those land within `loop_search_radius` of.
    /// Inliers are source points within sqrt(`loop_score_thresh`) of the target after ICP.
    /// Each round counts as (extended inliers / single-scan inliers) independent checks, each as
    /// good as the single scan's confidence times how well the overlap held up (extended inlier
    /// fraction / single inlier fraction, at most 1): confidence = 1 - (1 - c_round)^checks.
    /// (Mean squared fitness is not used: the stacked clouds see places the other side never did.)
    /// The first round that clears the gate is kept, and its transform becomes the loop. 0 = off.
    pub loop_extended_icp_max_keyframes: usize,
    /// Minimum keyframe-index separation between the two ends of a loop closure.
    /// A closure only corrects drift accumulated over the trajectory between the
    /// pair; a near-in-sequence pair spans negligible drift, so a "closure" there
    /// corrects nothing and can only inject error (structural alias). 0 disables.
    pub loop_min_id_gap: u64,
    /// Robust (M-estimator) kernel wrapping all loop factors.
    pub loop_robust_kernel: bool,
    pub loop_robust_huber_k: f64,
    /// Loop-verification ICP: iteration cap and correspondence gate (m).
    pub loop_icp_max_iterations: u64,
    pub loop_icp_max_correspondence_m: f64,
    /// Score ICP fitness over correspondences within the gate only (inlier
    /// mean squared error) instead of over every source point.
    pub loop_icp_inlier_fitness: bool,
    /// Reject a candidate whose source or target submap has fewer points. 0 = off.
    pub loop_icp_min_points: u64,
    /// Seed loop ICP at the current pose instead of at the matched keyframe
    /// rotated by the Scan Context yaw.
    pub loop_icp_seed_at_current_pose: bool,
    /// Loop factor noise: rotation variance (0 = the ICP fitness, as for
    /// translation) and a floor on the translation variance (the ICP fitness).
    pub loop_rotation_variance: f64,
    pub loop_translation_variance_floor: f64,
    /// iSAM2 update passes after an update that adds a closure.
    pub loop_refinement_passes: u64,
    /// After those, keep relinearizing until no keyframe moves more than this per pass (at
    /// most `loop_refinement_max_passes`), so a closure moves the map once, not over the
    /// following keyframes.
    pub loop_refinement_tolerance_m: f64,
    pub loop_refinement_max_passes: u64,
    /// After the incremental run, re-solve the whole graph once with a batch GNC
    /// (graduated non-convexity, TLS loss) optimizer that jointly classifies the
    /// committed loop closures inlier/outlier against the global residual — the
    /// odometry backbone is pinned as a known inlier. This rejects the mutually
    /// conflicting closures that Huber merely down-weights, so a loop that only
    /// under-closes incrementally actually snaps shut.
    pub loop_gnc_final: bool,
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
    /// Consecutive GNC solves that must agree before a loop enters or leaves the graph, so a
    /// verdict one solve gives and the next takes back never moves the map.
    pub loop_gnc_confirmations: u64,
    /// true: fold a background GNC solve on the keyframe schedule above, so a replay gives the same
    /// graph on any machine. false: fold it as soon as it finishes (see `poll_gnc`), keyframes or
    /// not, so a robot standing still at a loop still gets the closure; results then depend on timing.
    pub loop_gnc_deterministic: bool,
    /// Ingest LocationConstraint events (consumed by the module wiring, not
    /// by GscPgo itself).
    pub use_location_constraints: bool,
    // Odometry between-factor noise (anisotropic).
    pub odom_rot_roll_pitch_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,
    // Roll/pitch stiffness of the first-keyframe anchor prior: a tight value
    // hard-pins roll/pitch to the initial LIO attitude, a loose one frees it.
    pub anchor_roll_pitch_var: f64,
    /// Optional roll/pitch prior on every keyframe (yaw + translation left free).
    pub per_keyframe_roll_pitch_prior: bool,
    pub per_keyframe_roll_pitch_var: f64,
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
            loop_candidate_max_distance_m: 0.0,
            min_descriptor_std: 0.0,
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_max_lowe_ratio: 0.0,
            loop_max_yank_rotation_deg: 0.0,
            loop_yank_gate_max_distance_m: 0.0,
            loop_yank_gate: true,
            loop_yank_gate_width: 1.0,
            loop_yank_per_sqrt_gap: true,
            loop_icp_dump_dir: String::new(),
            loop_extended_icp_max_keyframes: 16,
            loop_min_id_gap: 0,
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
            loop_refinement_tolerance_m: 0.01,
            loop_refinement_max_passes: 200,
            loop_gnc_final: true,
            loop_gnc_var_scale: 10.0,
            loop_gnc_inlier_probability: 0.95,
            loop_gnc_apply_after_fraction: 0.05,
            loop_gnc_confirmations: 2,
            loop_gnc_deterministic: true,
            use_location_constraints: false,
            odom_rot_roll_pitch_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            anchor_roll_pitch_var: 1e-12,
            per_keyframe_roll_pitch_prior: false,
            per_keyframe_roll_pitch_var: 1e-4,
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

impl Config {
    /// Overwrite every false-closure gate with a preset for `level`, a single
    /// dial from 0 (accept nearly any match) to 4 (every gate plus a tight
    /// GNC). Level 2 reproduces this struct's own defaults, and a level
    /// outside the range leaves the individually-set fields untouched.
    ///
    /// The gates it does not touch are the ones that are not accept/reject
    /// tradeoffs: keyframing, the search radius, the odometry variances, and
    /// the Huber kernel (numerical conditioning, not a filter).
    pub fn apply_conservativeness(&mut self, level: i32) {
        if !(0..=4).contains(&level) {
            return;
        }
        let (score_thresh, scan_context_threshold, occupancy, degeneracy) = match level {
            0 => (0.30, 0.70, 0, 0.0),
            1 => (0.20, 0.50, 40, 0.02),
            2 => (0.15, 0.40, 80, 0.05),
            3 => (0.13, 0.30, 100, 0.07),
            _ => (0.10, 0.15, 120, 0.10),
        };
        let (lowe_ratio, yank_deg, yank_distance, id_gap) = match level {
            0..=2 => (0.0, 0.0, 0.0, 0),
            3 => (0.97, 30.0, 8.0, 10),
            _ => (0.85, 15.0, 5.0, 30),
        };
        let candidate_max = match level {
            0..=2 => 0.0,
            3 => 200.0,
            _ => 50.0,
        };
        let (gnc_final, gnc_var_scale, gnc_inlier_probability) = match level {
            0 => (false, 100.0, 0.999),
            1 => (true, 20.0, 0.999),
            2 => (true, 10.0, 0.99),
            3 => (true, 10.0, 0.95),
            _ => (true, 5.0, 0.90),
        };

        self.loop_score_thresh = score_thresh;
        self.scan_context_match_threshold = scan_context_threshold;
        self.loop_min_occupancy = occupancy;
        self.loop_min_degeneracy = degeneracy;
        self.loop_max_lowe_ratio = lowe_ratio;
        self.loop_max_yank_rotation_deg = yank_deg;
        self.loop_yank_gate_max_distance_m = yank_distance;
        self.loop_min_id_gap = id_gap;
        self.loop_candidate_max_distance_m = candidate_max;
        self.loop_gnc_final = gnc_final;
        self.loop_gnc_var_scale = gnc_var_scale;
        self.loop_gnc_inlier_probability = gnc_inlier_probability;
    }
}

/// A constraint factor staged this update cycle, with its position in the
/// graph so its assigned iSAM2 factor index can be captured afterwards.
struct StagedConstraintFactor {
    instance_id: String,
    graph_pos: usize,
}

pub struct GscPgo {
    config: Config,
    scan_context_config: scan_context::Config,
    key_poses: Vec<KeyPoseWithCloud>,
    history_pairs: Vec<(usize, usize)>,
    committed_loops: Vec<CommittedLoop>,
    cache_pairs: Vec<LoopPair>,
    scan_context_descriptors: Vec<scan_context::Descriptor>,
    scan_context_ring_keys: Vec<scan_context::RingKey>,
    rotation_offset: Mat3,
    translation_offset: Vec3,
    isam2: Isam2,
    initial_values: Values,
    graph: FactorGraph,
    node_poses: BTreeMap<u64, PoseStamped>,
    // --- Location-constraint bookkeeping ---------------------------------
    location_index: BTreeMap<String, i32>,
    next_location: i32,
    staged_constraint_factors: Vec<StagedConstraintFactor>,
    committed_by_instance: BTreeMap<String, Vec<u64>>,
    pending_removals: Vec<u64>,
    /// Committed loops a GNC solve accepted, to insert into iSAM2 at the next update (the
    /// location-constraint path, which folds GNC in without rebuilding iSAM2).
    accepted_loop_inserts: Vec<usize>,
    location_closure: bool,
    /// iSAM2 was just rebuilt at a GNC solution, which may hold loops the graph does not
    /// (unconfirmed ones), so the next update relinearizes until it settles.
    reseeded: bool,
    timing: TimingStats,
    loop_diagnostics: Vec<LoopCandidateDiag>,
    loop_edge_events: Vec<LoopEdgeEvent>,
    /// Per committed loop: whether a graph update has applied it since it last entered the graph.
    loop_applied: Vec<bool>,
    /// Per committed loop never in the graph: consecutive GNC votes against it, and whether that
    /// has been reported as a rejection (an edge GNC turns down without ever taking it in).
    loop_out_votes: Vec<(u64, bool)>,
    /// Rotation (deg) and translation (m) yank of every candidate ICP has scored (per sqrt keyframe
    /// gap with `loop_yank_per_sqrt_gap`), the scale new yanks are measured against. Every scored candidate, not just committed ones: a scale taken
    /// only from loops the gate let through would shrink with each rejection and reject the rest.
    scored_yanks: Vec<(f64, f64)>,
    /// Per committed loop: its yank_z and confidence (which scales its GNC votes'), for its
    /// loop-edge events.
    loop_yank_z: Vec<f64>,
    loop_fit_confidence: Vec<f64>,
    /// Keyframe time of the last loop search that got past the throttles.
    last_loop_search_time: Option<f64>,
    /// Background batch-GNC worker (spawned only when `loop_gnc_final` is on); see `step_gnc`.
    gnc_worker: Option<GncWorker>,
    /// Keyframe count at which the outstanding GNC solve is folded in, None when none is out.
    gnc_due_at: Option<usize>,
    /// Committed loops the newest dispatched GNC solve covers.
    gnc_loops_covered: usize,
}

/// Cumulative per-stage wall time, printed periodically when `debug` is on —
/// purely observational (never read back into the pipeline).
#[derive(Default)]
struct TimingStats {
    scan_context_s: f64,
    submap_s: f64,
    icp_s: f64,
    degeneracy_s: f64,
    gtsam_s: f64,
    smooth_calls: u64,
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
        let gnc_worker = if config.loop_gnc_final {
            let (job_sender, job_receiver) = std::sync::mpsc::channel::<GncJob>();
            let (result_sender, result_receiver) = std::sync::mpsc::channel::<GncResult>();
            std::thread::Builder::new()
                .name("gnc-worker".to_string())
                .stack_size(GTSAM_THREAD_STACK_BYTES)
                .spawn(move || gnc_worker_loop(job_receiver, result_sender))
                .expect("spawn gnc worker thread");
            Some(GncWorker {
                job_sender,
                result_receiver,
            })
        } else {
            None
        };
        GscPgo {
            config,
            scan_context_config,
            gnc_worker,
            gnc_due_at: None,
            gnc_loops_covered: 0,
            key_poses: Vec::new(),
            history_pairs: Vec::new(),
            committed_loops: Vec::new(),
            cache_pairs: Vec::new(),
            scan_context_descriptors: Vec::new(),
            scan_context_ring_keys: Vec::new(),
            rotation_offset: mat3::identity(),
            translation_offset: [0.0; 3],
            // The shim configures ISAM2 with relinearizeThreshold = 0.01,
            // relinearizeSkip = 1.
            isam2: Isam2::new(),
            initial_values: Values::new(),
            graph: FactorGraph::new(),
            node_poses: BTreeMap::new(),
            location_index: BTreeMap::new(),
            next_location: 0,
            staged_constraint_factors: Vec::new(),
            committed_by_instance: BTreeMap::new(),
            pending_removals: Vec::new(),
            accepted_loop_inserts: Vec::new(),
            location_closure: false,
            reseeded: false,
            timing: TimingStats::default(),
            loop_diagnostics: Vec::new(),
            loop_edge_events: Vec::new(),
            loop_applied: Vec::new(),
            loop_out_votes: Vec::new(),
            scored_yanks: Vec::new(),
            loop_yank_z: Vec::new(),
            loop_fit_confidence: Vec::new(),
            last_loop_search_time: None,
        }
    }

    /// All per-search loop-closure diagnostic records accumulated so far.
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

    /// Fill in the yank / confidence gate's inputs and outputs for a candidate ICP has scored, then
    /// add its yank to the scale.
    fn score_candidate(&mut self, diag: &mut LoopCandidateDiag) {
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
        if self.scored_yanks.is_empty() {
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
        self.scored_yanks.push((
            diag.yank_rotation_deg / drift,
            diag.yank_translation / drift,
        ));
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
        target_cloud: &PointCloud,
        first: &IcpResult,
        diag: &mut LoopCandidateDiag,
    ) -> Option<IcpResult> {
        let max_keyframes = self.config.loop_extended_icp_max_keyframes;
        if max_keyframes < 2 || !first.converged {
            return None;
        }
        let max_distance = self.config.loop_icp_max_correspondence_m;
        // a point overlaps when it is within the RMS distance the ICP score gate allows (fitness is a mean squared distance)
        let inliers = |source: &PointCloud, target: &PointCloud, icp: &IcpResult| {
            let tree = KdTree::build(target);
            let limit = self.config.loop_score_thresh as f32;
            transform_cloud(source, &icp.rotation, &icp.translation)
                .iter()
                .filter(|p| matches!(tree.nearest(p), Some((_, d)) if d <= limit))
                .count()
        };
        let single = inliers(source_cloud, target_cloud, first);
        diag.single_inliers = single;
        if single == 0 {
            return None;
        }
        let params = IcpParams {
            max_iterations: self.config.loop_icp_max_iterations as usize,
            max_correspondence_distance: max_distance,
            fitness_max_distance: if self.config.loop_icp_inlier_fitness {
                max_distance
            } else {
                0.0
            },
            ..IcpParams::default()
        };
        let half_range = self.config.loop_submap_half_range.max(0) as usize;
        let radius = self.config.loop_search_radius;
        let mut keyframes = 2;
        while keyframes <= max_keyframes {
            // never stack source keyframes into the stretch the target side is drawn from
            let window = half_range + keyframes;
            let source_first = cur_idx.saturating_sub(keyframes).max(loop_idx + window + 1);
            if source_first >= cur_idx {
                return None;
            }
            // where the first ICP puts each stacked source keyframe
            let placed: Vec<Vec3> = (source_first..=cur_idx)
                .map(|k| {
                    mat3::add(
                        &mat3::mat_vec(&first.rotation, &self.key_poses[k].translation_global),
                        &first.translation,
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
            let source = self.stacked_cloud(source_first..=cur_idx);
            let target = self.stacked_cloud(target_ids.filter(|&j| near(j)));
            let icp = icp_point_to_point(
                &source,
                &target,
                &first.rotation,
                &first.translation,
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
            let extended = if icp.converged {
                inliers(&source, &target, &icp)
            } else {
                0
            };
            let held_up = ((extended as f64 / source.len().max(1) as f64)
                / (single as f64 / source_cloud.len() as f64))
                .min(1.0);
            let per_scan = diag.confidence * held_up;
            // evidence = -ln(1 - confidence); the gate compares it to z^2 / 2k^2 without rounding to 1
            let evidence = -(1.0 - per_scan).ln() * extended as f64 / single as f64;
            let width = self.config.loop_yank_gate_width.max(f64::EPSILON);
            let required = diag.yank_z * diag.yank_z / (2.0 * width * width);
            let confidence = 1.0 - (-evidence).exp();
            diag.extended_keyframes = cur_idx - source_first + 1;
            diag.extended_source_points = source.len();
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
            keyframes *= 2;
        }
        None
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

    /// Total nonlinear error of the current factor graph at the best estimate
    /// (½·Σ whitened residual², robust kernels applied) — how much all the
    /// constraints conflict. `NaN` on failure.
    pub fn graph_error(&self) -> f64 {
        self.isam2.error()
    }

    /// Rebuild the odometry backbone (anchor prior + odom between-factors, plus
    /// the optional per-keyframe roll/pitch prior) into a fresh graph/values.
    /// Shared by the counterfactual edge-error probe and the batch GNC re-solve.
    /// Loop factors are added by the caller. `local_init` seeds every node (and
    /// the anchor prior) from the raw dead-reckoned local pose instead of the
    /// incrementally-corrected global — GNC's TLS loss is non-convex and must
    /// start from the odom backbone, not the under-closed Huber estimate, or it
    /// settles into the wrong inlier set on long trajectories.
    fn build_backbone(&self, graph: &mut FactorGraph, values: &mut Values, local_init: bool) {
        let backbone_keyframes: Vec<BackboneKeyframe> = self
            .key_poses
            .iter()
            .map(|key_pose| BackboneKeyframe {
                rotation_local: key_pose.rotation_local,
                translation_local: key_pose.translation_local,
                initial_rotation: if local_init {
                    key_pose.rotation_local
                } else {
                    key_pose.rotation_global
                },
                initial_translation: if local_init {
                    key_pose.translation_local
                } else {
                    key_pose.translation_global
                },
            })
            .collect();
        build_odometry_backbone(
            graph,
            values,
            &backbone_keyframes,
            &BackboneNoise::from_config(&self.config),
        );
    }

    /// For each committed loop, rebuild a fresh pose graph and report the total
    /// GTSAM error assuming that one edge were accepted at high confidence
    /// (`high_confidence_variance` on all six tangent components, no robust
    /// kernel so the conflict is not down-weighted).
    ///
    /// With `full_consensus` every other committed loop is present at its real
    /// noise, so the error reflects how much the tested edge fights the rest of
    /// the closures. Otherwise only the odometry backbone plus the tested edge
    /// are present, so the error reflects how much it fights dead-reckoning.
    /// Returned in `committed_loops` order.
    pub fn counterfactual_edge_errors(
        &self,
        high_confidence_variance: f64,
        full_consensus: bool,
    ) -> Vec<f64> {
        let high_conf_var = [high_confidence_variance; 6];
        let mut errors = Vec::with_capacity(self.committed_loops.len());
        for tested in 0..self.committed_loops.len() {
            let mut graph = FactorGraph::new();
            let mut values = Values::new();
            self.build_backbone(&mut graph, &mut values, false);
            for (edge_idx, loop_edge) in self.committed_loops.iter().enumerate() {
                let is_tested = edge_idx == tested;
                if !full_consensus && !is_tested {
                    continue;
                }
                let variances = if is_tested {
                    high_conf_var
                } else {
                    [loop_edge.score; 6]
                };
                let base = NoiseModel::diagonal_variances(&variances);
                let robust;
                let noise: &NoiseModel = if self.config.loop_robust_kernel && !is_tested {
                    robust = NoiseModel::robust_huber(self.config.loop_robust_huber_k, &base);
                    &robust
                } else {
                    &base
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
                    .expect("gtsam: counterfactual loop between factor");
            }
            let mut isam2 = Isam2::new();
            isam2
                .update(&graph, &values, &[])
                .expect("gtsam: counterfactual isam2 update");
            errors.push(isam2.error());
        }
        errors
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
        self.insert_pose_node(&cloud_with_pose.pose, cloud, &cloud_with_pose.frame_id);
        true
    }

    /// The constraint becomes its own pose node (at the interpolated-odometry
    /// pose supplied by the caller)
    /// linked to the backbone by an odom between-factor, plus a
    /// BetweenFactor(node, location) with the constraint's covariance.
    /// Returns false (and does nothing) if no keyframe exists yet.
    pub fn add_location_constraint(
        &mut self,
        pose: &PoseWithTime,
        frame_id: &str,
        constraint: &LocationConstraintObs,
    ) -> bool {
        if self.key_poses.is_empty() {
            return false;
        }
        let node_idx = self.insert_pose_node(pose, None, frame_id);
        self.add_location_constraint_factors(node_idx, constraint);
        true
    }

    pub fn has_loop(&self) -> bool {
        !self.cache_pairs.is_empty()
    }

    pub fn history_pairs(&self) -> &[(usize, usize)] {
        &self.history_pairs
    }

    /// The relative-pose data of every committed loop, in commit order.
    pub fn committed_loops(&self) -> &[CommittedLoop] {
        &self.committed_loops
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

    pub fn node_poses(&self) -> &BTreeMap<u64, PoseStamped> {
        &self.node_poses
    }

    pub fn descriptors(&self) -> &[scan_context::Descriptor] {
        &self.scan_context_descriptors
    }

    pub fn ring_keys(&self) -> &[scan_context::RingKey] {
        &self.scan_context_ring_keys
    }

    /// New node (key = next contiguous index) with initial value + backbone
    /// factor (gravity prior on the first,
    /// else an odom between-factor), the optional per-keyframe gravity
    /// anchor, the scan-context cache, and the frame record.
    fn insert_pose_node(
        &mut self,
        pose: &PoseWithTime,
        cloud: Option<Arc<PointCloud>>,
        frame_id: &str,
    ) -> usize {
        let idx = self.key_poses.len();
        let init_rotation = mat3::mat_mul(&self.rotation_offset, &pose.rotation);
        let init_translation = mat3::add(
            &mat3::mat_vec(&self.rotation_offset, &pose.translation),
            &self.translation_offset,
        );
        let init_pose = Pose3 {
            rotation: init_rotation,
            translation: init_translation,
        };
        self.initial_values
            .insert_pose3(idx as u64, &init_pose)
            .expect("gtsam: insert initial value");
        if idx == 0 {
            // Absolute anchor prior on the first keyframe (always present: a
            // relative-only pose graph is singular without one absolute anchor).
            // Pose3 tangent order is [rot(3), trans(3)]: components 0-1 are
            // roll/pitch, 2 is yaw. Yaw and translation are hard-pinned to the
            // gauge; only roll/pitch stiffness is exposed as a config knob.
            const ANCHOR_YAW_VAR: f64 = 1e-12;
            const ANCHOR_TRANS_VAR: f64 = 1e-12;
            let prior_var = [
                self.config.anchor_roll_pitch_var,
                self.config.anchor_roll_pitch_var,
                ANCHOR_YAW_VAR,
                ANCHOR_TRANS_VAR,
                ANCHOR_TRANS_VAR,
                ANCHOR_TRANS_VAR,
            ];
            let noise = NoiseModel::diagonal_variances(&prior_var);
            self.graph
                .add_prior_pose3(idx as u64, &init_pose, &noise)
                .expect("gtsam: add anchor prior");
        } else {
            // Odometry constraint to the previous keyframe. Anisotropic:
            // stiff relative roll/pitch (gravity-accurate), looser yaw.
            let last_item = self.key_poses.last().unwrap();
            let last_rotation_transpose = mat3::transpose(&last_item.rotation_local);
            let rotation_between = mat3::mat_mul(&last_rotation_transpose, &pose.rotation);
            let translation_between = mat3::mat_vec(
                &last_rotation_transpose,
                &mat3::sub(&pose.translation, &last_item.translation_local),
            );
            let noise = NoiseModel::diagonal_variances(&[
                self.config.odom_rot_roll_pitch_var,
                self.config.odom_rot_roll_pitch_var,
                self.config.odom_rot_yaw_var,
                self.config.odom_trans_xy_var,
                self.config.odom_trans_xy_var,
                self.config.odom_trans_z_var,
            ]);
            self.graph
                .add_between_pose3(
                    (idx - 1) as u64,
                    idx as u64,
                    &Pose3 {
                        rotation: rotation_between,
                        translation: translation_between,
                    },
                    &noise,
                )
                .expect("gtsam: add odom between factor");

            // Optional per-keyframe roll/pitch prior: pin this keyframe's
            // roll/pitch to its initial (LIO) attitude, leaving yaw + translation
            // free (huge variance).
            if self.config.per_keyframe_roll_pitch_prior {
                let grav_var = [
                    self.config.per_keyframe_roll_pitch_var,
                    self.config.per_keyframe_roll_pitch_var,
                    FREE_VARIANCE,
                    FREE_VARIANCE,
                    FREE_VARIANCE,
                    FREE_VARIANCE,
                ];
                let grav_noise = NoiseModel::diagonal_variances(&grav_var);
                self.graph
                    .add_prior_pose3(idx as u64, &init_pose, &grav_noise)
                    .expect("gtsam: add per-keyframe roll/pitch prior");
            }
        }
        self.key_poses.push(KeyPoseWithCloud {
            time: pose.second,
            rotation_local: pose.rotation,
            translation_local: pose.translation,
            body_cloud: cloud.clone(),
            rotation_global: init_rotation,
            translation_global: init_translation,
        });

        // Record this node's frame (from the odometry) alongside its pose.
        let q = mat3::quat_from_mat(&pose.rotation);
        self.node_poses.insert(
            idx as u64,
            PoseStamped {
                frame_id: frame_id.to_string(),
                sec: pose.second as i32,
                nsec: ((pose.second - (pose.second as i32) as f64) * 1e9) as u32,
                position: pose.translation,
                orientation: [q[1], q[2], q[3], q[0]],
            },
        );

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

        idx
    }

    /// Aggregate the body clouds of keyframes [idx-half_range,
    /// idx+half_range] transformed by their GLOBAL poses,
    /// then voxel-downsample at `resolution` (when > 0).
    pub fn get_sub_map(&self, idx: i32, half_range: i32, resolution: f64) -> PointCloud {
        assert!(idx >= 0 && (idx as usize) < self.key_poses.len());
        let min_idx = 0.max(idx - half_range) as usize;
        let max_idx = (self.key_poses.len() as i32 - 1).min(idx + half_range) as usize;

        let mut ret: PointCloud = Vec::new();
        for key_pose in &self.key_poses[min_idx..=max_idx] {
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
        if let Some(key_pose) = self.key_poses.last() {
            let mut diag = LoopCandidateDiag::new(
                self.key_poses.len() - 1,
                key_pose.time,
                key_pose.translation_global,
            );
            diag.outcome = "throttle_backlog";
            self.record_search(diag);
        }
    }

    pub fn search_for_loop_pairs(&mut self) {
        if self.key_poses.len() < 10 {
            return;
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
                    return;
                }
            }
        }

        if self.config.loop_search_min_interval > 0.0 {
            let current_time = self.key_poses[cur_idx].time;
            if let Some(last_search) = self.last_loop_search_time {
                if current_time - last_search < self.config.loop_search_min_interval {
                    diag.outcome = "throttle_search_interval";
                    self.record_search(diag);
                    return;
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
            if self.config.min_descriptor_std > 0.0
                && diag.descriptor_structure < self.config.min_descriptor_std as f32
            {
                diag.outcome = "no_structure";
                self.record_search(diag);
                return;
            }
            if self.config.loop_min_occupancy > 0
                && diag.descriptor_occupancy < self.config.loop_min_occupancy
            {
                diag.outcome = "low_occupancy";
                self.record_search(diag);
                return;
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
            return;
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

        // Arc-separation gate: a closure only corrects drift accumulated over the
        // trajectory between the pair. Too few keyframes apart = negligible drift
        // to correct, so the "closure" can only inject error (structural alias).
        if self.config.loop_min_id_gap > 0
            && (cur_idx.abs_diff(loop_idx) as u64) < self.config.loop_min_id_gap
        {
            diag.outcome = "id_gap_reject";
            self.record_search(diag);
            return;
        }

        if from_scan_context && sc_second > 1e-6 {
            diag.scan_context_best = sc_best;
            diag.scan_context_second = sc_second;
            diag.lowe_ratio = sc_best / sc_second;
        }

        // Aliasing gate: reject scan-context matches whose best and second-best
        // distances are near-tied (Lowe ratio -> 1). Runs before the expensive
        // ICP/submap work.
        if from_scan_context
            && self.config.loop_max_lowe_ratio > 0.0
            && sc_second > 1e-6
            && (sc_best / sc_second) as f64 > self.config.loop_max_lowe_ratio
        {
            diag.outcome = "lowe_reject";
            self.record_search(diag);
            return;
        }

        // Positional-plausibility gate on scan-context false matches.
        if self.config.loop_candidate_max_distance_m > 0.0
            && diag.candidate_distance > self.config.loop_candidate_max_distance_m
        {
            diag.outcome = "distance_reject";
            self.record_search(diag);
            return;
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

        if self.config.loop_icp_seed_at_current_pose {
            init_rotation = mat3::identity();
            init_translation = [0.0f64; 3];
        }

        let t0 = std::time::Instant::now();
        let target_cloud = self.get_sub_map(
            loop_idx as i32,
            self.config.loop_submap_half_range,
            self.config.submap_resolution,
        );
        let source_cloud = self.get_sub_map(cur_idx as i32, 0, self.config.submap_resolution);
        let min_points = self.config.loop_icp_min_points as usize;
        if min_points > 0 && (source_cloud.len() < min_points || target_cloud.len() < min_points) {
            diag.outcome = "icp_reject";
            self.record_search(diag);
            return;
        }
        self.timing.submap_s += t0.elapsed().as_secs_f64();

        let t0 = std::time::Instant::now();
        let icp = icp_point_to_point(
            &source_cloud,
            &target_cloud,
            &init_rotation,
            &init_translation,
            &IcpParams {
                max_iterations: self.config.loop_icp_max_iterations as usize,
                max_correspondence_distance: self.config.loop_icp_max_correspondence_m,
                fitness_max_distance: if self.config.loop_icp_inlier_fitness {
                    self.config.loop_icp_max_correspondence_m
                } else {
                    0.0
                },
                ..IcpParams::default()
            },
        );
        self.timing.icp_s += t0.elapsed().as_secs_f64();
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
        self.timing.degeneracy_s += t0.elapsed().as_secs_f64();
        diag.degeneracy_min = degeneracy_min;

        if self.config.loop_min_degeneracy > 0.0
            && degeneracy_min >= 0.0
            && (degeneracy_min as f64) < self.config.loop_min_degeneracy
        {
            diag.outcome = "degeneracy_reject";
            self.record_search(diag);
            return;
        }

        if !icp.converged || icp.fitness > self.config.loop_score_thresh {
            diag.outcome = "icp_reject";
            self.record_search(diag);
            return;
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

        self.score_candidate(&mut diag);
        if self.config.loop_yank_gate && diag.confidence < diag.required_confidence {
            let Some(extended) = self.extended_icp(
                cur_idx,
                loop_idx,
                &source_cloud,
                &target_cloud,
                &icp,
                &mut diag,
            ) else {
                diag.outcome = "yank_confidence_reject";
                self.record_search(diag);
                return;
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

        if self.config.loop_max_yank_rotation_deg > 0.0
            && diag.candidate_distance < self.config.loop_yank_gate_max_distance_m
            && diag.yank_rotation_deg > self.config.loop_max_yank_rotation_deg
        {
            diag.outcome = "yank_reject";
            self.record_search(diag);
            return;
        }

        let noise = NoiseModel::diagonal_variances(&loop_variances(&self.config, score));
        let pair = LoopPair {
            source_id: cur_idx,
            target_id: loop_idx,
            rotation_offset,
            translation_offset,
            score,
            noise,
        };

        diag.outcome = "accepted";
        self.record_search(diag);
        self.commit_loop_pair(pair);
    }

    /// Commit an accepted loop pair to the graph (between-factor cache) and the
    /// history used for closure throttling.
    fn commit_loop_pair(&mut self, pair: LoopPair) {
        self.history_pairs.push((pair.target_id, pair.source_id));
        // With a GNC worker, every loop waits for GNC to accept it before it enters the graph.
        let unvetted = self.gnc_worker.is_some();
        self.committed_loops.push(CommittedLoop {
            source_id: pair.source_id,
            target_id: pair.target_id,
            rotation_offset: pair.rotation_offset,
            translation_offset: pair.translation_offset,
            score: pair.score,
            gnc_weight: if unvetted { 0.0 } else { 1.0 },
            isam2_factor_index: None,
            active_in_isam2: !unvetted,
            gnc_vote_streak: 0,
        });
        self.loop_applied.push(false);
        self.loop_out_votes.push((0, false));
        self.cache_pairs.push(pair);
    }

    /// Ensure a graph variable for `to_id` (initialized from this node's
    /// global pose when new), add a
    /// BetweenFactor(node, location) with the constraint's covariance, and
    /// apply instance-id revision by scheduling removal of committed factors
    /// with the same instance id.
    fn add_location_constraint_factors(
        &mut self,
        node_idx: usize,
        constraint: &LocationConstraintObs,
    ) {
        let node = self.key_poses[node_idx].clone();

        // Ensure a graph variable for this location id (Symbol('l', index)).
        let is_new = !self.location_index.contains_key(&constraint.to_id);
        let loc_idx = if is_new {
            let loc_idx = self.next_location;
            self.next_location += 1;
            self.location_index
                .insert(constraint.to_id.clone(), loc_idx);
            loc_idx
        } else {
            self.location_closure = true; // re-sighting => a loop closure
            self.location_index[&constraint.to_id]
        };
        let loc_key = symbol_key('l', loc_idx as u64);

        if is_new {
            // Initialize the location in the world frame from this node.
            let rotation_loc_world =
                mat3::mat_mul(&node.rotation_global, &constraint.rotation_body_loc);
            let translation_loc_world = mat3::add(
                &mat3::mat_vec(&node.rotation_global, &constraint.translation_body_loc),
                &node.translation_global,
            );
            self.initial_values
                .insert_pose3(
                    loc_key,
                    &Pose3 {
                        rotation: rotation_loc_world,
                        translation: translation_loc_world,
                    },
                )
                .expect("gtsam: insert location initial value");
        }

        // Revision: a constraint reusing an existing constraint_instance_id
        // supersedes the committed factors carrying that id.
        if !constraint.constraint_instance_id.is_empty() {
            if let Some(committed) = self
                .committed_by_instance
                .get_mut(&constraint.constraint_instance_id)
            {
                if !committed.is_empty() {
                    self.pending_removals.append(committed);
                    self.location_closure = true; // removal earns the extra relin passes
                }
            }
        }

        // Noise model = the covariance carried in the message (already in
        // GTSAM Pose3 tangent order [rot(3), trans(3)]).
        let gaussian = NoiseModel::gaussian_covariance(&constraint.covariance);
        let robust;
        let noise: &NoiseModel = if self.config.loop_robust_kernel {
            robust = NoiseModel::robust_huber(self.config.loop_robust_huber_k, &gaussian);
            &robust
        } else {
            &gaussian
        };

        // Observation factor: node -> location relative pose = T_body_loc.
        let graph_pos = self.graph.len();
        self.graph
            .add_between_pose3(
                node_idx as u64,
                loc_key,
                &Pose3 {
                    rotation: constraint.rotation_body_loc,
                    translation: constraint.translation_body_loc,
                },
                noise,
            )
            .expect("gtsam: add location constraint factor");
        self.staged_constraint_factors.push(StagedConstraintFactor {
            instance_id: constraint.constraint_instance_id.clone(),
            graph_pos,
        });
    }

    /// Stage cached loop pairs as between factors, run the iSAM2 update
    /// sequence (with revision removals and
    /// the extra relinearization passes a closure needs), commit staged
    /// constraint-factor indices, then refresh all keyframe globals and the
    /// rotation/translation offsets from the best estimate.
    pub fn smooth_and_update(&mut self) {
        let smooth_t0 = std::time::Instant::now();
        let cache_pairs = std::mem::take(&mut self.cache_pairs);
        // Each cache pair was pushed to `committed_loops` in lockstep with
        // `commit_loop_pair`, so the drained pairs are exactly the tail of
        // `committed_loops`. Record, per pair, its committed index and the graph
        // position of its between factor so the iSAM2 factor index it is assigned
        // can be captured after the update (Approach B: background GNC can then
        // remove an outlier loop's factor by that index).
        let first_new_loop_index = self.committed_loops.len() - cache_pairs.len();
        let mut loop_factor_positions: Vec<(usize, usize)> = Vec::with_capacity(cache_pairs.len());
        // 添加回环因子 (add the loop factors). A loop that would move the map waits for a GNC
        // solve to accept it (see `step_gnc`) instead of going in now.
        for (offset, pair) in cache_pairs.iter().enumerate() {
            if !self.committed_loops[first_new_loop_index + offset].active_in_isam2 {
                continue;
            }
            let robust;
            let noise: &NoiseModel = if self.config.loop_robust_kernel {
                robust = NoiseModel::robust_huber(self.config.loop_robust_huber_k, &pair.noise);
                &robust
            } else {
                &pair.noise
            };
            loop_factor_positions.push((first_new_loop_index + offset, self.graph.len()));
            self.graph
                .add_between_pose3(
                    pair.target_id as u64,
                    pair.source_id as u64,
                    &Pose3 {
                        rotation: pair.rotation_offset,
                        translation: pair.translation_offset,
                    },
                    noise,
                )
                .expect("gtsam: add loop between factor");
        }
        drop(cache_pairs);
        // A loop GNC accepted goes in with its own ICP noise, as it would have gone in directly.
        for edge_index in std::mem::take(&mut self.accepted_loop_inserts) {
            let loop_edge = self.committed_loops[edge_index];
            let base =
                NoiseModel::diagonal_variances(&loop_variances(&self.config, loop_edge.score));
            let robust;
            let noise: &NoiseModel = if self.config.loop_robust_kernel {
                robust = NoiseModel::robust_huber(self.config.loop_robust_huber_k, &base);
                &robust
            } else {
                &base
            };
            loop_factor_positions.push((edge_index, self.graph.len()));
            self.graph
                .add_between_pose3(
                    loop_edge.target_id as u64,
                    loop_edge.source_id as u64,
                    &Pose3 {
                        rotation: loop_edge.rotation_offset,
                        translation: loop_edge.translation_offset,
                    },
                    noise,
                )
                .expect("gtsam: add accepted loop between factor");
        }
        let loop_factors_added = !loop_factor_positions.is_empty();
        // Smooth and map; removeFactorIndices applies constraint revision.
        let remove = std::mem::take(&mut self.pending_removals);
        // A re-sighted location closes a loop just like a lidar closure, and a
        // factor removal reshapes the graph just as much as an insertion — both
        // need the extra relinearization passes below.
        let has_closure =
            loop_factors_added || self.location_closure || self.reseeded || !remove.is_empty();
        let new_factor_indices = match self
            .isam2
            .update(&self.graph, &self.initial_values, &remove)
        {
            Ok(indices) => indices,
            // On a pathological graph (many conflicting closures mid-rejection)
            // GTSAM can throw and leave iSAM2 unusable; rebuilding from the
            // backbone + still-inlier loops is the only recovery. Location
            // constraints would be silently dropped by the rebuild, so with
            // them enabled this stays fatal.
            Err(gtsam_error) => {
                if self.config.use_location_constraints {
                    panic!("gtsam: isam2 update: {gtsam_error:?}");
                }
                eprintln!("gtsam: isam2 update failed ({gtsam_error:?}); rebuilding from backbone");
                self.graph.clear();
                self.initial_values.clear();
                self.staged_constraint_factors.clear();
                self.rebuild_isam2_from_backbone();
                Vec::new()
            }
        };
        // Refinement passes are best-effort: the estimate is still usable if a
        // forced relinearization throws, so log and stop refining.
        let refinement_passes = if has_closure {
            self.config.loop_refinement_passes.max(1)
        } else {
            1
        };
        let max_passes = if has_closure {
            self.config
                .loop_refinement_max_passes
                .max(refinement_passes)
        } else {
            refinement_passes
        };
        let mut previous_translations: Option<Vec<Vec3>> = None;
        for pass in 0..max_passes {
            if let Err(gtsam_error) = self.isam2.update_empty() {
                eprintln!(
                    "gtsam: relinearization pass failed ({gtsam_error:?}); \
                     skipping remaining passes"
                );
                break;
            }
            if pass + 1 < refinement_passes {
                continue;
            }
            let Ok(estimate) = self.isam2.calculate_best_estimate() else {
                break;
            };
            let translations: Vec<Vec3> = (0..self.key_poses.len())
                .filter_map(|i| estimate.pose3(i as u64).map(|pose| pose.translation))
                .collect();
            let settled = previous_translations.as_ref().is_some_and(|previous| {
                previous.iter().zip(&translations).all(|(before, after)| {
                    mat3::norm(&mat3::sub(after, before)) <= self.config.loop_refinement_tolerance_m
                })
            });
            if settled {
                break;
            }
            previous_translations = Some(translations);
        }

        // Record the iSAM2 factor index assigned to each staged constraint
        // factor so a future revision (same instance id) can remove it.
        for staged in &self.staged_constraint_factors {
            if !staged.instance_id.is_empty() && staged.graph_pos < new_factor_indices.len() {
                self.committed_by_instance
                    .entry(staged.instance_id.clone())
                    .or_default()
                    .push(new_factor_indices[staged.graph_pos]);
            }
        }
        self.staged_constraint_factors.clear();
        self.location_closure = false;
        self.reseeded = false;
        if has_closure {
            for index in 0..self.committed_loops.len() {
                let edge = &self.committed_loops[index];
                if edge.active_in_isam2 && !self.loop_applied[index] {
                    self.loop_applied[index] = true;
                    let (source, target) = (edge.source_id, edge.target_id);
                    let confidence = edge.gnc_weight
                        * self.loop_fit_confidence.get(index).copied().unwrap_or(1.0);
                    let yank_z = self.loop_yank_z.get(index).copied().unwrap_or(f64::NAN);
                    self.push_edge(source, target, LoopEdgeState::Applied, confidence, yank_z);
                }
            }
        }

        // Approach B: remember each newly-inserted loop factor's iSAM2 index so a
        // background GNC classification can later remove it if it is an outlier.
        for (committed_index, graph_pos) in &loop_factor_positions {
            if *graph_pos < new_factor_indices.len() {
                self.committed_loops[*committed_index].isam2_factor_index =
                    Some(new_factor_indices[*graph_pos]);
                self.committed_loops[*committed_index].active_in_isam2 = true;
            }
        }

        self.graph.clear();
        self.initial_values.clear();

        // Update key poses from the best estimate.
        let estimate_values = self
            .isam2
            .calculate_best_estimate()
            .expect("gtsam: best estimate");
        for (i, key_pose) in self.key_poses.iter_mut().enumerate() {
            let pose = estimate_values
                .pose3(i as u64)
                .expect("gtsam: keyframe missing from best estimate");
            key_pose.rotation_global = pose.rotation;
            key_pose.translation_global = pose.translation;
        }
        // Update offset.
        let last_item = self.key_poses.last().unwrap();
        self.rotation_offset = mat3::mat_mul(
            &last_item.rotation_global,
            &mat3::transpose(&last_item.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &last_item.translation_global,
            &mat3::mat_vec(&self.rotation_offset, &last_item.translation_local),
        );

        self.timing.gtsam_s += smooth_t0.elapsed().as_secs_f64();
        self.timing.smooth_calls += 1;
    }

    /// Snapshot everything the batch GNC re-solve needs into a `Send` job: the
    /// keyframes' local poses (backbone + local seed) and the committed loops.
    fn build_gnc_job(&self) -> GncJob {
        let keyframes = self
            .key_poses
            .iter()
            .map(|key_pose| BackboneKeyframe {
                rotation_local: key_pose.rotation_local,
                translation_local: key_pose.translation_local,
                initial_rotation: key_pose.rotation_local,
                initial_translation: key_pose.translation_local,
            })
            .collect();
        GncJob {
            keyframes,
            loops: self.committed_loops.clone(),
            backbone_noise: BackboneNoise::from_config(&self.config),
            loop_gnc_var_scale: self.config.loop_gnc_var_scale,
            loop_gnc_inlier_probability: self.config.loop_gnc_inlier_probability,
            loop_noise: RebuildLoopNoise::from_config(&self.config),
            loop_gnc_confirmations: self.config.loop_gnc_confirmations,
        }
    }

    /// Advance the background GNC after a keyframe: the outstanding solve is folded in once the
    /// map has grown by `loop_gnc_apply_after_fraction` of its size at dispatch (waiting for it
    /// if it is not done), then a new one is dispatched if loops were committed since the last. The
    /// schedule counts keyframes, not time, so a replay gives the same graph however fast the
    /// machine is. Returns whether the keyframe poses changed.
    pub fn step_gnc(&mut self) -> bool {
        if !self.config.loop_gnc_deterministic {
            return self.poll_gnc();
        }
        let due = self
            .gnc_due_at
            .is_some_and(|due_at| self.key_poses.len() >= due_at);
        let folded = due && self.fold_gnc(true);
        if self.gnc_due_at.is_none() && self.gnc_has_work() {
            let keyframes = self.key_poses.len();
            let delay = (self.config.loop_gnc_apply_after_fraction * keyframes as f64).ceil();
            self.dispatch_gnc(keyframes + (delay as usize).max(1));
        }
        folded
    }

    /// Nondeterministic GNC (`loop_gnc_deterministic` off): fold the outstanding solve if it has
    /// finished, then dispatch the next if there is work, never waiting. Call it often (every
    /// worker pass); returns whether the keyframe poses changed.
    pub fn poll_gnc(&mut self) -> bool {
        let folded = self.gnc_due_at.is_some() && self.fold_gnc(false);
        if self.gnc_due_at.is_none() && self.gnc_has_work() {
            self.dispatch_gnc(self.key_poses.len());
        }
        folded
    }

    /// Bring the graph up to date with every committed loop: fold in the outstanding solve and,
    /// if loops were committed after it was dispatched, solve once more. For the end of a stream.
    /// Returns whether the keyframe poses changed.
    pub fn finish_gnc(&mut self) -> bool {
        let mut changed = false;
        while self.gnc_pending() {
            changed |= self.poll_finish_gnc(true);
        }
        changed
    }

    /// One step of `finish_gnc` that, unless `wait`, returns at once when the outstanding solve
    /// is not done yet, so an idle pipeline can keep publishing while it finishes.
    pub fn poll_finish_gnc(&mut self, wait: bool) -> bool {
        if self.gnc_due_at.is_some() {
            return self.fold_gnc(wait);
        }
        if self.gnc_has_work() {
            self.dispatch_gnc(self.key_poses.len());
        }
        false
    }

    /// Whether a GNC solve is wanted: loops were committed since the last, or one awaits
    /// confirmation by another solve.
    fn gnc_has_work(&self) -> bool {
        self.committed_loops.len() > self.gnc_loops_covered
            || self
                .committed_loops
                .iter()
                .any(|loop_edge| loop_edge.gnc_vote_streak > 0)
    }

    /// Whether a dispatched GNC solve has not been folded in yet, or loops await one.
    pub fn gnc_pending(&self) -> bool {
        self.gnc_due_at.is_some() || self.gnc_has_work()
    }

    /// Queue a batch GNC re-solve of the current graph on the background worker, to be folded
    /// in once the graph has `due_at` keyframes. No-op without the worker.
    fn dispatch_gnc(&mut self, due_at: usize) {
        let Some(worker) = &self.gnc_worker else {
            self.gnc_loops_covered = self.committed_loops.len();
            return;
        };
        let job = self.build_gnc_job();
        self.gnc_loops_covered = job.loops.len();
        if worker.job_sender.send(job).is_ok() {
            self.gnc_due_at = Some(due_at);
        }
    }

    /// Fold in the outstanding GNC solve (waiting for it if `wait`, else only if it is done):
    /// insert the loops it accepted into iSAM2 and remove the ones it rejected, so keyframes
    /// move only by what those loops imply, never to GNC's own pose estimate. Returns whether
    /// the keyframe poses changed.
    fn fold_gnc(&mut self, wait: bool) -> bool {
        let Some(worker) = &self.gnc_worker else {
            self.gnc_due_at = None;
            return false;
        };
        let result = if wait {
            worker.result_receiver.recv().ok()
        } else {
            match worker.result_receiver.try_recv() {
                Err(std::sync::mpsc::TryRecvError::Empty) => return false,
                received => received.ok(),
            }
        };
        self.gnc_due_at = None;
        let Some(result) = result else {
            return false;
        };
        let mut changed = false;
        let confirmations = self.config.loop_gnc_confirmations.max(1);
        let mut edges: Vec<(usize, usize, LoopEdgeState, f64, f64)> = Vec::new();
        for (edge_index, weight) in result.loop_weights.iter().enumerate() {
            let loop_edge = &mut self.committed_loops[edge_index];
            let fit = self
                .loop_fit_confidence
                .get(edge_index)
                .copied()
                .unwrap_or(1.0);
            let yank_z = self
                .loop_yank_z
                .get(edge_index)
                .copied()
                .unwrap_or(f64::NAN);
            edges.push((
                loop_edge.source_id,
                loop_edge.target_id,
                LoopEdgeState::Vote,
                *weight * fit,
                yank_z,
            ));
            let out_votes = &mut self.loop_out_votes[edge_index];
            if *weight < LOOP_GNC_INLIER_THRESHOLD && !loop_edge.active_in_isam2 {
                out_votes.0 += 1;
                if out_votes.0 >= confirmations && !out_votes.1 {
                    out_votes.1 = true;
                    edges.push((
                        loop_edge.source_id,
                        loop_edge.target_id,
                        LoopEdgeState::Rejected,
                        *weight * fit,
                        yank_z,
                    ));
                }
            } else {
                *out_votes = (0, false);
            }
            let accepted = *weight >= LOOP_GNC_INLIER_THRESHOLD;
            loop_edge.gnc_vote_streak = if accepted != loop_edge.active_in_isam2 {
                loop_edge.gnc_vote_streak + 1
            } else {
                0
            };
            let mut in_graph = loop_edge.active_in_isam2;
            if loop_edge.gnc_vote_streak >= confirmations {
                loop_edge.gnc_vote_streak = 0;
                if in_graph {
                    if let Some(factor_index) = loop_edge.isam2_factor_index {
                        self.pending_removals.push(factor_index);
                    }
                    loop_edge.active_in_isam2 = false;
                    self.loop_applied[edge_index] = false;
                    self.loop_out_votes[edge_index] = (0, true);
                } else {
                    self.accepted_loop_inserts.push(edge_index);
                }
                in_graph = !in_graph;
                changed = true;
                let state = if in_graph {
                    LoopEdgeState::Accepted
                } else {
                    LoopEdgeState::Rejected
                };
                edges.push((
                    loop_edge.source_id,
                    loop_edge.target_id,
                    state,
                    *weight * fit,
                    yank_z,
                ));
            }
            // A rebuild keeps exactly the loops in the graph once this fold is done.
            loop_edge.gnc_weight = if in_graph { 1.0 } else { 0.0 };
        }
        for (source, target, state, confidence, yank_z) in edges {
            self.push_edge(source, target, state, confidence, yank_z);
        }
        if changed {
            self.reseed_isam2_from_gnc(&result.poses);
            self.smooth_and_update();
        }
        changed
    }

    /// Rebuild iSAM2 from the backbone and the loops now marked inliers, seeded at GNC's
    /// solution for that loop set (keyframes added since the solve hang off its last pose by
    /// odometry). Inserting a changed loop set into the old estimate instead can settle in a
    /// different, wrong minimum hundreds of meters away.
    fn reseed_isam2_from_gnc(&mut self, gnc_poses: &[Pose3]) {
        let Some(last_solved) = gnc_poses.len().checked_sub(1) else {
            return;
        };
        let anchor = &self.key_poses[last_solved];
        let anchor_rotation_local_inverse = mat3::transpose(&anchor.rotation_local);
        let backbone: Vec<BackboneKeyframe> = self
            .key_poses
            .iter()
            .enumerate()
            .map(|(index, key_pose)| {
                let (initial_rotation, initial_translation) = match gnc_poses.get(index) {
                    Some(pose) => (pose.rotation, pose.translation),
                    None => {
                        let solved = &gnc_poses[last_solved];
                        let relative_rotation =
                            mat3::mat_mul(&anchor_rotation_local_inverse, &key_pose.rotation_local);
                        let relative_translation = mat3::mat_vec(
                            &anchor_rotation_local_inverse,
                            &mat3::sub(&key_pose.translation_local, &anchor.translation_local),
                        );
                        (
                            mat3::mat_mul(&solved.rotation, &relative_rotation),
                            mat3::add(
                                &mat3::mat_vec(&solved.rotation, &relative_translation),
                                &solved.translation,
                            ),
                        )
                    }
                };
                BackboneKeyframe {
                    rotation_local: key_pose.rotation_local,
                    translation_local: key_pose.translation_local,
                    initial_rotation,
                    initial_translation,
                }
            })
            .collect();
        self.accepted_loop_inserts.clear();
        let rebuilt = build_isam2_from_backbone(
            &backbone,
            &self.committed_loops,
            &BackboneNoise::from_config(&self.config),
            &RebuildLoopNoise::from_config(&self.config),
        );
        self.swap_in_rebuilt_isam2(rebuilt);
        self.reseeded = true;
    }

    /// Discard the incremental iSAM2 and rebuild it, blocking, from the odometry
    /// backbone plus every committed loop whose GNC weight still marks it an
    /// inlier (unclassified loops carry 1.0 and re-enter — the next GNC vets
    /// them), seeded at the current keyframe globals. Recovery path for a throwing
    /// update; adoption gets the same rebuild prebuilt from the worker.
    fn rebuild_isam2_from_backbone(&mut self) {
        let backbone: Vec<BackboneKeyframe> = self
            .key_poses
            .iter()
            .map(|key_pose| BackboneKeyframe {
                rotation_local: key_pose.rotation_local,
                translation_local: key_pose.translation_local,
                initial_rotation: key_pose.rotation_global,
                initial_translation: key_pose.translation_global,
            })
            .collect();
        let rebuilt = build_isam2_from_backbone(
            &backbone,
            &self.committed_loops,
            &BackboneNoise::from_config(&self.config),
            &RebuildLoopNoise::from_config(&self.config),
        );
        self.swap_in_rebuilt_isam2(rebuilt);
    }

    /// Replace the incremental iSAM2 with a rebuilt one, repoint the loop
    /// bookkeeping at its factor indices, and catch it up with whatever was
    /// added after its snapshot. Staged factors and pending removals refer to the
    /// old iSAM2 and are dropped; the old iSAM2 is freed off-thread.
    fn swap_in_rebuilt_isam2(&mut self, rebuilt: RebuiltIsam2) {
        retire_isam2(std::mem::replace(&mut self.isam2, rebuilt.isam2));
        self.pending_removals.clear();
        self.graph.clear();
        self.initial_values.clear();
        self.staged_constraint_factors.clear();
        for loop_edge in &mut self.committed_loops {
            loop_edge.active_in_isam2 = false;
            loop_edge.isam2_factor_index = None;
        }
        for (edge_index, factor_index) in rebuilt.loop_factor_indices {
            self.committed_loops[edge_index].isam2_factor_index = Some(factor_index);
            self.committed_loops[edge_index].active_in_isam2 = true;
        }
        self.catch_up_isam2(
            rebuilt.keyframe_count,
            rebuilt.loop_count,
            rebuilt.loops_included,
        );
    }

    /// Add to a swapped-in iSAM2 what came after its snapshot: backbone factors
    /// and values (at the re-anchored globals) for the newer keyframes, and the
    /// still-inlier loops smoothed since, with the rebuild's noise. Loops still
    /// in `cache_pairs` reach it through the next `smooth_and_update` as usual.
    fn catch_up_isam2(&mut self, keyframe_count: usize, loop_count: usize, loops_included: bool) {
        let smoothed_loop_count = self.committed_loops.len() - self.cache_pairs.len();
        if keyframe_count >= self.key_poses.len() && loop_count >= smoothed_loop_count {
            return;
        }
        let mut graph = FactorGraph::new();
        let mut values = Values::new();
        let backbone_noise = BackboneNoise::from_config(&self.config);
        let seeded_at_global = |key_pose: &KeyPoseWithCloud| BackboneKeyframe {
            rotation_local: key_pose.rotation_local,
            translation_local: key_pose.translation_local,
            initial_rotation: key_pose.rotation_global,
            initial_translation: key_pose.translation_global,
        };
        for idx in keyframe_count..self.key_poses.len() {
            let previous = idx
                .checked_sub(1)
                .map(|previous_idx| seeded_at_global(&self.key_poses[previous_idx]));
            add_backbone_keyframe(
                &mut graph,
                &mut values,
                previous.as_ref(),
                &seeded_at_global(&self.key_poses[idx]),
                idx as u64,
                &backbone_noise,
            );
        }
        let mut loop_positions: Vec<(usize, usize)> = Vec::new();
        if loops_included {
            let loop_noise = RebuildLoopNoise::from_config(&self.config);
            for edge_index in loop_count..smoothed_loop_count {
                let loop_edge = &self.committed_loops[edge_index];
                if loop_edge.gnc_weight < LOOP_GNC_INLIER_THRESHOLD {
                    continue;
                }
                loop_positions.push((edge_index, graph.len()));
                add_rebuild_loop_factor(&mut graph, loop_edge, &loop_noise);
            }
        }
        match self.isam2.update(&graph, &values, &[]) {
            Ok(new_factor_indices) => {
                for (edge_index, graph_position) in loop_positions {
                    if graph_position < new_factor_indices.len() {
                        self.committed_loops[edge_index].isam2_factor_index =
                            Some(new_factor_indices[graph_position]);
                        self.committed_loops[edge_index].active_in_isam2 = true;
                    }
                }
            }
            Err(gtsam_error) => {
                eprintln!("gtsam: isam2 catch-up failed ({gtsam_error:?}); rebuilding in place");
                self.rebuild_isam2_from_backbone();
            }
        }
    }

    /// Direct access to the ICP used for loop verification — handy for
    /// standalone experiments; the loop path calls `icp_point_to_point`
    /// with these same defaults.
    pub fn icp_params() -> IcpParams {
        IcpParams::default()
    }
}

/// Free an iSAM2 on a short-lived thread: tearing down a large Bayes tree would
/// stall the pipeline thread.
/// Write one candidate's ICP inputs and result (see `Config::loop_icp_dump_dir`).
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

fn retire_isam2(isam2: Isam2) {
    let _ = std::thread::Builder::new()
        .name("isam2-retire".to_string())
        .stack_size(GTSAM_THREAD_STACK_BYTES)
        .spawn(move || drop(isam2));
}

// Re-export the point-cloud helpers alongside the PGO for wiring layers.
pub use pointcloud::IcpResult;

/// A loop's own factor variances (rotation x3, translation x3): its ICP fitness, unless the
/// config fixes the rotation variance or floors the translation one.
fn loop_variances(config: &Config, score: f64) -> [f64; 6] {
    loop_score_variances(
        config.loop_rotation_variance,
        config.loop_translation_variance_floor,
        score,
    )
}

/// A loop factor's diagonal variances: rotation from `rotation_variance` (0 = the ICP
/// score), translation the score floored at `translation_floor`.
pub(crate) fn loop_score_variances(
    rotation_variance: f64,
    translation_floor: f64,
    score: f64,
) -> [f64; 6] {
    let rotation = if rotation_variance > 0.0 {
        rotation_variance
    } else {
        score
    };
    let translation = score.max(translation_floor);
    [
        rotation,
        rotation,
        rotation,
        translation,
        translation,
        translation,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const RADIUS: f64 = 20.0;
    const STEPS_PER_LAP: usize = 120;
    /// Enough drift that the lap's closing loop no longer fits the map within its noise.
    const LARGE_YAW_BIAS: f64 = 0.0012;

    /// Biased odometry around a circle (it spirals open by `yaw_bias` per step); one keyframe
    /// per step, each smoothed. Step `i` is truly at circle angle `i / STEPS_PER_LAP`.
    fn drive_biased(pgo: &mut GscPgo, steps: std::ops::Range<usize>, yaw_bias: f64) {
        for i in steps {
            let mut rotation = mat3::identity();
            let mut translation = [0.0; 3];
            let step_length = 2.0 * RADIUS * (std::f64::consts::PI / STEPS_PER_LAP as f64).sin();
            let turn = 2.0 * std::f64::consts::PI / STEPS_PER_LAP as f64 + yaw_bias;
            for _ in 0..i {
                translation = mat3::add(
                    &translation,
                    &mat3::mat_vec(&rotation, &[step_length, 0.0, 0.0]),
                );
                rotation = mat3::mat_mul(&rotation, &mat3::rot_z(turn));
            }
            let mut pose = PoseWithTime::new(rotation, translation);
            pose.set_time(i as i32, 0);
            let cloud_with_pose = CloudWithPose {
                cloud: None,
                pose,
                frame_id: "odom".to_string(),
            };
            assert!(pgo.add_key_pose(&cloud_with_pose));
            pgo.smooth_and_update();
        }
    }

    /// Commit a loop source -> target with relative pose `offset_xyz` (identity
    /// rotation) and smooth it in.
    fn close(pgo: &mut GscPgo, source_id: usize, target_id: usize, offset_xyz: Vec3) {
        let score = 0.05;
        pgo.commit_loop_pair(LoopPair {
            source_id,
            target_id,
            rotation_offset: mat3::identity(),
            translation_offset: offset_xyz,
            score,
            noise: NoiseModel::diagonal_variances(&[score; 6]),
        });
        pgo.smooth_and_update();
    }

    /// A search's diagnostics for a candidate that passed every gate.
    fn passed_search(pgo: &GscPgo, source_id: usize, target_id: usize) -> LoopCandidateDiag {
        let source = &pgo.key_poses()[source_id];
        let target = &pgo.key_poses()[target_id];
        let mut diag = LoopCandidateDiag::new(source_id, source.time, source.translation_global);
        diag.outcome = "accepted";
        diag.target_id = target_id as i64;
        diag.target_time = target.time;
        diag.target_xyz = target.translation_global;
        diag.scan_context_best = 0.2;
        diag.icp_fitness = 0.05;
        diag
    }

    #[test]
    fn a_loop_edge_is_proposed_before_gnc_judges_it_and_applied_after_it_is_accepted() {
        let mut pgo = GscPgo::new(Config::default());
        drive_biased(&mut pgo, 0..STEPS_PER_LAP + 1, LARGE_YAW_BIAS);
        let diag = passed_search(&pgo, STEPS_PER_LAP, 0);
        pgo.record_search(diag);
        close(&mut pgo, STEPS_PER_LAP, 0, [0.0; 3]);
        let diag = passed_search(&pgo, STEPS_PER_LAP / 2, 0);
        pgo.record_search(diag);
        close(&mut pgo, STEPS_PER_LAP / 2, 0, [0.5, 0.0, 0.0]);
        assert!(pgo.finish_gnc());
        let life = |source: usize| -> Vec<LoopEdgeState> {
            pgo.loop_edge_events()
                .iter()
                .filter(|e| {
                    e.source_id == source && e.target_id == 0 && e.state != LoopEdgeState::Vote
                })
                .map(|e| e.state)
                .collect()
        };
        use LoopEdgeState::*;
        assert_eq!(
            life(STEPS_PER_LAP),
            vec![Proposed, Committed, Accepted, Applied]
        );
        assert_eq!(life(STEPS_PER_LAP / 2), vec![Proposed, Committed, Rejected]);
        let votes = pgo
            .loop_edge_events()
            .iter()
            .filter(|e| e.state == Vote)
            .count();
        assert!(votes >= 2 * pgo.config().loop_gnc_confirmations.max(1) as usize);
        assert!(pgo
            .loop_edge_events()
            .iter()
            .all(|e| (0.0..=1.0).contains(&e.confidence)));
    }

    #[test]
    fn a_loop_that_would_move_the_map_waits_for_gnc() {
        let mut pgo = GscPgo::new(Config::default());
        drive_biased(&mut pgo, 0..STEPS_PER_LAP + 1, LARGE_YAW_BIAS);
        let before: Vec<Vec3> = pgo
            .key_poses()
            .iter()
            .map(|k| k.translation_global)
            .collect();
        close(&mut pgo, STEPS_PER_LAP, 0, [0.0; 3]);
        close(&mut pgo, STEPS_PER_LAP / 2, 0, [0.5, 0.0, 0.0]);
        let unmoved = pgo
            .key_poses()
            .iter()
            .zip(&before)
            .all(|(k, b)| mat3::norm(&mat3::sub(&k.translation_global, b)) < 1e-6);
        assert!(unmoved, "an unvetted loop moved the map");
        assert!(pgo.committed_loops().iter().all(|l| !l.active_in_isam2));
        assert!(pgo.finish_gnc());
        let active: Vec<bool> = pgo
            .committed_loops()
            .iter()
            .map(|l| l.active_in_isam2)
            .collect();
        assert_eq!(active, vec![true, false]);
        let gap = mat3::norm(&mat3::sub(
            &pgo.key_poses()[STEPS_PER_LAP].translation_global,
            &pgo.key_poses()[0].translation_global,
        ));
        assert!(gap < 0.5, "the accepted loop left the lap {gap} m open");
    }
}
