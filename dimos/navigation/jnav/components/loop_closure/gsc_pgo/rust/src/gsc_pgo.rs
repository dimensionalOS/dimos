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
    build_odometry_backbone, gnc_worker_loop, solve_gnc, BackboneKeyframe, BackboneNoise, GncJob,
    GncResult, GncWorker,
};
use crate::gtsam::{symbol_key, FactorGraph, Isam2, NoiseModel, Pose3, Values};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    self, cloud_degeneracy, icp_point_to_point, transform_cloud, voxel_downsample, IcpParams,
    PointCloud,
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
    /// Final GNC weight from `finalize_gnc` (~1 inlier / kept, ~0 rejected).
    /// 1.0 until the batch GNC pass runs (or if `loop_gnc_final` is off).
    pub gnc_weight: f64,
    /// Live path (Approach B): the iSAM2 factor index of this loop's between
    /// factor, captured the update it was inserted, so a later background GNC
    /// classification can remove it via `removeFactorIndices` if it is judged an
    /// outlier. `None` until the containing update commits.
    pub isam2_factor_index: Option<u64>,
    /// Live path (Approach B): whether this loop's between factor is currently in
    /// the iSAM2 graph. Set false once removed as a GNC-classified outlier.
    pub active_in_isam2: bool,
}

/// Max buffered long jumps kept at once. Oldest is evicted FIFO past this.
const LONG_JUMP_BUFFER_MAX_LEN: usize = 4;

/// gtsam's elimination/Bayes-tree code recurses proportionally to graph size;
/// the 2 MB default thread stack overflows on multi-thousand-keyframe runs.
pub const GTSAM_THREAD_STACK_BYTES: usize = 64 * 1024 * 1024;

/// Floor on the per-loop ICP score before it is used as a GNC factor variance,
/// so a zero-score closure still gets a finite (very tight) noise model.
pub(crate) const LOOP_GNC_MIN_VARIANCE: f64 = 1e-6;

/// Approach B: a background GNC weight below this classifies the loop as an
/// outlier, so its between factor is dropped from iSAM2. GNC's TLS weights are
/// near 0 (reject) or near 1 (keep), so the midpoint is a safe cut.
const LOOP_GNC_INLIER_THRESHOLD: f64 = 0.5;

/// A would-be long-jump loop closure parked in the buffer. It carries the same
/// constraint a `LoopPair` does, plus the rigid map-correction SE3 it implies so
/// buffered jumps can be tested for mutual agreement before any is committed.
struct PendingJump {
    pair: LoopPair,
    /// Position-independent translation of the implied rigid correction SE3
    /// (`t` in `refined = R * odom + t`). Comparable across jumps taken at
    /// different points, so a shared drift correction reads as agreement.
    correction_translation: Vec3,
    /// Rotation of the implied rigid correction SE3 (`R`).
    correction_rotation: Mat3,
    /// Index into `loop_diagnostics` so the outcome can be rewritten to
    /// "accept_buffered" when this jump is promoted.
    diag_index: usize,
}

/// Per-search loop-closure diagnostics
/// one record every time `search_for_loop_pairs` evaluates a keyframe, capturing the metric at each
/// gate and the outcome (accepted, or the name of the gate that rejected it).
/// Lets an offline eval reconstruct WHERE and WHY closures are accepted/rejected
#[derive(Debug, Clone)]
pub struct LoopCandidateDiag {
    /// Outcome: a short accepted jump is "accepted"; a long jump is "buffered"
    /// while parked and "accept_buffered" once agreement promotes it. Otherwise
    /// the rejecting gate — one of "throttle_min_duration", "no_structure",
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
        }
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
    pub min_loop_detect_duration: f64,
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
    /// Minimum keyframe-index separation between the two ends of a loop closure.
    /// A closure only corrects drift accumulated over the trajectory between the
    /// pair; a near-in-sequence pair spans negligible drift, so a "closure" there
    /// corrects nothing and can only inject error (structural alias). 0 disables.
    pub loop_min_id_gap: u64,
    /// Long-jump buffer: an accepted closure whose candidate_distance is within
    /// this many meters is committed to the graph instantly. A longer jump is
    /// parked in a buffer and only committed once enough independent jumps agree
    /// on the same correction (see the `loop_buffer_agreement_*` gates and
    /// `loop_buffer_min_agree`). <= 0 disables the buffer (every accept instant).
    pub loop_instant_accept_distance_m: f64,
    /// Agreement gate: the two jumps' implied rigid map-correction SE3 must have
    /// translations (position-independent `t`) within this many meters.
    pub loop_buffer_agreement_trans_m: f64,
    /// Agreement gate: the two jumps' implied correction rotations must match
    /// within this many degrees.
    pub loop_buffer_agreement_rot_deg: f64,
    /// Number of mutually-agreeing buffered jumps required before the whole
    /// agreeing set is promoted to real graph constraints.
    pub loop_buffer_min_agree: u64,
    /// Robust (M-estimator) kernel wrapping all loop factors.
    pub loop_robust_kernel: bool,
    pub loop_robust_huber_k: f64,
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
            min_loop_detect_duration: 2.0,
            loop_candidate_max_distance_m: 0.0,
            min_descriptor_std: 0.0,
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_max_lowe_ratio: 0.0,
            loop_max_yank_rotation_deg: 0.0,
            loop_yank_gate_max_distance_m: 0.0,
            loop_min_id_gap: 0,
            loop_instant_accept_distance_m: 0.0,
            loop_buffer_agreement_trans_m: 1.0,
            loop_buffer_agreement_rot_deg: 10.0,
            loop_buffer_min_agree: 2,
            loop_robust_kernel: true,
            loop_robust_huber_k: 1.345,
            loop_gnc_final: true,
            loop_gnc_var_scale: 10.0,
            loop_gnc_inlier_probability: 0.99,
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
        let (instant_accept, min_agree, candidate_max) = match level {
            0..=2 => (0.0, 2, 0.0),
            3 => (10.0, 2, 200.0),
            _ => (3.0, 3, 50.0),
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
        self.loop_instant_accept_distance_m = instant_accept;
        self.loop_buffer_min_agree = min_agree;
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
    pending_long_jumps: Vec<PendingJump>,
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
    location_closure: bool,
    timing: TimingStats,
    loop_diagnostics: Vec<LoopCandidateDiag>,
    /// Background batch-GNC worker (spawned only when `loop_gnc_final` is on).
    /// The live path dispatches a re-solve here after each closure instead of
    /// blocking on it inline; `None` disables the async path (offline harness
    /// still calls the blocking `finalize_gnc`).
    gnc_worker: Option<GncWorker>,
    gnc_sequence_dispatched: u64,
    gnc_sequence_applied: u64,
    gnc_sequence_adopted: u64,
    /// Newest finished GNC solve, kept for idle adoption: the per-keyframe
    /// classification poll drains the worker channel, so without this the
    /// batch poses would be gone by the time the pipeline goes idle.
    latest_gnc_result: Option<GncResult>,
    /// Set when batch poses were adopted without rebuilding iSAM2 yet; the
    /// rebuild is deferred so the pipeline thread can publish the adopted
    /// poses before blocking on the (40-90s) rebuild.
    isam2_stale: bool,
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
            gnc_sequence_dispatched: 0,
            gnc_sequence_applied: 0,
            gnc_sequence_adopted: 0,
            latest_gnc_result: None,
            isam2_stale: false,
            key_poses: Vec::new(),
            history_pairs: Vec::new(),
            committed_loops: Vec::new(),
            cache_pairs: Vec::new(),
            pending_long_jumps: Vec::new(),
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
            location_closure: false,
            timing: TimingStats::default(),
            loop_diagnostics: Vec::new(),
        }
    }

    /// All per-search loop-closure diagnostic records accumulated so far.
    pub fn loop_diagnostics(&self) -> &[LoopCandidateDiag] {
        &self.loop_diagnostics
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
        self.insert_pose_node(
            &cloud_with_pose.pose,
            cloud_with_pose.cloud.clone(),
            &cloud_with_pose.frame_id,
        );
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
                    self.loop_diagnostics.push(diag);
                    return;
                }
            }
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
                self.loop_diagnostics.push(diag);
                return;
            }
            if self.config.loop_min_occupancy > 0
                && diag.descriptor_occupancy < self.config.loop_min_occupancy
            {
                diag.outcome = "low_occupancy";
                self.loop_diagnostics.push(diag);
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
            self.loop_diagnostics.push(diag);
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
            self.loop_diagnostics.push(diag);
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
            self.loop_diagnostics.push(diag);
            return;
        }

        // Positional-plausibility gate on scan-context false matches.
        if self.config.loop_candidate_max_distance_m > 0.0
            && diag.candidate_distance > self.config.loop_candidate_max_distance_m
        {
            diag.outcome = "distance_reject";
            self.loop_diagnostics.push(diag);
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

        let t0 = std::time::Instant::now();
        let target_cloud = self.get_sub_map(
            loop_idx as i32,
            self.config.loop_submap_half_range,
            self.config.submap_resolution,
        );
        let source_cloud = self.get_sub_map(cur_idx as i32, 0, self.config.submap_resolution);
        self.timing.submap_s += t0.elapsed().as_secs_f64();

        let t0 = std::time::Instant::now();
        let icp = icp_point_to_point(
            &source_cloud,
            &target_cloud,
            &init_rotation,
            &init_translation,
            &IcpParams::default(),
        );
        self.timing.icp_s += t0.elapsed().as_secs_f64();
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
            self.loop_diagnostics.push(diag);
            return;
        }

        if !icp.converged || icp.fitness > self.config.loop_score_thresh {
            diag.outcome = "icp_reject";
            self.loop_diagnostics.push(diag);
            return;
        }

        let score = icp.fitness;
        let rotation_refined =
            mat3::mat_mul(&icp.rotation, &self.key_poses[cur_idx].rotation_global);
        let translation_refined = mat3::add(
            &mat3::mat_vec(&icp.rotation, &self.key_poses[cur_idx].translation_global),
            &icp.translation,
        );
        let loop_rotation_transpose = mat3::transpose(&self.key_poses[loop_idx].rotation_global);
        let rotation_offset = mat3::mat_mul(&loop_rotation_transpose, &rotation_refined);
        let translation_offset = mat3::mat_vec(
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

        if self.config.loop_max_yank_rotation_deg > 0.0
            && diag.candidate_distance < self.config.loop_yank_gate_max_distance_m
            && diag.yank_rotation_deg > self.config.loop_max_yank_rotation_deg
        {
            diag.outcome = "yank_reject";
            self.loop_diagnostics.push(diag);
            return;
        }

        // Original isotropic noise = ICP fitness on all 6 DOF.
        let noise = NoiseModel::diagonal_variances(&[score; 6]);
        let pair = LoopPair {
            source_id: cur_idx,
            target_id: loop_idx,
            rotation_offset,
            translation_offset,
            score,
            noise,
        };

        // Short jumps carry almost no drift correction and are trusted on their
        // own; commit them instantly. A long jump could be a single spurious
        // yank, so it is parked in the buffer until enough independent jumps
        // agree on the same correction.
        let buffer_active = self.config.loop_instant_accept_distance_m > 0.0;
        if !buffer_active || diag.candidate_distance <= self.config.loop_instant_accept_distance_m {
            diag.outcome = "accepted";
            self.loop_diagnostics.push(diag);
            self.commit_loop_pair(pair);
            return;
        }

        // The rigid map-correction this constraint implies, as an SE3 transform
        // (R, t) mapping current odom pose -> ICP-refined pose:
        //   refined = R * odom + t   =>   t = refined - R * odom
        // Recovering t this way makes it position-independent: jumps taken from
        // different points along a return leg that imply the SAME rigid drift
        // correction share the same (R, t), even when their raw
        // (refined - odom) offsets differ because of the rotation. That is what
        // lets independent revisit jumps reinforce each other.
        let correction_rotation = mat3::mat_mul(
            &rotation_refined,
            &mat3::transpose(&self.key_poses[cur_idx].rotation_global),
        );
        let correction_translation = mat3::sub(
            &translation_refined,
            &mat3::mat_vec(
                &correction_rotation,
                &self.key_poses[cur_idx].translation_global,
            ),
        );
        diag.outcome = "buffered";
        let diag_index = self.loop_diagnostics.len();
        self.loop_diagnostics.push(diag);
        self.pending_long_jumps.push(PendingJump {
            pair,
            correction_translation,
            correction_rotation,
            diag_index,
        });
        self.try_promote_pending();
        while self.pending_long_jumps.len() > LONG_JUMP_BUFFER_MAX_LEN {
            self.pending_long_jumps.remove(0);
        }
    }

    /// Commit an accepted loop pair to the graph (between-factor cache) and the
    /// history used for closure throttling.
    fn commit_loop_pair(&mut self, pair: LoopPair) {
        self.history_pairs.push((pair.target_id, pair.source_id));
        self.committed_loops.push(CommittedLoop {
            source_id: pair.source_id,
            target_id: pair.target_id,
            rotation_offset: pair.rotation_offset,
            translation_offset: pair.translation_offset,
            score: pair.score,
            gnc_weight: 1.0,
            isam2_factor_index: None,
            active_in_isam2: true,
        });
        self.cache_pairs.push(pair);
    }

    /// After a long jump is parked, check whether the newest buffered jump now
    /// has at least `loop_buffer_min_agree` mutually-agreeing peers (itself
    /// included). If so, promote all agreeing jumps to real constraints and mark
    /// their diagnostics "accept_buffered".
    fn try_promote_pending(&mut self) {
        let min_agree = self.config.loop_buffer_min_agree.max(1) as usize;
        if self.pending_long_jumps.len() < min_agree {
            return;
        }
        let (anchor_translation, anchor_rotation) = {
            let anchor = self.pending_long_jumps.last().unwrap();
            (anchor.correction_translation, anchor.correction_rotation)
        };
        let trans_tol = self.config.loop_buffer_agreement_trans_m;
        let rot_tol = self.config.loop_buffer_agreement_rot_deg;
        let agreeing: Vec<usize> = (0..self.pending_long_jumps.len())
            .filter(|&index| {
                let jump = &self.pending_long_jumps[index];
                mat3::norm(&mat3::sub(
                    &anchor_translation,
                    &jump.correction_translation,
                )) <= trans_tol
                    && mat3::angular_distance(&anchor_rotation, &jump.correction_rotation)
                        .to_degrees()
                        <= rot_tol
            })
            .collect();
        if agreeing.len() < min_agree {
            return;
        }
        for &index in agreeing.iter().rev() {
            let jump = self.pending_long_jumps.remove(index);
            self.loop_diagnostics[jump.diag_index].outcome = "accept_buffered";
            self.commit_loop_pair(jump.pair);
        }
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
        let has_loop = !cache_pairs.is_empty();
        // Each cache pair was pushed to `committed_loops` in lockstep with
        // `commit_loop_pair`, so the drained pairs are exactly the tail of
        // `committed_loops`. Record, per pair, its committed index and the graph
        // position of its between factor so the iSAM2 factor index it is assigned
        // can be captured after the update (Approach B: background GNC can then
        // remove an outlier loop's factor by that index).
        let first_new_loop_index = self.committed_loops.len() - cache_pairs.len();
        let mut loop_factor_positions: Vec<(usize, usize)> = Vec::with_capacity(cache_pairs.len());
        // 添加回环因子 (add the loop factors)
        for (offset, pair) in cache_pairs.iter().enumerate() {
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
        // Smooth and map; removeFactorIndices applies constraint revision.
        let remove = std::mem::take(&mut self.pending_removals);
        // A re-sighted location closes a loop just like a lidar closure, and a
        // factor removal reshapes the graph just as much as an insertion — both
        // need the extra relinearization passes below.
        let has_closure = has_loop || self.location_closure || !remove.is_empty();
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
        let refinement_passes = if has_closure { 5 } else { 1 };
        for _ in 0..refinement_passes {
            if let Err(gtsam_error) = self.isam2.update_empty() {
                eprintln!(
                    "gtsam: relinearization pass failed ({gtsam_error:?}); \
                     skipping remaining passes"
                );
                break;
            }
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

        // Approach B: remember each newly-inserted loop factor's iSAM2 index so a
        // background GNC classification can later remove it if it is an outlier.
        for (committed_index, graph_pos) in &loop_factor_positions {
            if *graph_pos < new_factor_indices.len() {
                self.committed_loops[*committed_index].isam2_factor_index =
                    Some(new_factor_indices[*graph_pos]);
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
            sequence: self.gnc_sequence_dispatched,
        }
    }

    /// Write a finished GNC re-solve back into the keyframes and loop weights.
    /// The result covers the first `result.keyframe_globals.len()` keyframes;
    /// any keyframes added while the solve ran (async path) keep their local
    /// dead-reckoned shape but are rigidly re-anchored onto the corrected tail so
    /// the trajectory stays continuous.
    fn apply_gnc_result(&mut self, result: &GncResult) {
        let solved_count = result.keyframe_globals.len();
        for (idx, (rotation_global, translation_global)) in
            result.keyframe_globals.iter().enumerate()
        {
            self.key_poses[idx].rotation_global = *rotation_global;
            self.key_poses[idx].translation_global = *translation_global;
        }
        for (edge_idx, weight) in result.loop_weights.iter().enumerate() {
            if edge_idx < self.committed_loops.len() {
                self.committed_loops[edge_idx].gnc_weight = *weight;
            }
        }
        if solved_count >= 1 && solved_count < self.key_poses.len() {
            let anchor = &self.key_poses[solved_count - 1];
            let correction_rotation = mat3::mat_mul(
                &anchor.rotation_global,
                &mat3::transpose(&anchor.rotation_local),
            );
            let correction_translation = mat3::sub(
                &anchor.translation_global,
                &mat3::mat_vec(&correction_rotation, &anchor.translation_local),
            );
            for key_pose in &mut self.key_poses[solved_count..] {
                key_pose.rotation_global =
                    mat3::mat_mul(&correction_rotation, &key_pose.rotation_local);
                key_pose.translation_global = mat3::add(
                    &mat3::mat_vec(&correction_rotation, &key_pose.translation_local),
                    &correction_translation,
                );
            }
        }
        let last_item = self.key_poses.last().unwrap();
        self.rotation_offset = mat3::mat_mul(
            &last_item.rotation_global,
            &mat3::transpose(&last_item.rotation_local),
        );
        self.translation_offset = mat3::sub(
            &last_item.translation_global,
            &mat3::mat_vec(&self.rotation_offset, &last_item.translation_local),
        );
    }

    /// Blocking full-trajectory batch GNC re-solve: rebuild the odometry backbone
    /// plus every committed loop closure and run GNC (TLS loss) with the backbone
    /// pinned as known inliers. GNC jointly classifies the closures against the
    /// global residual, rejecting the mutually conflicting ones that the
    /// incremental Huber pass merely down-weights (so a loop that only
    /// under-closes incrementally actually snaps shut). The best estimate is
    /// written back to every keyframe and each closure's GNC weight is stored.
    /// Used by the offline harness, which runs it once at the end (all keyframes
    /// present, no more added, so the pose overwrite is safe). The live path uses
    /// `dispatch_gnc_async` + `poll_and_apply_gnc_classification` instead: iSAM2
    /// stays authoritative for poses and GNC only classifies loops off-thread.
    /// No-op when `loop_gnc_final` is off or no closures exist.
    pub fn finalize_gnc(&mut self) {
        if !self.config.loop_gnc_final || self.committed_loops.is_empty() {
            return;
        }
        let solve_t0 = std::time::Instant::now();
        self.gnc_sequence_dispatched += 1;
        let job = self.build_gnc_job();
        let result = solve_gnc(&job);
        self.apply_gnc_result(&result);
        self.gnc_sequence_applied = result.sequence;
        self.timing.gtsam_s += solve_t0.elapsed().as_secs_f64();
    }

    /// Live path: queue a fresh batch GNC re-solve on the background worker and
    /// return immediately. The heavy full-graph solve runs off the pipeline
    /// thread; the worker coalesces bursts, so a storm of closures collapses to a
    /// single solve. No-op when the async worker is disabled or no closures exist.
    pub fn dispatch_gnc_async(&mut self) {
        if self.committed_loops.is_empty() {
            return;
        }
        if self.gnc_worker.is_some() {
            self.gnc_sequence_dispatched += 1;
        }
        let job = self.build_gnc_job();
        if let Some(worker) = &self.gnc_worker {
            let _ = worker.job_sender.send(job);
        }
    }

    /// Whether a dispatched background GNC classification has not yet been
    /// applied — i.e. the latest full-graph verdict is still pending. The
    /// harness (and any shutdown path) should keep the pipeline alive while
    /// this is true or stale outlier loops stay committed.
    pub fn gnc_in_flight(&self) -> bool {
        self.gnc_sequence_applied < self.gnc_sequence_dispatched
    }

    /// Live path (Approach B): fold in the newest finished background GNC
    /// classification, if any (dropping superseded results). iSAM2 stays
    /// authoritative for poses — this NEVER overwrites keyframe globals. It only
    /// reads GNC's per-loop weights and, for any loop GNC now judges an outlier,
    /// queues removal of that loop's iSAM2 between factor (applied on the next
    /// `smooth_and_update` via `removeFactorIndices`). Returns whether any loop
    /// was newly removed. Cheap — a channel drain plus a weight scan — so it is
    /// safe to call every keyframe without blocking the pipeline thread.
    ///
    /// Removal is one-way for now: a loop dropped as an outlier is not re-added if
    /// a later GNC pass reclassifies it as an inlier (re-adding needs the original
    /// ICP noise model, which the plain-data `CommittedLoop` does not retain).
    /// GNC's joint full-graph classification is stable, so this is a safe first
    /// cut; re-admission is a future refinement.
    pub fn poll_and_apply_gnc_classification(&mut self) -> bool {
        let mut latest_result: Option<GncResult> = None;
        if let Some(worker) = &self.gnc_worker {
            while let Ok(result) = worker.result_receiver.try_recv() {
                latest_result = Some(result);
            }
        }
        let Some(result) = latest_result else {
            return false;
        };
        self.gnc_sequence_applied = self.gnc_sequence_applied.max(result.sequence);
        let mut removed_any = false;
        for (edge_index, weight) in result.loop_weights.iter().enumerate() {
            if edge_index >= self.committed_loops.len() {
                break;
            }
            self.committed_loops[edge_index].gnc_weight = *weight;
            let is_outlier = *weight < LOOP_GNC_INLIER_THRESHOLD;
            let loop_edge = &self.committed_loops[edge_index];
            if is_outlier && loop_edge.active_in_isam2 {
                if let Some(factor_index) = loop_edge.isam2_factor_index {
                    self.pending_removals.push(factor_index);
                    self.committed_loops[edge_index].active_in_isam2 = false;
                    removed_any = true;
                }
            }
        }
        self.latest_gnc_result = Some(result);
        removed_any
    }

    /// Idle path: adopt the newest background GNC solve wholesale — batch
    /// poses written back, iSAM2 marked stale — because iSAM2 linearization
    /// never recovers from mass false-closure removal. Uses the stored result
    /// if the per-keyframe classification poll already drained the channel.
    /// The caller publishes right after, then triggers the deferred rebuild
    /// via `rebuild_isam2_if_stale`. Rebuild drops location-constraint
    /// factors, so callers gate on `use_location_constraints`.
    pub fn poll_and_adopt_gnc_solution(&mut self) -> bool {
        if let Some(worker) = &self.gnc_worker {
            while let Ok(result) = worker.result_receiver.try_recv() {
                self.gnc_sequence_applied = self.gnc_sequence_applied.max(result.sequence);
                self.latest_gnc_result = Some(result);
            }
        }
        let Some(result) = self.latest_gnc_result.take() else {
            return false;
        };
        if result.sequence <= self.gnc_sequence_adopted {
            return false;
        }
        self.gnc_sequence_adopted = result.sequence;
        self.apply_gnc_result(&result);
        self.isam2_stale = true;
        true
    }

    /// Deferred half of adoption: rebuild iSAM2 from the adopted poses. Call
    /// only when no newer GNC solve is in flight — the rebuild blocks the
    /// pipeline thread for tens of seconds, and doing it while a solve is
    /// pending would silence republishes right when they matter most.
    pub fn rebuild_isam2_if_stale(&mut self) -> bool {
        if !self.isam2_stale {
            return false;
        }
        self.rebuild_isam2_from_backbone();
        true
    }

    /// Discard the incremental iSAM2 and rebuild it from the odometry backbone
    /// plus every committed loop whose GNC weight still marks it an inlier
    /// (unclassified loops carry 1.0 and re-enter — the next GNC vets them),
    /// seeded at the current keyframe globals. Loop factors keep the same
    /// robust kernel the live path uses. If GTSAM throws even on the fresh
    /// graph (mid-rejection it can hold hundreds of conflicting false
    /// closures), fall back to the backbone alone — pure odometry always
    /// solves, and the next GNC adoption restores the vetted loops.
    fn rebuild_isam2_from_backbone(&mut self) {
        for include_loops in [true, false] {
            let mut graph = FactorGraph::new();
            let mut values = Values::new();
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
            build_odometry_backbone(
                &mut graph,
                &mut values,
                &backbone,
                &BackboneNoise::from_config(&self.config),
            );
            let mut kept_loop_positions: Vec<(usize, usize)> = Vec::new();
            if include_loops {
                for (edge_index, loop_edge) in self.committed_loops.iter().enumerate() {
                    if loop_edge.gnc_weight < LOOP_GNC_INLIER_THRESHOLD {
                        continue;
                    }
                    let variance =
                        loop_edge.score.max(LOOP_GNC_MIN_VARIANCE) * self.config.loop_gnc_var_scale;
                    let gaussian = NoiseModel::diagonal_variances(&[variance; 6]);
                    let robust;
                    let noise: &NoiseModel = if self.config.loop_robust_kernel {
                        robust =
                            NoiseModel::robust_huber(self.config.loop_robust_huber_k, &gaussian);
                        &robust
                    } else {
                        &gaussian
                    };
                    kept_loop_positions.push((edge_index, graph.len()));
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
            }
            let mut rebuilt_isam2 = Isam2::new();
            let new_factor_indices = match rebuilt_isam2.update(&graph, &values, &[]) {
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
            self.isam2 = rebuilt_isam2;
            self.pending_removals.clear();
            self.graph.clear();
            self.initial_values.clear();
            for loop_edge in &mut self.committed_loops {
                loop_edge.active_in_isam2 = false;
                loop_edge.isam2_factor_index = None;
            }
            for (edge_index, graph_position) in kept_loop_positions {
                if graph_position < new_factor_indices.len() {
                    self.committed_loops[edge_index].isam2_factor_index =
                        Some(new_factor_indices[graph_position]);
                    self.committed_loops[edge_index].active_in_isam2 = true;
                }
            }
            self.isam2_stale = false;
            return;
        }
    }

    /// Direct access to the ICP used for loop verification — handy for
    /// standalone experiments; the loop path calls `icp_point_to_point`
    /// with these same defaults.
    pub fn icp_params() -> IcpParams {
        IcpParams::default()
    }
}

// Re-export the point-cloud helpers alongside the PGO for wiring layers.
pub use pointcloud::IcpResult;
