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

//! `GscPgo`: keyframe gating, the closure search and its gates, and the pose graph (odometry + gated closures), re-solved on a background thread when new closures arrive. Keyframe ids are indices; `mat3` is row-major f64.

use std::sync::Arc;

use crate::big_picture_fit::{closure_edges, Edge};
use crate::connection_fit::{dump_icp, IcpParams};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, voxel_downsample, KdTree, PointCloud};
use crate::pose_graph_solving::Solving;
use crate::similarity_discovery;
use crate::store::KeyFrames;
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
}

/// `planar_pose_graph`: a pose on the floor plane (x, y, yaw; z, roll and pitch dropped).
pub fn planar(rotation: &Mat3, translation: &Vec3) -> (Mat3, Vec3) {
    (
        mat3::yaw_and_tilt(rotation).0,
        [translation[0], translation[1], 0.0],
    )
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

/// A closure that passed every gate: relative pose (source in the target's frame), ICP fitness, newest solve's verdict.
#[derive(Clone, Copy)]
pub struct CommittedClosure {
    pub source_id: usize,
    pub target_id: usize,
    pub rotation_offset: Mat3,
    pub translation_offset: Vec3,
    pub score: f64,
    /// 1 kept, 0 rejected by the newest applied solve; NaN until a solve has judged it.
    pub gnc_weight: f64,
}

impl CommittedClosure {
    /// Whether the newest applied solve kept it.
    pub fn is_kept(&self) -> bool {
        self.gnc_weight > 0.5
    }
}

/// Per-search loop-closure diagnostics: each gate's metric and the outcome, so an offline eval sees where/why closures pass.
#[derive(Debug, Clone, serde::Serialize)]
pub struct ClosureCandidateDiag {
    /// "accepted", "outranked", "stale", "verification_dropped", or the rejecting gate's name (e.g. "icp_reject", "uninformative").
    pub outcome: &'static str,
    pub source_id: usize,
    pub source_time: f64,
    pub source_xyz: Vec3,
    /// Candidate target keyframe, or -1 if no candidate was found.
    pub target_id: i64,
    pub target_time: f64,
    pub target_xyz: Vec3,
    pub from_scan_context: bool,
    /// Occupied scan-context cells of the source keyframe (the occupancy gate input).
    pub descriptor_occupancy: i32,
    /// Best / second-best scan-context distances and their Lowe ratio (best/second); NaN for a position-fallback candidate.
    pub scan_context_best: f32,
    pub scan_context_second: f32,
    pub lowe_ratio: f32,
    /// Pressure at search start (0 before any); keep_rate = similar-yank judged closures' (kept+1)/(judged+2); look_alike_places 0 = not counted.
    pub pressure: f64,
    /// Odometry error estimate at search start, as ratios to the reported variances (xy, z, yaw, roll/pitch).
    pub learned_odometry_error_estimate: [f64; 4],
    /// The same as the uninformative gate measures the graph with (`GscPgo::odometry_error_estimate`, the stated odometry error estimate).
    pub odometry_error_estimate_gate: [f64; 4],
    /// Source keyframe's offset from where the graph put it, vs the target, world axes (m), and its graph covariance (m²); NaN before ICP.
    pub innovation: Vec3,
    pub innovation_covariance: Mat3,
    /// Chance the source scan overlaps the target keyframe's (`OverlapModel::overlap`); the extension budget scales with it.
    pub target_overlap: f64,
    pub keep_rate: f64,
    pub extension_budget: usize,
    pub look_alike_places: usize,
    /// Global distance between source and candidate keyframe.
    pub candidate_distance: f64,
    /// Minimum degeneracy (observability) of the source submap; -1 if not computed.
    pub degeneracy_min: f32,
    pub icp_converged: bool,
    /// Single ICP's iterations and stop reason (`IcpStop::name`; "" if ICP did not run).
    pub icp_iterations: u32,
    pub icp_stop: &'static str,
    /// How far ICP moved the source keyframe from the scan-context seed: distance (m) and rotation (deg); NaN if ICP did not run.
    pub icp_init_offset_m: f64,
    pub icp_init_offset_deg: f64,
    /// ICP fitness (the score-threshold gate input); NaN if ICP did not run.
    pub icp_fitness: f64,
    /// Fraction of the source scan that overlaps the target after ICP (see `IcpResult`); NaN if ICP did not run.
    pub icp_overlap: f64,
    /// Magnitude of the ICP translation correction, NaN if ICP did not run.
    pub icp_translation_norm: f64,
    /// Translation (m) / rotation (deg) between the ICP relative pose and the current graph's; a large yank flags a bad closure. NaN before ICP.
    pub yank_translation: f64,
    pub yank_rotation_deg: f64,
    /// Yank gate (NaN before ICP): yank_stdevs = sqrt(v^T (P + cI)^-1 v); confidence = factors' product; required = 1 - exp(-z²/2k²).
    pub yank_stdevs: f64,
    pub scan_context_factor: f64,
    pub icp_factor: f64,
    pub degeneracy_factor: f64,
    pub confidence: f64,
    pub required_confidence: f64,
    /// Extended ICP (`closure_extended_icp_max_keyframes`) stats (0 / NaN if not run): keyframes, cloud sizes, inliers, fitness, confidence.
    pub extended_keyframes: usize,
    pub extended_source_points: usize,
    pub extended_target_points: usize,
    pub single_inliers: usize,
    pub extended_inliers: usize,
    pub extended_fitness: f64,
    pub extended_confidence: f64,
    /// Extension confidence ratio per cloud (evidence / bar, yank units), single scan first; the candidate's score is their median.
    pub cloud_ratios: Vec<f64>,
    /// Each extension step's alignment distance from the previous at the source keyframe (inlier distances; `compounding_agreement_sensitivity`).
    pub cloud_agreement: Vec<f64>,
    /// Search cost in seconds per step (NaN where it did not run; lookup 0 for a retry), for fitting pre-search cost estimates.
    pub cost_lookup_s: f64,
    pub cost_begin_s: f64,
    pub cost_submap_s: f64,
    pub cost_degeneracy_s: f64,
    pub cost_icp_s: f64,
    pub cost_extension_s: f64,
    pub cost_extension_rounds_s: Vec<f64>,
    pub cost_gates_s: f64,
    pub cost_search_s: f64,
    /// Known before the search: map size, scan-context candidates, alternatives (try 1 = best), clouds, ICP points, speed (m/s), scan indices.
    pub map_keyframes: usize,
    pub scan_context_candidates: usize,
    pub alternatives_left: usize,
    pub alternative_try: usize,
    pub extension_clouds_planned: usize,
    pub extension_clouds_used: usize,
    pub single_source_points: usize,
    pub single_target_points: usize,
    pub single_target_points_thinned: usize,
    pub speed_mps: f64,
    pub scan_index_proposed: usize,
    pub scan_index_finished: usize,
    /// Deterministic schedule: scan count at dispatch, its estimate (raw, and in scans after dispatch), and whether it overshot.
    pub scan_index_dispatched: usize,
    pub estimate_raw_scans: f64,
    pub estimate_scans: usize,
    pub search_overshoot: bool,
    /// Points the dispatch estimate counted: the voxelized source keyframe, and the target submap's keyframes summed before merging.
    pub dispatch_source_points: usize,
    pub dispatch_target_points: usize,
    /// Two-stage search (`closure_search_schedule`): single-scan confidence ratio (verification rank), verification schedule, per-round points.
    pub promise: f64,
    pub verification_scan_index_dispatched: usize,
    pub verification_estimate_raw_scans: f64,
    pub verification_estimate_scans: usize,
    pub verification_overshoot: bool,
    pub extension_rounds_source_points: Vec<usize>,
    pub extension_rounds_target_points: Vec<usize>,
    pub agreement_multiplier: f64,
    /// `connection_fit::likelihood_ratio` (mean log ratio, yank units) of the single and last extended alignment, and its width; NaN unscored.
    pub likelihood_ratio: f64,
    pub extended_likelihood_ratio: f64,
    pub likelihood_width: f64,
}
impl ClosureCandidateDiag {
    fn new(source_id: usize, source_time: f64, source_xyz: Vec3) -> ClosureCandidateDiag {
        ClosureCandidateDiag {
            outcome: "no_candidate",
            source_id,
            source_time,
            source_xyz,
            target_id: -1,
            target_time: f64::NAN,
            target_xyz: [f64::NAN; 3],
            from_scan_context: false,
            descriptor_occupancy: -1,
            scan_context_best: f32::NAN,
            scan_context_second: f32::NAN,
            lowe_ratio: f32::NAN,
            pressure: 0.0,
            learned_odometry_error_estimate: [1.0; 4],
            odometry_error_estimate_gate: [1.0; 4],
            innovation: [f64::NAN; 3],
            innovation_covariance: [[f64::NAN; 3]; 3],
            target_overlap: 0.0,
            keep_rate: f64::NAN,
            extension_budget: 0,
            look_alike_places: 0,
            candidate_distance: f64::NAN,
            degeneracy_min: -1.0,
            icp_converged: false,
            icp_iterations: 0,
            icp_stop: "",
            icp_init_offset_m: f64::NAN,
            icp_init_offset_deg: f64::NAN,
            icp_fitness: f64::NAN,
            icp_overlap: f64::NAN,
            icp_translation_norm: f64::NAN,
            yank_translation: f64::NAN,
            yank_rotation_deg: f64::NAN,
            yank_stdevs: f64::NAN,
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
            cloud_ratios: Vec::new(),
            cloud_agreement: Vec::new(),
            cost_lookup_s: f64::NAN,
            cost_begin_s: f64::NAN,
            cost_submap_s: f64::NAN,
            cost_degeneracy_s: f64::NAN,
            cost_icp_s: f64::NAN,
            cost_extension_s: f64::NAN,
            cost_extension_rounds_s: Vec::new(),
            cost_gates_s: f64::NAN,
            cost_search_s: f64::NAN,
            map_keyframes: 0,
            scan_context_candidates: 0,
            alternatives_left: 0,
            alternative_try: 0,
            extension_clouds_planned: 0,
            extension_clouds_used: 0,
            single_source_points: 0,
            single_target_points: 0,
            single_target_points_thinned: 0,
            speed_mps: f64::NAN,
            scan_index_proposed: 0,
            scan_index_finished: 0,
            scan_index_dispatched: 0,
            estimate_raw_scans: f64::NAN,
            estimate_scans: 0,
            search_overshoot: false,
            dispatch_source_points: 0,
            dispatch_target_points: 0,
            promise: f64::NAN,
            verification_scan_index_dispatched: 0,
            verification_estimate_raw_scans: f64::NAN,
            verification_estimate_scans: 0,
            verification_overshoot: false,
            extension_rounds_source_points: Vec::new(),
            extension_rounds_target_points: Vec::new(),
            agreement_multiplier: 1.0,
            likelihood_ratio: f64::NAN,
            extended_likelihood_ratio: f64::NAN,
            likelihood_width: f64::NAN,
        }
    }
}

/// A closure candidate's lifecycle: proposed, gated, GNC's votes, and applied to the map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ClosureEdgeState {
    /// The search found a candidate keyframe for this one.
    Proposed = 0,
    /// A gate (distance, degeneracy, ICP, yank, ...) turned it down; it never reaches GNC.
    GateRejected = 1,
    /// Passed every gate; waits for GNC to vote it into the graph.
    Committed = 2,
    /// GNC voted it into the graph (3, a single GNC solve's weight, is retired).
    Accepted = 4,
    /// GNC voted it out of the graph (never in, or taken back out).
    Rejected = 5,
    /// The graph update that moved the map with it in force.
    Applied = 6,
}

/// One step in a closure candidate's life, for visualizing edges as they happen; observational only.
#[derive(Debug, Clone, Copy)]
pub struct ClosureEdgeEvent {
    /// Keyframe time of the newest keyframe when it happened.
    pub time: f64,
    pub source_id: usize,
    pub target_id: usize,
    pub state: ClosureEdgeState,
    /// How likely the edge is right in [0, 1]: scan-context similarity, then ICP confidence, then times GNC's weight.
    pub confidence: f64,
    /// How hard the edge pulls the accepted graph (`ClosureCandidateDiag::yank_stdevs`); NaN before ICP.
    pub yank_stdevs: f64,
}

/// Bytes per packed closure-edge record (`ClosureEdgeEvent::put_record`).
pub const CLOSURE_EDGE_RECORD_BYTES: usize = 52;

impl ClosureEdgeEvent {
    /// Append one packed record: source then target xyz (f32, current pose), ids u32, state u32, confidence f32, yank_stdevs f32, time f64.
    pub fn put_record(&self, data: &mut Vec<u8>, key_poses: &KeyFrames) {
        let at = |id: usize| {
            if id < key_poses.len() {
                key_poses.position(id)
            } else {
                [f64::NAN; 3]
            }
        };
        for v in at(self.source_id).into_iter().chain(at(self.target_id)) {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
        data.extend_from_slice(&(self.source_id as u32).to_le_bytes());
        data.extend_from_slice(&(self.target_id as u32).to_le_bytes());
        data.extend_from_slice(&(self.state as u32).to_le_bytes());
        data.extend_from_slice(&(self.confidence as f32).to_le_bytes());
        data.extend_from_slice(&(self.yank_stdevs as f32).to_le_bytes());
        data.extend_from_slice(&self.time.to_le_bytes());
    }
}

/// One LET kind's jobs and overshoots since the last warning (`Config::overshoot_warning_interval_s`).
#[derive(Default)]
pub(crate) struct OvershootTally {
    jobs: usize,
    overshoots: usize,
}

impl GscPgo {
    /// Count a landed job's overshoot; warn (log only) at most once per `overshoot_warning_interval_s` of data time.
    pub(crate) fn note_overshoot(&mut self, kind: &'static str, overshoot: bool) {
        let tally = self.overshoot_tally.entry(kind).or_default();
        tally.jobs += 1;
        if !overshoot {
            return;
        }
        tally.overshoots += 1;
        let now = self.newest_scan_time();
        let due = self
            .last_overshoot_warning
            .is_none_or(|last| now - last >= self.config.overshoot_warning_interval_s);
        if !due {
            return;
        }
        self.last_overshoot_warning = Some(now);
        let counts: Vec<String> = self
            .overshoot_tally
            .iter_mut()
            .filter(|(_, tally)| tally.jobs > 0)
            .map(|(kind, tally)| {
                let line = format!("{kind} {} of {}", tally.overshoots, tally.jobs);
                *tally = OvershootTally::default();
                line
            })
            .collect();
        tracing::warn!(
            "jobs ran past their deterministic_config estimates (overshoots since the last warning: {}); this machine is slower than the config's, so a replay waits for them and a live run lands them late; a fast-mode run logs a deterministic_config that fits it",
            counts.join(", ")
        );
    }
}

/// `Config::mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    Fast,
    Deterministic,
}

/// `Config::closure_sources`: where loop closures come from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClosureSources {
    /// Scan-context place recognition (else the nearest past keyframe by position), each match verified by ICP.
    ScanContext,
    /// No place recognition: the external inputs (landmark sightings, world positions) close loops; a landmark seen again on a later visit seeds an ICP closure between the two sighting keyframes (`landmark_closure_icp`) so the clouds line up.
    InputsOnly,
}

/// Deterministic-mode job estimates in scans (a job holds `ceil(estimate)`, at least one); defaults are the R1 Jetson fit.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeterministicConfig {
    /// Fast match (search up to the extension): fixed, per keyframe (scan-context lookup; none for a retry), per point, per source point.
    pub fast_match_scans_fixed_overhead: f64,
    pub fast_match_scans_per_keyframe: f64,
    pub fast_match_scans_per_point: f64,
    pub fast_match_scans_per_source_point: f64,
    /// Verification (extension): fixed at start, and per round plus per point of that round's source cloud and target.
    pub verification_scans_fixed_overhead: f64,
    pub verification_scans_per_round: f64,
    pub verification_scans_per_round_point: f64,
    /// Solve: scale * keyframes^keyframe_power * (1 + closures)^closure_power * (1 + rejected by newest adopted solve)^rejected_power.
    pub solve_scans_scale: f64,
    pub solve_keyframe_power: f64,
    pub solve_closure_power: f64,
    pub solve_rejected_power: f64,
    /// A solve with no closures (only landmark sightings or world positions): closure_free_scale * keyframes^closure_free_power.
    pub solve_closure_free_scans_scale: f64,
    pub solve_closure_free_keyframe_power: f64,
    /// The debounce (`adoption_schedule`): the graph change heat's cost of a finished solve, per keyframe.
    pub debounce_scans_per_keyframe: f64,
    /// The merge (`adoption_schedule`): preparing the adoption's new graph version, per keyframe.
    pub merge_scans_per_keyframe: f64,
    /// The loop-overlap pressure: fixed, per keyframe and per committed closure.
    pub pressure_scans_fixed_overhead: f64,
    pub pressure_scans_per_keyframe: f64,
    pub pressure_scans_per_closure: f64,
    /// The solver fast path (`fast_path`): fixed, per keyframe it frees, and per keyframe of the graph (its copy and heat cost).
    pub fast_path_scans_fixed_overhead: f64,
    pub fast_path_scans_per_window_keyframe: f64,
    pub fast_path_scans_per_keyframe: f64,
}

impl Config {
    /// Whether every LET job is held to its estimate (`Mode::Deterministic`).
    pub fn deterministic(&self) -> bool {
        self.mode == Mode::Deterministic
    }
}

/// Tunable configuration for the PGO core; its defaults are `defaults.json`, which module.py reads too.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// `deterministic` (default): every LET job is held to its estimate so replays match on any machine; `fast`: no holds, prints a fitted config.
    pub mode: Mode,
    /// The deterministic mode's estimates, every coefficient in scans (the exponents unitless).
    pub deterministic_config: DeterministicConfig,
    /// Overshoot warnings (log only) at most once per this many seconds of data.
    pub overshoot_warning_interval_s: f64,
    pub keyframe_min_rotation_degrees: f64,
    pub keyframe_min_distance_meters: f64,
    pub closure_search_radius: f64,
    /// ICP pairs a source point with its nearest target point only within this distance (m).
    pub closure_icp_inlier_distance_m: f64,
    /// A closure whose ICP fit has a larger rms inlier distance (m, sqrt of the fitness) is rejected.
    pub closure_icp_max_fitness_m: f64,
    /// Point-to-plane closure ICP (false = point-to-point).
    pub closure_icp_point_to_plane: bool,
    /// Single-scan ICP aligns against the target voxelized at this (m; fitness/overlap still use the full target); 0 = full target.
    pub closure_icp_target_resolution_m: f64,
    /// Single-scan ICP stops early (an ICP reject) once its fitness can no longer pass `closure_icp_max_fitness_m`².
    pub closure_icp_early_stop: bool,
    /// Drop points a scan re-sends from the previous scan (e.g. the Go2 L1 republishing) from every keyframe cloud.
    pub drop_reseen_points: bool,
    /// Min points (voxelized at submap_resolution) a keyframe cloud gathers before a due keyframe is made; 0 = the keyframe's own scan.
    pub auto_aggregate_cloud_size: usize,
    /// Target submap half range = ceil(clouds x (1 + extra_fraction)) + extra_keyframes (`Config::target_half_range`).
    pub closure_target_extra_fraction: f64,
    pub closure_target_extra_keyframes: usize,
    pub submap_resolution: f64,
    /// Structure-spread gate: minimum occupied Scan-Context cells. 0 disables.
    pub closure_min_occupancy: i32,
    /// Observability gate (Zhang 2016 / X-ICP degeneracy factor). 0 disables.
    pub closure_min_degeneracy: f64,
    /// k in the yank gate's required confidence 1 - exp(-yank_stdevs^2 / 2k^2); yank is per sqrt(keyframe gap).
    pub closure_yank_gate_width: f64,
    /// How many times the stated odometry error estimate the true drift may be (`proportional_confidence::yank_evidence`); 1 = trust it.
    pub closure_yank_drift_range: f64,
    /// Skip extending a candidate whose bar a full stack could not reach at its single scan's evidence per keyframe.
    pub closure_extension_skip_unreachable: bool,
    /// Uncertainty range as evidence/bar ratios: >= pass (> 1) accepts, <= fail (< 1) rejects, between extends (`UncertaintyRange`).
    pub barely_fail_uncertainty_range: f64,
    pub barely_pass_uncertainty_range: f64,
    /// How much the pressure keeps the uncertainty range wide: its narrowing is stretched by 1 / (1 - pressure)^this.
    pub decisive_pressure_rate: f64,
    /// Odometry distance between an extension's source clouds, as a fraction of the mean scan reach.
    pub extension_cloud_spacing: f64,
    /// Yank gate scores an extension by median cloud ratio x exp(s (1 - min(d, 5)^2)) per step (d: disagreement, inlier distances); 0 = median.
    pub compounding_agreement_sensitivity: f64,
    /// Graph change heat: a solve is adopted only at heat <= threshold, adding its curbed yank above floor; decays by decay per keyframe.
    pub graph_change_heat_enabled: bool,
    pub graph_change_heat_threshold: f64,
    pub graph_change_heat_floor: f64,
    pub graph_change_heat_decay: f64,
    /// Seed a solve from the accepted graph when it already fits every closure GNC would weigh; otherwise from the odometry.
    pub solve_seed_from_accepted: bool,
    /// Solver fast path (`fast_path`): a new closure whose older end is within `solver_fast_path_max_keyframes` of the newest keyframe is applied at once by a local least squares (no GNC) over those keyframes; the GNC solve covering it supersedes.
    pub solver_fast_path_enabled: bool,
    pub solver_fast_path_max_keyframes: usize,
    /// Each of its Levenberg-Marquardt passes gives up (declines) after this many steps, bounding its cost estimate.
    pub solver_fast_path_max_lm_steps: usize,
    /// GNC starts from the newest adopted solve's verdicts (rejected closures start out).
    pub solve_seed_gnc_verdicts: bool,
    /// Search pipeline (`closure_search_schedule`): candidate buffer -> fast-match workers; uncertain matches -> verification buffer -> workers.
    pub fast_match_workers: usize,
    pub sc_buffer_size: usize,
    pub verification_workers: usize,
    pub verification_buffer_slots: usize,
    /// Recent speed (`recent_speed`) is the median over the newest this many steps between distinct odometry poses.
    pub solve_speed_window_poses: usize,
    /// When set, each candidate ICP writes `<dir>/<source>_<target>.icp` (clouds as ICP got them, then its result; see `dump_icp`).
    pub closure_icp_dump_dir: String,
    /// Extension: an uncertain candidate's source grows to the last 2, 4, 8, ... keyframes (up to this many), each seeded by the last; 0 = off.
    pub closure_extended_icp_max_keyframes: usize,
    /// Huber threshold on a kept closure's whitened residual in the final solve; 0 = plain least squares.
    pub closure_robust_huber_k: f64,
    /// Closure-verification ICP: iteration cap and correspondence gate (m).
    pub closure_icp_max_iterations: u64,
    pub closure_icp_max_correspondence_m: f64,
    /// GNC closure-factor variance = `score * closure_gnc_var_scale` (the raw ICP score makes GNC over-reject).
    pub closure_gnc_var_scale: f64,
    /// Probability a true inlier's residual falls under GNC's chi-squared threshold, in (0, 1); lower rejects more.
    pub closure_gnc_inlier_probability: f64,
    /// False: no closure is rejected as uninformative (graph surer of its two ends than ICP).
    pub closure_uninformative_gate: bool,
    /// Odometry drift random walk per source (xy, z, yaw, tilt): sd per sqrt m travelled and per sqrt rad turned; defaults fit Mid-360 FAST-LIO.
    pub odometry_error_estimate_xy_per_sqrt_m: f64,
    pub odometry_error_estimate_xy_per_sqrt_rad: f64,
    pub odometry_error_estimate_z_per_sqrt_m: f64,
    pub odometry_error_estimate_z_per_sqrt_rad: f64,
    pub odometry_error_estimate_yaw_per_sqrt_m: f64,
    pub odometry_error_estimate_yaw_per_sqrt_rad: f64,
    pub odometry_error_estimate_tilt_per_sqrt_m: f64,
    pub odometry_error_estimate_tilt_per_sqrt_rad: f64,
    /// Odometry error estimate follows the lidar range (`lidar_range`), replacing `odometry_error_estimate_*` on each update.
    pub odometry_error_estimate_from_lidar_range: bool,
    pub lidar_range: crate::lidar_range::LidarRangeConfig,
    /// Learner plasticity: how many closed loops' evidence the stated estimate counts as (0 = closures alone, infinite = never learn).
    pub learned_odometry_error_estimate_stiffness: f64,
    pub scan_context_num_rings: i32,
    pub scan_context_num_sectors: i32,
    pub scan_context_max_range_m: f64,
    pub scan_context_top_k: i32,
    pub scan_context_match_threshold: f64,
    pub scan_context_lidar_height_m: f64,
    /// Evidence (yank units) is the likelihood ratio (`connection_fit::likelihood_ratio`) not scan context x ICP x degeneracy; sets the budget.
    pub closure_likelihood_confidence: bool,
    /// On rejection, try the keyframe's next scan-context match, up to 1 + floor(sqrt(min(ceil(1 / (1 - pressure)), top_k - 1))) tries.
    pub closure_alternative_places: bool,
    /// Alternative places are tried only at or above this pressure.
    pub closure_alternative_places_min_pressure: f64,
    /// With `closure_likelihood_confidence`: score only source points a target scan could see (`connection_fit::likelihood_ratio`).
    pub closure_likelihood_visibility: bool,
    /// Estimate the odometry noise variances from kept closures (`learned_odometry_error_estimate`); logged only, nothing uses them.
    pub learned_odometry_error_estimate_from_closures: bool,
    /// Noise learner reads only kept closures that agree pairwise around raw-odometry cycles (the PCM core).
    pub odometry_error_estimate_evidence_core: bool,
    /// Recompute the loop-overlap pressure (`loop_overlap`) on every this-many-th scan, reusing it in between; 0 = never.
    pub pressure_check_frequency: usize,
    /// `soft_pressure` (display only): 50% where log10 of the loop hazard is this, 0% this many decades below (log-compressed).
    pub soft_pressure_half_log10_hazard: f64,
    pub soft_pressure_decades: f64,
    /// Reduced pose graph for long runs (`graph_simplify`): keyframes revisiting a mapped, localized place ride a kept one.
    pub graph_simplify: bool,
    /// The newest this many keyframes are never simplified (each is decided once it is this far behind the newest).
    pub graph_simplify_frontier_keyframes: usize,
    /// A keyframe revisits a place only if the kept keyframe there is at least this many keyframes older (an older pass).
    pub graph_simplify_pass_gap_keyframes: usize,
    /// A keyframe may ride a keyframe kept on an older pass within this distance (m).
    pub graph_simplify_place_radius_m: f64,
    /// ...only when a closure the newest solve kept has an end within this many keyframes of it.
    pub graph_simplify_localized_keyframes: usize,
    /// Closures per node pair that go to the solve (the rest keep their last verdict); 0 = all.
    pub graph_simplify_closures_per_pair: usize,
    /// How far a removed keyframe's frozen offset from its node may be off (m, rad, one sd): an edge reaching a node through one carries this much more variance per such end, so the adopted graph it was read from is not taken as exact.
    pub graph_simplify_offset_sd_m: f64,
    pub graph_simplify_offset_sd_rad: f64,
    /// Odometry folds between consecutive kept keyframes (true), or follows the nodes removed keyframes ride (false).
    pub graph_simplify_chain_odometry: bool,
    /// ...folding at most this many metres between kept keyframes; past it the walk rides the current place's node.
    pub graph_simplify_chain_fold_max_m: f64,
    /// `scan_context` (default): scan-context proposals; `inputs_only`: none, only landmark sightings and world positions (`ClosureSources`).
    pub closure_sources: ClosureSources,
    /// With `inputs_only`: a landmark re-seen on a later visit proposes an ICP closure between its two sighting keyframes.
    pub landmark_closure_icp: bool,
    /// 2D: odometry is taken as x, y, yaw (z, roll, pitch dropped, the cloud levelled), closures, sightings and world positions are measured in that level frame, and the solve holds every keyframe's z, roll and pitch at 0 (no z or tilt drift).
    pub planar_pose_graph: bool,
    /// A solve with landmark sightings starts from one with the odometry's xy variances times this (<= 1 = off): a long loop's re-sighting is otherwise out of reach of a solve from the drifted odometry, and GNC drops it as an outlier.
    pub landmark_warm_start_xy_variance_scale: f64,
    /// ...then a solve per stage, each from the last with that scale divided by this, down to the stated variances.
    pub landmark_warm_start_xy_variance_ladder: f64,
}

impl Config {
    /// The closure target submap's half range with `clouds` source clouds in the extension (1: the single scan).
    pub fn target_half_range(&self, clouds: usize) -> usize {
        (clouds.max(1) as f64 * (1.0 + self.closure_target_extra_fraction.max(0.0))).ceil() as usize
            + self.closure_target_extra_keyframes
    }
    /// An error naming the first setting out of its allowed range, if any (the negated comparisons reject NaN too).
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), String> {
        if self.odometry_error_estimate_from_lidar_range {
            self.lidar_range.validate()?;
        }
        if !(self.barely_fail_uncertainty_range < 1.0) {
            return Err(format!(
                "barely_fail_uncertainty_range must be < 1 (got {})",
                self.barely_fail_uncertainty_range
            ));
        }
        if !(self.barely_pass_uncertainty_range > 1.0) {
            return Err(format!(
                "barely_pass_uncertainty_range must be > 1 (got {})",
                self.barely_pass_uncertainty_range
            ));
        }
        let estimates = [
            (
                "fast_match_scans_fixed_overhead",
                self.deterministic_config.fast_match_scans_fixed_overhead,
            ),
            (
                "fast_match_scans_per_keyframe",
                self.deterministic_config.fast_match_scans_per_keyframe,
            ),
            (
                "fast_match_scans_per_point",
                self.deterministic_config.fast_match_scans_per_point,
            ),
            (
                "fast_match_scans_per_source_point",
                self.deterministic_config.fast_match_scans_per_source_point,
            ),
            (
                "verification_scans_fixed_overhead",
                self.deterministic_config.verification_scans_fixed_overhead,
            ),
            (
                "verification_scans_per_round",
                self.deterministic_config.verification_scans_per_round,
            ),
            (
                "verification_scans_per_round_point",
                self.deterministic_config.verification_scans_per_round_point,
            ),
            (
                "pressure_scans_fixed_overhead",
                self.deterministic_config.pressure_scans_fixed_overhead,
            ),
            (
                "pressure_scans_per_keyframe",
                self.deterministic_config.pressure_scans_per_keyframe,
            ),
            (
                "pressure_scans_per_closure",
                self.deterministic_config.pressure_scans_per_closure,
            ),
            (
                "solve_scans_scale",
                self.deterministic_config.solve_scans_scale,
            ),
            (
                "solve_closure_free_scans_scale",
                self.deterministic_config.solve_closure_free_scans_scale,
            ),
            (
                "debounce_scans_per_keyframe",
                self.deterministic_config.debounce_scans_per_keyframe,
            ),
            (
                "fast_path_scans_fixed_overhead",
                self.deterministic_config.fast_path_scans_fixed_overhead,
            ),
            (
                "fast_path_scans_per_window_keyframe",
                self.deterministic_config
                    .fast_path_scans_per_window_keyframe,
            ),
            (
                "fast_path_scans_per_keyframe",
                self.deterministic_config.fast_path_scans_per_keyframe,
            ),
        ];
        for (name, count) in [
            ("fast_match_workers", self.fast_match_workers),
            ("sc_buffer_size", self.sc_buffer_size),
            ("verification_workers", self.verification_workers),
            ("verification_buffer_slots", self.verification_buffer_slots),
        ] {
            if count == 0 {
                return Err(format!("{name} must be >= 1"));
            }
        }
        if !(self.overshoot_warning_interval_s >= 0.0) {
            return Err("overshoot_warning_interval_s must be >= 0".into());
        }
        if let Some((name, value)) = estimates.iter().find(|(_, v)| !(*v >= 0.0)) {
            return Err(format!("{name} must be >= 0 (got {value})"));
        }
        Ok(())
    }
    /// `closure_icp_inlier_distance_m` squared, the form ICP compares against.
    pub fn closure_inlier_squared_distance(&self) -> f64 {
        self.closure_icp_inlier_distance_m.powi(2)
    }
}

impl Default for Config {
    /// `defaults.json`, with the stated odometry error estimate the Mid-360's (`lidar_range.mid360_odometry_error_estimate`).
    fn default() -> Config {
        let mut defaults: serde_json::Value =
            serde_json::from_str(include_str!("../defaults.json")).expect("defaults.json parses");
        let mid360 = defaults["lidar_range"]["mid360_odometry_error_estimate"].clone();
        for (source_index, source) in ["xy", "z", "yaw", "tilt"].iter().enumerate() {
            for (unit_index, unit) in ["m", "rad"].iter().enumerate() {
                defaults[format!("odometry_error_estimate_{source}_per_sqrt_{unit}")] =
                    mid360[source_index][unit_index].clone();
            }
        }
        defaults["odometry_error_estimate_from_lidar_range"] = true.into();
        serde_json::from_value(defaults).expect("defaults.json is a Config")
    }
}

#[derive(Default)]
pub struct GscPgo {
    pub(crate) config: Config,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) key_poses: KeyFrames,
    pub(crate) committed_closures: crate::store::Closures,
    /// The newest accepted search's fit confidence and yank, for the closure `commit_closure` commits next.
    pub(crate) committed_stats: Option<(f64, f64)>,
    /// ln of each odometry variance's ratio to the reported one (0 = reported), set from kept closures after each solve.
    pub(crate) learned_odometry_error_estimate_ln: [f64; 4],
    /// Each `learned_odometry_error_estimate_ln`'s posterior standard deviation once the residuals have measured it (the prior's before).
    pub(crate) learned_odometry_error_estimate_ln_sd: [Option<f64>; 4],
    /// A solve is due only to measure the learned estimate's evidence (at `finish`).
    pub(crate) evidence_solve_due: bool,
    /// Next solve also measures the learned estimate's evidence: on a longer kept loop than ever (`learned_evidence_gap`), and at `finish`.
    pub(crate) learned_evidence_requested: bool,
    pub(crate) learned_evidence_gap: usize,
    pub(crate) learned_evidence_measured: bool,
    /// A closure was committed since the last `update`.
    pub(crate) new_closure: bool,
    /// Committed closures a solve has been started for.
    pub(crate) closures_dispatched: usize,
    /// World-frame position fixes (`add_coordinate_factor`): time, frame id, body sensor offset, position, covariance; dispatched; alignments.
    pub(crate) world_positions: Vec<(f64, u32, Vec3, Vec3, Mat3)>,
    pub(crate) world_positions_dispatched: usize,
    pub(crate) world_alignment: Vec<(u32, f64, Vec3)>,
    /// Landmark sightings (`add_landmark_factor`), how many were dispatched, and the newest solve's landmarks (id, map rotation, position).
    pub(crate) landmark_sightings: Vec<LandmarkSightingRecord>,
    pub(crate) landmark_sightings_dispatched: usize,
    /// Each superseding sighting id's index in `landmark_sightings` (`replace_landmark_factor`), and whether one was replaced since the newest solve started.
    pub(crate) landmark_sighting_index: std::collections::HashMap<u64, usize>,
    pub(crate) landmark_sighting_replaced: bool,
    pub(crate) landmarks: Vec<(u64, Mat3, Vec3)>,
    /// `inputs_only`: how many sightings were placed on their nearest keyframes, and each landmark's visits (`LandmarkVisits`).
    pub(crate) sightings_placed: usize,
    pub(crate) landmark_visits: std::collections::BTreeMap<u64, LandmarkVisits>,
    /// `planar_pose_graph`: per keyframe, the tilt (body -> level frame) dropped from its odometry; empty when off.
    pub(crate) keyframe_tilts: Vec<Mat3>,
    pub(crate) solving: Option<Solving>,
    pub(crate) graph_error: f64,
    /// Graph change heat, the keyframe count it was last raised at, and the solve waiting for it to cool.
    pub(crate) heat: f64,
    pub(crate) heat_keyframes: usize,
    pub(crate) pending_solve: Option<(crate::pose_graph_solving::FinishedSolve, f64)>,
    /// Debounce and merge (`adoption_schedule`): jobs out, solves waiting, workers, and whether they run inline (end of data).
    pub(crate) debounce_job:
        Option<crate::adoption_schedule::AdoptionJob<crate::adoption_schedule::Debounced>>,
    pub(crate) debounce_waiting: Option<crate::pose_graph_solving::FinishedSolve>,
    pub(crate) merge_job:
        Option<crate::adoption_schedule::AdoptionJob<crate::adoption_schedule::Merged>>,
    pub(crate) merge_waiting: Option<crate::pose_graph_solving::FinishedSolve>,
    pub(crate) adoption_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) merge_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) adoption_inline: bool,
    /// A merge landed (keyframes moved) since the last `update`.
    pub(crate) adopted_since_update: bool,
    /// Newest `solve_speed_window_poses` + 1 distinct odometry poses (time, odometry-frame position), for `recent_speed`.
    pub(crate) recent_motion: std::collections::VecDeque<(f64, Vec3)>,
    /// The newest scan's recorded time.
    pub(crate) scan_time: f64,
    /// Newest advancing scan time and count of scans whose stamp advanced (LET holds count these; repeated stamps span no time).
    pub(crate) newest_advancing_time: Option<f64>,
    pub(crate) advancing_scans: usize,
    pub(crate) first_advancing_time: Option<f64>,
    /// Each landed job's kind and measured / estimated scans, for `runtime_profile`; never read by a decision.
    pub(crate) job_ratios: Vec<(&'static str, f64)>,
    /// Per LET kind, its jobs and overshoots since the last warning, and the data time of that warning.
    pub(crate) overshoot_tally: std::collections::BTreeMap<&'static str, OvershootTally>,
    pub(crate) last_overshoot_warning: Option<f64>,
    /// Dispatched, unlanded closure searches in order, the next order number, advancing-scan times since the oldest's dispatch, the pool.
    pub(crate) closure_searches:
        std::collections::VecDeque<crate::closure_search_schedule::SearchJob>,
    pub(crate) search_order: u64,
    pub(crate) search_scan_times: std::collections::VecDeque<f64>,
    pub(crate) search_scan_base: usize,
    pub(crate) search_pool: Option<crate::closure_search_schedule::SearchPool>,
    /// Two-stage search: candidates awaiting fast match (with arrival numbers), matches awaiting verification, its pool.
    pub(crate) sc_buffer: Vec<(ClosureProposal, u64)>,
    pub(crate) sc_arrivals: u64,
    pub(crate) verification_buffer: Vec<(Box<PendingExtension>, u64)>,
    pub(crate) verification_pool: Option<crate::closure_search_schedule::SearchPool>,
    /// Scans offered to `add_key_pose`.
    pub(crate) scans_seen: usize,
    /// The running lidar range estimate (`odometry_error_estimate_from_lidar_range`).
    pub(crate) lidar_range: crate::lidar_range::LidarRange,
    pub(crate) pressure: Option<(usize, crate::loop_overlap::LoopOverlap)>,
    /// Bumped on each keyframe or adopted solve; pressure recomputes only when it differs from `pressure_version`.
    pub(crate) graph_version: u64,
    pub(crate) pressure_version: Option<u64>,
    /// The pressure job running or held (`pressure_schedule`).
    pub(crate) pressure_job: Option<crate::pressure_schedule::PressureJob>,
    /// Solves adopted so far: the accepted graph's version (keyframes added since only ride it).
    pub(crate) adopted_solves: u64,
    pub(crate) pressure_requested: bool,
    pub(crate) pressure_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) pressure_late: bool,
    /// Sum and count of the keyframes' scan reaches (95th-percentile planar range), the sensor range loop_overlap uses.
    pub(crate) keyframe_reach: (f64, usize),
    /// `drop_reseen_points`: the previous scan's points (odometry frame, 0.1 mm grid).
    pub(crate) reseen_previous: std::collections::HashSet<[i64; 3]>,
    /// `auto_aggregate_cloud_size`: the scans since the last keyframe, in the odometry frame, voxelized.
    pub(crate) gathered: PointCloud,
    /// The newest applied solve's tip keyframe and every solved keyframe's position covariance relative to it.
    pub(crate) solved_relative_covariance: Option<(usize, Arc<Vec<Mat3>>)>,
    /// The reduced pose graph (`graph_simplify`).
    pub(crate) simplify: crate::graph_simplify::GraphSimplify,
    pub(crate) scan_context_ring_keys: Vec<similarity_discovery::RingKey>,
    pub(crate) rotation_offset: Mat3,
    pub(crate) translation_offset: Vec3,
    pub(crate) timing: TimingStats,
    pub(crate) closure_diagnostics: Vec<ClosureCandidateDiag>,
    pub(crate) closure_edge_events: Vec<ClosureEdgeEvent>,
    /// The newest proposal waiting for the search thread (`offer_proposal`).
    pub(crate) waiting_proposal: Option<ClosureProposal>,
    /// The ICP cache every closure search's cloud computations go through (`set_icp_cache`), if any.
    pub(crate) icp_cache: Option<Arc<crate::icp_cache::IcpCache>>,
    /// Each keyframe's voxelized scan and range image, built once and shared by every closure search.
    pub(crate) keyframe_clouds: Arc<crate::proportional_confidence::KeyframeClouds>,
    /// Called when a background solve finishes (`notify_when_solved`).
    pub(crate) solved: Option<Arc<dyn Fn() + Send + Sync>>,
    /// Solver fast path (`fast_path`): the job out, its worker, the committed closures the graph reflects, and its records.
    pub(crate) fast_path_job: Option<crate::fast_path::FastPathJob>,
    pub(crate) fast_path_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) fast_path_reflected: usize,
    pub(crate) fast_path_records: Vec<crate::fast_path::FastPathRecord>,
    /// Per committed closure, when it was committed, first reflected by the fast path, and judged by an adopted GNC solve.
    pub(crate) closure_latency: Vec<crate::fast_path::ClosureLatency>,
}

/// Cumulative per-stage wall time (`GscPgo::stage_seconds`), purely observational (never read back into the pipeline).
#[derive(Default)]
pub(crate) struct TimingStats {
    pub(crate) scan_context_s: f64,
    pub(crate) submap_s: f64,
    pub(crate) icp_s: f64,
    pub(crate) degeneracy_s: f64,
    pub(crate) extended_icp_s: f64,
    pub(crate) solve_s: f64,
    /// `add_key_pose` without the pressure check (drop-reseen, aggregation, voxelizing, keyframe, descriptor).
    pub(crate) scan_s: f64,
    /// The loop-overlap pressure checks (`pressure_check_frequency`).
    pub(crate) pressure_s: f64,
    /// `propose_closure` and `begin_closure_search`, the scan-context lookup (`scan_context_s`) included.
    pub(crate) propose_s: f64,
    /// `ClosureSearch::run` (its ICP, gates and extension), debug dumps left out.
    pub(crate) search_s: f64,
}

/// A keyframe through stages 1 and 2 awaiting the search thread: target, scan-context column shift, cosine distance (lower = better).
pub struct ClosureProposal {
    pub(crate) diag: ClosureCandidateDiag,
    pub(crate) target_idx: usize,
    pub(crate) sector_shift: i32,
    pub(crate) similarity: f32,
    /// Every earlier keyframe's ring-key distance (the top k in order), for the search's look-alike count.
    pub(crate) ranked: Vec<(f32, usize)>,
    /// The keyframe's other scan-context matches still untried (distance, index, shift), and the tries so far.
    pub(crate) alternatives: Vec<(f32, usize, i32)>,
    pub(crate) tries: usize,
    /// A landmark seen from both keyframes (`inputs_only`), which seeds the ICP instead of a place match.
    pub(crate) landmark_pair: Option<LandmarkPair>,
}

/// A landmark sighting on its nearest keyframe (`inputs_only`): keyframe, body-frame position and its covariance (m²).
#[derive(Clone, Copy, Debug)]
pub(crate) struct PlacedSighting {
    pub(crate) keyframe: usize,
    pub(crate) position: Vec3,
    pub(crate) covariance: Mat3,
}

/// One landmark's sightings so far: its first visit's best (nearest) sighting, that visit's newest keyframe and whether it ended, and the newest keyframe a closure was proposed from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LandmarkVisits {
    pub(crate) first_visit_best: PlacedSighting,
    pub(crate) first_visit_newest: usize,
    pub(crate) first_visit_over: bool,
    pub(crate) newest_proposed: Option<usize>,
}

/// The two sightings of one landmark a closure candidate joins: the source keyframe's and the (earlier) target keyframe's.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LandmarkPair {
    pub(crate) source: PlacedSighting,
    pub(crate) target: PlacedSighting,
}

/// The ICP half of a closure search, on its own keyframe copy so it runs off the scan thread (`begin_closure_search` -> `finish_closure_search`).
pub struct ClosureSearch {
    pub(crate) key_poses: KeyFrames,
    pub(crate) config: Config,
    /// The odometry's mean xy variance per keyframe step at the reported noise (`big_picture_fit::mean_step_variances`).
    pub(crate) mean_step_xy_variance: f64,
    /// The closures the newest solve kept.
    pub(crate) kept_closures: Vec<Edge>,
    pub(crate) diag: ClosureCandidateDiag,
    pub(crate) target_idx: usize,
    pub(crate) init_rotation: Mat3,
    pub(crate) init_translation: Vec3,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) ranked: Vec<(f32, usize)>,
    /// Every closure the solve has judged: its yank_stdevs when committed, and whether the solve kept it.
    pub(crate) judged_yanks: Vec<(f64, bool)>,
    /// Snapshots the search's setup (`prepare`) reads: committed closures, keyframe reach, odometry error estimate, solved relative covariances.
    pub(crate) closures: crate::store::Closures,
    pub(crate) keyframe_reach: (f64, usize),
    pub(crate) error_estimate: [f64; 4],
    pub(crate) solved_relative_covariance: Option<(usize, Arc<Vec<Mat3>>)>,
    pub(crate) alternatives: Vec<(f32, usize, i32)>,
    pub(crate) tries: usize,
    /// The ICP cache the search's cloud computations go through (`icp_cache`), if any.
    pub(crate) icp_cache: Option<Arc<crate::icp_cache::IcpCache>>,
    /// Each keyframe's voxelized scan and range image, built once and shared by every closure search.
    pub(crate) keyframe_clouds: Arc<crate::proportional_confidence::KeyframeClouds>,
    /// Every debug dump's seconds in this search (`closure_icp_dump_dir`), left out of the stage timings.
    pub(crate) dump_s: Arc<std::sync::Mutex<f64>>,
    /// The keyframes' mean scan reach (m), which spaces an extension's separate source clouds.
    pub(crate) mean_reach: f64,
    /// Where the search reports each extension round's start and end.
    pub(crate) progress: Option<Arc<crate::closure_search_schedule::SearchProgress>>,
    /// A landmark-seeded search (`inputs_only`): the covariance (m², map frame) of the seed the two sightings imply, which the yank is then measured from instead of the current graph.
    pub(crate) landmark_seed_covariance: Option<Mat3>,
}

/// What a `ClosureSearch` found: diagnostics, the accepted closure (relative rotation, translation, fitness), stage timings.
pub struct ClosureSearchResult {
    pub(crate) diag: ClosureCandidateDiag,
    accepted: Option<(Mat3, Vec3, f64)>,
    timing: TimingStats,
    /// The search's place-recognition inputs, for a retry on the next alternative (`closure_alternative_places`).
    retry: Option<RetryInputs>,
}

impl ClosureSearchResult {
    /// The cost fields of its diagnostics, from its timing (seconds, NaN where a step did not run).
    fn finalize_costs(&mut self) {
        let (t, diag) = (&self.timing, &mut self.diag);
        let ran = |seconds: f64, ran: bool| if ran { seconds } else { f64::NAN };
        diag.cost_submap_s = t.submap_s;
        diag.cost_degeneracy_s = t.degeneracy_s;
        diag.cost_icp_s = ran(t.icp_s, !diag.icp_fitness.is_nan());
        diag.cost_extension_s = ran(t.extended_icp_s, !diag.cost_extension_rounds_s.is_empty());
        diag.cost_search_s = t.search_s;
        diag.cost_gates_s = t.search_s - t.submap_s - t.degeneracy_s - t.icp_s - t.extended_icp_s;
    }
    /// A match the verification buffer dropped (`verification_buffer_slots`): a rejection, its alternatives still open.
    pub(crate) fn dropped(pending: PendingExtension) -> ClosureSearchResult {
        let mut result = rejected(pending.diag, "verification_dropped", pending.timing);
        result.timing.search_s = pending.fast_s;
        result.finalize_costs();
        result.retry = pending.retry;
        result
    }
}

/// A landmark sighting (`add_landmark_factor`): time, landmark id, rotation, position and 6x6 information in the body frame.
pub(crate) type LandmarkSightingRecord = (f64, u64, Mat3, Vec3, [[f64; 6]; 6]);
/// A search's place-recognition inputs for a retry: ring-key distances, the top-k matches under the threshold, and the next alternative.
type RetryInputs = (Vec<(f32, usize)>, Vec<(f32, usize, i32)>, usize);

/// Where a search stands after its fast match: decided, or inside the uncertainty range and handed to the extension.
#[allow(clippy::large_enum_variant)]
pub enum SearchStage {
    Done(ClosureSearchResult),
    Extend(Box<PendingExtension>),
}

/// A match inside the uncertainty range and what its extension needs (snapshot, single fit, ratio = verification promise, bar base).
pub struct PendingExtension {
    pub(crate) search: ClosureSearch,
    pub(crate) diag: ClosureCandidateDiag,
    timing: TimingStats,
    icp: crate::connection_fit::IcpResult,
    pub(crate) ratio: f64,
    look_alikes: similarity_discovery::LookAlikes,
    error_estimate_sums: Option<Vec<crate::loop_overlap::Sums>>,
    base: f64,
    retry: Option<RetryInputs>,
    pub(crate) fast_s: f64,
}

impl PendingExtension {
    /// Verification stage: extension rounds and the yank verdict; the result's seconds include the fast match's.
    pub(crate) fn run(mut self) -> ClosureSearchResult {
        let retry = self.retry.take();
        let fast_s = self.fast_s;
        let dumps = self.search.dump_s.clone();
        let dumped_before = *dumps.lock().expect("dump seconds");
        let started = std::time::Instant::now();
        let mut result = self.extend();
        let dumped = *dumps.lock().expect("dump seconds") - dumped_before;
        result.timing.search_s = fast_s + started.elapsed().as_secs_f64() - dumped;
        result.finalize_costs();
        result.retry = retry;
        result
    }
    fn extend(self) -> ClosureSearchResult {
        let PendingExtension {
            search,
            mut diag,
            mut timing,
            icp,
            ratio,
            mut look_alikes,
            mut error_estimate_sums,
            base,
            ..
        } = self;
        let (cur_idx, target_idx) = (diag.source_id, search.target_idx);
        let key_suffix = search.icp_key_suffix();
        let memo_alignment =
            |tag: &str, at: usize, compute: &mut dyn FnMut() -> Vec<f64>| -> Vec<f64> {
                search.memo_alignment(&key_suffix, tag, at, compute)
            };
        let mut required = |evidence: f64, diag: &mut ClosureCandidateDiag| -> f64 {
            search.bar_with_places(
                evidence,
                diag,
                &mut look_alikes,
                &mut error_estimate_sums,
                base,
            )
        };
        let (extended_started, dumped_before) = (
            std::time::Instant::now(),
            *search.dump_s.lock().expect("dump seconds"),
        );
        let extended = search.extended_icp(
            (cur_idx, target_idx),
            &icp,
            ratio,
            &mut diag,
            &mut required,
            &memo_alignment,
        );
        timing.extended_icp_s += extended_started.elapsed().as_secs_f64()
            - (*search.dump_s.lock().expect("dump seconds") - dumped_before);
        let Some(extended) = extended else {
            return rejected(diag, "yank_confidence_reject", timing);
        };
        // the closure measurement comes from the extended alignment, which saw more of both places
        let (rotation_offset, translation_offset) =
            search.measured(&extended.rotation, &extended.translation);
        let score = extended.inlier_mean_square();
        diag.outcome = "accepted";
        ClosureSearchResult {
            diag,
            accepted: Some((rotation_offset, translation_offset, score)),
            timing,
            retry: None,
        }
    }
}

/// A search that ended at the gate `outcome`.
fn rejected(
    mut diag: ClosureCandidateDiag,
    outcome: &'static str,
    timing: TimingStats,
) -> ClosureSearchResult {
    diag.outcome = outcome;
    ClosureSearchResult {
        diag,
        accepted: None,
        timing,
        retry: None,
    }
}

impl ClosureSearch {
    /// Fast match (`closure_search_schedule`): up to the extension; a clear pass/fail is decided here, else `SearchStage::Extend`.
    pub(crate) fn run_stage(mut self) -> SearchStage {
        let retry = (
            self.ranked.clone(),
            std::mem::take(&mut self.alternatives),
            self.tries,
        );
        let (started, dumps) = (std::time::Instant::now(), self.dump_s.clone());
        self.prepare();
        let stage = self.search_stage();
        let seconds = started.elapsed().as_secs_f64() - *dumps.lock().expect("dump seconds");
        match stage {
            SearchStage::Done(mut result) => {
                result.timing.search_s = seconds;
                result.finalize_costs();
                result.retry = Some(retry);
                SearchStage::Done(result)
            }
            SearchStage::Extend(mut pending) => {
                pending.fast_s = seconds;
                pending.retry = Some(retry);
                SearchStage::Extend(pending)
            }
        }
    }
    /// Search setup from its snapshots on its own thread: kept closures, judged yanks, mean step variance, target overlap and covariance.
    fn prepare(&mut self) {
        self.mean_step_xy_variance =
            crate::big_picture_fit::mean_step_variances(&self.key_poses, &self.config)[0];
        // kept, or not judged yet
        self.kept_closures = closure_edges(
            self.closures
                .iter()
                .filter(|closure| closure.gnc_weight.is_nan() || closure.is_kept()),
        );
        self.judged_yanks = (0..self.closures.len())
            .map(|i| (self.closures.fixed(i).yank_stdevs, self.closures.weight(i)))
            .filter(|(z, weight)| !weight.is_nan() && z.is_finite())
            .map(|(z, weight)| (z, weight > 0.5))
            .collect();
        let (source_id, target_idx) = (self.diag.source_id, self.target_idx);
        let inputs = crate::loop_overlap::OverlapInputs {
            key_poses: &self.key_poses,
            config: &self.config,
            keyframe_reach: self.keyframe_reach,
            error_estimate: self.error_estimate,
            solved_relative_covariance: self
                .solved_relative_covariance
                .as_ref()
                .map(|(tip, covariance)| (*tip, covariance.as_slice())),
        };
        // extension doubles, so the budget is the doubling step covering max keyframes x the chance a real loop is here
        let model = inputs.overlap_model(source_id);
        let diag = &mut self.diag;
        diag.target_overlap = model
            .as_ref()
            .map_or(0.0, |model| model.overlap(target_idx).0);
        // the graph's covariance of the source's offset from this target, at the odometry error estimate in use
        if let Some(model) = &model {
            diag.innovation_covariance = model.relative(target_idx);
        }
        // either a likely unresolved loop (pressure) or likely overlap with this target makes extending worthwhile
        let worth = 1.0 - (1.0 - diag.pressure) * (1.0 - diag.target_overlap);
        diag.extension_budget = ((self.config.closure_extended_icp_max_keyframes as f64 * worth)
            .ceil() as usize)
            .next_power_of_two()
            .min(self.config.closure_extended_icp_max_keyframes);
    }
    /// A cloud alignment through the ICP cache, stored relative to the target (`relative_icp`).
    pub(crate) fn memo_alignment(
        &self,
        key_suffix: &str,
        tag: &str,
        at: usize,
        compute: &mut dyn FnMut() -> Vec<f64>,
    ) -> Vec<f64> {
        let (cur_idx, target_idx) = (self.diag.source_id, self.target_idx);
        let Some(cache) = &self.icp_cache else {
            return compute();
        };
        let (source_pose, target_pose) = (
            (
                &self.key_poses.rotation(cur_idx),
                &self.key_poses.position(cur_idx),
            ),
            (
                &self.key_poses.rotation(target_idx),
                &self.key_poses.position(target_idx),
            ),
        );
        let mut computed = None;
        let stored = cache.get_or(format!("{cur_idx}:{target_idx}:{tag}{key_suffix}"), || {
            let value = compute();
            let mut relative = value.clone();
            crate::icp_cache::relative_icp(&mut relative[at..], source_pose, target_pose, true);
            computed = Some(value);
            relative
        });
        computed.unwrap_or_else(|| {
            let mut value = stored;
            crate::icp_cache::relative_icp(&mut value[at..], source_pose, target_pose, false);
            value
        })
    }
    /// The closure measurement an alignment gives: the source keyframe moved by it, in the target's frame.
    pub(crate) fn measured(&self, rotation: &Mat3, translation: &Vec3) -> (Mat3, Vec3) {
        let (source, target) = (
            &self.key_poses.get(self.diag.source_id),
            &self.key_poses.get(self.target_idx),
        );
        let closure_rotation_transpose = mat3::transpose(&target.rotation_global);
        let moved = mat3::add(
            &mat3::mat_vec(rotation, &source.translation_global),
            translation,
        );
        let measured = (
            mat3::mat_mul(
                &closure_rotation_transpose,
                &mat3::mat_mul(rotation, &source.rotation_global),
            ),
            mat3::mat_vec(
                &closure_rotation_transpose,
                &mat3::sub(&moved, &target.translation_global),
            ),
        );
        // planar_pose_graph: ICP's z, roll and pitch are projected out of the measurement
        if self.config.planar_pose_graph {
            planar(&measured.0, &measured.1)
        } else {
            measured
        }
    }
    /// Evidence a match needs (yank units): `base` + ln(look-alike places fitting as well), counted only as far as `evidence` needs.
    pub(crate) fn bar_with_places(
        &self,
        evidence: f64,
        diag: &mut ClosureCandidateDiag,
        look_alikes: &mut similarity_discovery::LookAlikes,
        error_estimate_sums: &mut Option<Vec<crate::loop_overlap::Sums>>,
        base: f64,
    ) -> f64 {
        let cur_idx = diag.source_id;
        let same_place_cutoff =
            crate::loop_overlap::chi2_inverse_3dof(self.config.closure_gnc_inlier_probability);
        if !diag.from_scan_context {
            return base;
        }
        let sums = error_estimate_sums.get_or_insert_with(|| {
            crate::loop_overlap::odometry_error_estimate_sums(
                &self.key_poses,
                &self.config,
                diag.learned_odometry_error_estimate,
            )
        });
        let places = look_alikes.count_past(
            (evidence - base).exp(),
            &self.key_poses.descriptor(cur_idx),
            |k| self.key_poses.descriptor(k),
            self.scan_context_config.match_threshold,
            |a, b| {
                crate::loop_overlap::could_be_same_place(
                    &self.key_poses,
                    sums,
                    a,
                    b,
                    self.config.closure_search_radius,
                    same_place_cutoff,
                )
            },
        );
        diag.look_alike_places = places;
        base + (places as f64).ln()
    }
    /// The ICP cache key suffix: what besides the pair changes a cloud computation's result.
    pub(crate) fn icp_key_suffix(&self) -> String {
        format!(
            "{}{}{}",
            if self.config.closure_icp_point_to_plane {
                ":pl"
            } else {
                ""
            },
            if self.config.closure_icp_target_resolution_m > 0.0 {
                format!(":t{}", self.config.closure_icp_target_resolution_m)
            } else {
                String::new()
            },
            if self.config.closure_icp_early_stop {
                ":es"
            } else {
                ""
            },
        ) + if self.landmark_seed_covariance.is_some() {
            ":lm"
        } else {
            ""
        }
    }

    fn search_stage(self) -> SearchStage {
        let mut diag = self.diag.clone();
        let (cur_idx, target_idx) = (diag.source_id, self.target_idx);
        let (init_rotation, init_translation) = (self.init_rotation, self.init_translation);
        let mut timing = TimingStats::default();
        let half_range = self.config.target_half_range(1);
        // source scan (all the degeneracy gate reads) and target submap + tree, built lazily (never on full cache hits); build seconds kept apart
        let built_s = std::cell::Cell::new(0.0);
        let source_cell = std::cell::OnceCell::new();
        let source_cloud = || -> &PointCloud {
            source_cell.get_or_init(|| {
                let started = std::time::Instant::now();
                let cloud = self.stacked_cloud(std::iter::once(cur_idx));
                built_s.set(built_s.get() + started.elapsed().as_secs_f64());
                cloud
            })
        };
        let clouds_cell = std::cell::OnceCell::new();
        let clouds = || -> &(PointCloud, PointCloud, KdTree) {
            let source = source_cloud();
            clouds_cell.get_or_init(|| {
                let started = std::time::Instant::now();
                let target_cloud = self.stacked_cloud(
                    target_idx.saturating_sub(half_range)
                        ..=(target_idx + half_range).min(self.key_poses.len() - 1),
                );
                let target_tree = KdTree::build(&target_cloud);
                built_s.set(built_s.get() + started.elapsed().as_secs_f64());
                (source.clone(), target_cloud, target_tree)
            })
        };
        // a cloud computation, through the ICP cache when there is one
        let cache = self.icp_cache.clone();
        // point-to-plane, the thinned target and the early stop change ICP's result, so they are part of the key
        let key_suffix = self.icp_key_suffix();
        let memo = |tag: &str, compute: &mut dyn FnMut() -> Vec<f64>| -> Vec<f64> {
            match &cache {
                Some(cache) => {
                    cache.get_or(format!("{cur_idx}:{target_idx}:{tag}{key_suffix}"), compute)
                }
                None => compute(),
            }
        };
        // an alignment through the cache (`memo_alignment`)
        let memo_alignment =
            |tag: &str, at: usize, compute: &mut dyn FnMut() -> Vec<f64>| -> Vec<f64> {
                self.memo_alignment(&key_suffix, tag, at, compute)
            };

        // Observability gate: a degenerate source scan leaves in-plane alignment unconstrained; source-only, so it runs before ICP
        let degenerate = |degeneracy_min: f32| {
            self.config.closure_min_degeneracy > 0.0
                && degeneracy_min >= 0.0
                && (degeneracy_min as f64) < self.config.closure_min_degeneracy
        };
        let (t0, built_before) = (std::time::Instant::now(), built_s.get());
        diag.degeneracy_min = memo("deg", &mut || {
            vec![uniqueness::degeneracy(source_cloud()) as f64]
        })[0] as f32;
        timing.degeneracy_s += t0.elapsed().as_secs_f64() - (built_s.get() - built_before);
        if degenerate(diag.degeneracy_min) {
            return SearchStage::Done(rejected(diag, "degeneracy_reject", timing));
        }

        // uninformative gate: when the graph is already surer of the two ends' relative position than ICP's fitness, the closure adds nothing
        let gate_error_estimate = diag.odometry_error_estimate_gate;
        let odometry_step = self.mean_step_xy_variance * gate_error_estimate[0];
        let step = |a: usize, b: usize| a.abs_diff(b) as f64 * odometry_step;
        let by_graph = diag
            .innovation_covariance
            .iter()
            .flatten()
            .all(|v| v.is_finite());
        let known = if by_graph {
            // the graph's covariance of the two ends' offset (heading swing and solved closures included): its largest eigenvalue
            mat3::jacobi_eigen(diag.innovation_covariance)
                .0
                .into_iter()
                .fold(0.0, f64::max)
        } else {
            self.kept_closures
                .iter()
                .fold(step(cur_idx, target_idx), |known, l| {
                    let via = |a: usize, b: usize| {
                        step(target_idx, a) + l.variances[3] + step(cur_idx, b)
                    };
                    known.min(via(l.from, l.to)).min(via(l.to, l.from))
                })
        };
        let (t0, built_before) = (std::time::Instant::now(), built_s.get());
        let icp = crate::icp_cache::decode_icp(&memo_alignment("icpr", 0, &mut || {
            let (source_cloud, target_cloud, target_tree) = clouds();
            crate::icp_cache::encode_icp_logged(&crate::connection_fit::icp_thinned(
                source_cloud,
                target_cloud,
                target_tree,
                (&init_rotation, &init_translation),
                &IcpParams {
                    max_iterations: self.config.closure_icp_max_iterations as usize,
                    max_correspondence_distance: self.config.closure_icp_max_correspondence_m,
                    inlier_squared_distance: self.config.closure_inlier_squared_distance(),
                    point_to_plane: self.config.closure_icp_point_to_plane,
                },
                self.config
                    .closure_icp_early_stop
                    .then(|| self.config.closure_icp_max_fitness_m.powi(2)),
                self.config.closure_icp_target_resolution_m,
            ))
        }));
        timing.icp_s += t0.elapsed().as_secs_f64() - (built_s.get() - built_before);
        timing.submap_s = built_s.get();
        if let Some((source, target, _)) = clouds_cell.get() {
            diag.single_source_points = source.len();
            diag.single_target_points = target.len();
        }
        diag.single_target_points_thinned = icp.target_points;
        if !self.config.closure_icp_dump_dir.is_empty() && cache.is_none() {
            let dump_started = std::time::Instant::now();
            let (source_cloud, target_cloud, _) = clouds();
            dump_icp(
                &self.config.closure_icp_dump_dir,
                cur_idx,
                target_idx,
                source_cloud,
                target_cloud,
                (&init_rotation, &init_translation),
                &icp,
                "",
            );
            *self.dump_s.lock().expect("dump seconds") += dump_started.elapsed().as_secs_f64();
        }
        diag.icp_converged = icp.converged;
        diag.icp_iterations = icp.iterations;
        diag.icp_stop = icp.stop.name();
        // where each puts the source keyframe
        let source_position = &self.key_poses.position(cur_idx);
        diag.icp_init_offset_m = mat3::norm(&mat3::sub(
            &mat3::add(
                &mat3::mat_vec(&icp.rotation, source_position),
                &icp.translation,
            ),
            &mat3::add(
                &mat3::mat_vec(&init_rotation, source_position),
                &init_translation,
            ),
        ));
        diag.icp_init_offset_deg =
            mat3::angular_distance(&icp.rotation, &init_rotation).to_degrees();
        diag.icp_fitness = icp.fitness;
        diag.icp_overlap = icp.overlap;
        diag.icp_translation_norm = mat3::norm(&icp.translation);

        if !icp.converged || icp.fitness > self.config.closure_icp_max_fitness_m.powi(2) {
            return SearchStage::Done(rejected(diag, "icp_reject", timing));
        }
        // the closure's translation variance as the graph takes it (`IcpResult::inlier_mean_square`)
        let closure_variance = icp.inlier_mean_square();
        if self.config.closure_uninformative_gate && closure_variance >= known {
            return SearchStage::Done(rejected(diag, "uninformative", timing));
        }
        if self.config.closure_likelihood_confidence {
            let window: Vec<usize> = (target_idx.saturating_sub(half_range)
                ..=(target_idx + half_range).min(self.key_poses.len() - 1))
                .collect();
            let lr = memo("lr", &mut || {
                let (source_cloud, target_cloud, target_tree) = clouds();
                let (ratio, count, width) =
                    self.likelihood_ratio(source_cloud, target_cloud, target_tree, &icp, &window);
                vec![ratio, count as f64, width]
            });
            let (ratio, width) = (lr[0], lr[2]);
            diag.likelihood_ratio = ratio;
            diag.likelihood_width = width;
        }

        // the closure's variance per axis: its inliers' mean squared distance, as LIO-SAM takes ICP's fitness score
        let score = icp.inlier_mean_square();
        // the closure measurement an alignment gives: the source keyframe moved by it, in the target's frame
        let measured = |rotation: &Mat3, translation: &Vec3| self.measured(rotation, translation);
        let (rotation_offset, translation_offset) = measured(&icp.rotation, &icp.translation);

        // graph yank: how far this constraint pulls the source from the current graph's relative pose (large = likely false closure); a landmark-seeded search measures it from the pose the landmark implies, at that seed's covariance
        let (current_relative_rotation, current_relative_translation) =
            match self.landmark_seed_covariance {
                Some(covariance) => {
                    diag.innovation_covariance = covariance;
                    measured(&init_rotation, &init_translation)
                }
                None => measured(&mat3::identity(), &[0.0; 3]),
            };
        diag.yank_translation = mat3::norm(&mat3::sub(
            &translation_offset,
            &current_relative_translation,
        ));
        diag.yank_rotation_deg =
            mat3::angular_distance(&rotation_offset, &current_relative_rotation).to_degrees();
        diag.innovation = mat3::mat_vec(
            &self.key_poses.rotation(target_idx),
            &mat3::sub(&translation_offset, &current_relative_translation),
        );

        self.score_candidate(&mut diag);
        // the solve's record on closures that pulled about as hard as this one, so a gentle closure isn't charged for rejected hard pullers
        let (kept, judged) =
            self.judged_yanks
                .iter()
                .fold((0.0, 0.0), |(kept, judged), &(z, was_kept)| {
                    let weight = (-0.5 * (z - diag.yank_stdevs).powi(2)).exp();
                    (kept + weight * was_kept as u8 as f64, judged + weight)
                });
        diag.keep_rate = (kept + 1.0) / (judged + 2.0);
        // stage 4b, look-alike places: N equally fitting places cost ln N of evidence; the candidates checked grow with pressure
        let cap = (self.scan_context_config.candidate_top_k as f64
            / (1.0 - diag.pressure).max(f64::EPSILON))
        .ceil()
        .min(usize::MAX as f64) as usize;
        let mut look_alikes = similarity_discovery::LookAlikes::new(self.ranked.clone(), cap);
        let base = self.required_evidence(&diag);
        // two matching keyframes are one place if the odometry between them could have drifted that far apart
        let mut error_estimate_sums: Option<Vec<crate::loop_overlap::Sums>> = None;
        let mut required = |evidence: f64, diag: &mut ClosureCandidateDiag| -> f64 {
            self.bar_with_places(
                evidence,
                diag,
                &mut look_alikes,
                &mut error_estimate_sums,
                base,
            )
        };
        let evidence = -(1.0 - diag.confidence).ln();
        let single_required = required(evidence, &mut diag);
        // evidence over its bar, both in yank units (an unmet bar of 0 counts as passed)
        let ratio = if single_required > 0.0 {
            evidence / single_required
        } else {
            f64::INFINITY
        };
        let (low, high) = (
            self.config.barely_fail_uncertainty_range,
            self.config.barely_pass_uncertainty_range,
        );
        if self.config.closure_likelihood_confidence {
            // extend as far as reaching the range's top takes at this scan's evidence per keyframe (at least one bit)
            let per_keyframe = evidence.max(std::f64::consts::LN_2);
            diag.extension_budget = ((single_required * high / per_keyframe).ceil().max(2.0)
                as usize)
                .next_power_of_two()
                .min(self.config.closure_extended_icp_max_keyframes);
        }
        diag.required_confidence = 1.0 - (-single_required).exp();
        // a stack earns at most about the single scan's evidence per keyframe: one that can't reach its bar at full stack is decided now
        let reachable = !self.config.closure_extension_skip_unreachable
            || !self.config.closure_likelihood_confidence
            || evidence.max(std::f64::consts::LN_2)
                * self.config.closure_extended_icp_max_keyframes as f64
                >= single_required;
        // outside the uncertainty range the single scan decides; inside it the match is extended until it does
        if ratio <= low || (ratio < 1.0 && !reachable) {
            return SearchStage::Done(rejected(diag, "yank_confidence_reject", timing));
        }
        // inside the uncertainty range the extension decides (`PendingExtension::extend`, the verification stage)
        if ratio < high {
            diag.promise = ratio;
            return SearchStage::Extend(Box::new(PendingExtension {
                search: self,
                diag,
                timing,
                icp,
                ratio,
                look_alikes,
                error_estimate_sums,
                base,
                retry: None,
                fast_s: 0.0,
            }));
        }

        diag.outcome = "accepted";
        SearchStage::Done(ClosureSearchResult {
            diag,
            accepted: Some((rotation_offset, translation_offset, score)),
            timing,
            retry: None,
        })
    }
}

impl GscPgo {
    pub fn new(config: Config) -> GscPgo {
        if let Err(e) = config.validate() {
            panic!("gsc_pgo config: {e}");
        }
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
            rotation_offset: mat3::identity(),
            ..Default::default()
        }
    }

    /// Add a scan's range to the running range estimate and, when it updates, set the odometry error estimate it implies.
    fn follow_lidar_range(&mut self, cloud: &[[f32; 3]]) {
        let range = crate::lidar_range::scan_range(cloud);
        let Some(range_m) = self.lidar_range.add(range, &self.config.lidar_range) else {
            return;
        };
        let [xy, z, yaw, tilt] = self.config.lidar_range.odometry_error_estimate_at(range_m);
        let c = &mut self.config;
        [
            c.odometry_error_estimate_xy_per_sqrt_m,
            c.odometry_error_estimate_xy_per_sqrt_rad,
        ] = xy;
        [
            c.odometry_error_estimate_z_per_sqrt_m,
            c.odometry_error_estimate_z_per_sqrt_rad,
        ] = z;
        [
            c.odometry_error_estimate_yaw_per_sqrt_m,
            c.odometry_error_estimate_yaw_per_sqrt_rad,
        ] = yaw;
        [
            c.odometry_error_estimate_tilt_per_sqrt_m,
            c.odometry_error_estimate_tilt_per_sqrt_rad,
        ] = tilt;
    }

    /// The stated odometry error estimate in use (per source xy, z, yaw, tilt: [per sqrt m, per sqrt rad]).
    pub fn config_odometry_error_estimate(&self) -> crate::lidar_range::OdometryErrorEstimate {
        let c = &self.config;
        [
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
        ]
    }

    /// Factor on each odometry variance everything uses: always 1 (the stated estimate; the learned one is only logged).
    pub fn odometry_error_estimate(&self) -> [f64; 4] {
        [1.0; 4]
    }

    /// The learned odometry error estimate (`learned_odometry_error_estimate_from_closures`), as factors on the stated variances; logged, never used.
    pub fn learned_odometry_error_estimate(&self) -> [f64; 4] {
        self.learned_odometry_error_estimate_ln.map(f64::exp)
    }

    /// Stated odometry error estimate the learned one implies, in config units, to start the next run from; and which sources were measured.
    pub fn odometry_error_estimate_for_next_run(&self) -> ([[f64; 2]; 4], [bool; 4]) {
        let stated = self.config_odometry_error_estimate();
        let noise = self.learned_odometry_error_estimate();
        let measured =
            std::array::from_fn(|k| self.learned_odometry_error_estimate_ln_sd[k].is_some());
        // the learned odometry error estimate scales each source's variances, so its drift (a standard deviation) by the square root
        (
            std::array::from_fn(|k| stated[k].map(|d| d * noise[k].sqrt())),
            measured,
        )
    }

    /// Prior sd for each source's ln estimate: the sd weighing `learned_odometry_error_estimate_stiffness` residual checks (infinite at 0).
    pub(crate) fn learned_odometry_error_estimate_prior(&self) -> [f64; 4] {
        [(2.0 / self.config.learned_odometry_error_estimate_stiffness).sqrt(); 4]
    }

    /// After a solve: learn the odometry error estimate from its kept closures; request a re-solve if it moved past its standard error.
    pub(crate) fn update_learned_odometry_error_estimate(
        &mut self,
        evidence: Option<[(f64, f64); 6]>,
    ) {
        if !self.config.learned_odometry_error_estimate_from_closures
            || !self
                .config
                .learned_odometry_error_estimate_stiffness
                .is_finite()
        {
            return;
        }
        // from the solved graph's residuals per source against the checks the graph gave it (`odometry_error_estimate_evidence`)
        let mut ln_odometry = self.learned_odometry_error_estimate_ln;
        if let Some(evidence) = evidence {
            for c in 0..4 {
                let (sum, r) = evidence[c];
                if r > 0.5 && sum.is_finite() {
                    // the solve whitens at the stated estimate, so the residuals' ln scale is 0
                    let (dispatched, prior_sd) =
                        (0.0, self.learned_odometry_error_estimate_prior()[c]);
                    ln_odometry[c] = crate::odometry_error_estimate::residual_ln_ratio(
                        sum, r, dispatched, c, prior_sd,
                    );
                    self.learned_odometry_error_estimate_ln_sd[c] =
                        Some(crate::odometry_error_estimate::residual_ln_ratio_sd(
                            sum,
                            dispatched,
                            ln_odometry[c],
                            c,
                            prior_sd,
                        ));
                }
            }
        }
        self.learned_odometry_error_estimate_ln = ln_odometry;
    }

    /// Cumulative wall seconds per stage: scan context, submaps, ICP, degeneracy, extended ICP, pose-graph solves.
    pub fn stage_seconds(&self) -> [(&'static str, f64); 10] {
        let t = &self.timing;
        [
            ("scan", t.scan_s),
            ("pressure", t.pressure_s),
            ("propose", t.propose_s),
            ("search", t.search_s),
            ("scan_context", t.scan_context_s),
            ("submap", t.submap_s),
            ("icp", t.icp_s),
            ("degeneracy", t.degeneracy_s),
            ("extended_icp", t.extended_icp_s),
            ("solve", t.solve_s),
        ]
    }
    pub fn closure_diagnostics(&self) -> &[ClosureCandidateDiag] {
        &self.closure_diagnostics
    }
    /// Every closure-edge lifecycle event so far, oldest first.
    pub fn closure_edge_events(&self) -> &[ClosureEdgeEvent] {
        &self.closure_edge_events
    }
    fn newest_keyframe_time(&self) -> f64 {
        self.key_poses.last().map_or(f64::NAN, |k| k.time)
    }
    pub(crate) fn push_edge(
        &mut self,
        source_id: usize,
        target_id: usize,
        state: ClosureEdgeState,
        confidence: f64,
        yank_stdevs: f64,
    ) {
        self.closure_edge_events.push(ClosureEdgeEvent {
            time: self.newest_keyframe_time(),
            source_id,
            target_id,
            state,
            confidence: confidence.clamp(0.0, 1.0),
            yank_stdevs,
        });
    }
    /// Keep a search's diagnostics and, if it found a candidate, its proposal and gate outcome as closure-edge events.
    pub(crate) fn record_search(&mut self, mut diag: ClosureCandidateDiag) {
        // a search that ended before its proposal was built never saw the noise scales
        if diag.target_id < 0 {
            diag.learned_odometry_error_estimate = self.learned_odometry_error_estimate();
            diag.odometry_error_estimate_gate = self.odometry_error_estimate();
        } else {
            let (source, target) = (diag.source_id, diag.target_id as usize);
            let proposed = if diag.scan_context_best.is_finite() {
                1.0 - diag.scan_context_best as f64
            } else {
                1.0 - diag.candidate_distance / self.config.closure_search_radius.max(f64::EPSILON)
            };
            self.push_edge(
                source,
                target,
                ClosureEdgeState::Proposed,
                proposed,
                f64::NAN,
            );
            let checked = if diag.confidence.is_finite() {
                diag.confidence
            } else {
                proposed
            };
            let outcome = if diag.outcome == "accepted" {
                ClosureEdgeState::Committed
            } else {
                ClosureEdgeState::GateRejected
            };
            if outcome == ClosureEdgeState::Committed {
                self.committed_stats = Some((checked.clamp(0.0, 1.0), diag.yank_stdevs));
            }
            self.push_edge(source, target, outcome, checked, diag.yank_stdevs);
        }
        self.closure_diagnostics.push(diag);
    }
    /// Keyframe gating by translation / rotation delta against the last keyframe's local pose.
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
    /// Add the scan as a keyframe if it moved far enough (returns whether it did); every `pressure_check_frequency`-th scan checks pressure.
    pub fn add_key_pose(&mut self, cloud_with_pose: &CloudWithPose) -> bool {
        let started = std::time::Instant::now();
        if self.config.odometry_error_estimate_from_lidar_range {
            if let Some(cloud) = &cloud_with_pose.cloud {
                self.follow_lidar_range(cloud);
            }
        }
        // planar_pose_graph: the pose on the floor plane, and the tilt that levels its body-frame cloud
        let level;
        let (pose, tilt) = if self.config.planar_pose_graph {
            let (turn, tilt) = mat3::yaw_and_tilt(&cloud_with_pose.pose.rotation);
            let t = &cloud_with_pose.pose.translation;
            level = PoseWithTime {
                rotation: turn,
                translation: [t[0], t[1], 0.0],
                ..cloud_with_pose.pose.clone()
            };
            (&level, Some(tilt))
        } else {
            (&cloud_with_pose.pose, None)
        };
        // an odometry that repeats each pose with a fresh stamp (huge's fastlio_odometry, ~2.7x) counts each pose once
        if self
            .recent_motion
            .back()
            .is_none_or(|last| last.1 != pose.translation)
        {
            if self.recent_motion.len() > self.config.solve_speed_window_poses {
                self.recent_motion.pop_front();
            }
            self.recent_motion
                .push_back((pose.second, pose.translation));
        }
        let due = self.is_key_pose(pose);
        // re-sent points are matched where the odometry as given places them
        let cloud = if self.config.drop_reseen_points {
            self.drop_reseen(cloud_with_pose)
        } else {
            cloud_with_pose.cloud.clone()
        };
        let cloud = match (&tilt, cloud) {
            (Some(tilt), Some(cloud)) => Some(Arc::new(transform_cloud(&cloud, tilt, &[0.0; 3]))),
            (_, cloud) => cloud,
        };
        let (keyframe, cloud) = if self.config.auto_aggregate_cloud_size > 0 {
            self.gather(pose, cloud, due)
        } else {
            (due, cloud)
        };
        if keyframe {
            self.insert_keyframe(pose, cloud);
            self.keyframe_tilts.extend(tilt);
        }
        self.scans_seen += 1;
        self.scan_time = pose.second;
        if self
            .newest_advancing_time
            .is_none_or(|newest| pose.second > newest)
        {
            self.advancing_scans += 1;
            self.newest_advancing_time = Some(pose.second);
            self.first_advancing_time.get_or_insert(pose.second);
            if !self.closure_searches.is_empty() {
                if self.search_scan_times.is_empty() {
                    self.search_scan_base = self.advancing_scans;
                }
                self.search_scan_times.push_back(pose.second);
            }
        }
        // the running solve's estimate ends at this scan: its data time, to judge an overshoot by
        if let Some(solving) = &mut self.solving {
            if solving.schedule.due_time.is_none() && self.scans_seen >= solving.schedule.due_scan()
            {
                solving.schedule.due_time = Some(pose.second);
            }
        }
        let checked = std::time::Instant::now();
        self.timing.scan_s += (checked - started).as_secs_f64();
        let frequency = self.config.pressure_check_frequency;
        let check_due = frequency > 0 && self.scans_seen.is_multiple_of(frequency);
        // pressure is its own job (`pressure_schedule`); debounce, merge and fast path also land every scan, not only at keyframes
        self.step_pressure(check_due);
        if !self.adoption_inline && self.step_adoption(false) {
            self.adopted_since_update = true;
        }
        // the solver fast path too, so its correction lands on the scan its hold ends at
        if self.step_fast_path() {
            self.adopted_since_update = true;
        }
        self.timing.pressure_s += checked.elapsed().as_secs_f64();
        keyframe
    }
    /// Gather the scan; a due keyframe is made once gathered points reach `auto_aggregate_cloud_size` (cloud in its body frame), else it waits.
    fn gather(
        &mut self,
        pose: &PoseWithTime,
        cloud: Option<Arc<PointCloud>>,
        due: bool,
    ) -> (bool, Option<Arc<PointCloud>>) {
        if let Some(cloud) = &cloud {
            self.gathered
                .extend(transform_cloud(cloud, &pose.rotation, &pose.translation));
            if self.config.submap_resolution > 0.0 {
                self.gathered = voxel_downsample(&self.gathered, self.config.submap_resolution);
            }
        }
        if !due || self.gathered.len() < self.config.auto_aggregate_cloud_size {
            return (false, None);
        }
        // odometry frame -> body: p_body = R^T (p - t)
        let to_body = mat3::transpose(&pose.rotation);
        let offset = mat3::mat_vec(&to_body, &pose.translation).map(|v| -v);
        let body = transform_cloud(&std::mem::take(&mut self.gathered), &to_body, &offset);
        (true, Some(Arc::new(body)))
    }
    /// The scan's cloud without the points the scan before already sent (`drop_reseen_points`).
    fn drop_reseen(&mut self, cloud_with_pose: &CloudWithPose) -> Option<Arc<PointCloud>> {
        let cloud = cloud_with_pose.cloud.as_ref()?;
        let pose = &cloud_with_pose.pose;
        let key = |p: &[f32; 3]| p.map(|v| (v as f64 * 1e4).round() as i64);
        let placed: Vec<[i64; 3]> = transform_cloud(cloud, &pose.rotation, &pose.translation)
            .iter()
            .map(key)
            .collect();
        let fresh: PointCloud = cloud
            .iter()
            .zip(&placed)
            .filter(|(_, k)| !self.reseen_previous.contains(*k))
            .map(|(p, _)| *p)
            .collect();
        self.reseen_previous = placed.into_iter().collect();
        Some(Arc::new(fresh))
    }
    /// Body speed (m/s): median over the newest `solve_speed_window_poses` steps between distinct odometry poses; NaN before two.
    pub(crate) fn recent_speed(&self) -> f64 {
        let mut speeds: Vec<f64> = self
            .recent_motion
            .iter()
            .zip(self.recent_motion.iter().skip(1))
            .filter(|(a, b)| b.0 > a.0)
            .map(|(a, b)| mat3::norm(&mat3::sub(&b.1, &a.1)) / (b.0 - a.0))
            .collect();
        if speeds.is_empty() {
            return f64::NAN;
        }
        speeds.sort_by(f64::total_cmp);
        speeds[speeds.len() / 2]
    }
    /// The newest loop-overlap pressure and the scan count it was computed at (see `pressure_check_frequency`).
    pub fn pressure(&self) -> Option<(usize, crate::loop_overlap::LoopOverlap)> {
        self.pressure
    }
    pub fn key_poses(&self) -> &KeyFrames {
        &self.key_poses
    }
    pub fn rotation_offset(&self) -> Mat3 {
        self.rotation_offset
    }
    pub fn translation_offset(&self) -> Vec3 {
        self.translation_offset
    }

    /// Stages 1 and 2 for the newest keyframe: it becomes the waiting proposal if its match beats the waiting one (loser: "outranked").
    pub fn propose_closure(&mut self) {
        let started = std::time::Instant::now();
        self.propose();
        self.timing.propose_s += started.elapsed().as_secs_f64();
    }
    fn propose(&mut self) {
        if self.config.closure_sources == ClosureSources::InputsOnly {
            self.propose_from_landmarks();
            return;
        }
        // nothing to close onto until a keyframe exists whose submap leaves out this one's scan
        if self.key_poses.is_empty() || self.newest_closure_target() == 0 {
            return;
        }
        let cur_idx = self.key_poses.len() - 1;
        // an older keyframe's waiting proposal is dropped (recorded "stale"), not queued
        if let Some(stale) = self
            .waiting_proposal
            .take_if(|waiting| waiting.diag.source_id < cur_idx)
        {
            let mut diag = stale.diag;
            diag.outcome = "stale";
            self.record_search(diag);
        }
        let current = &self.key_poses.get(cur_idx);
        let mut diag = ClosureCandidateDiag::new(cur_idx, current.time, current.translation_global);

        // stage 1, universal uniqueness: a scan whose structure barely fills the descriptor can't place itself
        let descriptor = self.key_poses.descriptor(cur_idx);
        if !descriptor.is_empty() {
            diag.descriptor_occupancy =
                similarity_discovery::descriptor_occupancy(&descriptor) as i32;
            if diag.descriptor_occupancy < self.config.closure_min_occupancy {
                diag.outcome = "low_occupancy";
                self.record_search(diag);
                return;
            }
        }

        // stage 2, similarity discovery: scan context, else the nearest past keyframe by position
        let t0 = std::time::Instant::now();
        let (scan_context_target, sector_shift, sc_best, sc_second, ranked, alternatives) =
            self.search_by_scan_context();
        let lookup_s = t0.elapsed().as_secs_f64();
        self.timing.scan_context_s += lookup_s;
        diag.cost_lookup_s = lookup_s;
        diag.speed_mps = self.recent_speed();
        diag.scan_index_proposed = self.scans_seen;
        diag.from_scan_context = scan_context_target.is_some();
        let Some(target_idx) = scan_context_target.or_else(|| self.search_by_position()) else {
            diag.outcome = "no_candidate";
            self.record_search(diag);
            return;
        };
        diag.target_id = target_idx as i64;
        diag.target_time = self.key_poses.fixed(target_idx).time;
        diag.target_xyz = self.key_poses.position(target_idx);
        diag.candidate_distance = mat3::norm(&mat3::sub(&diag.source_xyz, &diag.target_xyz));
        if diag.from_scan_context && sc_second > 1e-6 {
            diag.scan_context_best = sc_best;
            diag.scan_context_second = sc_second;
            diag.lowe_ratio = sc_best / sc_second;
        }
        // a position-only candidate ranks below any scan-context match (cosine distance is at most 2)
        let similarity = if diag.from_scan_context { sc_best } else { 2.0 };
        let proposal = ClosureProposal {
            diag,
            target_idx,
            sector_shift,
            similarity,
            ranked,
            alternatives,
            tries: 0,
            landmark_pair: None,
        };
        self.offer_proposal(proposal);
    }
    /// `inputs_only`: place the sightings up to the newest keyframe; a landmark seen on a later visit than its first proposes a closure from that sighting's keyframe onto the first visit's nearest sighting, at most one per target submap span per landmark.
    fn propose_from_landmarks(&mut self) {
        let Some(newest_time) = self.key_poses.last().map(|k| k.time) else {
            return;
        };
        let half_range = self.config.target_half_range(1);
        let mut candidate: Option<LandmarkPair> = None;
        // a sighting after the newest keyframe waits: a later keyframe may be nearer
        while let Some(&(time, landmark, rotation, position, information)) =
            self.landmark_sightings.get(self.sightings_placed)
        {
            if time > newest_time {
                break;
            }
            self.sightings_placed += 1;
            let Some((keyframe, moved)) = self.nearest_keyframe(time) else {
                continue;
            };
            // planar_pose_graph: seen from the keyframe's level frame
            let (_, position, information) = match self.keyframe_tilts.get(keyframe) {
                Some(tilt) => crate::landmarks::leveled(tilt, &rotation, &position, &information),
                None => (rotation, position, information),
            };
            let full = crate::landmarks::known_inverse(&information);
            // the position block only (a rotation-only sighting places nothing)
            let mut covariance: Mat3 =
                std::array::from_fn(|i| std::array::from_fn(|j| full[3 + i][3 + j]));
            if (0..3).any(|i| covariance[i][i] <= 0.0) {
                continue;
            }
            for (axis, row) in covariance.iter_mut().enumerate() {
                row[axis] += moved;
            }
            let sighting = PlacedSighting {
                keyframe,
                position,
                covariance,
            };
            let visits = self
                .landmark_visits
                .entry(landmark)
                .or_insert(LandmarkVisits {
                    first_visit_best: sighting,
                    first_visit_newest: keyframe,
                    first_visit_over: false,
                    newest_proposed: None,
                });
            // the first visit lasts while sightings keep coming within the target submap's half range of each other
            if !visits.first_visit_over {
                if keyframe <= visits.first_visit_newest + half_range {
                    visits.first_visit_newest = visits.first_visit_newest.max(keyframe);
                    if mat3::norm(&position) < mat3::norm(&visits.first_visit_best.position) {
                        visits.first_visit_best = sighting;
                    }
                    continue;
                }
                visits.first_visit_over = true;
            }
            // the two keyframes' submaps must not share a scan, and one proposal per half range of source keyframes
            let target = visits.first_visit_best;
            if !self.config.landmark_closure_icp
                || keyframe <= target.keyframe + 2 * half_range + 1
                || visits
                    .newest_proposed
                    .is_some_and(|proposed| keyframe < proposed + half_range)
            {
                continue;
            }
            visits.newest_proposed = Some(keyframe);
            candidate = Some(LandmarkPair {
                source: sighting,
                target,
            });
        }
        let Some(pair) = candidate else {
            return;
        };
        let (source_id, target_idx) = (pair.source.keyframe, pair.target.keyframe);
        let source = self.key_poses.get(source_id);
        let mut diag = ClosureCandidateDiag::new(source_id, source.time, source.translation_global);
        diag.speed_mps = self.recent_speed();
        diag.scan_index_proposed = self.scans_seen;
        diag.target_id = target_idx as i64;
        diag.target_time = self.key_poses.fixed(target_idx).time;
        diag.target_xyz = self.key_poses.position(target_idx);
        diag.candidate_distance = mat3::norm(&mat3::sub(&diag.source_xyz, &diag.target_xyz));
        // the landmark already names the place, so it ranks above any place match
        self.offer_proposal(ClosureProposal {
            diag,
            target_idx,
            sector_shift: 0,
            similarity: 0.0,
            ranked: Vec::new(),
            alternatives: Vec::new(),
            tries: 0,
            landmark_pair: Some(pair),
        });
    }
    /// Make `proposal` the waiting one if its match beats the one already waiting; the loser is recorded as "outranked".
    fn offer_proposal(&mut self, proposal: ClosureProposal) {
        let (winner, loser) = match self.waiting_proposal.take() {
            None => (proposal, None),
            Some(waiting) if waiting.similarity <= proposal.similarity => (waiting, Some(proposal)),
            Some(waiting) => (proposal, Some(waiting)),
        };
        self.waiting_proposal = Some(winner);
        let Some(loser) = loser else {
            return;
        };
        let mut diag = loser.diag;
        diag.outcome = "outranked";
        self.record_search(diag);
    }
    /// Take the waiting proposal as a `ClosureSearch` (stages 3 and 4) to run on any thread and hand to `finish_closure_search`.
    pub fn begin_closure_search(&mut self) -> Option<ClosureSearch> {
        let started = std::time::Instant::now();
        let mut search = self.begin();
        let seconds = started.elapsed().as_secs_f64();
        self.timing.propose_s += seconds;
        if let Some(search) = &mut search {
            let diag = &mut search.diag;
            diag.cost_begin_s = seconds;
            diag.map_keyframes = search.key_poses.len();
            diag.scan_context_candidates = search.ranked.len();
            diag.alternatives_left = search.alternatives.len();
            diag.alternative_try = search.tries;
        }
        search
    }
    fn begin(&mut self) -> Option<ClosureSearch> {
        let ClosureProposal {
            diag,
            target_idx,
            sector_shift,
            ranked,
            alternatives,
            tries,
            landmark_pair,
            ..
        } = self.waiting_proposal.take()?;
        let mut diag = diag;
        diag.pressure = self
            .pressure
            .map_or(0.0, |(_, overlap)| overlap.unresolved_loop_probability);
        diag.learned_odometry_error_estimate = self.learned_odometry_error_estimate();
        diag.odometry_error_estimate_gate = self.odometry_error_estimate();
        let source_position = self.key_poses.position(diag.source_id);
        let target_position = self.key_poses.position(target_idx);
        // ICP seeds the source at the target's position turned by the scan-context yaw (after long drift its own position wouldn't overlap)
        let yaw = if sector_shift != 0 {
            similarity_discovery::yaw_from_shift(sector_shift, self.scan_context_config.n_sectors)
        } else {
            0.0
        };
        let mut init_rotation = mat3::rot_z(yaw);
        let mut init_translation = mat3::sub(
            &target_position,
            &mat3::mat_vec(&init_rotation, &source_position),
        );
        // a landmark seen from both: shift the source so its sighting lands where the target's puts the landmark (rotation kept)
        let mut landmark_seed_covariance = None;
        if let Some(pair) = landmark_pair {
            let (source, target) = (
                self.key_poses.get(diag.source_id),
                self.key_poses.get(target_idx),
            );
            let placed = |key_pose: &KeyPoseWithCloud, sighting: &PlacedSighting| {
                (
                    mat3::add(
                        &mat3::mat_vec(&key_pose.rotation_global, &sighting.position),
                        &key_pose.translation_global,
                    ),
                    mat3::mat_mul(
                        &mat3::mat_mul(&key_pose.rotation_global, &sighting.covariance),
                        &mat3::transpose(&key_pose.rotation_global),
                    ),
                )
            };
            let ((from_target, target_covariance), (from_source, source_covariance)) =
                (placed(&target, &pair.target), placed(&source, &pair.source));
            init_rotation = mat3::identity();
            init_translation = mat3::sub(&from_target, &from_source);
            landmark_seed_covariance = Some(mat3::add_mat(&target_covariance, &source_covariance));
        }
        Some(ClosureSearch {
            key_poses: self.key_poses.clone(),
            config: self.config.clone(),
            // set where the search starts (`prepare`), from the snapshots
            mean_step_xy_variance: f64::NAN,
            kept_closures: Vec::new(),
            closures: self.committed_closures.clone(),
            keyframe_reach: self.keyframe_reach,
            error_estimate: self.odometry_error_estimate(),
            solved_relative_covariance: self.solved_relative_covariance.clone(),
            diag,
            target_idx,
            init_rotation,
            init_translation,
            scan_context_config: self.scan_context_config,
            ranked,
            judged_yanks: Vec::new(),
            alternatives,
            tries: tries + 1,
            icp_cache: self.icp_cache.clone(),
            keyframe_clouds: self.keyframe_clouds.clone(),
            dump_s: Arc::new(std::sync::Mutex::new(0.0)),
            progress: None,
            mean_reach: if self.keyframe_reach.1 > 0 {
                self.keyframe_reach.0 / self.keyframe_reach.1 as f64
            } else {
                self.config.closure_search_radius
            },
            landmark_seed_covariance,
        })
    }
    /// Apply a `ClosureSearch`'s result: record its diagnostics and commit the closure it accepted.
    pub fn finish_closure_search(&mut self, result: ClosureSearchResult) {
        let ClosureSearchResult {
            mut diag,
            accepted,
            timing,
            retry,
        } = result;
        diag.scan_index_finished = self.scans_seen;
        self.timing.submap_s += timing.submap_s;
        self.timing.icp_s += timing.icp_s;
        self.timing.degeneracy_s += timing.degeneracy_s;
        self.timing.extended_icp_s += timing.extended_icp_s;
        self.timing.search_s += timing.search_s;
        let Some((rotation_offset, translation_offset, score)) = accepted else {
            if let Some((ranked, alternatives, tries)) = retry {
                self.retry_alternative(&diag, ranked, alternatives, tries);
            }
            self.record_search(diag);
            return;
        };
        let (source, target) = (diag.source_id, diag.target_id as usize);
        self.record_search(diag);
        self.commit_closure(source, target, rotation_offset, translation_offset, score);
    }
    /// Offer the rejected keyframe again at its next scan-context match while tries remain (`closure_alternative_places`).
    fn retry_alternative(
        &mut self,
        rejected: &ClosureCandidateDiag,
        ranked: Vec<(f32, usize)>,
        mut alternatives: Vec<(f32, usize, i32)>,
        tries: usize,
    ) {
        if !self.config.closure_alternative_places
            || rejected.pressure < self.config.closure_alternative_places_min_pressure
            || alternatives.is_empty()
            || rejected.source_id >= self.key_poses.len()
        {
            return;
        }
        // extra tries grow with pressure under a square root: retries far down the match list rarely close
        let extra = (1.0 / (1.0 - rejected.pressure).max(f64::EPSILON))
            .ceil()
            .min(self.scan_context_config.candidate_top_k.saturating_sub(1) as f64);
        let allowed = 1 + extra.sqrt().floor() as usize;
        if tries >= allowed {
            return;
        }
        let (distance, target_idx, sector_shift) = alternatives.remove(0);
        let mut diag = ClosureCandidateDiag::new(
            rejected.source_id,
            rejected.source_time,
            rejected.source_xyz,
        );
        diag.descriptor_occupancy = rejected.descriptor_occupancy;
        diag.from_scan_context = true;
        diag.target_id = target_idx as i64;
        diag.target_time = self.key_poses.fixed(target_idx).time;
        diag.target_xyz = self.key_poses.position(target_idx);
        diag.candidate_distance = mat3::norm(&mat3::sub(&diag.source_xyz, &diag.target_xyz));
        diag.scan_context_best = distance;
        diag.scan_context_second = rejected.scan_context_second;
        diag.cost_lookup_s = 0.0;
        diag.speed_mps = self.recent_speed();
        diag.scan_index_proposed = self.scans_seen;
        self.offer_proposal(ClosureProposal {
            diag,
            target_idx,
            sector_shift,
            similarity: distance,
            ranked,
            alternatives,
            tries,
            landmark_pair: None,
        });
    }
    /// Add a keyframe at `pose`, placed by the current correction.
    fn insert_keyframe(&mut self, pose: &PoseWithTime, cloud: Option<Arc<PointCloud>>) {
        // a solve waiting on the graph change heat is adopted at the last keyframe count's end once the heat is under
        let keyframes = self.key_poses.len();
        self.release_pending_solve(keyframes);
        self.graph_version += 1;
        // stored voxelized at submap_resolution, as every use voxelizes it (a dense scan would otherwise be kept whole)
        let stored = match &cloud {
            Some(cloud) if self.config.submap_resolution > 0.0 => Some(Arc::new(voxel_downsample(
                cloud,
                self.config.submap_resolution,
            ))),
            _ => cloud.clone(),
        };
        // how far this keyframe's scan reaches, for loop_overlap's sensor range
        if let Some(stored) = &stored {
            let reach = similarity_discovery::auto_max_range(stored);
            if reach > 0.0 {
                self.keyframe_reach.0 += reach;
                self.keyframe_reach.1 += 1;
            }
        }
        // scan context descriptor and ring key (empty without a cloud); a non-positive max range is set once from the first cloud
        if let Some(cloud) = &cloud {
            if self.scan_context_config.max_range_m <= 0.0 {
                let auto_range = similarity_discovery::auto_max_range(cloud);
                if auto_range > 0.0 {
                    self.scan_context_config.max_range_m = auto_range;
                    tracing::info!(
                        "scan-context: auto ring range = {auto_range:.2} m (from cloud extent)"
                    );
                }
            }
        }
        let descriptor = match &cloud {
            Some(cloud) if self.scan_context_config.max_range_m > 0.0 => {
                similarity_discovery::make_descriptor(cloud, &self.scan_context_config)
            }
            _ => similarity_discovery::Descriptor::empty(),
        };
        // an empty descriptor's ring key is empty too
        self.scan_context_ring_keys
            .push(similarity_discovery::make_ring_key(&descriptor));
        // placed by the current correction (`KeyFrames::global` rides it on the odometry pose when read)
        self.key_poses.push(crate::store::KeyFixed::new(
            pose.rotation,
            pose.translation,
            pose.second,
            stored,
            Arc::new(descriptor),
        ));
        self.simplify_step();
    }

    /// Commit a closure that passed every gate; the next `update` starts a solve with it.
    pub fn commit_closure(
        &mut self,
        source_id: usize,
        target_id: usize,
        rotation_offset: Mat3,
        translation_offset: Vec3,
        score: f64,
    ) {
        // a closure committed directly (no search) has no search confidence or yank recorded
        let (fit_confidence, yank_stdevs) =
            self.committed_stats.take().unwrap_or((f64::NAN, f64::NAN));
        let (rotation_offset, translation_offset) = if self.config.planar_pose_graph {
            planar(&rotation_offset, &translation_offset)
        } else {
            (rotation_offset, translation_offset)
        };
        self.committed_closures.push(crate::store::ClosureFixed {
            source_id,
            target_id,
            rotation_offset,
            translation_offset,
            score,
            fit_confidence,
            yank_stdevs,
        });
        self.new_closure = true;
        self.closure_latency
            .push(crate::fast_path::ClosureLatency::committed(
                self.scans_seen,
                self.newest_scan_time(),
            ));
    }
}

#[cfg(test)]
mod config_tests {
    use super::Config;

    /// The target submap's half range is 5 for the single scan and grows with the extension's source clouds.
    #[test]
    fn the_target_range_grows_with_the_extension() {
        let config = Config::default();
        assert_eq!(config.target_half_range(1), 5);
        assert_eq!(config.target_half_range(2), 6);
        assert_eq!(config.target_half_range(4), 9);
        assert_eq!(config.target_half_range(0), 5);
    }

    /// The uncertainty range must straddle the bar: barely_fail below 1, barely_pass above it.
    #[test]
    fn the_uncertainty_range_must_straddle_one() {
        assert!(Config::default().validate().is_ok());
        for (fail, pass) in [
            (1.0, 1.05),
            (1.2, 1.5),
            (0.95, 1.0),
            (0.95, 0.9),
            (f64::NAN, 1.05),
        ] {
            let config = Config {
                barely_fail_uncertainty_range: fail,
                barely_pass_uncertainty_range: pass,
                ..Config::default()
            };
            assert!(config.validate().is_err(), "{fail} {pass}");
        }
    }

    /// The learned odometry error estimate is only logged: the solve and the gates use the stated one.
    #[test]
    fn the_solve_and_gates_use_the_stated_odometry_error_estimate() {
        let config = Config {
            learned_odometry_error_estimate_from_closures: true,
            ..Config::default()
        };
        assert_eq!(
            super::GscPgo::new(config).odometry_error_estimate(),
            [1.0; 4]
        );
    }
}
