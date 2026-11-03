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

//! `GscPgo`: keyframe gating, the closure search and its gates, and the pose graph (each keyframe's
//! odometry plus the closures that passed the gates), re-solved by `pose_graph` on a background thread
//! whenever new closures arrive. Keyframe ids are their indices; rotations and vectors are `mat3`
//! (row-major f64).

use std::sync::Arc;

use crate::store::KeyFrames;
use crate::big_picture_fit::{closure_edges, Edge};
use crate::connection_fit::{dump_icp, IcpParams};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{transform_cloud, voxel_downsample, KdTree, PointCloud};
use crate::pose_graph_solving::Solving;
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

/// A closure that passed every gate: its relative pose (source in the target's frame), its ICP
/// fitness, and the newest applied solve's verdict.
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

/// Per-search loop-closure diagnostics
/// one record every time `search_for_closure_pairs` evaluates a keyframe, capturing the metric at each
/// gate and the outcome (accepted, or the name of the gate that rejected it).
/// Lets an offline eval reconstruct WHERE and WHY closures are accepted/rejected
#[derive(Debug, Clone)]
#[cfg_attr(feature = "native", derive(serde::Serialize))]
pub struct ClosureCandidateDiag {
    /// "accepted", "outranked" (a better proposal took the search thread), "stale" (still waiting for the busy search
    /// thread when a newer keyframe arrived), or the gate that rejected it:
    /// "low_occupancy", "no_candidate", "degeneracy_reject", "icp_reject", "uninformative", "yank_confidence_reject".
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
    /// Best / second-best scan-context distances and their Lowe ratio
    /// (best/second). NaN when the candidate came from the position fallback.
    pub scan_context_best: f32,
    pub scan_context_second: f32,
    pub lowe_ratio: f32,
    /// The search's inputs from the rest of the run: the loop-overlap pressure when it started (`GscPgo::pressure`, 0
    /// before any), the share of judged closures with a similar yank_stdevs that the solve kept ((kept + 1) / (judged + 2),
    /// each weighted by e^(-(yank_stdevs difference)^2 / 2)), the extension budget in
    /// keyframes (closure_extended_icp_max_keyframes x (1 - (1 - pressure)(1 - target_overlap)), up to the next doubling step), and the separate places the scan matched as well as
    /// this target, counted only as far as the decision needed (0 when it was never counted).
    pub pressure: f64,
    /// Pressure 2 when the search started (`LoopOverlap::unclosed_overlap_probability`).
    pub pressure2: f64,
    /// The odometry error estimate when the search started, as ratios to the reported variances (xy, z, yaw, roll/pitch;
    /// `GscPgo::odometry_error_estimate`).
    pub learned_odometry_error_estimate: [f64; 4],
    /// The same as the uninformative gate measures the graph with (`GscPgo::odometry_error_estimate`, the stated odometry error estimate).
    pub odometry_error_estimate_gate: [f64; 4],
    /// The closure's innovation: the source keyframe's offset from where the graph put it, relative to the target, in
    /// world axes (m), and the graph's covariance of that offset at the odometry error estimate in use (m²); NaN before ICP.
    pub innovation: Vec3,
    pub innovation_covariance: Mat3,
    /// The chance the source scan overlaps the target keyframe's (`OverlapModel::overlap`): what the extension budget
    /// scales with.
    pub target_overlap: f64,
    pub keep_rate: f64,
    pub extension_budget: usize,
    pub look_alike_places: usize,
    /// Global distance between source and candidate keyframe.
    pub candidate_distance: f64,
    /// Minimum degeneracy (observability) of the source submap, -1 if not
    /// computed on this path.
    pub degeneracy_min: f32,
    pub icp_converged: bool,
    /// The single ICP's iterations and why it stopped (`IcpStop::name`; "" if ICP did not run, 0 iterations when read
    /// from a cache written before these were logged).
    pub icp_iterations: u32,
    pub icp_stop: &'static str,
    /// How far ICP moved the source from its initial guess (the scan-context seed): the distance (m) between where the
    /// guess and the result put the source keyframe, and the rotation (deg) between them; NaN if ICP did not run.
    pub icp_init_offset_m: f64,
    pub icp_init_offset_deg: f64,
    /// ICP fitness (the score-threshold gate input); NaN if ICP did not run.
    pub icp_fitness: f64,
    /// Fraction of the source scan that overlaps the target after ICP (see `IcpResult`); NaN if ICP did not run.
    pub icp_overlap: f64,
    /// Magnitude of the ICP translation correction, NaN if ICP did not run.
    pub icp_translation_norm: f64,
    /// How far the accepted constraint pulls the graph away from the current
    /// odom-chain estimate: translation (m) and rotation (deg) between the
    /// ICP-refined relative pose and the current relative pose. A large yank on
    /// a plausible-distance candidate is the signature of a bad closure that
    /// disagrees with odometry. NaN if ICP did not run.
    pub yank_translation: f64,
    pub yank_rotation_deg: f64,
    /// The inputs and outputs of the yank / confidence gate , recorded for every
    /// candidate that reaches ICP (NaN otherwise) whether the gate is on or not:
    /// yank_stdevs = the closure's innovation in standard deviations of its predicted spread, sqrt(v^T (P + c I)^-1 v):
    /// P the graph's covariance of the two ends' relative position, c the closure's variance (its ICP inliers' mean square);
    /// (infinite before any, so the first candidate only seeds the scale); each factor is its metric normalized by its own gate and clamped to [0, 1];
    /// confidence = their product; required_confidence = 1 - exp(-yank_stdevs^2 / 2k^2), k = `closure_yank_gate_width`.
    pub yank_stdevs: f64,
    pub scan_context_factor: f64,
    pub icp_factor: f64,
    pub degeneracy_factor: f64,
    pub confidence: f64,
    pub required_confidence: f64,
    /// Extended ICP (`closure_extended_icp_max_keyframes`), recorded when it ran (0 / NaN otherwise):
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
    /// The extension's confidence ratio per cloud (evidence over the bar, yank units), the single scan's first; the
    /// candidate's score is their median.
    pub cloud_ratios: Vec<f64>,
    /// How far each extension step's alignment is from the one before it (first fit, cloud 1, cloud 2, ...) at the
    /// source keyframe, in inlier distances, and the agreement multiplier through the last step (1 before any; it
    /// scales only the yank gate's score, `compounding_agreement_sensitivity`).
    pub cloud_agreement: Vec<f64>,
    /// The search's cost, for fitting estimates of it from what is known before it starts (seconds, NaN where a step did
    /// not run): the scan-context lookup (0 for a retry, which reuses its keyframe's), taking the proposal as a search
    /// (`begin_closure_search`), building the source scan and target submap, the degeneracy check, the single ICP, the
    /// extension (each round, and in all), the rest of the search (gates, likelihoods, sharpness), and the whole search.
    pub cost_lookup_s: f64,
    pub cost_begin_s: f64,
    pub cost_submap_s: f64,
    pub cost_degeneracy_s: f64,
    pub cost_icp_s: f64,
    pub cost_extension_s: f64,
    pub cost_extension_rounds_s: Vec<f64>,
    pub cost_gates_s: f64,
    pub cost_search_s: f64,
    /// What was known before the search: the map's keyframes, the scan-context candidates ranked and the alternative
    /// places left (and this one's try, 1 = the best match), the extension clouds planned and fitted, the single ICP's
    /// source, target and thinned-target points, the body's speed (m/s, `recent_speed`), and the scan count when the search was proposed and when its result landed.
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
    /// The deterministic schedule (`closure_search_hold_enabled`): the scan count at the search's dispatch, its estimate
    /// (seconds, the extension rounds it ran included, and scans after dispatch), and whether it ran past the data time
    /// its estimate spanned.
    pub scan_index_dispatched: usize,
    pub estimate_raw_scans: f64,
    pub estimate_scans: usize,
    pub search_overshoot: bool,
    /// The points the dispatch-time estimate counted: the voxelized source keyframe's, and the single target submap's
    /// keyframes' summed before merging.
    pub dispatch_source_points: usize,
    pub dispatch_target_points: usize,
    /// The two-stage search (`closure_search_schedule`): the single scan's confidence ratio a match inside the
    /// uncertainty range is ranked by in the verification buffer, and its verification job's schedule (as the fast
    /// match's above); each extension round's separate source cloud and target points.
    pub promise: f64,
    pub verification_scan_index_dispatched: usize,
    pub verification_estimate_raw_scans: f64,
    pub verification_estimate_scans: usize,
    pub verification_overshoot: bool,
    pub extension_rounds_source_points: Vec<usize>,
    pub extension_rounds_target_points: Vec<usize>,
    pub agreement_multiplier: f64,
    /// `connection_fit::sharpness` of the single-scan and the last extended alignment (NaN when not scored).
    pub sharpness: f64,
    pub extended_sharpness: f64,
    /// `connection_fit::likelihood_ratio` (mean log ratio, yank units) of the single-scan and the last extended alignment, and
    /// the aligned-distance width it measured (NaN when not scored).
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
            pressure2: 0.0,
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
            sharpness: f64::NAN,
            extended_sharpness: f64::NAN,
            likelihood_ratio: f64::NAN,
            extended_likelihood_ratio: f64::NAN,
            likelihood_width: f64::NAN,
        }
    }
}

/// Where a closure candidate is in its life: proposed by the search, through the gates, GNC's
/// votes, in or out of the graph, and applied to the map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ClosureEdgeState {
    /// The search found a candidate keyframe for this one.
    Proposed = 0,
    /// A gate (distance, degeneracy, ICP, yank, ...) turned it down; it never reaches GNC.
    GateRejected = 1,
    /// Passed every gate; waits for GNC to vote it into the graph.
    Committed = 2,
    // 3 was a single GNC solve's weight, no longer sent
    /// GNC voted it into the graph.
    Accepted = 4,
    /// GNC voted it out of the graph (never in, or taken back out).
    Rejected = 5,
    /// The graph update that moved the map with it in force.
    Applied = 6,
}

/// One step in a closure candidate's life, for visualizing proposed / accepted /
/// rejected / applied edges as they happen. Purely observational.
#[derive(Debug, Clone, Copy)]
pub struct ClosureEdgeEvent {
    /// Keyframe time of the newest keyframe when it happened.
    pub time: f64,
    pub source_id: usize,
    pub target_id: usize,
    pub state: ClosureEdgeState,
    /// How likely the edge is right, in [0, 1]: scan-context similarity while only proposed,
    /// ICP fitness against the fitness its yank requires once ICP ran, then GNC's weight times that.
    pub confidence: f64,
    /// How hard the edge pulls the accepted pose graph, in robust standard deviations of the
    /// yanks of closures committed so far (`ClosureCandidateDiag::yank_stdevs`); NaN before ICP.
    pub yank_stdevs: f64,
}

/// Bytes per packed closure-edge record (`ClosureEdgeEvent::put_record`).
pub const CLOSURE_EDGE_RECORD_BYTES: usize = 52;

impl ClosureEdgeEvent {
    /// Append this event as one packed record: both keyframes where they are now (x y z, then
    /// target x y z, f32), source and target ids, state as u32, confidence f32, yank_stdevs f32, time f64.
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

/// One LET kind's overshoots: over the run, and since the last warning (`Config::overshoot_warning_interval_s`).
#[derive(Default)]
pub(crate) struct OvershootTally {
    pub(crate) jobs: usize,
    pub(crate) overshoots: usize,
    jobs_since_warning: usize,
    overshoots_since_warning: usize,
}

impl GscPgo {
    /// Count a landed job's overshoot verdict; on an overshoot, warn (a log line only: nothing reads it) when
    /// `overshoot_warning_interval_s` of data has passed since the last warning, with each kind's overshoots and jobs since.
    pub(crate) fn note_overshoot(&mut self, kind: &'static str, overshoot: bool) {
        let tally = self.overshoot_tally.entry(kind).or_default();
        tally.jobs += 1;
        tally.jobs_since_warning += 1;
        if !overshoot {
            return;
        }
        tally.overshoots += 1;
        tally.overshoots_since_warning += 1;
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
            .filter(|(_, tally)| tally.jobs_since_warning > 0)
            .map(|(kind, tally)| {
                let line = format!("{kind} {} of {}", tally.overshoots_since_warning, tally.jobs_since_warning);
                tally.jobs_since_warning = 0;
                tally.overshoots_since_warning = 0;
                line
            })
            .collect();
        eprintln!(
            "gsc_pgo: WARNING: jobs ran past their deterministic_config estimates (overshoots since the last warning: {}); this machine is slower than the config's, so a replay waits for them and a live run lands them late; a fast-mode run prints a deterministic_config that fits it",
            counts.join(", ")
        );
    }
    /// Per LET kind, its jobs and overshoots over the run (a report only).
    pub fn overshoot_report(&self) -> Vec<(&'static str, usize, usize)> {
        self.overshoot_tally
            .iter()
            .map(|(&kind, tally)| (kind, tally.jobs, tally.overshoots))
            .collect()
    }
}

/// `Config::mode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(
    feature = "native",
    derive(serde::Serialize, serde::Deserialize),
    serde(rename_all = "lowercase")
)]
pub enum Mode {
    Fast,
    Deterministic,
}

/// The deterministic mode's estimates (`Config::deterministic_config`), in scans: a job holds `ceil(estimate)` scans, at
/// least one. The defaults are the Jetson (R1) fit: each job's measured seconds turned into scans by its recording's scan
/// rate, bounded at >= 99% of each recording's jobs.
#[derive(Debug, Clone)]
#[cfg_attr(
    feature = "native",
    derive(serde::Serialize, serde::Deserialize),
    serde(default, deny_unknown_fields)
)]
pub struct DeterministicConfig {
    /// The fast match (a search up to the extension), at dispatch: fixed, per keyframe (the scan-context lookup; none for
    /// a retry), per point (the voxelized source keyframe's and the single target submap's keyframes' before merging)
    /// and per source point.
    pub fast_match_scans_fixed_overhead: f64,
    pub fast_match_scans_per_keyframe: f64,
    pub fast_match_scans_per_point: f64,
    pub fast_match_scans_per_source_point: f64,
    /// The verification (the extension, in a whole search or its own job): fixed as it starts, and before each round
    /// that round's own: per round + per point of its source cloud and target (both known as it starts).
    pub verification_scans_fixed_overhead: f64,
    pub verification_scans_per_round: f64,
    pub verification_scans_per_round_point: f64,
    /// The solve: `solve_scans_scale * keyframes^solve_keyframe_power * (1 + closures)^solve_closure_power * (1 +
    /// rejected)^solve_rejected_power`, the closures in the solve and the ones the newest adopted solve rejected.
    pub solve_scans_scale: f64,
    pub solve_keyframe_power: f64,
    pub solve_closure_power: f64,
    pub solve_rejected_power: f64,
    /// The debounce (`adoption_schedule`): the graph change heat's cost of a finished solve, per keyframe.
    pub debounce_scans_per_keyframe: f64,
    /// The merge (`adoption_schedule`): preparing the adoption's new graph version, per keyframe.
    pub merge_scans_per_keyframe: f64,
    /// The loop-overlap pressure: fixed, per keyframe and per committed closure.
    pub pressure_scans_fixed_overhead: f64,
    pub pressure_scans_per_keyframe: f64,
    pub pressure_scans_per_closure: f64,
}

impl Default for DeterministicConfig {
    fn default() -> DeterministicConfig {
        // the R1 Jetson's (gsc_stable_0.0.3 fast mode at 1x under MLS + raytracer load): each key's largest suggested
        // value over nine maps, solve powers from the Mac fit; a faster machine replays a Jetson run with these
        DeterministicConfig {
            fast_match_scans_fixed_overhead: 1.3885433563125162,
            fast_match_scans_per_keyframe: 1.448251991030438e-07,
            fast_match_scans_per_point: 7.970467536600849e-05,
            fast_match_scans_per_source_point: 0.00019838511484150277,
            verification_scans_fixed_overhead: 5.036066465159226,
            verification_scans_per_round: 0.7489339387672495,
            verification_scans_per_round_point: 6.563647896724165e-05,
            solve_scans_scale: 9.097483642429608e-05,
            solve_keyframe_power: 2.017,
            solve_closure_power: -0.215,
            solve_rejected_power: 0.558,
            debounce_scans_per_keyframe: 6.093487031356368e-06,
            merge_scans_per_keyframe: 3.0032671115943386e-07,
            pressure_scans_fixed_overhead: 0.0007777038336858567,
            pressure_scans_per_keyframe: 8.708997476399469e-06,
            pressure_scans_per_closure: 1.2578234731596874e-05,
        }
    }
}

impl Config {
    /// Whether every LET job is held to its estimate (`Mode::Deterministic`).
    pub fn deterministic(&self) -> bool {
        self.mode == Mode::Deterministic
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
    /// `deterministic` (the default): every logical-execution-time job (solve, closure search, verification, pressure) is
    /// held to its estimate (`deterministic_config`), so a replay gives the same graph on any machine. `fast` (a real-life
    /// run): every job lands as soon as it finishes, no holds; the run measures what each job cost in scans and prints the
    /// `deterministic_config` it suggests (`GscPgo::runtime_profile`), ready to paste.
    pub mode: Mode,
    /// The deterministic mode's estimates, every coefficient in scans (the exponents unitless).
    pub deterministic_config: DeterministicConfig,
    /// A job past its estimate (an overshoot: a replay waits for it, a live run lands it late) logs a warning at most
    /// once per this many seconds of data, with each LET kind's overshoots since the last; a warning only, it changes
    /// nothing (a slower machine running a faster one's `deterministic_config` just waits, the output the same).
    pub overshoot_warning_interval_s: f64,
    pub keyframe_min_rotation_degrees: f64,
    pub keyframe_min_distance_meters: f64,
    pub closure_search_radius: f64,
    /// ICP pairs a source point with its nearest target point only within this distance (m); farther points are
    /// treated as not seen by the other scan.
    pub closure_icp_inlier_distance_m: f64,
    /// A closure whose ICP fit has a larger rms inlier distance (m, sqrt of the fitness) is rejected.
    pub closure_icp_max_fitness_m: f64,
    /// Closure ICP steps minimise distances along the target's surface normals (point-to-plane); false is the old
    /// point-to-point.
    pub closure_icp_point_to_plane: bool,
    /// The single-scan closure ICP aligns against its target voxelized at this (m; cheaper search, fewer normals;
    /// fitness and overlap still measured against the full target). 0 = the full target. Default 1.5 x
    /// submap_resolution: on 200 small Mid-360 candidates it ran 18% faster and its 5 changed decisions all rejected
    /// matches GT puts 7-55 m apart (half the inlier distance was faster but accepted one 62 m look-alike).
    pub closure_icp_target_resolution_m: f64,
    /// The single-scan closure ICP gives up (an ICP reject) once its fitness can no longer reach the gate's largest
    /// passing fitness (`closure_icp_max_fitness_m`², `connection_fit::icp_with_early_stop`).
    pub closure_icp_early_stop: bool,
    /// Drop the points a scan re-sends from the scan before (placed by the odometry they land exactly on their copies;
    /// the Go2 L1 republishes its last seconds of points) from every keyframe cloud, so scan context, ICP and the maps
    /// see each surface once. On by default.
    pub drop_reseen_points: bool,
    /// A keyframe's cloud needs at least this many points (voxelized at the submap resolution): scans are gathered,
    /// placed by the odometry, and a due keyframe waits until they reach it, so a sparse lidar gives fewer, fuller
    /// keyframes and everything downstream (scan context, ICP, submaps) sees the gathered cloud (0 = the keyframe's scan).
    /// 2000 is about the 10th percentile of a Mid-360 scan voxelized at 0.1 m, so about 1 in 10 of its keyframes waits.
    pub auto_aggregate_cloud_size: usize,
    /// The closure target submap's half range (keyframes either side of the target) grows with the extension:
    /// ceil(clouds x (1 + closure_target_extra_fraction)) + closure_target_extra_keyframes, clouds being the source clouds
    /// fitted so far (1 for the single scan: 5 at the defaults); see `Config::target_half_range`.
    pub closure_target_extra_fraction: f64,
    pub closure_target_extra_keyframes: usize,
    pub submap_resolution: f64,
    /// Structure-spread gate: minimum occupied Scan-Context cells. 0 disables.
    pub closure_min_occupancy: i32,
    /// Observability gate (Zhang 2016 / X-ICP degeneracy factor). 0 disables.
    pub closure_min_degeneracy: f64,
    /// A closure's match confidence must clear 1 - exp(-yank_stdevs^2 / 2k^2) (see `ClosureCandidateDiag`): a small yank
    /// needs little confidence, a large one a near-perfect match. This is k, the sigmas of yank before that.
    /// Yank is measured per sqrt(keyframe gap), since odometry error estimate is a random walk (on huge_loop this ranks
    /// closures that help above closures that hurt at AUC 0.82 vs 0.70).
    pub closure_yank_gate_width: f64,
    /// How many times the stated odometry error estimate (standard deviation) the true drift may be, unknown within that range
    /// (`proportional_confidence::yank_evidence`): a candidate's yank is priced averaged over it, so a large jump costs
    /// about 3 ln z of evidence instead of z^2 / 2 and GNC judges it. 1 = the stated odometry error estimate is trusted exactly.
    pub closure_yank_drift_range: f64,
    /// Don't extend a candidate whose required evidence a full stack could not reach at its single scan's evidence per
    /// keyframe.
    pub closure_extension_skip_unreachable: bool,
    /// Reject before ICP a candidate whose least reachable yank needs more evidence than any stack of scans could give.
    pub closure_yank_pre_check: bool,
    /// Stop extending once a step's evidence is no higher than the step before.
    pub closure_extension_stop_when_flat: bool,
    /// Stop extending once a step's fit moves a stacked keyframe farther than the search radius from the single fit.
    pub closure_extension_stop_on_swing: bool,
    /// The uncertainty range, as ratios of a candidate's evidence to its bar (the required evidence; both in yank units):
    /// at or above `barely_pass_uncertainty_range` (> 1) the single scan accepts, at or below
    /// `barely_fail_uncertainty_range` (< 1) it rejects, and in between the match is extended until the ratio leaves the
    /// range, which narrows toward 1 each step (`proportional_confidence::UncertaintyRange`).
    pub barely_fail_uncertainty_range: f64,
    pub barely_pass_uncertainty_range: f64,
    /// How much the pressure keeps the uncertainty range wide: its narrowing is stretched by 1 / (1 - pressure)^this.
    pub decisive_pressure_rate: f64,
    /// The odometry distance between an extension's separate source clouds, as a fraction of the mean scan reach: small
    /// enough that each still lands inside the target submap's coverage.
    pub extension_cloud_spacing: f64,
    /// The compounding agreement sensitivity: the yank gate scores an extension by the median cloud ratio times
    /// exp(sensitivity (1 - min(d, 5)^2)) per step, where d is how far that step's alignment is from the one before it at
    /// the source keyframe (shift and mean reach x turn, in inlier distances): clouds that agree to within an inlier
    /// distance raise the score, ones that disagree lower it. It never changes a closure's fit or the confidence the
    /// solve takes. 0 = the plain median.
    #[cfg_attr(feature = "native", serde(alias = "closure_extension_agreement_alpha"))]
    pub compounding_agreement_sensitivity: f64,
    /// Graph change heat: a finished solve is adopted only while the heat is at or under `graph_change_heat_threshold`;
    /// adopting one adds its cost, the curbed yank (yank units) of the largest keyframe move it makes from the accepted
    /// graph, in standard deviations of the stated odometry error accumulated along the path, above
    /// `graph_change_heat_floor`; the heat decays by `graph_change_heat_decay` per keyframe. A solve finishing while the
    /// heat is over waits, a newer one replacing it, and is adopted at the first keyframe the heat is back under (always
    /// at `finish`). The solver itself never sees it.
    pub graph_change_heat_enabled: bool,
    pub graph_change_heat_threshold: f64,
    pub graph_change_heat_floor: f64,
    pub graph_change_heat_decay: f64,
    /// Seeded solve: each solve starts from the accepted graph (the newest adopted solve's poses, the keyframes since
    /// riding its correction) instead of from the odometry alone.
    pub solve_seed_from_accepted: bool,
    /// GNC starts from the newest adopted solve's verdicts: a closure it rejected starts out, and GNC's starting mu comes
    /// from the rest's residuals, so one still beyond twice the worst of them stays out.
    pub solve_seed_gnc_verdicts: bool,
    /// Deterministic solve schedule: a finished solve is held `ceil(estimate)` scans (at least one) after its dispatch,
    /// its estimate an upper bound on its cost in scans known before it starts, so the graph is the same however fast the
    /// machine is; one still running then is an overshoot (logged loudly, counted) and is adopted as soon as it
    /// finishes. The estimate is `solve_scans_scale * keyframes^solve_keyframe_power * (1 + closures)^solve_closure_power *
    /// (1 + rejected)^solve_rejected_power`: the closures in the solve and the ones the newest adopted solve rejected
    /// (GNC takes longer with outliers in). The debounce and merge after it are their own jobs (`adoption_schedule`).
    /// Deterministic closure-search schedule: a search's result (its closure, or its next alternative) takes effect only
    /// once `ceil(estimate)` scans (at least one) have passed since its dispatch, an upper bound on its cost in scans fixed
    /// before each piece of work starts, so which searches run and when their closures land is the same however fast the
    /// machine is; one still running then is an overshoot (logged loudly, counted) and lands as soon as it finishes. The
    /// fast match (everything up to the extension) at dispatch: `fast_match_scans_fixed_overhead` +
    /// `fast_match_scans_per_keyframe` x keyframes (the scan-context lookup, none for a retry) +
    /// `fast_match_scans_per_point` x (source + target points) + `fast_match_scans_per_source_point` x source points (the
    /// voxelized source keyframe's, and the single target submap's keyframes' before merging); the verification (the
    /// extension, in a whole search or its own job) `verification_scans_fixed_overhead` as it starts and
    /// `verification_scans_per_round` before each round.
    /// The closure search's pipeline (`closure_search_schedule`): each keyframe's scan-context candidate and each retry
    /// alternative wait in a priority buffer of `sc_buffer_size`, the most similar first (ties by arrival), the least
    /// similar dropped when it overflows; each of `fast_match_workers` takes the best one as it frees (at its job's
    /// landing) for the fast match (cloud build, single ICP, degeneracy and the gates, the `fast_match_scans_*` terms). A
    /// clear pass or fail lands decided; a match inside the uncertainty range waits in a priority buffer of
    /// `verification_buffer_slots`, the highest single-scan confidence ratio first (ties by dispatch), the lowest dropped
    /// when it overflows, for one of `verification_workers` to run its extension rounds and yank verdict as it frees:
    /// held `verification_scans_fixed_overhead` + each round's own estimate as it starts (one job reporting each round's
    /// tick).
    pub fast_match_workers: usize,
    pub sc_buffer_size: usize,
    pub verification_workers: usize,
    pub verification_buffer_slots: usize,
    /// The body's recent speed (`recent_speed`: the solve estimate's speed, each closure diagnostic's) is the median over
    /// the newest this many steps between distinct odometry poses.
    pub solve_speed_window_poses: usize,
    /// When set, every candidate ICP scores writes `<dir>/<source>_<target>.icp`: the source cloud
    /// and target submap exactly as ICP got them (global frame, current graph), then ICP's
    /// rotation (row-major) and translation, fitness and convergence. Little-endian: u32 source
    /// count, u32 target count, f32 xyz points, 9 + 3 f64, f64 fitness, u8 converged.
    pub closure_icp_dump_dir: String,
    /// A candidate whose single-scan confidence is too low for its yank gets
    /// a second look on more data instead of an outright reject: each round seeded with the previous
    /// round's ICP (the first round with the single-scan ICP), the source grows to the last 2, 4, 8, ... keyframes (up to this many), and the target to the
    /// keyframes near the matched one in time that those land within `closure_search_radius` of.
    /// Inliers are source points within `closure_icp_inlier_distance_m` of the target after ICP.
    /// Each round counts as (extended inliers / single-scan inliers) independent checks, each as
    /// good as the single scan's confidence times how well the overlap held up (extended inlier
    /// fraction / single inlier fraction, at most 1): confidence = 1 - (1 - c_round)^checks.
    /// (Mean squared fitness is not used: the stacked clouds see places the other side never did.)
    /// The first round that clears the gate is kept, and its transform becomes the closure. 0 = off.
    pub closure_extended_icp_max_keyframes: usize,
    /// Huber threshold on a kept closure's whitened residual in the final solve; 0 = plain least squares.
    pub closure_robust_huber_k: f64,
    /// Closure-verification ICP: iteration cap and correspondence gate (m).
    pub closure_icp_max_iterations: u64,
    pub closure_icp_max_correspondence_m: f64,
    /// GNC closure-factor variance is `score * closure_gnc_var_scale`. The raw ICP
    /// score gives a variance (~1e-3) tight enough that GNC over-rejects; scaling
    /// it up loosens each closure so GNC keeps the true consensus set.
    pub closure_gnc_var_scale: f64,
    /// Probability that a true inlier's residual falls under GNC's chi-squared
    /// cost threshold, in (0, 1). Lowering it tightens the test so GNC rejects more closures.
    pub closure_gnc_inlier_probability: f64,
    /// When true, a closure is uninformative only where the graph's own covariance of its two ends (heading swing and
    /// solved closures included) is surer than ICP in every direction, instead of the per-keyframe xy step times the
    /// keyframe gap.
    pub closure_informative_by_graph_covariance: bool,
    /// When false, no closure is rejected as uninformative (the graph surer of its two ends than ICP): every one that
    /// passes the other gates is committed, and repeats of a kept closure count again.
    pub closure_uninformative_gate: bool,
    /// The odometry's drift as a random walk in the motion itself, so it does not depend on how far apart keyframes
    /// are: per noise source (xy translation, z translation, yaw, roll/pitch tilt), the standard deviation it adds per
    /// square root of metres travelled (m/√m for translation, rad/√m for rotation) and per square root of radians
    /// turned (m/√rad, rad/√rad); a step's variance is the first squared times its distance plus the second squared
    /// times its turn. Defaults: a Mid-360 with FAST-LIO / Point-LIO, fitted to ground truth over every span of five
    /// recordings (small_mid, gir1, gir2, athens, huge; report_build/odometry_rate_pooled.py); the drift is almost all
    /// from turning. After each solve that keeps closures, the module logs the values these measure (debug_log kind
    /// "odometry_drift"), to start the next run from.
    #[cfg_attr(feature = "native", serde(alias = "odom_xy_drift_per_sqrt_m"))]
    pub odometry_error_estimate_xy_per_sqrt_m: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_xy_drift_per_sqrt_rad"))]
    pub odometry_error_estimate_xy_per_sqrt_rad: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_z_drift_per_sqrt_m"))]
    pub odometry_error_estimate_z_per_sqrt_m: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_z_drift_per_sqrt_rad"))]
    pub odometry_error_estimate_z_per_sqrt_rad: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_yaw_drift_per_sqrt_m"))]
    pub odometry_error_estimate_yaw_per_sqrt_m: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_yaw_drift_per_sqrt_rad"))]
    pub odometry_error_estimate_yaw_per_sqrt_rad: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_tilt_drift_per_sqrt_m"))]
    pub odometry_error_estimate_tilt_per_sqrt_m: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_tilt_drift_per_sqrt_rad"))]
    pub odometry_error_estimate_tilt_per_sqrt_rad: f64,
    /// How far (standard deviation of ln) each noise source's true size spreads around its rates from one recording to
    /// the next (xy, z, yaw, roll/pitch): the prior that `learned_odometry_error_estimate_from_closures` holds its estimate to. 0 = one
    /// closure's worth of information (a unit-information prior). Defaults: the spread of those five recordings around
    /// the fit (their held-out error at 32-keyframe spans).
    #[cfg_attr(feature = "native", serde(alias = "odom_noise_prior_ln_sd_xy"))]
    pub learned_odometry_error_estimate_prior_ln_sd_xy: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_noise_prior_ln_sd_z"))]
    pub learned_odometry_error_estimate_prior_ln_sd_z: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_noise_prior_ln_sd_yaw"))]
    pub learned_odometry_error_estimate_prior_ln_sd_yaw: f64,
    #[cfg_attr(feature = "native", serde(alias = "odom_noise_prior_ln_sd_tilt"))]
    pub learned_odometry_error_estimate_prior_ln_sd_tilt: f64,
    /// How many closed loops' worth of evidence the stated odometry error estimate counts as for the learner (its plasticity): each
    /// source's estimate is the stated odometry error estimate held with that many residual checks against the closures' (a closed loop
    /// gives a source about one), so the first loop nudges it and many agreeing loops decide it. 0 = the closures alone;
    /// infinite = the stated odometry error estimate is never moved (no learning: the gates and solves use the config's drift). Default 30
    /// (huge_l1: yaw x4.7 against x40000 at 1, where one loop's misfit moved it x30; small_l1 unchanged).
    #[cfg_attr(feature = "native", serde(alias = "odom_error_stiffness"))]
    pub learned_odometry_error_estimate_stiffness: f64,
    pub scan_context_num_rings: i32,
    pub scan_context_num_sectors: i32,
    pub scan_context_max_range_m: f64,
    pub scan_context_top_k: i32,
    pub scan_context_match_threshold: f64,
    pub scan_context_lidar_height_m: f64,
    /// When above 0, a closure's ICP factor is its sharpness s (`connection_fit::sharpness`) as s / (s + this), and an
    /// extended stack is scored the same way from its own sharpness, each round started from the single-scan alignment;
    /// 0 keeps the fitness factor and the inlier-count evidence of extended ICP.
    pub closure_sharpness_half: f64,
    /// The nudge `sharpness` measures against: this far (m) along x and y, and this many degrees about z.
    pub closure_sharpness_nudge_m: f64,
    pub closure_sharpness_nudge_deg: f64,
    /// When true, a closure's evidence (yank units) is its likelihood ratio (`connection_fit::likelihood_ratio`, scales measured
    /// from the clouds, misfit reach = `closure_icp_max_correspondence_m`), as confidence 1 - e^-ratio, in place of scan
    /// context x ICP x degeneracy, and an extended stack's is the stack's own ratio. It also sets the extension budget: the keyframes the still-needed evidence takes at the
    /// single scan's evidence per keyframe (at least one bit), up to the next doubling, at most
    /// `closure_extended_icp_max_keyframes`; a stack's evidence is its ratio per point x its size over the single scan's
    /// (the new surface it covers).
    pub closure_likelihood_confidence: bool,
    /// When true, a keyframe whose search is rejected tries its next scan-context match (other places under the match
    /// threshold, closest first), up to 1 + floor(sqrt(min(ceil(1 / (1 - pressure)), scan_context_top_k - 1))) tries per
    /// keyframe (3 at pressure 0.8, 4 at 0.9 and above with top_k 10); a newer keyframe's better match still outranks a
    /// waiting retry.
    pub closure_alternative_places: bool,
    /// Alternative places are tried only at or above this pressure; below it a keyframe tries only its best match.
    /// Retries were two-thirds of all closure ICPs (small Mid-360: 2,012 of 3,256 candidates for 48 of 173 right kept
    /// loops; huge: 12,012 of 16,867 for 69 right and 150 wrong).
    pub closure_alternative_places_min_pressure: f64,
    /// With `closure_likelihood_confidence`: score only the source points the target can speak to, those within ICP's
    /// inlier distance of it or in space a target scan looked through (`connection_fit::likelihood_ratio_visible`);
    /// a point no target scan saw counts for nothing instead of as a misfit.
    pub closure_likelihood_visibility: bool,
    /// When true, the odometry's four noise variances (xy, z, yaw, roll/pitch per keyframe) are estimated from how far the
    /// kept closures disagreed with the raw odometry (`learned_odometry_error_estimate`), the reported ones as priors, and logged
    /// (each search's diagnostics, `GSC_LEARN_LOG`); nothing uses them: the solve, pressure and gates take the stated odometry error estimate.
    #[cfg_attr(feature = "native", serde(alias = "odometry_noise_from_closures"))]
    pub learned_odometry_error_estimate_from_closures: bool,
    /// When true, the noise learner reads only the kept closures that agree pairwise around their raw-odometry cycles
    /// (the PCM core) on every solve: a bend two independent loops agree on is drift, a lone closure's may be a wrong
    /// one (huge: kept wrong closures read as yaw x41000).
    pub odometry_error_estimate_evidence_core: bool,
    /// Recompute the loop-overlap pressure (`loop_overlap`) on every this-many-th scan, reusing it in between; 0 = never.
    pub pressure_check_frequency: usize,
    /// The pressure off the scan thread (`pressure_schedule`): a job on one worker, started with a snapshot of its inputs
    /// when the worker is free (the newest request pending meanwhile), landing at its dispatch + `pressure_scans_fixed_overhead` +
    /// `pressure_scans_per_keyframe` x keyframes + `pressure_scans_per_closure` x committed closures, rounded up (scans). Off: computed inline on the scan thread. The coefficients
    /// are the cheapest nonnegative ones bounding every Mac pressure computation of small L1, gir1 and huge L1 but each
    /// recording's hardest 1% (re-chosen over 3 refits), 99% of each one's under it.
    /// `soft_pressure` (display only, the same probability without the cliff): 50% where log10 of the loop hazard
    /// (unresolved_loop_probability = 1 - e^-hazard) is this, and 0% this many decades below it (log-compressed).
    pub soft_pressure_half_log10_hazard: f64,
    pub soft_pressure_decades: f64,
}

impl Config {
    /// The closure target submap's half range with `clouds` source clouds in the extension (1: the single scan).
    pub fn target_half_range(&self, clouds: usize) -> usize {
        (clouds.max(1) as f64 * (1.0 + self.closure_target_extra_fraction.max(0.0))).ceil() as usize
            + self.closure_target_extra_keyframes
    }
    /// An error naming the first setting out of its allowed range, if any.
    // the negated comparisons reject NaN too
    #[allow(clippy::neg_cmp_op_on_partial_ord)]
    pub fn validate(&self) -> Result<(), String> {
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
            ("fast_match_scans_fixed_overhead", self.deterministic_config.fast_match_scans_fixed_overhead),
            ("fast_match_scans_per_keyframe", self.deterministic_config.fast_match_scans_per_keyframe),
            ("fast_match_scans_per_point", self.deterministic_config.fast_match_scans_per_point),
            ("fast_match_scans_per_source_point", self.deterministic_config.fast_match_scans_per_source_point),
            ("verification_scans_fixed_overhead", self.deterministic_config.verification_scans_fixed_overhead),
            ("verification_scans_per_round", self.deterministic_config.verification_scans_per_round),
            ("verification_scans_per_round_point", self.deterministic_config.verification_scans_per_round_point),
            ("pressure_scans_fixed_overhead", self.deterministic_config.pressure_scans_fixed_overhead),
            ("pressure_scans_per_keyframe", self.deterministic_config.pressure_scans_per_keyframe),
            ("pressure_scans_per_closure", self.deterministic_config.pressure_scans_per_closure),
            ("solve_scans_scale", self.deterministic_config.solve_scans_scale),
            ("debounce_scans_per_keyframe", self.deterministic_config.debounce_scans_per_keyframe),
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
        if !(self.overshoot_warning_interval_s >= 0.0) {
            return Err("overshoot_warning_interval_s must be >= 0".into());
        }
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
    fn default() -> Config {
        Config {
            mode: Mode::Deterministic,
            deterministic_config: DeterministicConfig::default(),
            overshoot_warning_interval_s: 10.0,
            keyframe_min_rotation_degrees: 10.0,
            keyframe_min_distance_meters: 0.5,
            closure_search_radius: 3.0,
            closure_icp_inlier_distance_m: 0.3872983346207417,
            closure_icp_max_fitness_m: 0.3872983346207417,
            closure_icp_point_to_plane: true,
            closure_icp_target_resolution_m: 0.15,
            closure_icp_early_stop: true,
            drop_reseen_points: true,
            auto_aggregate_cloud_size: 2000,
            closure_target_extra_fraction: 0.3,
            closure_target_extra_keyframes: 3,
            submap_resolution: 0.1,
            closure_min_occupancy: 80,
            closure_min_degeneracy: 0.05,
            closure_yank_gate_width: 1.0,
            closure_yank_drift_range: 100.0,
            closure_extension_skip_unreachable: true,
            closure_yank_pre_check: false,
            closure_extension_stop_when_flat: true,
            closure_extension_stop_on_swing: true,
            barely_fail_uncertainty_range: 0.05,
            barely_pass_uncertainty_range: 3.0,
            decisive_pressure_rate: 1.0,
            extension_cloud_spacing: 0.25,
            compounding_agreement_sensitivity: 1.5,
            graph_change_heat_enabled: true,
            graph_change_heat_threshold: 30.0,
            graph_change_heat_floor: 20.0,
            graph_change_heat_decay: 0.995,
            solve_seed_from_accepted: false,
            solve_seed_gnc_verdicts: false,
            fast_match_workers: 2,
            sc_buffer_size: 4,
            verification_workers: 2,
            verification_buffer_slots: 2,
            solve_speed_window_poses: 500,
            closure_icp_dump_dir: String::new(),
            closure_extended_icp_max_keyframes: 16,
            closure_robust_huber_k: 1.345,
            closure_icp_max_iterations: 50,
            closure_icp_max_correspondence_m: 10.0,
            closure_gnc_var_scale: 10.0,
            closure_gnc_inlier_probability: 0.95,
            closure_informative_by_graph_covariance: true,
            closure_uninformative_gate: true,
            odometry_error_estimate_xy_per_sqrt_m: 3.91e-4,
            odometry_error_estimate_xy_per_sqrt_rad: 6.35e-3,
            odometry_error_estimate_z_per_sqrt_m: 6.42e-4,
            odometry_error_estimate_z_per_sqrt_rad: 3.86e-3,
            odometry_error_estimate_yaw_per_sqrt_m: 1e-6,
            odometry_error_estimate_yaw_per_sqrt_rad: 8.66e-3,
            odometry_error_estimate_tilt_per_sqrt_m: 6.91e-4,
            odometry_error_estimate_tilt_per_sqrt_rad: 3.3e-3,
            learned_odometry_error_estimate_prior_ln_sd_xy: 1.72,
            learned_odometry_error_estimate_prior_ln_sd_z: 2.05,
            learned_odometry_error_estimate_prior_ln_sd_yaw: 1.15,
            learned_odometry_error_estimate_prior_ln_sd_tilt: 0.96,
            learned_odometry_error_estimate_stiffness: 10.0,
            scan_context_num_rings: 20,
            scan_context_num_sectors: 60,
            scan_context_max_range_m: 0.0,
            scan_context_top_k: 10,
            scan_context_match_threshold: 0.4,
            scan_context_lidar_height_m: 2.0,
            closure_sharpness_half: 0.0,
            closure_sharpness_nudge_m: 1.0,
            closure_sharpness_nudge_deg: 5.0,
            closure_likelihood_confidence: true,
            closure_alternative_places: true,
            closure_alternative_places_min_pressure: 0.8,
            closure_likelihood_visibility: true,
            learned_odometry_error_estimate_from_closures: true,
            odometry_error_estimate_evidence_core: true,
            pressure_check_frequency: 5,
            soft_pressure_half_log10_hazard: -0.5,
            soft_pressure_decades: 100.0,
        }
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
    /// ln of each odometry variance's ratio to the reported one (0: the reported noise), set from the kept closures
    /// after each solve.
    pub(crate) learned_odometry_error_estimate_ln: [f64; 4],
    /// Each `learned_odometry_error_estimate_ln`'s posterior standard deviation once the residuals have measured it (the prior's before).
    pub(crate) learned_odometry_error_estimate_ln_sd: [Option<f64>; 4],
    /// The ln scale the running (or newest) solve was started with, and whether a re-solve at a moved scale is due.
    pub(crate) learned_odometry_error_estimate_ln_dispatched: [f64; 4],
    pub(crate) odometry_error_estimate_rescale: bool,
    /// The next solve also measures the learned odometry error estimate's evidence: requested after a solve keeps a loop
    /// longer (in keyframes between its ends) than any kept before, `learned_evidence_gap`, and by `finish`.
    pub(crate) learned_evidence_requested: bool,
    pub(crate) learned_evidence_gap: usize,
    pub(crate) learned_evidence_measured: bool,
    /// Whether the newest solve was itself a re-solve at a moved noise (no new closure or fix): one per new piece of
    /// evidence, since the estimate need not settle (GNC's verdicts can change with the noise) and would re-solve forever.
    pub(crate) odometry_error_estimate_rescaled: bool,
    /// A closure was committed since the last `update`.
    pub(crate) new_closure: bool,
    /// Committed closures a solve has been started for.
    pub(crate) closures_dispatched: usize,
    /// World-frame position fixes (`add_coordinate_factor`): time, frame id, sensor offset in the body frame, position,
    /// covariance; how many a solve has been started for; and the newest solve's map -> frame alignments.
    pub(crate) world_positions: Vec<(f64, u32, Vec3, Vec3, Mat3)>,
    pub(crate) world_positions_dispatched: usize,
    pub(crate) world_alignment: Vec<(u32, f64, Vec3)>,
    /// Landmark sightings (`add_landmark_factor`): time, landmark id, rotation, position and 6x6 information in the body
    /// frame; how many a solve has been started for; and the newest solve's landmarks (id, map rotation and position).
    pub(crate) landmark_sightings: Vec<LandmarkSightingRecord>,
    pub(crate) landmark_sightings_dispatched: usize,
    pub(crate) landmarks: Vec<(u64, Mat3, Vec3)>,
    pub(crate) solving: Option<Solving>,
    pub(crate) graph_error: f64,
    /// Per finished solve (adopted or not): the keyframes it was dispatched over, its newest keyframe's time, and the
    /// seconds it took on its thread.
    pub(crate) solve_records: Vec<(usize, f64, f64)>,
    /// The graph change heat (`graph_change_heat_enabled`), the keyframe count it was last raised at, and the finished
    /// solve waiting for it to cool.
    pub(crate) heat: f64,
    pub(crate) heat_keyframes: usize,
    pub(crate) pending_solve: Option<(crate::pose_graph_solving::FinishedSolve, f64)>,
    /// The debounce and the merge (`adoption_schedule`): each job out, the newest solve waiting for each, their workers,
    /// and whether they run inline (the end of the data).
    pub(crate) debounce_job: Option<crate::adoption_schedule::AdoptionJob<crate::adoption_schedule::Debounced>>,
    pub(crate) debounce_waiting: Option<crate::pose_graph_solving::FinishedSolve>,
    pub(crate) merge_job: Option<crate::adoption_schedule::AdoptionJob<crate::adoption_schedule::Merged>>,
    pub(crate) merge_waiting: Option<MergeWaiting>,
    pub(crate) adoption_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) merge_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) adoption_inline: bool,
    /// A merge landed (keyframes moved) since the last `update`.
    pub(crate) adopted_since_update: bool,
    /// The newest `solve_speed_window_poses` + 1 distinct odometry poses' times and positions (odometry frame), for the
    /// body's recent speed.
    pub(crate) recent_motion: std::collections::VecDeque<(f64, Vec3)>,
    /// Solves slower than the data their estimate spanned (`solve_hold_enabled`), and the newest scan's recorded time.
    pub(crate) solve_overshoots: usize,
    pub(crate) scan_time: f64,
    /// The newest advancing scan's recorded time (the data time holds are judged by), and the scans whose recorded time
    /// advanced past every earlier one's, the count logical-execution-time holds run on (a scan repeating an earlier stamp
    /// spans no data time).
    pub(crate) newest_advancing_time: Option<f64>,
    pub(crate) advancing_scans: usize,
    pub(crate) first_advancing_time: Option<f64>,
    /// Each landed job's kind, its measured scans (seconds x the scan rate then) over its estimate in scans: what a run
    /// reports as its measured coefficients (`runtime_profile`); never read by a decision.
    pub(crate) job_ratios: Vec<(&'static str, f64)>,
    /// Per LET kind, its jobs and overshoots (over the run and since the last warning), and the data time of that warning.
    pub(crate) overshoot_tally: std::collections::BTreeMap<&'static str, OvershootTally>,
    pub(crate) last_overshoot_warning: Option<f64>,
    /// The closure searches dispatched and not landed yet, in dispatch order (`closure_search_hold_enabled`), the number
    /// the next one gets, the recorded times of the advancing scans since the oldest one's dispatch (the first is advancing
    /// scan `search_scan_base`), the worker pool they run on, the searches that overshot
    /// their estimate and the seconds a lockstep replay waited on them.
    pub(crate) closure_searches: std::collections::VecDeque<crate::closure_search_schedule::SearchJob>,
    pub(crate) search_order: u64,
    pub(crate) search_scan_times: std::collections::VecDeque<f64>,
    pub(crate) search_scan_base: usize,
    pub(crate) search_pool: Option<crate::closure_search_schedule::SearchPool>,
    /// The two-stage search (`closure_search_schedule`): the candidates waiting for a fast-match worker (and their
    /// arrival numbers), the matches waiting for verification (and their fast match's dispatch numbers), the
    /// verification pool, and the counts its buffers dropped and its verification jobs overshot.
    pub(crate) sc_buffer: Vec<(ClosureProposal, u64)>,
    pub(crate) sc_arrivals: u64,
    pub(crate) verification_buffer: Vec<(Box<PendingExtension>, u64)>,
    pub(crate) verification_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) sc_dropped: usize,
    pub(crate) verification_dropped: usize,
    pub(crate) verification_overshoots: usize,
    pub(crate) closure_search_overshoots: usize,
    pub(crate) closure_search_wait_s: f64,
    /// Scans offered to `add_key_pose`, and the newest loop-overlap pressure with the scan count it was computed at.
    pub(crate) scans_seen: usize,
    pub(crate) pressure: Option<(usize, crate::loop_overlap::LoopOverlap)>,
    /// Bumped whenever a keyframe is added or a solve is adopted: what loop_overlap reads changes only then, so pressure
    /// is recomputed only when it differs from `pressure_version`, the version it was last computed at.
    pub(crate) graph_version: u64,
    pub(crate) pressure_version: Option<u64>,
    /// The pressure job running or held (`pressure_let_enabled`), whether a newer one is pending, its worker, each
    /// pressure computation's keyframes, closures and seconds, the jobs that overshot, whether the running one is late,
    /// and whether a lockstep replay waits for due jobs (`set_wait_for_due_jobs`).
    pub(crate) pressure_job: Option<crate::pressure_schedule::PressureJob>,
    /// The graph change heat's cost per adopted solve measured on its solve's thread (keyframes, seconds), and the solves
    /// whose cost was recomputed here because a solve was adopted while they ran.
    pub(crate) change_cost_calls: Vec<(usize, f64)>,
    /// Seconds the scan thread blocked joining finished-or-waited-for solves (a lockstep replay's wait, not work).
    pub(crate) solve_join_wait_s: f64,
    pub(crate) graph_change_cost_recomputed: usize,
    /// Solves adopted so far: the accepted graph's version (keyframes added since only ride it).
    pub(crate) adopted_solves: u64,
    pub(crate) pressure_requested: bool,
    pub(crate) pressure_pool: Option<crate::closure_search_schedule::SearchPool>,
    pub(crate) pressure_calls: Vec<(usize, usize, f64)>,
    pub(crate) pressure_overshoots: usize,
    pub(crate) pressure_late: bool,
    pub(crate) wait_for_due_jobs: bool,
    /// Sum and count of the keyframes' scan reaches (95th-percentile planar range), the sensor range loop_overlap uses.
    pub(crate) keyframe_reach: (f64, usize),
    /// `drop_reseen_points`: the previous scan's points (odometry frame, 0.1 mm grid).
    pub(crate) reseen_previous: std::collections::HashSet<[i64; 3]>,
    /// `auto_aggregate_cloud_size`: the scans since the last keyframe, in the odometry frame, voxelized.
    pub(crate) gathered: PointCloud,
    /// The newest applied solve's tip keyframe and every solved keyframe's position covariance relative to it.
    pub(crate) solved_relative_covariance: Option<(usize, Arc<Vec<Mat3>>)>,
    /// Shared, so a closure search can count look-alike places without copying them.
    pub(crate) scan_context_descriptors: crate::store::Appended<Arc<similarity_discovery::Descriptor>>,
    pub(crate) scan_context_ring_keys: Vec<similarity_discovery::RingKey>,
    pub(crate) rotation_offset: Mat3,
    pub(crate) translation_offset: Vec3,
    pub(crate) timing: TimingStats,
    pub(crate) closure_diagnostics: Vec<ClosureCandidateDiag>,
    pub(crate) closure_edge_events: Vec<ClosureEdgeEvent>,
    /// Rotation (deg) and translation (m) yank of every candidate ICP has scored (per sqrt keyframe
    /// gap), the scale new yanks are measured against. Every scored
    /// candidate, not just committed ones: a scale taken only from closures the gate let through would
    /// shrink with each rejection and reject the rest.
    pub(crate) waiting_proposal: Option<ClosureProposal>,
    /// The ICP cache every closure search's cloud computations go through (`set_icp_cache`), if any.
    pub(crate) icp_cache: Option<Arc<crate::icp_cache::IcpCache>>,
    /// Each keyframe's voxelized scan and range image, built once and shared by every closure search.
    pub(crate) keyframe_clouds: Arc<crate::proportional_confidence::KeyframeClouds>,
    /// Called when a background solve finishes (`notify_when_solved`).
    pub(crate) solved: Option<Arc<dyn Fn() + Send + Sync>>,
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
    /// `add_key_pose` without the pressure check: drop-reseen, aggregation, voxelizing, the keyframe and its scan-context
    /// descriptor.
    pub(crate) scan_s: f64,
    /// The loop-overlap pressure checks (`pressure_check_frequency`).
    pub(crate) pressure_s: f64,
    /// `propose_closure` and `begin_closure_search`, the scan-context lookup (`scan_context_s`) included.
    pub(crate) propose_s: f64,
    /// `ClosureSearch::run` (its ICP, gates and extension), debug dumps left out.
    pub(crate) search_s: f64,
}

/// A keyframe through stages 1 and 2, waiting for the search thread: its candidate target, the
/// scan-context column shift between them, and the match's cosine distance (lower is better).
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
}

/// The ICP half of one closure search, on its own copy of the keyframes so it can run on another thread
/// while scans keep arriving (`GscPgo::begin_closure_search` makes it, `finish_closure_search` applies it).
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
    pub(crate) scan_context_descriptors: crate::store::Appended<Arc<similarity_discovery::Descriptor>>,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) ranked: Vec<(f32, usize)>,
    /// Every closure the solve has judged: its yank_stdevs when committed, and whether the solve kept it.
    pub(crate) judged_yanks: Vec<(f64, bool)>,
    /// The snapshots the search's setup reads where it starts (`prepare`): the committed closures and their weights,
    /// the keyframes' reach, the odometry error estimate and the newest solve's relative covariances.
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
    /// Seconds spent writing debug dumps (`closure_icp_dump_dir`), taken out of the stage timings.
    pub(crate) debug_io_s: std::cell::Cell<f64>,
    /// Every debug dump's seconds in this search (`closure_icp_dump_dir`), left out of `TimingStats::search_s`.
    pub(crate) dump_s: Arc<std::sync::Mutex<f64>>,
    /// The keyframes' mean scan reach (m), which spaces an extension's separate source clouds.
    pub(crate) mean_reach: f64,
    /// Where the search reports each extension round before it starts and its end (`closure_search_hold_enabled`).
    pub(crate) progress: Option<Arc<crate::closure_search_schedule::SearchProgress>>,
}

/// What a `ClosureSearch` found: its diagnostics, the closure it accepts (relative rotation, translation,
/// fitness), and the time each stage took.
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
/// A finished solve waiting for the merge, with what its debounce measured.
pub(crate) type MergeWaiting = (crate::pose_graph_solving::FinishedSolve, Option<(f64, f64, f64)>);
/// A search's place-recognition inputs for a retry: ring-key distances, the top-k matches under the threshold, and the next alternative.
type RetryInputs = (Vec<(f32, usize)>, Vec<(f32, usize, i32)>, usize);

/// Where a search stands after its fast match: decided, or inside the uncertainty range and handed to the extension.
#[allow(clippy::large_enum_variant)]
pub enum SearchStage {
    Done(ClosureSearchResult),
    Extend(Box<PendingExtension>),
}

/// A match inside the uncertainty range, with what its extension needs: the search (its snapshot), the single fit and
/// its confidence ratio (the verification buffer's promise), the source points, the look-alike count so far, the bar's
/// base, the retry inputs and the fast match's seconds.
pub struct PendingExtension {
    pub(crate) search: ClosureSearch,
    pub(crate) diag: ClosureCandidateDiag,
    timing: TimingStats,
    icp: crate::connection_fit::IcpResult,
    pub(crate) ratio: f64,
    source_points: usize,
    look_alikes: similarity_discovery::LookAlikes,
    error_estimate_sums: Option<Vec<crate::loop_overlap::Sums>>,
    base: f64,
    retry: Option<RetryInputs>,
    pub(crate) fast_s: f64,
}

impl PendingExtension {
    /// The verification stage: the extension rounds and the yank verdict on them, the fast match's seconds included in
    /// the result's.
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
            source_points,
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
            search.bar_with_places(evidence, diag, &mut look_alikes, &mut error_estimate_sums, base)
        };
        let t0 = std::time::Instant::now();
        let extended = search.extended_icp(
            cur_idx,
            target_idx,
            source_points,
            &icp,
            ratio,
            &mut diag,
            &mut required,
            &memo_alignment,
        );
        timing.extended_icp_s += t0.elapsed().as_secs_f64() - search.debug_io_s.take();
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
    /// The ICP half of a closure search: submaps, ICP, its gates, the yank / confidence gate and extended ICP.
    pub fn run(self) -> ClosureSearchResult {
        match self.run_stage() {
            SearchStage::Done(result) => result,
            SearchStage::Extend(pending) => pending.run(),
        }
    }
    /// The fast match (`closure_search_schedule`): everything up to the extension, which a match inside the uncertainty
    /// range is handed on to (`SearchStage::Extend`); a clear pass or fail is decided here.
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
    /// The search's setup from its snapshots, on its own thread (it was the scan thread's): the closures the newest solve
    /// kept (or has not judged), every judged closure's yank and verdict, the odometry's mean step variance, and the
    /// overlap model's chance this scan overlaps the target and the graph's covariance of their offset, which set the
    /// extension's budget.
    fn prepare(&mut self) {
        self.mean_step_xy_variance =
            crate::big_picture_fit::mean_step_variances(&self.key_poses, &self.config)[0];
        // kept, or not judged yet
        self.kept_closures = closure_edges(
            self.closures
                .iter()
                .filter(|l| l.gnc_weight.is_nan() || l.gnc_weight > 0.5),
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
            committed_closures: &self.closures,
        };
        // extending is worth its cost as far as a real loop is likely here; extension grows by doubling, so the budget is
        // the doubling step that covers max keyframes x that chance
        let model = inputs.overlap_model(source_id);
        let diag = &mut self.diag;
        diag.target_overlap = model
            .as_ref()
            .map_or(0.0, |model| model.overlap(target_idx).0);
        // the graph's covariance of the source's offset from this target, at the odometry error estimate in use
        if let Some(model) = &model {
            diag.innovation_covariance = model.relative(target_idx);
        }
        // either reason makes extending worthwhile: a loop is likely somewhere unresolved (pressure), or this scan likely
        // overlaps this target
        let worth = 1.0 - (1.0 - diag.pressure) * (1.0 - diag.target_overlap);
        diag.extension_budget = ((self.config.closure_extended_icp_max_keyframes as f64 * worth)
            .ceil() as usize)
            .next_power_of_two()
            .min(self.config.closure_extended_icp_max_keyframes);
    }
    /// A cloud alignment through the ICP cache (`icp_cache`): stored as the source's pose relative to the target
    /// (`relative_icp`), the computed value itself returned when it was computed here.
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
        (
            mat3::mat_mul(
                &closure_rotation_transpose,
                &mat3::mat_mul(rotation, &source.rotation_global),
            ),
            mat3::mat_vec(
                &closure_rotation_transpose,
                &mat3::sub(&moved, &target.translation_global),
            ),
        )
    }
    /// The evidence a match needs (yank units): `base` plus ln of the look-alike places it fits as well as this one,
    /// counted only as far as `evidence` makes the count matter (stage 4b).
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
                &self.scan_context_descriptors[cur_idx],
                &self.scan_context_descriptors,
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
        )
    }

    fn search_stage(self) -> SearchStage {
        let mut diag = self.diag.clone();
        let (cur_idx, target_idx) = (diag.source_id, self.target_idx);
        let (init_rotation, init_translation) = (self.init_rotation, self.init_translation);
        let mut timing = TimingStats::default();
        let half_range = self.config.target_half_range(1);
        // the source scan, the target submap and its tree, built only when a computation needs them (never when every
        // result comes from the ICP cache)
        // the source scan alone (all the degeneracy gate reads), built apart from the target and its tree
        // seconds spent building them, kept out of the steps that first needed them
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
        // a cloud computation, through the ICP cache when there is one; a point-to-plane alignment is a different
        // result from a point-to-point one, so it is part of the key
        let cache = self.icp_cache.clone();
        // the thinned target and the early stop change the single ICP's result too, so they are part of the key
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
        let memo_alignment = |tag: &str, at: usize, compute: &mut dyn FnMut() -> Vec<f64>| -> Vec<f64> {
            self.memo_alignment(&key_suffix, tag, at, compute)
        };

        // Observability gate: a planar/degenerate source scan leaves the alignment unconstrained in-plane, so fitness
        // lies. It reads only the source scan, so it runs before ICP (computed for every candidate, gate on or off)
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

        // the pose graph takes ICP's fitness as the closure's variance: when the graph is already surer of the two
        // ends' relative position than that (along the odometry, or across one kept closure; with per-motion noise,
        // by the graph's own covariance of it), the closure adds only ICP's noise, and repeats of a closure already
        // kept would count the same evidence twice
        let gate_error_estimate = diag.odometry_error_estimate_gate;
        let odometry_step = self.mean_step_xy_variance * gate_error_estimate[0];
        let step = |a: usize, b: usize| a.abs_diff(b) as f64 * odometry_step;
        let by_graph = diag
            .innovation_covariance
            .iter()
            .flatten()
            .all(|v| v.is_finite());
        let known = if by_graph {
            // the graph's own covariance of the two ends' relative position (its heading swing and the solved
            // closures included): the closure adds something only where it is surer than the graph in some direction.
            // Per-keyframe xy steps say nothing of the heading's swing, which per-motion noise puts most of the drift in
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
        // a candidate whose least yank ICP could leave still needs more evidence than any stack of scans could give is
        // rejected unaligned: ICP pairs points only within its match distance, so it moves the source from the seed (the
        // target's position) by about that much at most; the closure's own variance is at most the fitness limit squared;
        // and a point's likelihood ratio is at most ln(2 reach / (width sqrt(2 pi))) with width no less than a quarter
        // voxel, times the largest stack
        if self.config.closure_likelihood_confidence && self.config.closure_yank_pre_check {
            let reach = self.config.closure_icp_max_correspondence_m;
            let source_position = self.key_poses.position(cur_idx);
            let seeded = self.key_poses.position(target_idx);
            let jump = mat3::norm(&mat3::sub(&seeded, &source_position));
            let spread = mat3::jacobi_eigen(diag.innovation_covariance)
                .0
                .into_iter()
                .fold(0.0, f64::max)
                + self.config.closure_icp_max_fitness_m.powi(2);
            if spread.is_finite() && spread > 0.0 && jump > reach {
                let least_yank = (jump - reach) / spread.sqrt();
                let width = self.config.submap_resolution / 4.0;
                let most_evidence = (2.0 * reach / (width * (2.0 * std::f64::consts::PI).sqrt()))
                    .ln()
                    * self.config.closure_extended_icp_max_keyframes.max(1) as f64;
                let least_required = crate::proportional_confidence::yank_evidence(
                    least_yank / self.config.closure_yank_gate_width.max(f64::EPSILON),
                    self.config.closure_yank_drift_range,
                );
                if least_required > most_evidence {
                    return SearchStage::Done(rejected(diag, "yank_confidence_reject", timing));
                }
            }
        }
        let (t0, built_before) = (std::time::Instant::now(), built_s.get());
        let icp = crate::icp_cache::decode_icp(&memo_alignment("icpr", 0, &mut || {
            let (source_cloud, target_cloud, target_tree) = clouds();
            crate::icp_cache::encode_icp_logged(&crate::connection_fit::icp_thinned(
                source_cloud,
                target_cloud,
                target_tree,
                &init_rotation,
                &init_translation,
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
        // the closure's translation variance as the graph will take it: its inliers' mean squared distance
        // (`IcpResult::inlier_mean_square`)
        let closure_variance = icp.inlier_mean_square();
        if self.config.closure_uninformative_gate && closure_variance >= known {
            return SearchStage::Done(rejected(diag, "uninformative", timing));
        }
        if self.config.closure_sharpness_half > 0.0 {
            diag.sharpness = memo("sharp", &mut || {
                let (source_cloud, _, target_tree) = clouds();
                vec![self.sharpness(source_cloud, target_tree, &icp)]
            })[0];
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
        let (rotation_offset, translation_offset) =
            measured(&icp.rotation, &icp.translation);

        // Graph yank: how far this constraint pulls the source keyframe from the
        // current odom-chain estimate (relative pose closure->cur, in the closure's
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
        diag.innovation = mat3::mat_vec(
            &self.key_poses.rotation(target_idx),
            &mat3::sub(&translation_offset, &current_relative_translation),
        );

        self.score_candidate(&mut diag);
        // the solve's record on closures that pulled about as hard as this one (within about one yank unit): rejected
        // closures are the hard pullers, so a gentle closure is not charged for them
        let (kept, judged) =
            self.judged_yanks
                .iter()
                .fold((0.0, 0.0), |(kept, judged), &(z, was_kept)| {
                    let weight = (-0.5 * (z - diag.yank_stdevs).powi(2)).exp();
                    (kept + weight * was_kept as u8 as f64, judged + weight)
                });
        diag.keep_rate = (kept + 1.0) / (judged + 2.0);
        // stage 4b, look-alike places, counted only as far as the decision needs: picking one of N places the scan fits
        // equally well costs ln N of evidence, so with `evidence` in hand only a count above e^(evidence - bar) matters.
        // Not desperate (low pressure), the candidates checked stay near scan_context_top_k; desperate, they grow
        // toward every keyframe.
        let cap = (self.scan_context_config.candidate_top_k as f64
            / (1.0 - diag.pressure).max(f64::EPSILON))
        .ceil()
        .min(usize::MAX as f64) as usize;
        let mut look_alikes = similarity_discovery::LookAlikes::new(self.ranked.clone(), cap);
        let base = self.required_evidence(&diag);
        // two matching keyframes are one place if the odometry between them could have drifted that far apart
        let mut error_estimate_sums: Option<Vec<crate::loop_overlap::Sums>> = None;
        let mut required = |evidence: f64, diag: &mut ClosureCandidateDiag| -> f64 {
            self.bar_with_places(evidence, diag, &mut look_alikes, &mut error_estimate_sums, base)
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
            // extend as far as reaching the uncertainty range's top takes at this scan's own evidence per keyframe (at least one bit):
            // the yank and the look-alike places it matched set what is needed, not the pressure
            let per_keyframe = evidence.max(std::f64::consts::LN_2);
            diag.extension_budget = ((single_required * high / per_keyframe).ceil().max(2.0)
                as usize)
                .next_power_of_two()
                .min(self.config.closure_extended_icp_max_keyframes);
        }
        diag.required_confidence = 1.0 - (-single_required).exp();
        // a stack of keyframes earns about the single scan's evidence per keyframe at best: one that could not climb to
        // its bar even at the full stack is decided now
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
            let source_points = memo("srcn", &mut || vec![source_cloud().len() as f64])[0] as usize;
            diag.promise = ratio;
            return SearchStage::Extend(Box::new(PendingExtension {
                search: self,
                diag,
                timing,
                icp,
                ratio,
                source_points,
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

    /// Send every closure search's cloud computations through `cache` (`icp_cache`): recording them, or answering from
    /// a recording.
    pub fn set_icp_cache(&mut self, cache: Arc<crate::icp_cache::IcpCache>) {
        self.icp_cache = Some(cache);
    }

    /// The odometry error estimate in use as ratios to the reported variances (xy, z, yaw, roll/pitch;
    /// `learned_odometry_error_estimate_from_closures`); all 1 when off or before a kept closure.
    /// The factor on each odometry variance everything uses: always the stated odometry error estimate (the learned estimate,
    /// `learned_odometry_error_estimate`, is only logged).
    pub fn odometry_error_estimate(&self) -> [f64; 4] {
        [1.0; 4]
    }

    /// The learned odometry error estimate (`learned_odometry_error_estimate_from_closures`), as factors on the stated variances; logged, never used.
    pub fn learned_odometry_error_estimate(&self) -> [f64; 4] {
        self.learned_odometry_error_estimate_ln.map(f64::exp)
    }

    /// The stated odometry error estimate this run's learned one implies, in the config's own units
    /// (`odometry_error_estimate_*_per_sqrt_m` and `_per_sqrt_rad`, per source xy, z, yaw, tilt), to start the next run
    /// from; and per source whether the closures measured it (an unmeasured source repeats the configured one).
    pub fn odometry_error_estimate_for_next_run(&self) -> ([[f64; 2]; 4], [bool; 4]) {
        let c = &self.config;
        let stated = [
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
        ];
        let noise = self.learned_odometry_error_estimate();
        let measured =
            std::array::from_fn(|k| self.learned_odometry_error_estimate_ln_sd[k].is_some());
        // the learned odometry error estimate scales each source's variances, so its drift (a standard deviation) by the square root
        (
            std::array::from_fn(|k| stated[k].map(|d| d * noise[k].sqrt())),
            measured,
        )
    }

    /// The prior sd the learner holds each source's ln estimate to: the sd whose weight is `learned_odometry_error_estimate_stiffness`
    /// residual checks (-2 ln curvature 2 / sd^2 = N; infinite at 0, no prior).
    pub(crate) fn learned_odometry_error_estimate_prior(&self) -> [f64; 4] {
        [(2.0 / self.config.learned_odometry_error_estimate_stiffness).sqrt(); 4]
    }

    /// After a solve: the odometry error estimate from the closures it kept, and a re-solve when it moved by more than its own
    /// standard error since the solve began, so the graph is solved at the noise its closures imply.
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
        // the odometry error estimate from the solved graph's own residuals: how far the solve bent each source's part of the
        // odometry edges against how many independent checks the graph gave it (`odometry_error_estimate_evidence`), which tells
        // turning from sliding where a closure's single innovation cannot
        let mut ln_odometry = self.learned_odometry_error_estimate_ln;
        let mut redundancy = [0.0; 4];
        if let Some(evidence) = evidence {
            for c in 0..4 {
                let (sum, r) = evidence[c];
                redundancy[c] = r;
                if r > 0.5 && sum.is_finite() {
                    let (dispatched, prior_sd) = (
                        self.learned_odometry_error_estimate_ln_dispatched[c],
                        self.learned_odometry_error_estimate_prior()[c],
                    );
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
        // what the next solve would use (the estimate, or its lower bound) against what the last one did
        let previous = std::mem::replace(&mut self.learned_odometry_error_estimate_ln, ln_odometry);
        let next_solve = self.odometry_error_estimate().map(f64::ln);
        self.learned_odometry_error_estimate_ln = previous;
        let moved = (0..4).any(|c| {
            redundancy[c] > 0.5
                && (next_solve[c] - self.learned_odometry_error_estimate_ln_dispatched[c]).abs()
                    > (2.0 / redundancy[c]).sqrt()
        });
        if moved && !self.odometry_error_estimate_rescaled {
            self.odometry_error_estimate_rescale = true;
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
    /// Per finished solve: the keyframes it was dispatched over, its newest keyframe's time, and its seconds.
    pub fn solve_records(&self) -> &[(usize, f64, f64)] {
        &self.solve_records
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
    /// Keep a search's diagnostics, and when it found a candidate, its proposal and gate outcome
    /// as closure-edge events.
    pub(crate) fn record_search(&mut self, mut diag: ClosureCandidateDiag) {
        // a search that ended before its proposal was built never saw the noise scales
        if diag.target_id < 0 {
            diag.learned_odometry_error_estimate = self.learned_odometry_error_estimate();
            diag.odometry_error_estimate_gate = self.odometry_error_estimate();
        }
        if diag.target_id >= 0 {
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
    /// Add `cloud_with_pose` as a keyframe if it moved far enough from the last one (returns whether it did); every
    /// `pressure_check_frequency`-th scan also recomputes the loop-overlap pressure.
    pub fn add_key_pose(&mut self, cloud_with_pose: &CloudWithPose) -> bool {
        let started = std::time::Instant::now();
        let pose = &cloud_with_pose.pose;
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
        let due = self.is_key_pose(&cloud_with_pose.pose);
        let cloud = if self.config.drop_reseen_points {
            self.drop_reseen(cloud_with_pose)
        } else {
            cloud_with_pose.cloud.clone()
        };
        let (keyframe, cloud) = if self.config.auto_aggregate_cloud_size > 0 {
            self.gather(&cloud_with_pose.pose, cloud, due)
        } else {
            (due, cloud)
        };
        if keyframe {
            self.insert_keyframe(&cloud_with_pose.pose, cloud);
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
        // the pressure is a job on its own worker (`pressure_schedule`), held to its estimate in deterministic mode; the
        // debounce and the merge land every scan too, so an adoption is not held to the next keyframe
        self.step_pressure(check_due);
        if !self.adoption_inline && self.step_adoption(false) {
            self.adopted_since_update = true;
        }
        self.timing.pressure_s += checked.elapsed().as_secs_f64();
        keyframe
    }
    /// `auto_aggregate_cloud_size`: add the scan to the gathered points; a due keyframe is made once they reach the minimum,
    /// its cloud the gathered points in its body frame (and the gathering starts over), else it waits.
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
    /// The body's speed (m/s): the median over the newest `solve_speed_window_poses` steps between distinct odometry
    /// poses, by their recorded times (NaN before two).
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
    /// Whether a closure was committed since the last `update`.
    pub fn has_closure(&self) -> bool {
        self.new_closure
    }

    /// Stages 1 and 2 for the newest keyframe, inline on every keyframe: it becomes the waiting
    /// proposal if its scan-context match beats the one already waiting (the loser is recorded as
    /// "outranked"), so a busy search thread still gets the most promising keyframe it missed.
    pub fn propose_closure(&mut self) {
        let started = std::time::Instant::now();
        self.propose();
        self.timing.propose_s += started.elapsed().as_secs_f64();
    }
    fn propose(&mut self) {
        // nothing to close onto until a keyframe exists whose submap leaves out this one's scan
        if self.key_poses.is_empty() || self.newest_closure_target() == 0 {
            return;
        }
        let cur_idx = self.key_poses.len() - 1;
        // a proposal still waiting from an older keyframe (the search thread was busy) is dropped, not queued: the
        // newest keyframe's search replaces it
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
        let descriptor = &self.scan_context_descriptors[cur_idx];
        if !descriptor.is_empty() {
            diag.descriptor_occupancy =
                similarity_discovery::descriptor_occupancy(descriptor) as i32;
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
        };
        self.offer_proposal(proposal);
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
    /// Take the waiting proposal as a `ClosureSearch` (stages 3 and 4) to run on any thread and hand to
    /// `finish_closure_search`.
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
            ..
        } = self.waiting_proposal.take()?;
        let mut diag = diag;
        diag.pressure = self
            .pressure
            .map_or(0.0, |(_, overlap)| overlap.unresolved_loop_probability);
        diag.pressure2 = self
            .pressure
            .map_or(0.0, |(_, overlap)| overlap.unclosed_overlap_probability);
        diag.learned_odometry_error_estimate = self.learned_odometry_error_estimate();
        diag.odometry_error_estimate_gate = self.odometry_error_estimate();
        let source_position = self.key_poses.position(diag.source_id);
        let target_position = self.key_poses.position(target_idx);
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
            scan_context_descriptors: self.scan_context_descriptors.clone(),
            scan_context_config: self.scan_context_config,
            ranked,
            judged_yanks: Vec::new(),
            alternatives,
            tries: tries + 1,
            icp_cache: self.icp_cache.clone(),
            keyframe_clouds: self.keyframe_clouds.clone(),
            debug_io_s: std::cell::Cell::new(0.0),
            dump_s: Arc::new(std::sync::Mutex::new(0.0)),
            progress: None,
            mean_reach: if self.keyframe_reach.1 > 0 {
                self.keyframe_reach.0 / self.keyframe_reach.1 as f64
            } else {
                self.config.closure_search_radius
            },
        })
    }
    /// Search for a closure at the newest keyframe as a lockstep replay does (`step_closure_search`): deterministic, it
    /// waits for a job only once its hold has passed; fast, it waits for none, the searches landing as they finish, as
    /// live (the keyframe thread only dispatches).
    pub fn search_for_closure_pairs(&mut self) {
        self.step_closure_search(self.config.deterministic());
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
    /// Offer the rejected search's keyframe again at its next scan-context match, while it has tries left: 1 +
    /// floor(sqrt(min(ceil(1 / (1 - pressure)), scan_context_top_k - 1))) (`closure_alternative_places`).
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
        // the extra tries grow with pressure as before, but under a square root: retries far down the match list
        // rarely close (small Mid-360: 1 accept in 419 tries past the second)
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
        });
    }
    /// Add a keyframe at `pose`, placed by the current correction.
    fn insert_keyframe(&mut self, pose: &PoseWithTime, cloud: Option<Arc<PointCloud>>) {
        // a solve waiting on the graph change heat is adopted at the last keyframe count's end once the heat is under
        let keyframes = self.key_poses.len();
        self.release_pending_solve(keyframes);
        self.graph_version += 1;
        // every use of a keyframe's cloud (submaps, stacked clouds) voxelizes it at submap_resolution, so it is
        // stored that way: a dense scan (the Go2 L1's ~30k-point local map) would otherwise be kept whole
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
        // placed by the current correction (`KeyFrames::global` rides it on the odometry pose when read)
        self.key_poses.push(crate::store::KeyFixed {
            time: pose.second,
            rotation_local: pose.rotation,
            translation_local: pose.translation,
            body_cloud: stored,
        });

        // the scan context descriptor and ring key (empty without a cloud); a non-positive max range is set once from
        // the first cloud's extent, so every descriptor shares one binning
        if let Some(cloud) = &cloud {
            if self.scan_context_config.max_range_m <= 0.0 {
                let auto_range = similarity_discovery::auto_max_range(cloud);
                if auto_range > 0.0 {
                    self.scan_context_config.max_range_m = auto_range;
                    eprintln!(
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
        self.scan_context_descriptors.push(Arc::new(descriptor));
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
        let (fit_confidence, yank_stdevs) = self.committed_stats.take().unwrap_or((f64::NAN, f64::NAN));
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
    }
}

#[cfg(test)]
mod config_tests {
    use super::Config;

    /// The target submap's half range is 5 for the single scan (today's fixed submap) and grows with the extension's
    /// source clouds.
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
}
