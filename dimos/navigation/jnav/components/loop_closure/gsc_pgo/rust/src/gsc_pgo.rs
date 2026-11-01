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
    /// "accepted", "outranked" (a better proposal took the search thread), or the gate that rejected it:
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
        data.extend_from_slice(&(self.yank_stdevs as f32).to_le_bytes());
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
    /// threshold, closest first), up to 1 + ceil(1 / (1 - pressure)) tries per keyframe (2 at no pressure), at most
    /// scan_context_top_k; a newer keyframe's better match still outranks a waiting retry.
    pub closure_alternative_places: bool,
    /// Alternative places are tried only at or above this pressure; below it a keyframe tries only its best match.
    /// Retries were two-thirds of all closure ICPs (small Mid-360: 2,012 of 3,256 candidates for 48 of 173 right kept
    /// loops; huge: 12,012 of 16,867 for 69 right and 150 wrong).
    pub closure_alternative_places_min_pressure: f64,
    /// When true, a closure the previous solve kept is a commitment: each solve holds the committed closures in, and
    /// drops them only when a cold solve (every closure re-decided) keeps more closures and rejects some committed one,
    /// a larger consistent group contradicting it; a cold solve that merely loses track of a group (GNC's start dragged
    /// by newer wrong closures) no longer drops it (huge overtrust: 24 kept, 0, 24, 0).
    pub closure_commitments: bool,
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
            barely_fail_uncertainty_range: 0.8,
            barely_pass_uncertainty_range: 1.05,
            decisive_pressure_rate: 1.0,
            extension_cloud_spacing: 0.25,
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
            closure_commitments: true,
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
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) committed_closures: Vec<CommittedClosure>,
    /// Per committed closure: its yank_stdevs and gate confidence, for its closure-edge events.
    pub(crate) closure_yank_stdevs: Vec<f64>,
    /// ln of each odometry variance's ratio to the reported one (0: the reported noise), set from the kept closures
    /// after each solve.
    pub(crate) learned_odometry_error_estimate_ln: [f64; 4],
    /// Each `learned_odometry_error_estimate_ln`'s posterior standard deviation once the residuals have measured it (the prior's before).
    pub(crate) learned_odometry_error_estimate_ln_sd: [Option<f64>; 4],
    /// The ln scale the running (or newest) solve was started with, and whether a re-solve at a moved scale is due.
    pub(crate) learned_odometry_error_estimate_ln_dispatched: [f64; 4],
    pub(crate) odometry_error_estimate_rescale: bool,
    /// The next solve also measures the learned odometry error estimate's evidence (only `finish`'s last solve does).
    pub(crate) learned_evidence_requested: bool,
    /// Whether the newest solve was itself a re-solve at a moved noise (no new closure or fix): one per new piece of
    /// evidence, since the estimate need not settle (GNC's verdicts can change with the noise) and would re-solve forever.
    pub(crate) odometry_error_estimate_rescaled: bool,
    pub(crate) closure_fit_confidence: Vec<f64>,
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
    pub(crate) landmark_sightings: Vec<(f64, u64, Mat3, Vec3, [[f64; 6]; 6])>,
    pub(crate) landmark_sightings_dispatched: usize,
    pub(crate) landmarks: Vec<(u64, Mat3, Vec3)>,
    pub(crate) solving: Option<Solving>,
    pub(crate) graph_error: f64,
    /// Scans offered to `add_key_pose`, and the newest loop-overlap pressure with the scan count it was computed at.
    pub(crate) scans_seen: usize,
    pub(crate) pressure: Option<(usize, crate::loop_overlap::LoopOverlap)>,
    /// Sum and count of the keyframes' scan reaches (95th-percentile planar range), the sensor range loop_overlap uses.
    pub(crate) keyframe_reach: (f64, usize),
    /// `drop_reseen_points`: the previous scan's points (odometry frame, 0.1 mm grid).
    pub(crate) reseen_previous: std::collections::HashSet<[i64; 3]>,
    /// `auto_aggregate_cloud_size`: the scans since the last keyframe, in the odometry frame, voxelized.
    pub(crate) gathered: PointCloud,
    /// The newest applied solve's tip keyframe and every solved keyframe's position covariance relative to it.
    pub(crate) solved_relative_covariance: Option<(usize, Vec<Mat3>)>,
    /// Shared, so a closure search can count look-alike places without copying them.
    pub(crate) scan_context_descriptors: Vec<Arc<similarity_discovery::Descriptor>>,
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
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) config: Config,
    /// The odometry's mean xy variance per keyframe step at the reported noise (`big_picture_fit::mean_step_variances`).
    pub(crate) mean_step_xy_variance: f64,
    /// The closures the newest solve kept.
    pub(crate) kept_closures: Vec<Edge>,
    pub(crate) diag: ClosureCandidateDiag,
    pub(crate) target_idx: usize,
    pub(crate) init_rotation: Mat3,
    pub(crate) init_translation: Vec3,
    pub(crate) scan_context_descriptors: Vec<Arc<similarity_discovery::Descriptor>>,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) ranked: Vec<(f32, usize)>,
    /// Every closure the solve has judged: its yank_stdevs when committed, and whether the solve kept it.
    pub(crate) judged_yanks: Vec<(f64, bool)>,
    pub(crate) alternatives: Vec<(f32, usize, i32)>,
    pub(crate) tries: usize,
    /// The ICP cache the search's cloud computations go through (`icp_cache`), if any.
    pub(crate) icp_cache: Option<Arc<crate::icp_cache::IcpCache>>,
    /// Each keyframe's voxelized scan and range image, built once and shared by every closure search.
    pub(crate) keyframe_clouds: Arc<crate::proportional_confidence::KeyframeClouds>,
    /// Seconds spent writing debug dumps (`closure_icp_dump_dir`), taken out of the stage timings.
    pub(crate) debug_io_s: std::cell::Cell<f64>,
    /// The keyframes' mean scan reach (m), which spaces an extension's separate source clouds.
    pub(crate) mean_reach: f64,
}

/// What a `ClosureSearch` found: its diagnostics, the closure it accepts (relative rotation, translation,
/// fitness), and the time each stage took.
pub struct ClosureSearchResult {
    diag: ClosureCandidateDiag,
    accepted: Option<(Mat3, Vec3, f64)>,
    timing: TimingStats,
    /// The search's place-recognition inputs, for a retry on the next alternative (`closure_alternative_places`).
    retry: Option<(Vec<(f32, usize)>, Vec<(f32, usize, i32)>, usize)>,
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
    pub fn run(mut self) -> ClosureSearchResult {
        let retry = (
            self.ranked.clone(),
            std::mem::take(&mut self.alternatives),
            self.tries,
        );
        let mut result = self.search();
        result.retry = Some(retry);
        result
    }
    fn search(self) -> ClosureSearchResult {
        let mut diag = self.diag.clone();
        let (cur_idx, target_idx) = (diag.source_id, self.target_idx);
        let (init_rotation, init_translation) = (self.init_rotation, self.init_translation);
        let mut timing = TimingStats::default();
        let half_range = self.config.target_half_range(1);
        // the source scan, the target submap and its tree, built only when a computation needs them (never when every
        // result comes from the ICP cache)
        // the source scan alone (all the degeneracy gate reads), built apart from the target and its tree
        let source_cell = std::cell::OnceCell::new();
        let source_cloud = || -> &PointCloud {
            source_cell.get_or_init(|| self.stacked_cloud(std::iter::once(cur_idx)))
        };
        let clouds_cell = std::cell::OnceCell::new();
        let clouds = || -> &(PointCloud, PointCloud, KdTree) {
            clouds_cell.get_or_init(|| {
                let target_cloud = self.stacked_cloud(
                    target_idx.saturating_sub(half_range)
                        ..=(target_idx + half_range).min(self.key_poses.len() - 1),
                );
                let target_tree = KdTree::build(&target_cloud);
                (source_cloud().clone(), target_cloud, target_tree)
            })
        };
        // a cloud computation, through the ICP cache when there is one; a point-to-plane alignment is a different
        // result from a point-to-point one, so it is part of the key
        let cache = self.icp_cache.clone();
        // the thinned target and the early stop change the single ICP's result too, so they are part of the key
        let key_suffix = format!(
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
        );
        let memo = |tag: &str, compute: &mut dyn FnMut() -> Vec<f64>| -> Vec<f64> {
            match &cache {
                Some(cache) => {
                    cache.get_or(format!("{cur_idx}:{target_idx}:{tag}{key_suffix}"), compute)
                }
                None => compute(),
            }
        };
        // an alignment through the cache: stored as the source's pose relative to the target (`relative_icp`), the
        // computed value itself returned when it was computed here
        let (source_pose, target_pose) = (
            (
                &self.key_poses[cur_idx].rotation_global,
                &self.key_poses[cur_idx].translation_global,
            ),
            (
                &self.key_poses[target_idx].rotation_global,
                &self.key_poses[target_idx].translation_global,
            ),
        );
        let memo_alignment = |tag: &str,
                              at: usize,
                              compute: &mut dyn FnMut() -> Vec<f64>|
         -> Vec<f64> {
            let Some(cache) = &cache else {
                return compute();
            };
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
        };

        // Observability gate: a planar/degenerate source scan leaves the alignment unconstrained in-plane, so fitness
        // lies. It reads only the source scan, so it runs before ICP (computed for every candidate, gate on or off)
        let degenerate = |degeneracy_min: f32| {
            self.config.closure_min_degeneracy > 0.0
                && degeneracy_min >= 0.0
                && (degeneracy_min as f64) < self.config.closure_min_degeneracy
        };
        let t0 = std::time::Instant::now();
        diag.degeneracy_min = memo("deg", &mut || {
            vec![uniqueness::degeneracy(source_cloud()) as f64]
        })[0] as f32;
        timing.degeneracy_s += t0.elapsed().as_secs_f64();
        if degenerate(diag.degeneracy_min) {
            return rejected(diag, "degeneracy_reject", timing);
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
        // at the upper bound: scaled by the most any component's bound exceeds its estimate
        let widen = (0..4)
            .map(|c| gate_error_estimate[c] / diag.learned_odometry_error_estimate[c])
            .fold(1.0, f64::max);
        let known = if by_graph {
            // the graph's own covariance of the two ends' relative position (its heading swing and the solved
            // closures included): the closure adds something only where it is surer than the graph in some direction.
            // Per-keyframe xy steps say nothing of the heading's swing, which per-motion noise puts most of the drift in
            widen
                * mat3::jacobi_eigen(diag.innovation_covariance)
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
        // no alignment of two voxelized scans fits tighter than the spread of a surface's voxel centroids between them
        // (two independent offsets within a voxel of the submap resolution r: variance r^2 / 6 along the normal), so a
        // graph surer than that makes the closure uninformative whatever ICP returns: it is not aligned at all
        let closure_variance_floor = self.config.submap_resolution.powi(2) / 6.0;
        if self.config.closure_uninformative_gate && closure_variance_floor >= known {
            return rejected(diag, "uninformative", timing);
        }
        // a candidate whose least yank ICP could leave still needs more evidence than any stack of scans could give is
        // rejected unaligned: ICP pairs points only within its match distance, so it moves the source from the seed (the
        // target's position) by about that much at most; the closure's own variance is at most the fitness limit squared;
        // and a point's likelihood ratio is at most ln(2 reach / (width sqrt(2 pi))) with width no less than a quarter
        // voxel, times the largest stack
        if self.config.closure_likelihood_confidence && self.config.closure_yank_pre_check {
            let reach = self.config.closure_icp_max_correspondence_m;
            let source_position = self.key_poses[cur_idx].translation_global;
            let seeded = self.key_poses[target_idx].translation_global;
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
                    return rejected(diag, "yank_confidence_reject", timing);
                }
            }
        }
        let t0 = std::time::Instant::now();
        let icp = crate::icp_cache::decode_icp(&memo_alignment("icpr", 0, &mut || {
            let (source_cloud, target_cloud, target_tree) = clouds();
            crate::icp_cache::encode_icp(&crate::connection_fit::icp_thinned(
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
        timing.icp_s += t0.elapsed().as_secs_f64();
        if !self.config.closure_icp_dump_dir.is_empty() && cache.is_none() {
            let (source_cloud, target_cloud, _) = clouds();
            dump_icp(
                &self.config.closure_icp_dump_dir,
                cur_idx,
                target_idx,
                source_cloud,
                target_cloud,
                &icp,
                "",
            );
        }
        diag.icp_converged = icp.converged;
        diag.icp_fitness = icp.fitness;
        diag.icp_overlap = icp.overlap;
        diag.icp_translation_norm = mat3::norm(&icp.translation);

        if !icp.converged || icp.fitness > self.config.closure_icp_max_fitness_m.powi(2) {
            return rejected(diag, "icp_reject", timing);
        }
        // the closure's translation variance as the graph will take it: its inliers' mean squared distance
        // (`IcpResult::inlier_mean_square`)
        let closure_variance = icp.inlier_mean_square();
        if self.config.closure_uninformative_gate && closure_variance >= known {
            return rejected(diag, "uninformative", timing);
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
        let mut score = icp.inlier_mean_square();
        let (source, target) = (&self.key_poses[cur_idx], &self.key_poses[target_idx]);
        let closure_rotation_transpose = mat3::transpose(&target.rotation_global);
        // the closure measurement an alignment gives: the source keyframe moved by it, in the target's frame
        let measured = |rotation: &Mat3, translation: &Vec3| {
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
        };
        let (mut rotation_offset, mut translation_offset) =
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
            &target.rotation_global,
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
        let same_place_cutoff =
            crate::loop_overlap::chi2_inverse_3dof(self.config.closure_gnc_inlier_probability);
        let mut required = |evidence: f64, diag: &mut ClosureCandidateDiag| -> f64 {
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
            return rejected(diag, "yank_confidence_reject", timing);
        }
        if ratio < high {
            let t0 = std::time::Instant::now();
            let source_points = memo("srcn", &mut || vec![source_cloud().len() as f64])[0] as usize;
            let extended = self.extended_icp(
                cur_idx,
                target_idx,
                source_points,
                &icp,
                ratio,
                &mut diag,
                &mut required,
                &memo_alignment,
            );
            timing.extended_icp_s += t0.elapsed().as_secs_f64() - self.debug_io_s.take();
            let Some(extended) = extended else {
                return rejected(diag, "yank_confidence_reject", timing);
            };
            // the closure measurement comes from the extended alignment, which saw more of both places
            (rotation_offset, translation_offset) =
                measured(&extended.rotation, &extended.translation);
            score = extended.inlier_mean_square();
        }

        diag.outcome = "accepted";
        ClosureSearchResult {
            diag,
            accepted: Some((rotation_offset, translation_offset, score)),
            timing,
            retry: None,
        }
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
                self.closure_fit_confidence.push(checked.clamp(0.0, 1.0));
                self.closure_yank_stdevs.push(diag.yank_stdevs);
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
        let frequency = self.config.pressure_check_frequency;
        if frequency > 0 && self.scans_seen.is_multiple_of(frequency) {
            if let Some(overlap) = self.loop_overlap() {
                self.pressure = Some((self.scans_seen, overlap));
            }
        }
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
    /// The newest loop-overlap pressure and the scan count it was computed at (see `pressure_check_frequency`).
    pub fn pressure(&self) -> Option<(usize, crate::loop_overlap::LoopOverlap)> {
        self.pressure
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
    /// Whether a closure was committed since the last `update`.
    pub fn has_closure(&self) -> bool {
        self.new_closure
    }

    /// Stages 1 and 2 for the newest keyframe, inline on every keyframe: it becomes the waiting
    /// proposal if its scan-context match beats the one already waiting (the loser is recorded as
    /// "outranked"), so a busy search thread still gets the most promising keyframe it missed.
    pub fn propose_closure(&mut self) {
        // nothing to close onto until a keyframe exists whose submap leaves out this one's scan
        if self.key_poses.is_empty() || self.newest_closure_target() == 0 {
            return;
        }
        let cur_idx = self.key_poses.len() - 1;
        let current = &self.key_poses[cur_idx];
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
        self.timing.scan_context_s += t0.elapsed().as_secs_f64();
        diag.from_scan_context = scan_context_target.is_some();
        let Some(target_idx) = scan_context_target.or_else(|| self.search_by_position()) else {
            diag.outcome = "no_candidate";
            self.record_search(diag);
            return;
        };
        diag.target_id = target_idx as i64;
        diag.target_time = self.key_poses[target_idx].time;
        diag.target_xyz = self.key_poses[target_idx].translation_global;
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
        // extending is worth its cost as far as a real loop is likely here; extension grows by doubling, so the budget is
        // the doubling step that covers max keyframes x that chance
        let model = self.overlap_model(diag.source_id);
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
        let source_position = self.key_poses[diag.source_id].translation_global;
        let target_position = self.key_poses[target_idx].translation_global;
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
            mean_step_xy_variance: crate::big_picture_fit::mean_step_variances(
                &self.key_poses,
                &self.config,
            )[0],
            // kept, or not judged yet
            kept_closures: closure_edges(
                self.committed_closures
                    .iter()
                    .filter(|l| l.gnc_weight.is_nan() || l.gnc_weight > 0.5),
            ),
            diag,
            target_idx,
            init_rotation,
            init_translation,
            scan_context_descriptors: self.scan_context_descriptors.clone(),
            scan_context_config: self.scan_context_config,
            ranked,
            judged_yanks: self
                .committed_closures
                .iter()
                .zip(&self.closure_yank_stdevs)
                .filter(|(c, z)| !c.gnc_weight.is_nan() && z.is_finite())
                .map(|(c, z)| (*z, c.gnc_weight > 0.5))
                .collect(),
            alternatives,
            tries: tries + 1,
            icp_cache: self.icp_cache.clone(),
            keyframe_clouds: self.keyframe_clouds.clone(),
            debug_io_s: std::cell::Cell::new(0.0),
            mean_reach: if self.keyframe_reach.1 > 0 {
                self.keyframe_reach.0 / self.keyframe_reach.1 as f64
            } else {
                self.config.closure_search_radius
            },
        })
    }
    /// Search for a closure at the newest keyframe, all on this thread.
    pub fn search_for_closure_pairs(&mut self) {
        self.propose_closure();
        // a rejected search may queue the keyframe's next alternative, which this thread tries right away
        while let Some(search) = self.begin_closure_search() {
            let result = search.run();
            self.finish_closure_search(result);
        }
    }
    /// Apply a `ClosureSearch`'s result: record its diagnostics and commit the closure it accepted.
    pub fn finish_closure_search(&mut self, result: ClosureSearchResult) {
        let ClosureSearchResult {
            diag,
            accepted,
            timing,
            retry,
        } = result;
        self.timing.submap_s += timing.submap_s;
        self.timing.icp_s += timing.icp_s;
        self.timing.degeneracy_s += timing.degeneracy_s;
        self.timing.extended_icp_s += timing.extended_icp_s;
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
    /// Offer the rejected search's keyframe again at its next scan-context match, while it has tries left: 1 + ceil(1 /
    /// (1 - pressure)), at most scan_context_top_k (`closure_alternative_places`).
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
        let allowed = (1.0 + (1.0 / (1.0 - rejected.pressure).max(f64::EPSILON)).ceil())
            .min(self.scan_context_config.candidate_top_k as f64) as usize;
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
        diag.target_time = self.key_poses[target_idx].time;
        diag.target_xyz = self.key_poses[target_idx].translation_global;
        diag.candidate_distance = mat3::norm(&mat3::sub(&diag.source_xyz, &diag.target_xyz));
        diag.scan_context_best = distance;
        diag.scan_context_second = rejected.scan_context_second;
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
        self.key_poses.push(KeyPoseWithCloud {
            time: pose.second,
            rotation_local: pose.rotation,
            translation_local: pose.translation,
            body_cloud: stored,
            rotation_global: mat3::mat_mul(&self.rotation_offset, &pose.rotation),
            translation_global: mat3::add(
                &mat3::mat_vec(&self.rotation_offset, &pose.translation),
                &self.translation_offset,
            ),
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
        self.committed_closures.push(CommittedClosure {
            source_id,
            target_id,
            rotation_offset,
            translation_offset,
            score,
            gnc_weight: f64::NAN,
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
