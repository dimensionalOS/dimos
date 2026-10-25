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

use crate::big_picture_fit::{closure_edges, Edge, Solving};
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
    /// Global distance between source and candidate keyframe (the
    /// candidate-distance gate input).
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
    /// yank_drift_scale = sqrt(keyframe gap);
    /// sigma_* = 1.4826 x the median yank / drift scale of the candidates ICP scored before it (NaN before any);
    /// yank_z = hypot(yank_rotation_deg / drift / sigma_rotation_deg, yank_translation / drift / sigma_translation_m)
    /// (infinite before any, so the first candidate only seeds the scale); each factor is its metric normalized by its own gate and clamped to [0, 1];
    /// confidence = their product; required_confidence = 1 - exp(-yank_z^2 / 2k^2), k = `closure_yank_gate_width`.
    pub yank_drift_scale: f64,
    pub sigma_rotation_deg: f64,
    pub sigma_translation_m: f64,
    pub yank_z: f64,
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
            candidate_distance: f64::NAN,
            degeneracy_min: -1.0,
            icp_converged: false,
            icp_fitness: f64::NAN,
            icp_overlap: f64::NAN,
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
    /// One GNC solve's weight for it.
    Vote = 3,
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
    /// yanks of closures committed so far (`ClosureCandidateDiag::yank_z`); NaN before ICP.
    pub yank_z: f64,
}

/// Bytes per packed closure-edge record (`ClosureEdgeEvent::put_record`).
pub const CLOSURE_EDGE_RECORD_BYTES: usize = 52;

impl ClosureEdgeEvent {
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
    pub closure_search_radius: f64,
    pub closure_score_thresh: f64,
    pub closure_submap_half_range: i32,
    pub submap_resolution: f64,
    /// Structure-spread gate: minimum occupied Scan-Context cells. 0 disables.
    pub closure_min_occupancy: i32,
    /// Observability gate (Zhang 2016 / X-ICP degeneracy factor). 0 disables.
    pub closure_min_degeneracy: f64,
    /// A closure's match confidence must clear 1 - exp(-yank_z^2 / 2k^2) (see `ClosureCandidateDiag`): a small yank
    /// needs little confidence, a large one a near-perfect match. This is k, the sigmas of yank before that.
    /// Yank is measured per sqrt(keyframe gap), since odometry drift is a random walk (on huge_loop this ranks
    /// closures that help above closures that hurt at AUC 0.82 vs 0.70).
    pub closure_yank_gate_width: f64,
    /// When set, every candidate ICP scores writes `<dir>/<source>_<target>.icp`: the source cloud
    /// and target submap exactly as ICP got them (global frame, current graph), then ICP's
    /// rotation (row-major) and translation, fitness and convergence. Little-endian: u32 source
    /// count, u32 target count, f32 xyz points, 9 + 3 f64, f64 fitness, u8 converged.
    pub closure_icp_dump_dir: String,
    /// A candidate whose single-scan confidence is too low for its yank gets
    /// a second look on more data instead of an outright reject: each round seeded with the previous
    /// round's ICP (the first round with the single-scan ICP), the source grows to the last 2, 4, 8, ... keyframes (up to this many), and the target to the
    /// keyframes near the matched one in time that those land within `closure_search_radius` of.
    /// Inliers are source points within sqrt(`closure_score_thresh`) of the target after ICP.
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
    /// Hold keyframes with two or more independent cycles nearby (see `big_picture_fit::dispatch_solve`), so a solve
    /// covers only the part of the map that can still move.
    pub closure_cooling: bool,
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
            closure_search_radius: 3.0,
            closure_score_thresh: 0.15,
            closure_submap_half_range: 5,
            submap_resolution: 0.1,
            closure_min_occupancy: 80,
            closure_min_degeneracy: 0.05,
            closure_yank_gate_width: 1.0,
            closure_icp_dump_dir: String::new(),
            closure_extended_icp_max_keyframes: 16,
            closure_robust_huber_k: 1.345,
            closure_icp_max_iterations: 50,
            closure_icp_max_correspondence_m: 10.0,
            closure_gnc_var_scale: 10.0,
            closure_gnc_inlier_probability: 0.95,
            closure_cooling: false,
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

#[derive(Default)]
pub struct GscPgo {
    pub(crate) config: Config,
    pub(crate) scan_context_config: similarity_discovery::Config,
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) committed_closures: Vec<CommittedClosure>,
    /// Per committed closure: its yank_z and gate confidence, for its closure-edge events.
    pub(crate) closure_yank_z: Vec<f64>,
    pub(crate) closure_fit_confidence: Vec<f64>,
    /// A closure was committed since the last `update`.
    pub(crate) new_closure: bool,
    /// Committed closures a solve has been started for.
    pub(crate) closures_dispatched: usize,
    pub(crate) solving: Option<Solving>,
    pub(crate) graph_error: f64,
    pub(crate) scan_context_descriptors: Vec<similarity_discovery::Descriptor>,
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
    pub(crate) scored_yanks: Vec<(f64, f64)>,
    pub(crate) waiting_proposal: Option<ClosureProposal>,
    /// Called when a background solve finishes (`notify_when_solved`).
    pub(crate) solved: Option<Arc<dyn Fn() + Send + Sync>>,
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

/// The ICP half of one closure search, on its own copy of the keyframes so it can run on another thread
/// while scans keep arriving (`GscPgo::begin_closure_search` makes it, `finish_closure_search` applies it).
/// A keyframe through stages 1 and 2, waiting for the search thread: its candidate target, the
/// scan-context column shift between them, and the match's cosine distance (lower is better).
pub struct ClosureProposal {
    pub(crate) diag: ClosureCandidateDiag,
    pub(crate) target_idx: usize,
    pub(crate) sector_shift: i32,
    pub(crate) similarity: f32,
}

pub struct ClosureSearch {
    pub(crate) key_poses: Vec<KeyPoseWithCloud>,
    pub(crate) config: Config,
    pub(crate) scored_yanks: Vec<(f64, f64)>,
    /// The closures the newest solve kept.
    pub(crate) kept_closures: Vec<Edge>,
    pub(crate) diag: ClosureCandidateDiag,
    pub(crate) target_idx: usize,
    pub(crate) init_rotation: Mat3,
    pub(crate) init_translation: Vec3,
}

/// What a `ClosureSearch` found: its diagnostics, the closure it accepts (relative rotation, translation,
/// fitness), the yank it scored, and the time each stage took.
pub struct ClosureSearchResult {
    diag: ClosureCandidateDiag,
    accepted: Option<(Mat3, Vec3, f64)>,
    scored_yank: Option<(f64, f64)>,
    timing: TimingStats,
}

/// A search that ended at the gate `outcome`.
fn rejected(
    mut diag: ClosureCandidateDiag,
    outcome: &'static str,
    scored_yank: Option<(f64, f64)>,
    timing: TimingStats,
) -> ClosureSearchResult {
    diag.outcome = outcome;
    ClosureSearchResult {
        diag,
        accepted: None,
        scored_yank,
        timing,
    }
}

impl ClosureSearch {
    /// The ICP half of a closure search: submaps, ICP, its gates, the yank / confidence gate and extended ICP.
    pub fn run(self) -> ClosureSearchResult {
        let mut diag = self.diag.clone();
        let (cur_idx, target_idx) = (diag.source_id, self.target_idx);
        let (init_rotation, init_translation) = (self.init_rotation, self.init_translation);
        let mut timing = TimingStats::default();
        let mut scored_yank = None;
        let t0 = std::time::Instant::now();
        let target_cloud = sub_map(
            &self.key_poses,
            target_idx as i32,
            self.config.closure_submap_half_range,
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
                max_iterations: self.config.closure_icp_max_iterations as usize,
                max_correspondence_distance: self.config.closure_icp_max_correspondence_m,
                inlier_squared_distance: self.config.closure_score_thresh,
            },
        );
        timing.icp_s += t0.elapsed().as_secs_f64();
        if !self.config.closure_icp_dump_dir.is_empty() {
            dump_icp(
                &self.config.closure_icp_dump_dir,
                cur_idx,
                target_idx,
                &source_cloud,
                &target_cloud,
                &icp,
                "",
            );
        }
        diag.icp_converged = icp.converged;
        diag.icp_fitness = icp.fitness;
        diag.icp_overlap = icp.overlap;
        diag.icp_translation_norm = mat3::norm(&icp.translation);

        // Observability gate: a planar/degenerate source scan leaves the
        // alignment unconstrained in-plane — fitness lies. Computed for every
        // candidate that reaches ICP (not just when the gate is on) so the
        // metric is always available to compare good vs bad closures.
        let t0 = std::time::Instant::now();
        let degeneracy_min = uniqueness::degeneracy(&source_cloud);
        timing.degeneracy_s += t0.elapsed().as_secs_f64();
        diag.degeneracy_min = degeneracy_min;

        if self.config.closure_min_degeneracy > 0.0
            && degeneracy_min >= 0.0
            && (degeneracy_min as f64) < self.config.closure_min_degeneracy
        {
            return rejected(diag, "degeneracy_reject", scored_yank, timing);
        }

        if !icp.converged || icp.fitness > self.config.closure_score_thresh {
            return rejected(diag, "icp_reject", scored_yank, timing);
        }
        // the pose graph takes ICP's fitness as the closure's variance: when the graph is already surer of the two
        // ends' relative position than that (along the odometry, or across one kept closure), the closure adds only
        // ICP's noise, and repeats of a closure already kept would count the same evidence twice
        let step = |a: usize, b: usize| a.abs_diff(b) as f64 * self.config.odom_trans_xy_var;
        let known = self
            .kept_closures
            .iter()
            .fold(step(cur_idx, target_idx), |known, l| {
                let via =
                    |a: usize, b: usize| step(target_idx, a) + l.variances[3] + step(cur_idx, b);
                known.min(via(l.from, l.to)).min(via(l.to, l.from))
            });
        if icp.fitness >= known {
            return rejected(diag, "uninformative", scored_yank, timing);
        }

        let mut score = icp.fitness;
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

        scored_yank = Some(self.score_candidate(&mut diag));
        if diag.confidence < diag.required_confidence {
            let t0 = std::time::Instant::now();
            let extended = self.extended_icp(cur_idx, target_idx, &source_cloud, &icp, &mut diag);
            timing.extended_icp_s += t0.elapsed().as_secs_f64();
            let Some(extended) = extended else {
                return rejected(diag, "yank_confidence_reject", scored_yank, timing);
            };
            // the closure measurement comes from the extended alignment, which saw more of both places
            (rotation_offset, translation_offset) =
                measured(&extended.rotation, &extended.translation);
            score = extended.fitness;
        }

        diag.outcome = "accepted";
        ClosureSearchResult {
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
            rotation_offset: mat3::identity(),
            ..Default::default()
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
        yank_z: f64,
    ) {
        self.closure_edge_events.push(ClosureEdgeEvent {
            time: self.newest_keyframe_time(),
            source_id,
            target_id,
            state,
            confidence: confidence.clamp(0.0, 1.0),
            yank_z,
        });
    }
    /// Keep a search's diagnostics, and when it found a candidate, its proposal and gate outcome
    /// as closure-edge events.
    pub(crate) fn record_search(&mut self, diag: ClosureCandidateDiag) {
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
                self.closure_yank_z.push(diag.yank_z);
            }
            self.push_edge(source, target, outcome, checked, diag.yank_z);
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
    /// Insert a keyframe if the pose clears the keyframe gate; returns
    /// whether it was added.
    pub fn add_key_pose(&mut self, cloud_with_pose: &CloudWithPose) -> bool {
        if !self.is_key_pose(&cloud_with_pose.pose) {
            return false;
        }
        self.insert_keyframe(&cloud_with_pose.pose, cloud_with_pose.cloud.clone());
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
    /// Whether a closure was committed since the last `update`.
    pub fn has_closure(&self) -> bool {
        self.new_closure
    }

    /// Candidate search (scan context with position fallback) + all the
    /// gates, ICP verification, and LoopPair construction.
    /// Record that this keyframe's closure search was skipped because newer scans
    /// are already queued behind it: a search (an ICP on a large cloud can take
    /// seconds) must never be what makes the module fall behind.
    /// Stages 1 and 2 for the newest keyframe, inline on every keyframe: it becomes the waiting
    /// proposal if its scan-context match beats the one already waiting (the loser is recorded as
    /// "outranked"), so a busy search thread still gets the most promising keyframe it missed.
    pub fn propose_closure(&mut self) {
        if self.key_poses.len() < 10 {
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
        let (mut target_idx, sector_shift, sc_best, sc_second) = self.search_by_scan_context();
        self.timing.scan_context_s += t0.elapsed().as_secs_f64();
        diag.from_scan_context = target_idx >= 0;
        if target_idx < 0 {
            target_idx = self.search_by_position();
        }
        if target_idx < 0 {
            diag.outcome = "no_candidate";
            self.record_search(diag);
            return;
        }
        let target_idx = target_idx as usize;
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
    /// Take the waiting proposal as a `ClosureSearch` (stages 3 and 4) to run on any thread and hand to
    /// `finish_closure_search`.
    pub fn begin_closure_search(&mut self) -> Option<ClosureSearch> {
        let ClosureProposal {
            diag,
            target_idx,
            sector_shift,
            ..
        } = self.waiting_proposal.take()?;
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
            scored_yanks: self.scored_yanks.clone(),
            // kept, or not judged yet
            kept_closures: closure_edges(
                self.committed_closures
                    .iter()
                    .filter(|l| !(l.gnc_weight <= 0.5)),
            ),
            diag,
            target_idx,
            init_rotation,
            init_translation,
        })
    }
    /// Search for a closure at the newest keyframe, all on this thread.
    pub fn search_for_closure_pairs(&mut self) {
        self.propose_closure();
        if let Some(search) = self.begin_closure_search() {
            let result = search.run();
            self.finish_closure_search(result);
        }
    }
    /// Apply a `ClosureSearch`'s result: record its diagnostics and commit the closure it accepted.
    pub fn finish_closure_search(&mut self, result: ClosureSearchResult) {
        let ClosureSearchResult {
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
        self.commit_closure(source, target, rotation_offset, translation_offset, score);
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
        self.scan_context_descriptors.push(descriptor);
    }

    /// Commit a closure that passed every gate; the next `update` starts a solve with it.
    fn commit_closure(
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
