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

use crate::gtsam::{symbol_key, FactorGraph, Isam2, NoiseModel, Pose3, Values};
use crate::mat3::{self, Mat3, Vec3};
use crate::pointcloud::{
    self, cloud_degeneracy, icp_point_to_point, transform_cloud, voxel_downsample, IcpParams,
    PointCloud,
};
use crate::scan_context;

/// Variance that leaves a Pose3 tangent component effectively free while still
/// contributing a tiny bit of information (keeps the linear system non-singular).
const FREE_VARIANCE: f64 = 1e8;

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
    /// Robust (M-estimator) kernel wrapping all loop factors.
    pub loop_robust_kernel: bool,
    pub loop_robust_huber_k: f64,
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
            keyframe_min_distance_meters: 1.0,
            loop_search_radius: 1.0,
            loop_time_thresh: 60.0,
            loop_score_thresh: 0.15,
            loop_submap_half_range: 5,
            submap_resolution: 0.1,
            min_loop_detect_duration: 10.0,
            loop_candidate_max_distance_m: 30.0,
            min_descriptor_std: 0.0,
            loop_min_occupancy: 80,
            loop_min_degeneracy: 0.05,
            loop_max_lowe_ratio: 0.0,
            loop_robust_kernel: false,
            loop_robust_huber_k: 1.345,
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
            scan_context_max_range_m: 80.0,
            scan_context_top_k: 10,
            scan_context_match_threshold: 0.4,
            scan_context_lidar_height_m: 2.0,
        }
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
    location_closure: bool,
    timing: TimingStats,
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
        GscPgo {
            config,
            scan_context_config,
            key_poses: Vec::new(),
            history_pairs: Vec::new(),
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
            location_closure: false,
            timing: TimingStats::default(),
        }
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
        if self.config.min_loop_detect_duration > 0.0 {
            if let Some(&(_, last_source)) = self.history_pairs.last() {
                let current_time = self.key_poses.last().unwrap().time;
                let last_time = self.key_poses[last_source].time;
                if current_time - last_time < self.config.min_loop_detect_duration {
                    return;
                }
            }
        }

        let cur_idx = self.key_poses.len() - 1;

        // Feature-poverty gate: a scan with no spatially-spread structure
        // can't reliably place itself; skip loop search entirely.
        if self.config.use_scan_context
            && !self.scan_context_descriptors.is_empty()
            && !self.scan_context_descriptors.last().unwrap().is_empty()
        {
            let descriptor = self.scan_context_descriptors.last().unwrap();
            if self.config.min_descriptor_std > 0.0
                && scan_context::descriptor_structure(descriptor)
                    < self.config.min_descriptor_std as f32
            {
                return;
            }
            if self.config.loop_min_occupancy > 0
                && (scan_context::descriptor_occupancy(descriptor) as i32)
                    < self.config.loop_min_occupancy
            {
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
            return;
        }
        let loop_idx = loop_idx as usize;

        // Aliasing gate: reject scan-context matches whose best and second-best
        // distances are near-tied (Lowe ratio -> 1). Runs before the expensive
        // ICP/submap work.
        if from_scan_context
            && self.config.loop_max_lowe_ratio > 0.0
            && sc_second > 1e-6
            && (sc_best / sc_second) as f64 > self.config.loop_max_lowe_ratio
        {
            return;
        }

        // Positional-plausibility gate on scan-context false matches.
        if self.config.loop_candidate_max_distance_m > 0.0 {
            let candidate_distance = mat3::norm(&mat3::sub(
                &self.key_poses[cur_idx].translation_global,
                &self.key_poses[loop_idx].translation_global,
            ));
            if candidate_distance > self.config.loop_candidate_max_distance_m {
                return;
            }
        }

        // Seed ICP with the scan-context yaw, rotating about the source
        // keyframe's own global position (both submaps are in global frame):
        //     init = T(source_position) * Rz(yaw) * T(-source_position)
        // Built in f32, then widened to f64 for the icp_point_to_point API —
        // which narrows it back losslessly.
        let mut init_rotation = mat3::identity();
        let mut init_translation = [0.0f64; 3];
        if self.config.use_scan_context && sector_shift != 0 {
            let yaw =
                scan_context::yaw_from_shift(sector_shift, self.scan_context_config.n_sectors);
            let yaw_f = yaw as f32;
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
            // rotation * source_position: fixed-size Eigen Matrix3f *
            // Vector3f, unrolled binary-tree redux t0 + (t1 + t2).
            for i in 0..3 {
                let rotated_source = source_position[0] * rotation[i][0]
                    + (source_position[1] * rotation[i][1] + source_position[2] * rotation[i][2]);
                init_translation[i] = f64::from(source_position[i] - rotated_source);
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

        // Observability gate: a planar/degenerate source scan leaves the
        // alignment unconstrained in-plane — fitness lies.
        let mut degeneracy_min = -1.0f32;
        if self.config.loop_min_degeneracy > 0.0 {
            let t0 = std::time::Instant::now();
            (degeneracy_min, _) = cloud_degeneracy(&source_cloud);
            self.timing.degeneracy_s += t0.elapsed().as_secs_f64();
        }

        if self.config.loop_min_degeneracy > 0.0
            && degeneracy_min >= 0.0
            && (degeneracy_min as f64) < self.config.loop_min_degeneracy
        {
            return;
        }

        if !icp.converged || icp.fitness > self.config.loop_score_thresh {
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
        // Original isotropic noise = ICP fitness on all 6 DOF.
        let noise = NoiseModel::diagonal_variances(&[score; 6]);
        self.cache_pairs.push(LoopPair {
            source_id: cur_idx,
            target_id: loop_idx,
            rotation_offset,
            translation_offset,
            score,
            noise,
        });
        self.history_pairs.push((loop_idx, cur_idx));
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
        // 添加回环因子 (add the loop factors)
        for pair in &cache_pairs {
            let robust;
            let noise: &NoiseModel = if self.config.loop_robust_kernel {
                robust = NoiseModel::robust_huber(self.config.loop_robust_huber_k, &pair.noise);
                &robust
            } else {
                &pair.noise
            };
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
        // A re-sighted location closes a loop just like a lidar closure.
        let has_closure = has_loop || self.location_closure;

        // Smooth and map; removeFactorIndices applies constraint revision.
        let remove = std::mem::take(&mut self.pending_removals);
        let new_factor_indices = self
            .isam2
            .update(&self.graph, &self.initial_values, &remove)
            .expect("gtsam: isam2 update");
        self.isam2.update_empty().expect("gtsam: isam2 update");
        if has_closure {
            self.isam2.update_empty().expect("gtsam: isam2 update");
            self.isam2.update_empty().expect("gtsam: isam2 update");
            self.isam2.update_empty().expect("gtsam: isam2 update");
            self.isam2.update_empty().expect("gtsam: isam2 update");
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

    /// Direct access to the ICP used for loop verification — handy for
    /// standalone experiments; the loop path calls `icp_point_to_point`
    /// with these same defaults.
    pub fn icp_params() -> IcpParams {
        IcpParams::default()
    }
}

// Re-export the point-cloud helpers alongside the PGO for wiring layers.
pub use pointcloud::IcpResult;
