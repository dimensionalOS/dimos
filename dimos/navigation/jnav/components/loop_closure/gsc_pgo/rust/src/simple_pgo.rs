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

//! `SimplePgo` — line-faithful port of gsc_pgo's `SimplePGO`
//! (simple_pgo.{h,cpp} @ remove-tag-handling). Same keyframe gating, same
//! loop-candidate search + gates, same factor graph construction, same
//! iSAM2 update sequence, same outputs, given the same inputs.
//!
//! Key structural mapping:
//! - keyframe node keys are the plain contiguous indices (0, 1, 2, ...);
//! - location variables are `Symbol('l', i)` keys;
//! - `CloudType::Ptr` -> `Option<Arc<PointCloud>>` (a constraint-triggered
//!   node has no cloud);
//! - Eigen M3D/V3D -> `mat3::Mat3` / `mat3::Vec3` (row-major f64).

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

/// `PoseWithTime` from commons.h.
#[derive(Debug, Clone)]
pub struct PoseWithTime {
    pub t: Vec3,
    pub r: Mat3,
    pub sec: i32,
    pub nsec: u32,
    pub second: f64,
}

impl PoseWithTime {
    pub fn new(r: Mat3, t: Vec3) -> PoseWithTime {
        PoseWithTime {
            t,
            r,
            sec: 0,
            nsec: 0,
            second: 0.0,
        }
    }

    /// `PoseWithTime::setTime` from commons.cpp.
    pub fn set_time(&mut self, sec: i32, nsec: u32) {
        self.sec = sec;
        self.nsec = nsec;
        self.second = sec as f64 + nsec as f64 / 1e9;
    }
}

/// `LocationConstraintObs` from commons.h: a relative-pose measurement from
/// the body frame to a location variable, plus its 6x6 covariance
/// (row-major, GTSAM Pose3 tangent order [rot(3), trans(3)]).
#[derive(Debug, Clone)]
pub struct LocationConstraintObs {
    pub to_id: String,
    pub constraint_instance_id: String,
    pub r_body_loc: Mat3,
    pub t_body_loc: Vec3,
    pub covariance: [f64; 36],
    pub ts: f64,
}

/// `CloudWithPose` from commons.h.
#[derive(Debug, Clone)]
pub struct CloudWithPose {
    pub cloud: Option<Arc<PointCloud>>,
    pub pose: PoseWithTime,
    pub frame_id: String,
}

/// `KeyPoseWithCloud` from simple_pgo.h.
#[derive(Debug, Clone)]
pub struct KeyPoseWithCloud {
    pub r_local: Mat3,
    pub t_local: Vec3,
    pub r_global: Mat3,
    pub t_global: Vec3,
    pub time: f64,
    pub body_cloud: Option<Arc<PointCloud>>,
}

/// `LoopPair` from simple_pgo.h: a detected loop with the ICP-refined
/// relative pose and the per-constraint noise model built at detection.
pub struct LoopPair {
    pub source_id: usize,
    pub target_id: usize,
    pub r_offset: Mat3,
    pub t_offset: Vec3,
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

/// `Config` from simple_pgo.h — every field, same defaults.
#[derive(Debug, Clone)]
pub struct Config {
    pub key_pose_delta_deg: f64,
    pub key_pose_delta_trans: f64,
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
    /// Log one "PGO_DIAG ..." line per loop candidate that reaches ICP.
    pub debug: bool,
    /// Robust (M-estimator) kernel wrapping all loop factors.
    pub loop_robust_kernel: bool,
    pub loop_robust_huber_k: f64,
    /// Ingest LocationConstraint events (consumed by the module wiring, not
    /// by SimplePgo itself — kept for config parity).
    pub use_location_constraints: bool,
    // Odometry between-factor noise (anisotropic).
    pub odom_rot_rp_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,
    // First-keyframe absolute anchor prior (always added; fixes the pose graph
    // gauge). Per-axis variances double as stiffness knobs: a tight anchor_rp_var
    // hard-pins roll/pitch to the initial LIO attitude, a loose one frees it.
    pub anchor_rp_var: f64,
    pub anchor_yaw_var: f64,
    pub anchor_trans_var: f64,
    /// Optional roll/pitch prior on every keyframe (yaw + translation left free).
    pub per_keyframe_rp_prior: bool,
    pub per_keyframe_rp_var: f64,
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
            key_pose_delta_deg: 10.0,
            key_pose_delta_trans: 1.0,
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
            debug: false,
            loop_robust_kernel: false,
            loop_robust_huber_k: 1.345,
            use_location_constraints: false,
            odom_rot_rp_var: 1e-8,
            odom_rot_yaw_var: 1e-5,
            odom_trans_xy_var: 1e-4,
            odom_trans_z_var: 1e-6,
            anchor_rp_var: 1e-12,
            anchor_yaw_var: 1e-12,
            anchor_trans_var: 1e-12,
            per_keyframe_rp_prior: false,
            per_keyframe_rp_var: 1e-4,
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

pub struct SimplePgo {
    config: Config,
    scan_context_config: scan_context::Config,
    key_poses: Vec<KeyPoseWithCloud>,
    history_pairs: Vec<(usize, usize)>,
    cache_pairs: Vec<LoopPair>,
    scan_context_descriptors: Vec<scan_context::Descriptor>,
    scan_context_ring_keys: Vec<scan_context::RingKey>,
    r_offset: Mat3,
    t_offset: Vec3,
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

impl SimplePgo {
    pub fn new(config: Config) -> SimplePgo {
        let scan_context_config = scan_context::Config {
            n_rings: config.scan_context_num_rings.max(0) as usize,
            n_sectors: config.scan_context_num_sectors.max(0) as usize,
            max_range_m: config.scan_context_max_range_m,
            candidate_top_k: config.scan_context_top_k.max(0) as usize,
            match_threshold: config.scan_context_match_threshold,
            lidar_height_m: config.scan_context_lidar_height_m,
        };
        SimplePgo {
            config,
            scan_context_config,
            key_poses: Vec::new(),
            history_pairs: Vec::new(),
            cache_pairs: Vec::new(),
            scan_context_descriptors: Vec::new(),
            scan_context_ring_keys: Vec::new(),
            r_offset: mat3::identity(),
            t_offset: [0.0; 3],
            // The shim configures ISAM2 like the C++ constructor:
            // relinearizeThreshold = 0.01, relinearizeSkip = 1.
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

    /// `SimplePGO::isKeyPose`: keyframe gating by translation / rotation
    /// delta against the last keyframe's LOCAL pose.
    pub fn is_key_pose(&self, pose: &PoseWithTime) -> bool {
        if self.key_poses.is_empty() {
            return true;
        }
        let last_item = self.key_poses.last().unwrap();
        let delta_trans = mat3::norm(&mat3::sub(&pose.t, &last_item.t_local));
        // NOTE: 57.324 (not 57.2958) — kept verbatim from the C++.
        let delta_deg = mat3::angular_distance(&pose.r, &last_item.r_local) * 57.324;
        delta_trans > self.config.key_pose_delta_trans || delta_deg > self.config.key_pose_delta_deg
    }

    /// `SimplePGO::addKeyPose`.
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

    /// `SimplePGO::addLocationConstraint`: the constraint becomes its own
    /// pose node (at the interpolated-odometry pose supplied by the caller)
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

    pub fn offset_r(&self) -> Mat3 {
        self.r_offset
    }

    pub fn offset_t(&self) -> Vec3 {
        self.t_offset
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

    /// `SimplePGO::insertPoseNode`: new node (key = next contiguous index)
    /// with initial value + backbone factor (gravity prior on the first,
    /// else an odom between-factor), the optional per-keyframe gravity
    /// anchor, the scan-context cache, and the frame record.
    fn insert_pose_node(
        &mut self,
        pose: &PoseWithTime,
        cloud: Option<Arc<PointCloud>>,
        frame_id: &str,
    ) -> usize {
        let idx = self.key_poses.len();
        let init_r = mat3::mat_mul(&self.r_offset, &pose.r);
        let init_t = mat3::add(&mat3::mat_vec(&self.r_offset, &pose.t), &self.t_offset);
        let init_pose = Pose3 {
            r: init_r,
            t: init_t,
        };
        self.initial_values
            .insert_pose3(idx as u64, &init_pose)
            .expect("gtsam: insert initial value");
        if idx == 0 {
            // Absolute anchor prior on the first keyframe (always present: a
            // relative-only pose graph is singular without one absolute anchor).
            // Pose3 tangent order is [rot(3), trans(3)]: components 0-1 are
            // roll/pitch, 2 is yaw. Each stiffness is a config knob — the smaller
            // the variance, the harder that axis is pinned to the initial (LIO)
            // pose. A tight anchor_rp_var pins roll/pitch to the LIO attitude; a
            // loose one lets odom/loops decide it.
            let prior_var = [
                self.config.anchor_rp_var,
                self.config.anchor_rp_var,
                self.config.anchor_yaw_var,
                self.config.anchor_trans_var,
                self.config.anchor_trans_var,
                self.config.anchor_trans_var,
            ];
            let noise = NoiseModel::diagonal_variances(&prior_var);
            self.graph
                .add_prior_pose3(idx as u64, &init_pose, &noise)
                .expect("gtsam: add anchor prior");
        } else {
            // Odometry constraint to the previous keyframe. Anisotropic:
            // stiff relative roll/pitch (gravity-accurate), looser yaw.
            let last_item = self.key_poses.last().unwrap();
            let last_r_t = mat3::transpose(&last_item.r_local);
            let r_between = mat3::mat_mul(&last_r_t, &pose.r);
            let t_between = mat3::mat_vec(&last_r_t, &mat3::sub(&pose.t, &last_item.t_local));
            let noise = NoiseModel::diagonal_variances(&[
                self.config.odom_rot_rp_var,
                self.config.odom_rot_rp_var,
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
                        r: r_between,
                        t: t_between,
                    },
                    &noise,
                )
                .expect("gtsam: add odom between factor");

            // Optional per-keyframe roll/pitch prior: pin this keyframe's
            // roll/pitch to its initial (LIO) attitude, leaving yaw + translation
            // free (huge variance).
            if self.config.per_keyframe_rp_prior {
                let grav_var = [
                    self.config.per_keyframe_rp_var,
                    self.config.per_keyframe_rp_var,
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
            r_local: pose.r,
            t_local: pose.t,
            body_cloud: cloud.clone(),
            r_global: init_r,
            t_global: init_t,
        });

        // Record this node's frame (from the odometry) alongside its pose.
        let q = mat3::quat_from_mat(&pose.r);
        self.node_poses.insert(
            idx as u64,
            PoseStamped {
                frame_id: frame_id.to_string(),
                sec: pose.second as i32,
                nsec: ((pose.second - (pose.second as i32) as f64) * 1e9) as u32,
                position: pose.t,
                orientation: [q[1], q[2], q[3], q[0]],
            },
        );

        // Cache the Scan Context descriptor + ring-key (empty when the node
        // has no cloud, e.g. a constraint-triggered node).
        if let Some(cloud) = cloud {
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

        idx
    }

    /// `SimplePGO::getSubMap`: aggregate the body clouds of keyframes
    /// [idx-half_range, idx+half_range] transformed by their GLOBAL poses,
    /// then voxel-downsample at `resolution` (when > 0).
    pub fn get_sub_map(&self, idx: i32, half_range: i32, resolution: f64) -> PointCloud {
        assert!(idx >= 0 && (idx as usize) < self.key_poses.len());
        let min_idx = 0.max(idx - half_range) as usize;
        let max_idx = (self.key_poses.len() as i32 - 1).min(idx + half_range) as usize;

        let mut ret: PointCloud = Vec::new();
        for key_pose in &self.key_poses[min_idx..=max_idx] {
            if let Some(body_cloud) = &key_pose.body_cloud {
                let global_cloud =
                    transform_cloud(body_cloud, &key_pose.r_global, &key_pose.t_global);
                ret.extend_from_slice(&global_cloud);
            }
        }
        if resolution > 0.0 {
            ret = voxel_downsample(&ret, resolution);
        }
        ret
    }

    /// `SimplePGO::searchByPosition`: radius search on past key-pose global
    /// positions (candidates ascending by distance, like PCL's radiusSearch),
    /// returning the first far-enough-in-time hit.
    fn search_by_position(&self) -> i64 {
        let cur_idx = self.key_poses.len() - 1;
        let last_item = self.key_poses.last().unwrap();
        let last = [
            last_item.t_global[0] as f32,
            last_item.t_global[1] as f32,
            last_item.t_global[2] as f32,
        ];
        let radius_sq = (self.config.loop_search_radius * self.config.loop_search_radius) as f32;
        let mut neighbors: Vec<(f32, usize)> = Vec::new();
        for (i, key_pose) in self.key_poses[..cur_idx].iter().enumerate() {
            let dx = key_pose.t_global[0] as f32 - last[0];
            let dy = key_pose.t_global[1] as f32 - last[1];
            let dz = key_pose.t_global[2] as f32 - last[2];
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

    /// `SimplePGO::searchByScanContext`. Returns (loop_idx or -1,
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

    /// `SimplePGO::searchForLoopPairs`: candidate search (scan context with
    /// position fallback) + all the gates, ICP verification, and LoopPair
    /// construction.
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
        if loop_idx < 0 {
            // Fallback (or sole path if SC disabled): search past positions.
            loop_idx = self.search_by_position();
        }
        if loop_idx < 0 {
            return;
        }
        let loop_idx = loop_idx as usize;

        // Positional-plausibility gate on scan-context false matches.
        if self.config.loop_candidate_max_distance_m > 0.0 {
            let candidate_distance = mat3::norm(&mat3::sub(
                &self.key_poses[cur_idx].t_global,
                &self.key_poses[loop_idx].t_global,
            ));
            if candidate_distance > self.config.loop_candidate_max_distance_m {
                return;
            }
        }

        // Seed ICP with the scan-context yaw, rotating about the source
        // keyframe's own global position (both submaps are in global frame):
        //     init = T(source_position) * Rz(yaw) * T(-source_position)
        // Built in f32 like the C++ (`Eigen::Matrix4f init_guess` from
        // AngleAxisf / Matrix3f / Vector3f), then widened to f64 for the
        // icp_point_to_point API — which narrows it back losslessly.
        let mut init_r = mat3::identity();
        let mut init_t = [0.0f64; 3];
        if self.config.use_scan_context && sector_shift != 0 {
            let yaw =
                scan_context::yaw_from_shift(sector_shift, self.scan_context_config.n_sectors);
            let yaw_f = yaw as f32;
            // Eigen AngleAxisf(yaw_f, UnitZ()).toRotationMatrix(): note the
            // (2,2) entry is computed as (1 - cos) + cos, which is not
            // always exactly 1.0f.
            let (s, c) = (yaw_f.sin(), yaw_f.cos());
            let r22 = (1.0f32 - c) + c;
            let rotation = [[c, -s, 0.0f32], [s, c, 0.0f32], [0.0f32, 0.0f32, r22]];
            let sp = [
                self.key_poses[cur_idx].t_global[0] as f32,
                self.key_poses[cur_idx].t_global[1] as f32,
                self.key_poses[cur_idx].t_global[2] as f32,
            ];
            // rotation * source_position: fixed-size Eigen Matrix3f *
            // Vector3f, unrolled binary-tree redux t0 + (t1 + t2).
            for i in 0..3 {
                let rsp =
                    sp[0] * rotation[i][0] + (sp[1] * rotation[i][1] + sp[2] * rotation[i][2]);
                init_t[i] = f64::from(sp[i] - rsp);
                init_r[i] = [
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
            &init_r,
            &init_t,
            &IcpParams::default(),
        );
        self.timing.icp_s += t0.elapsed().as_secs_f64();

        // Observability gate: a planar/degenerate source scan leaves the
        // alignment unconstrained in-plane — fitness lies.
        let mut degeneracy_min = -1.0f32;
        let mut degeneracy_mid = -1.0f32;
        if self.config.loop_min_degeneracy > 0.0 || self.config.debug {
            let t0 = std::time::Instant::now();
            (degeneracy_min, degeneracy_mid) = cloud_degeneracy(&source_cloud);
            self.timing.degeneracy_s += t0.elapsed().as_secs_f64();
        }

        if self.config.debug {
            let cand_dist = mat3::norm(&mat3::sub(
                &self.key_poses[cur_idx].t_global,
                &self.key_poses[loop_idx].t_global,
            ));
            let have_desc =
                self.config.use_scan_context && !self.scan_context_descriptors.is_empty();
            let structure = if have_desc {
                scan_context::descriptor_structure(self.scan_context_descriptors.last().unwrap())
            } else {
                -1.0
            };
            let occupancy = if have_desc {
                scan_context::descriptor_occupancy(self.scan_context_descriptors.last().unwrap())
                    as i64
            } else {
                -1
            };
            // Lowe ratio: best / second-best Scan-Context distance.
            let lowe_ratio = if sc_second > 1e-6 {
                sc_best / sc_second
            } else {
                -1.0
            };
            let accepted = icp.converged && icp.fitness <= self.config.loop_score_thresh;
            eprintln!(
                "PGO_DIAG kf={} cand={} dist={:.2} fitness={:e} converged={} structure={:.2} \
                 occ={} sc_best={:.3} sc_2nd={:.3} lowe={:.3} degen_min={:.4} degen_mid={:.4} \
                 src_pts={} tgt_pts={} accepted={}",
                cur_idx,
                loop_idx,
                cand_dist,
                icp.fitness,
                if icp.converged { 1 } else { 0 },
                structure,
                occupancy,
                sc_best,
                sc_second,
                lowe_ratio,
                degeneracy_min,
                degeneracy_mid,
                source_cloud.len(),
                target_cloud.len(),
                if accepted { 1 } else { 0 }
            );
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
        let r_refined = mat3::mat_mul(&icp.r, &self.key_poses[cur_idx].r_global);
        let t_refined = mat3::add(
            &mat3::mat_vec(&icp.r, &self.key_poses[cur_idx].t_global),
            &icp.t,
        );
        let loop_r_t = mat3::transpose(&self.key_poses[loop_idx].r_global);
        let r_offset = mat3::mat_mul(&loop_r_t, &r_refined);
        let t_offset = mat3::mat_vec(
            &loop_r_t,
            &mat3::sub(&t_refined, &self.key_poses[loop_idx].t_global),
        );
        // Original isotropic noise = ICP fitness on all 6 DOF.
        let noise = NoiseModel::diagonal_variances(&[score; 6]);
        self.cache_pairs.push(LoopPair {
            source_id: cur_idx,
            target_id: loop_idx,
            r_offset,
            t_offset,
            score,
            noise,
        });
        self.history_pairs.push((loop_idx, cur_idx));
    }

    /// `SimplePGO::addLocationConstraintFactors`: ensure a graph variable for
    /// `to_id` (initialized from this node's global pose when new), add a
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
            let r_loc_world = mat3::mat_mul(&node.r_global, &constraint.r_body_loc);
            let t_loc_world = mat3::add(
                &mat3::mat_vec(&node.r_global, &constraint.t_body_loc),
                &node.t_global,
            );
            self.initial_values
                .insert_pose3(
                    loc_key,
                    &Pose3 {
                        r: r_loc_world,
                        t: t_loc_world,
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
                    r: constraint.r_body_loc,
                    t: constraint.t_body_loc,
                },
                noise,
            )
            .expect("gtsam: add location constraint factor");
        self.staged_constraint_factors.push(StagedConstraintFactor {
            instance_id: constraint.constraint_instance_id.clone(),
            graph_pos,
        });

        if self.config.debug {
            eprintln!(
                "PGO_LOCATION node={} to_id={} new={} |t_body_loc|={:.2} instance={}",
                node_idx,
                constraint.to_id,
                if is_new { 1 } else { 0 },
                mat3::norm(&constraint.t_body_loc),
                constraint.constraint_instance_id
            );
        }
    }

    /// `SimplePGO::smoothAndUpdate`: stage cached loop pairs as between
    /// factors, run the iSAM2 update sequence (with revision removals and
    /// the extra relinearization passes a closure needs), commit staged
    /// constraint-factor indices, then refresh all keyframe globals and the
    /// r/t offsets from the best estimate.
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
                        r: pair.r_offset,
                        t: pair.t_offset,
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
            key_pose.r_global = pose.r;
            key_pose.t_global = pose.t;
        }
        // Update offset.
        let last_item = self.key_poses.last().unwrap();
        self.r_offset = mat3::mat_mul(&last_item.r_global, &mat3::transpose(&last_item.r_local));
        self.t_offset = mat3::sub(
            &last_item.t_global,
            &mat3::mat_vec(&self.r_offset, &last_item.t_local),
        );

        self.timing.gtsam_s += smooth_t0.elapsed().as_secs_f64();
        self.timing.smooth_calls += 1;
        if self.config.debug && self.timing.smooth_calls.is_multiple_of(100) {
            let t = &self.timing;
            eprintln!(
                "PGO_TIMING kf={} sc={:.1}s submap={:.1}s icp={:.1}s degen={:.1}s gtsam={:.1}s",
                self.key_poses.len(),
                t.scan_context_s,
                t.submap_s,
                t.icp_s,
                t.degeneracy_s,
                t.gtsam_s,
            );
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
