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

//! Native Rust PGO module — a 1:1 port of gsc_pgo's main.cpp on top of the
//! `dimos_module` framework (same wire protocol as the C++
//! dimos_native_module.hpp, but config arrives as stdin JSON instead of CLI
//! args).
//!
//! Structure mirrors the C++ exactly:
//! - LCM handlers stash odometry (latest + a sliding interpolation buffer),
//!   pair each lidar scan with the LATEST odometry pose (CloudWithPose), and
//!   buffer LocationConstraints.
//! - A 20 Hz worker loop drains constraints (each becomes its own pose node
//!   placed via odometry interpolated at the constraint's own timestamp),
//!   feeds one scan per tick into SimplePgo, and publishes:
//!   corrected_odometry (offset-corrected pose), correction (map->odom
//!   TFMessage), pose_graph (Graph3D snapshot per keyframe),
//!   loop_closure_event (GraphDelta3D of pre->post iSAM2 deltas when a loop
//!   fired), tf_deformation_nodes (per-keyframe DeformationNode, re-published
//!   only when the optimizer moves a node past an epsilon), and the optional
//!   throttled _global_map debug cloud.
//!
//! SimplePgo wraps gtsam, which is thread-unsafe (`!Send`), so the worker is
//! a dedicated OS thread that OWNS the SimplePgo; publishes hop back onto the
//! tokio runtime via a captured Handle.

use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::msgs::{
    DeformationNode, DeltaTransform, Edge, Graph3D, GraphDelta3D, LocationConstraint, Node3D,
    PoseStamped as WirePose,
};
use dimos_gsc_pgo::pointcloud::{self, PointCloud};
use dimos_gsc_pgo::simple_pgo::{
    CloudWithPose, Config as PgoConfig, KeyPoseWithCloud, LocationConstraintObs, PoseWithTime,
    SimplePgo,
};
use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::geometry_msgs::{Point, Quaternion, Transform, TransformStamped, Vector3};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use lcm_msgs::tf2_msgs::TFMessage;
use tracing::{error, info, warn};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Output/map frame (the C++ module's `frame_id`; renamed because the
    /// Python coordinator strips base-config field names — `frame_id` is one —
    /// from the stdin config dict).
    pub world_frame: String,
    /// The corrected edge's child (odom) frame.
    pub child_frame_id: String,
    /// Robot body frame; LocationConstraints must be expressed in it.
    pub body_frame: String,

    // Keyframe detection
    pub key_pose_delta_deg: f64,
    pub key_pose_delta_trans: f64,

    // Loop closure
    pub loop_search_radius: f64,
    pub loop_time_thresh: f64,
    pub loop_score_thresh: f64,
    pub loop_submap_half_range: i32,
    pub submap_resolution: f64,
    pub min_loop_detect_duration: f64,
    pub min_descriptor_std: f64,
    pub loop_min_occupancy: i32,
    pub loop_min_degeneracy: f64,

    /// Transform world-frame scans to body-frame using the paired odometry.
    pub unregister_input: bool,

    // Debug global-map publishing (rate <= 0 disables; see module.py).
    pub global_map_voxel_size: f64,
    pub global_map_publish_rate: f64,

    // Scan Context place recognition
    pub use_scan_context: bool,
    pub scan_context_num_rings: i32,
    pub scan_context_num_sectors: i32,
    pub scan_context_max_range_m: f64,
    pub scan_context_top_k: i32,
    pub scan_context_match_threshold: f64,
    pub scan_context_lidar_height_m: f64,

    pub loop_candidate_max_distance_m: f64,

    // Robust (Huber) kernel on loop factors
    pub loop_robust_kernel: bool,
    pub loop_robust_huber_k: f64,

    // Location constraints
    pub use_location_constraints: bool,
    pub odom_buffer_window: f64,

    // First-keyframe absolute anchor prior (per-axis stiffness) + optional
    // per-keyframe roll/pitch prior.
    pub anchor_rp_var: f64,
    pub anchor_yaw_var: f64,
    pub anchor_trans_var: f64,
    pub per_keyframe_rp_prior: bool,
    pub per_keyframe_rp_var: f64,

    // Anisotropic odometry between-factor
    pub odom_rot_rp_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,

    /// Bounded scan FIFO depth (<= 0 = unbounded).
    pub max_scan_queue: i32,

    pub debug: bool,
}

impl Config {
    fn pgo(&self) -> PgoConfig {
        PgoConfig {
            key_pose_delta_deg: self.key_pose_delta_deg,
            key_pose_delta_trans: self.key_pose_delta_trans,
            loop_search_radius: self.loop_search_radius,
            loop_time_thresh: self.loop_time_thresh,
            loop_score_thresh: self.loop_score_thresh,
            loop_submap_half_range: self.loop_submap_half_range,
            submap_resolution: self.submap_resolution,
            min_loop_detect_duration: self.min_loop_detect_duration,
            loop_candidate_max_distance_m: self.loop_candidate_max_distance_m,
            min_descriptor_std: self.min_descriptor_std,
            loop_min_occupancy: self.loop_min_occupancy,
            loop_min_degeneracy: self.loop_min_degeneracy,
            debug: self.debug,
            loop_robust_kernel: self.loop_robust_kernel,
            loop_robust_huber_k: self.loop_robust_huber_k,
            use_location_constraints: self.use_location_constraints,
            odom_rot_rp_var: self.odom_rot_rp_var,
            odom_rot_yaw_var: self.odom_rot_yaw_var,
            odom_trans_xy_var: self.odom_trans_xy_var,
            odom_trans_z_var: self.odom_trans_z_var,
            anchor_rp_var: self.anchor_rp_var,
            anchor_yaw_var: self.anchor_yaw_var,
            anchor_trans_var: self.anchor_trans_var,
            per_keyframe_rp_prior: self.per_keyframe_rp_prior,
            per_keyframe_rp_var: self.per_keyframe_rp_var,
            use_scan_context: self.use_scan_context,
            scan_context_num_rings: self.scan_context_num_rings,
            scan_context_num_sectors: self.scan_context_num_sectors,
            scan_context_max_range_m: self.scan_context_max_range_m,
            scan_context_top_k: self.scan_context_top_k,
            scan_context_match_threshold: self.scan_context_match_threshold,
            scan_context_lidar_height_m: self.scan_context_lidar_height_m,
        }
    }
}

// ---- shared state (the C++ file's globals) --------------------------------------

#[derive(Clone)]
struct OdomSample {
    ts: f64,
    r: Mat3,
    t: Vec3,
    frame_id: String,
}

#[derive(Default)]
struct SharedState {
    latest_odom: Option<OdomSample>,
    /// Reject out-of-order scans (mirrors g_last_message_time).
    last_message_time: f64,
    /// Sliding odometry history for interpolating a constraint's pose at its
    /// own timestamp (mirrors g_odom_buffer).
    odom_buffer: VecDeque<OdomSample>,
    /// Bounded scan FIFO (mirrors g_cloud_buffer + g_max_scan_queue).
    cloud_buffer: VecDeque<CloudWithPose>,
    /// Pending LocationConstraints, drained by the worker loop.
    constraints: Vec<(LocationConstraintObs, String)>,
    /// LocationConstraint frames already warned about (once each).
    frame_warned: HashSet<String>,
}

type Shared<T> = Arc<Mutex<T>>;

/// Interpolate the odometry pose at `ts` (slerp rotation, lerp translation).
/// Port of interpolate_odom: `None` if the buffer is empty or `ts` predates it
/// by more than 1 ms; clamps to the newest sample when `ts` is past it.
fn interpolate_odom(buffer: &VecDeque<OdomSample>, ts: f64) -> Option<(Mat3, Vec3, String)> {
    let front = buffer.front()?;
    if ts <= front.ts {
        if front.ts - ts > 1e-3 {
            return None;
        }
        return Some((front.r, front.t, front.frame_id.clone()));
    }
    let back = buffer.back()?;
    if ts >= back.ts {
        return Some((back.r, back.t, back.frame_id.clone()));
    }
    for i in 1..buffer.len() {
        let hi = &buffer[i];
        if hi.ts < ts {
            continue;
        }
        let lo = &buffer[i - 1];
        let span = hi.ts - lo.ts;
        let alpha = if span > 0.0 { (ts - lo.ts) / span } else { 0.0 };
        let t = [
            lo.t[0] + alpha * (hi.t[0] - lo.t[0]),
            lo.t[1] + alpha * (hi.t[1] - lo.t[1]),
            lo.t[2] + alpha * (hi.t[2] - lo.t[2]),
        ];
        let q = slerp(
            &mat3::quat_from_mat(&lo.r),
            &mat3::quat_from_mat(&hi.r),
            alpha,
        );
        return Some((mat3::mat_from_quat(&q), t, lo.frame_id.clone()));
    }
    None
}

/// Quaternion slerp with Eigen's shortest-path semantics (quats are [w,x,y,z]).
fn slerp(a: &[f64; 4], b: &[f64; 4], alpha: f64) -> [f64; 4] {
    let mut dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    dot *= sign;
    let (wa, wb) = if dot > 0.9995 {
        (1.0 - alpha, alpha)
    } else {
        let theta = dot.clamp(-1.0, 1.0).acos();
        let sin_theta = theta.sin();
        (
            ((1.0 - alpha) * theta).sin() / sin_theta,
            (alpha * theta).sin() / sin_theta,
        )
    };
    let mut out = [0.0; 4];
    for i in 0..4 {
        out[i] = wa * a[i] + sign * wb * b[i];
    }
    let norm = out.iter().map(|v| v * v).sum::<f64>().sqrt();
    if norm > 0.0 {
        for v in out.iter_mut() {
            *v /= norm;
        }
    }
    out
}

// ---- module ----------------------------------------------------------------------

#[derive(Module)]
#[module(setup = spawn_worker, teardown = stop_worker)]
struct GscPgo {
    // Named "lidar" to match the jnav LoopClosure spec: the scan is paired
    // with the LATEST odometry pose internally, so a raw sensor-frame lidar +
    // odometry is what's expected.
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    // Optional decoupled LocationConstraint events; only consumed when
    // config.use_location_constraints is set (the C++ skips the subscription
    // entirely; here the handler is a no-op instead).
    #[input(decode = LocationConstraint::decode, handler = on_location_constraint)]
    location_constraints: Input<LocationConstraint>,

    #[output(encode = Odometry::encode)]
    corrected_odometry: Output<Odometry>,

    #[output(encode = TFMessage::encode)]
    correction: Output<TFMessage>,

    #[output(encode = Graph3D::encode)]
    pose_graph: Output<Graph3D>,

    #[output(encode = GraphDelta3D::encode)]
    loop_closure_event: Output<GraphDelta3D>,

    #[output(encode = DeformationNode::encode)]
    tf_deformation_nodes: Output<DeformationNode>,

    // Debug-only; leading underscore keeps autoconnect from wiring it.
    // Publishing is gated on global_map_publish_rate > 0.
    #[output(encode = PointCloud2::encode)]
    _global_map: Output<PointCloud2>,

    #[config]
    config: Config,

    state: Shared<SharedState>,
    worker: Option<WorkerHandle>,
}

struct WorkerHandle {
    stop: Arc<AtomicBool>,
    thread: std::thread::JoinHandle<()>,
}

impl GscPgo {
    async fn spawn_worker(&mut self) {
        let stop = Arc::new(AtomicBool::new(false));
        let worker = Worker {
            state: Arc::clone(&self.state),
            config: self.config.clone(),
            corrected_odometry: self.corrected_odometry.clone(),
            correction: self.correction.clone(),
            pose_graph: self.pose_graph.clone(),
            loop_closure_event: self.loop_closure_event.clone(),
            tf_deformation_nodes: self.tf_deformation_nodes.clone(),
            global_map: self._global_map.clone(),
            rt: tokio::runtime::Handle::current(),
            stop: Arc::clone(&stop),
        };
        // SimplePgo (gtsam) is !Send: a dedicated OS thread owns it for the
        // module's whole life instead of a tokio task.
        let thread = std::thread::Builder::new()
            .name("gsc-pgo-worker".into())
            .spawn(move || worker.run())
            .expect("spawn PGO worker thread");
        self.worker = Some(WorkerHandle { stop, thread });
        info!("PGO native module started (Rust iSAM2 port)");
    }

    async fn stop_worker(&mut self) {
        if let Some(WorkerHandle { stop, thread }) = self.worker.take() {
            stop.store(true, Ordering::Relaxed);
            // The worker sleeps <= 50 ms per tick; a blocking join here at
            // teardown is bounded and simpler than an async dance.
            let _ = tokio::task::spawn_blocking(move || thread.join()).await;
        }
    }

    /// Port of Handlers::on_odometry.
    async fn on_odometry(&mut self, msg: Odometry) {
        let q = &msg.pose.pose.orientation;
        let r = mat3::mat_from_quat(&[q.w, q.x, q.y, q.z]);
        let p = &msg.pose.pose.position;
        let t = [p.x, p.y, p.z];
        let ts = msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 / 1e9;
        let sample = OdomSample {
            ts,
            r,
            t,
            frame_id: msg.header.frame_id.clone(),
        };
        let mut state = self.state.lock().expect("state");
        state.latest_odom = Some(sample.clone());
        state.odom_buffer.push_back(sample);
        let window = self.config.odom_buffer_window;
        while state
            .odom_buffer
            .front()
            .is_some_and(|front| ts - front.ts > window)
        {
            state.odom_buffer.pop_front();
        }
    }

    /// Port of Handlers::on_registered_scan.
    async fn on_lidar(&mut self, msg: PointCloud2) {
        let mut state = self.state.lock().expect("state");
        let Some(latest) = state.latest_odom.clone() else {
            return;
        };
        // Reject out-of-order messages (by the paired odometry's timestamp).
        if latest.ts < state.last_message_time {
            return;
        }
        state.last_message_time = latest.ts;

        let cloud = match extract_xyz(&msg) {
            Ok(points) => points,
            Err(e) => {
                error!(error = %e, "lidar PointCloud2 parse failed; dropped");
                return;
            }
        };
        let mut pose = PoseWithTime::new(latest.r, latest.t);
        pose.set_time(
            latest.ts as i32,
            ((latest.ts - (latest.ts as i32) as f64) * 1e9) as u32,
        );
        state.cloud_buffer.push_back(CloudWithPose {
            cloud: Some(Arc::new(cloud)),
            pose,
            frame_id: latest.frame_id,
        });
        let cap = self.config.max_scan_queue;
        while cap > 0 && state.cloud_buffer.len() as i32 > cap {
            state.cloud_buffer.pop_front(); // drop oldest stale scan
        }
    }

    /// Port of Handlers::on_location_constraint.
    async fn on_location_constraint(&mut self, msg: LocationConstraint) {
        if !self.config.use_location_constraints {
            return;
        }
        let frame_id = if msg.frame_id.is_empty() {
            self.config.body_frame.clone()
        } else {
            msg.frame_id.clone()
        };
        let mut state = self.state.lock().expect("state");
        if frame_id != self.config.body_frame {
            if state.frame_warned.insert(frame_id.clone()) {
                warn!(
                    frame = %frame_id,
                    body_frame = %self.config.body_frame,
                    "LocationConstraint frame != body frame; dropping \
                     (only body-frame constraints supported for now)"
                );
            }
            return;
        }
        let [qx, qy, qz, qw] = msg.orientation;
        let obs = LocationConstraintObs {
            to_id: msg.to_id.clone(),
            constraint_instance_id: msg.constraint_instance_id.clone(),
            r_body_loc: mat3::mat_from_quat(&[qw, qx, qy, qz]),
            t_body_loc: msg.position,
            covariance: msg.covariance,
            ts: msg.ts,
        };
        state.constraints.push((obs, frame_id));
    }
}

// ---- worker (the C++ main loop) -----------------------------------------------------

struct Worker {
    state: Shared<SharedState>,
    config: Config,
    corrected_odometry: Output<Odometry>,
    correction: Output<TFMessage>,
    pose_graph: Output<Graph3D>,
    loop_closure_event: Output<GraphDelta3D>,
    tf_deformation_nodes: Output<DeformationNode>,
    global_map: Output<PointCloud2>,
    rt: tokio::runtime::Handle,
    stop: Arc<AtomicBool>,
}

impl Worker {
    fn publish<T>(&self, out: &Output<T>, msg: &T, what: &str) {
        if let Err(e) = self.rt.block_on(out.publish(msg)) {
            error!(error = %e, "{what} failed to publish");
        }
    }

    fn run(self) {
        let mut pgo = SimplePgo::new(self.config.pgo());
        let frame_id = self.config.world_frame.clone();
        let child_frame_id = self.config.child_frame_id.clone();
        let body_frame = self.config.body_frame.clone();
        let debug = self.config.debug;

        let publish_global_map = self.config.global_map_publish_rate > 0.0;
        let global_map_interval = if publish_global_map {
            1.0 / self.config.global_map_publish_rate
        } else {
            0.0
        };
        let mut last_global_map_time = 0.0f64;
        let timer_period = Duration::from_millis(50); // 20 Hz, matching original

        // Per-node deformation stream state: a stable random id per keyframe
        // plus its last published pose (re-publish only on new/moved nodes).
        let deformation_tf_id = dimos_gsc_pgo::msgs::tf_id_for(&frame_id, &child_frame_id);
        let mut deformation_rng = SplitMix64::from_entropy();
        let mut deformation_ids: Vec<u64> = Vec::new();
        let mut deformation_last: Vec<(Mat3, Vec3)> = Vec::new();

        while !self.stop.load(Ordering::Relaxed) {
            // Drain pending LocationConstraints first, independent of scans,
            // so a constraint is handled promptly even with no scans arriving.
            if self.config.use_location_constraints {
                let pending: Vec<(LocationConstraintObs, String)> = {
                    let mut state = self.state.lock().expect("state");
                    std::mem::take(&mut state.constraints)
                };
                for (constraint, _frame) in pending {
                    let interp = {
                        let state = self.state.lock().expect("state");
                        interpolate_odom(&state.odom_buffer, constraint.ts)
                    };
                    let Some((r, t, interp_frame)) = interp else {
                        warn!(
                            to_id = %constraint.to_id,
                            ts = constraint.ts,
                            "no odometry within the buffer window for constraint; dropping"
                        );
                        continue;
                    };
                    let mut node_pose = PoseWithTime::new(r, t);
                    node_pose.set_time(
                        constraint.ts as i32,
                        ((constraint.ts - (constraint.ts as i32) as f64) * 1e9) as u32,
                    );
                    if pgo.add_location_constraint(&node_pose, &interp_frame, &constraint) {
                        pgo.smooth_and_update();
                    } else {
                        warn!(
                            to_id = %constraint.to_id,
                            "LocationConstraint arrived before any keyframe; dropping \
                             (no node to anchor from)"
                        );
                    }
                }
            }

            // Strict FIFO: one scan per tick (backlog bounded at enqueue time).
            let cloud_with_pose = {
                let mut state = self.state.lock().expect("state");
                state.cloud_buffer.pop_front()
            };
            let Some(mut cloud_with_pose) = cloud_with_pose else {
                std::thread::sleep(timer_period);
                continue;
            };

            // Optionally transform world-frame scan to body-frame:
            // body = R_odom^T * (world_pts - t_odom).
            if self.config.unregister_input {
                if let Some(cloud) = cloud_with_pose.cloud.take() {
                    if !cloud.is_empty() {
                        let r_inv = mat3::transpose(&cloud_with_pose.pose.r);
                        let neg_t = [
                            -cloud_with_pose.pose.t[0],
                            -cloud_with_pose.pose.t[1],
                            -cloud_with_pose.pose.t[2],
                        ];
                        let t_body = mat3::mat_vec(&r_inv, &neg_t);
                        cloud_with_pose.cloud = Some(Arc::new(pointcloud::transform_cloud(
                            &cloud, &r_inv, &t_body,
                        )));
                    } else {
                        cloud_with_pose.cloud = Some(cloud);
                    }
                }
            }

            let cur_time = cloud_with_pose.pose.second;

            if !pgo.add_key_pose(&cloud_with_pose) {
                // Not a keyframe — still broadcast the corrected odom + TF.
                self.publish_corrected(
                    &pgo,
                    &cloud_with_pose.pose,
                    cur_time,
                    &frame_id,
                    &body_frame,
                    &child_frame_id,
                );
                std::thread::sleep(timer_period);
                continue;
            }

            // Keyframe added. Snapshot global poses BEFORE search + smooth so
            // we can publish the delta iSAM2 applies if a loop actually fires.
            pgo.search_for_loop_pairs();
            let had_loop = pgo.has_loop();
            let pre_poses: Vec<(Mat3, Vec3)> = if had_loop {
                pgo.key_poses()
                    .iter()
                    .map(|kp| (kp.r_global, kp.t_global))
                    .collect()
            } else {
                Vec::new()
            };

            pgo.smooth_and_update();

            if had_loop {
                let msg =
                    build_loop_closure_event(&pre_poses, pgo.key_poses(), cur_time, &frame_id);
                self.publish(&self.loop_closure_event, &msg, "loop_closure_event");
                if debug {
                    info!(
                        deltas = pre_poses.len(),
                        "PGO: loop_closure_event published"
                    );
                }
            }

            if debug {
                info!(
                    keyframes = pgo.key_poses().len(),
                    x = cloud_with_pose.pose.t[0],
                    y = cloud_with_pose.pose.t[1],
                    z = cloud_with_pose.pose.t[2],
                    "PGO: keyframe added"
                );
            }

            self.publish_corrected(
                &pgo,
                &cloud_with_pose.pose,
                cur_time,
                &frame_id,
                &body_frame,
                &child_frame_id,
            );

            // Pose graph on every keyframe (iSAM2 may have re-optimized prior
            // poses on loop closure).
            let graph = build_pose_graph(pgo.key_poses(), pgo.history_pairs(), cur_time, &frame_id);
            self.publish(&self.pose_graph, &graph, "pose_graph");

            // Same keyframes, streamed individually (new + moved nodes only).
            self.publish_deformation_nodes(
                pgo.key_poses(),
                deformation_tf_id,
                &frame_id,
                &mut deformation_ids,
                &mut deformation_last,
                &mut deformation_rng,
            );

            // Throttled debug global map.
            if publish_global_map
                && cur_time - last_global_map_time >= global_map_interval
                && !pgo.key_poses().is_empty()
            {
                last_global_map_time = cur_time;
                let mut global_cloud: PointCloud = Vec::new();
                for kp in pgo.key_poses() {
                    if let Some(body_cloud) = &kp.body_cloud {
                        global_cloud.extend(pointcloud::transform_cloud(
                            body_cloud,
                            &kp.r_global,
                            &kp.t_global,
                        ));
                    }
                }
                let filtered =
                    pointcloud::voxel_downsample(&global_cloud, self.config.global_map_voxel_size);
                let msg = build_pointcloud2(&filtered, &frame_id, cur_time);
                self.publish(&self.global_map, &msg, "_global_map");
            }

            std::thread::sleep(timer_period);
        }

        if debug {
            info!("PGO native module shutting down");
        }
    }

    fn publish_corrected(
        &self,
        pgo: &SimplePgo,
        pose: &PoseWithTime,
        ts: f64,
        frame_id: &str,
        body_frame: &str,
        child_frame_id: &str,
    ) {
        let corr_r = mat3::mat_mul(&pgo.offset_r(), &pose.r);
        let corr_t = mat3::add(&mat3::mat_vec(&pgo.offset_r(), &pose.t), &pgo.offset_t());
        let corrected = build_odometry(&corr_r, &corr_t, ts, frame_id, body_frame);
        self.publish(&self.corrected_odometry, &corrected, "corrected_odometry");

        let tf_msg = build_tf_message(
            &pgo.offset_r(),
            &pgo.offset_t(),
            ts,
            frame_id,
            child_frame_id,
        );
        self.publish(&self.correction, &tf_msg, "correction");
    }

    /// Port of publish_deformation_nodes: each keyframe keeps a stable random
    /// id; a node is (re)published only when it's new or the optimizer moved
    /// it past an epsilon.
    fn publish_deformation_nodes(
        &self,
        key_poses: &[KeyPoseWithCloud],
        tf_id: u64,
        frame_id: &str,
        ids: &mut Vec<u64>,
        last_published: &mut Vec<(Mat3, Vec3)>,
        rng: &mut SplitMix64,
    ) {
        const POS_EPS: f64 = 1e-4; // 0.1 mm
        const ROT_EPS: f64 = 1e-5; // Frobenius-norm threshold on the rotation matrix
        for (i, kp) in key_poses.iter().enumerate() {
            let is_new = i >= ids.len();
            if !is_new {
                let (last_r, last_t) = &last_published[i];
                let pos_moved = mat3::norm(&mat3::sub(&kp.t_global, last_t));
                let rot_moved = frobenius_diff(&kp.r_global, last_r);
                if pos_moved <= POS_EPS && rot_moved <= ROT_EPS {
                    continue;
                }
                last_published[i] = (kp.r_global, kp.t_global);
            } else {
                ids.push(rng.next());
                last_published.push((kp.r_global, kp.t_global));
            }
            let q = mat3::quat_from_mat(&kp.r_global);
            let node = DeformationNode {
                id: ids[i],
                tf_id,
                pose: WirePose {
                    ts: kp.time,
                    frame_id: frame_id.to_string(),
                    position: kp.t_global,
                    orientation: [q[1], q[2], q[3], q[0]],
                },
            };
            self.publish(&self.tf_deformation_nodes, &node, "tf_deformation_nodes");
        }
    }
}

fn frobenius_diff(a: &Mat3, b: &Mat3) -> f64 {
    let mut sum = 0.0;
    for row in 0..3 {
        for col in 0..3 {
            let d = a[row][col] - b[row][col];
            sum += d * d;
        }
    }
    sum.sqrt()
}

// ---- message builders (ports of the C++ helpers) ---------------------------------------

fn make_time(ts: f64) -> Time {
    Time {
        sec: ts as i32,
        nsec: ((ts - (ts as i32) as f64) * 1e9) as i32,
    }
}

fn make_header(frame_id: &str, ts: f64) -> Header {
    Header {
        seq: 0,
        stamp: make_time(ts),
        frame_id: frame_id.to_string(),
    }
}

fn build_odometry(r: &Mat3, t: &Vec3, ts: f64, frame_id: &str, child_frame_id: &str) -> Odometry {
    let q = mat3::quat_from_mat(r);
    let mut odom = Odometry {
        header: make_header(frame_id, ts),
        child_frame_id: child_frame_id.to_string(),
        ..Default::default()
    };
    odom.pose.pose.position = Point {
        x: t[0],
        y: t[1],
        z: t[2],
    };
    odom.pose.pose.orientation = Quaternion {
        x: q[1],
        y: q[2],
        z: q[3],
        w: q[0],
    };
    odom
}

fn build_tf_message(
    correction_r: &Mat3,
    correction_t: &Vec3,
    ts: f64,
    frame_id: &str,
    child_frame_id: &str,
) -> TFMessage {
    let q = mat3::quat_from_mat(correction_r);
    TFMessage {
        transforms: vec![TransformStamped {
            header: make_header(frame_id, ts),
            child_frame_id: child_frame_id.to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: correction_t[0],
                    y: correction_t[1],
                    z: correction_t[2],
                },
                rotation: Quaternion {
                    x: q[1],
                    y: q[2],
                    z: q[3],
                    w: q[0],
                },
            },
        }],
    }
}

// Pose-graph metadata ids (match the C++ constants).
const NODE_KEYFRAME: u64 = 0;
const EDGE_ODOMETRY: u64 = 0;
const EDGE_LOOP_CLOSURE: u64 = 1;

fn build_pose_graph(
    key_poses: &[KeyPoseWithCloud],
    loop_pairs: &[(usize, usize)],
    ts: f64,
    frame_id: &str,
) -> Graph3D {
    let mut msg = Graph3D {
        ts,
        nodes: Vec::with_capacity(key_poses.len()),
        edges: Vec::with_capacity(key_poses.len() + loop_pairs.len()),
    };
    for (i, kp) in key_poses.iter().enumerate() {
        let q = mat3::quat_from_mat(&kp.r_global);
        msg.nodes.push(Node3D {
            pose: WirePose {
                ts: kp.time,
                frame_id: frame_id.to_string(),
                position: kp.t_global,
                orientation: [q[1], q[2], q[3], q[0]],
            },
            id: i as u64,
            metadata_id: NODE_KEYFRAME,
        });
    }
    for (i, key_pose) in key_poses.iter().enumerate().skip(1) {
        msg.edges.push(Edge {
            start_id: (i - 1) as u64,
            end_id: i as u64,
            timestamp: key_pose.time,
            metadata_id: EDGE_ODOMETRY,
        });
    }
    for &(first, second) in loop_pairs {
        if first >= key_poses.len() || second >= key_poses.len() {
            continue;
        }
        msg.edges.push(Edge {
            start_id: first as u64,
            end_id: second as u64,
            timestamp: ts,
            metadata_id: EDGE_LOOP_CLOSURE,
        });
    }
    msg
}

const NODE_KEYFRAME_DELTA: u64 = 0;

/// Port of build_loop_closure_event: each pair is (pre-smooth node, SE(3)
/// delta such that post = delta * pre).
fn build_loop_closure_event(
    pre_poses: &[(Mat3, Vec3)],
    post_poses: &[KeyPoseWithCloud],
    ts: f64,
    frame_id: &str,
) -> GraphDelta3D {
    let count = pre_poses.len().min(post_poses.len());
    let mut msg = GraphDelta3D {
        ts,
        nodes: Vec::with_capacity(count),
        transforms: Vec::with_capacity(count),
    };
    for i in 0..count {
        let (pre_r, pre_t) = &pre_poses[i];
        let post_r = &post_poses[i].r_global;
        let post_t = &post_poses[i].t_global;

        // SE(3) delta such that post = delta * pre.
        let r_delta = mat3::mat_mul(post_r, &mat3::transpose(pre_r));
        let t_delta = mat3::sub(post_t, &mat3::mat_vec(&r_delta, pre_t));
        let q_pre = mat3::quat_from_mat(pre_r);
        let q_delta = mat3::quat_from_mat(&r_delta);

        msg.nodes.push(Node3D {
            pose: WirePose {
                ts: post_poses[i].time,
                frame_id: frame_id.to_string(),
                position: *pre_t,
                orientation: [q_pre[1], q_pre[2], q_pre[3], q_pre[0]],
            },
            id: i as u64,
            metadata_id: NODE_KEYFRAME_DELTA,
        });
        msg.transforms.push(DeltaTransform {
            translation: t_delta,
            rotation: [q_delta[1], q_delta[2], q_delta[3], q_delta[0]],
        });
    }
    msg
}

/// The `from_pcl` wire layout (x/y/z/intensity float32, 16-byte points).
/// The Rust core's PointCloud carries no intensity, so it's published as 0.
fn build_pointcloud2(points: &[[f32; 3]], frame_id: &str, ts: f64) -> PointCloud2 {
    let mut data = Vec::with_capacity(points.len() * 16);
    for p in points {
        data.extend_from_slice(&p[0].to_le_bytes());
        data.extend_from_slice(&p[1].to_le_bytes());
        data.extend_from_slice(&p[2].to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
    }
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let n = points.len() as i32;
    PointCloud2 {
        header: make_header(frame_id, ts),
        height: 1,
        width: n,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: true,
    }
}

#[derive(Debug)]
struct ExtractError(String);
impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// Port of smartnav::parse_pointcloud2 (x/y/z float32 offsets; intensity is
/// dropped — the PGO core never reads it).
fn extract_xyz(msg: &PointCloud2) -> Result<PointCloud, ExtractError> {
    let mut ox = None;
    let mut oy = None;
    let mut oz = None;
    for f in &msg.fields {
        match f.name.as_str() {
            "x" => ox = Some(f.offset as usize),
            "y" => oy = Some(f.offset as usize),
            "z" => oz = Some(f.offset as usize),
            _ => {}
        }
    }
    let (ox, oy, oz) = match (ox, oy, oz) {
        (Some(a), Some(b), Some(c)) => (a, b, c),
        _ => return Err(ExtractError("missing x/y/z fields".into())),
    };
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("zero point_step".into()));
    }
    let num_points = (msg.width as usize) * (msg.height as usize);
    let data = &msg.data;
    let mut out = Vec::with_capacity(num_points);
    for i in 0..num_points {
        let base = i * step;
        if base + step > data.len() {
            break;
        }
        let read = |off: usize| -> f32 {
            let b = &data[base + off..base + off + 4];
            f32::from_le_bytes([b[0], b[1], b[2], b[3]])
        };
        out.push([read(ox), read(oy), read(oz)]);
    }
    Ok(out)
}

/// Tiny non-crypto RNG for the stable deformation-node ids (the C++ uses
/// std::mt19937_64 seeded from random_device; the ids only need to be stable
/// within a run and unlikely to collide, so no rand crate dependency).
struct SplitMix64(u64);

impl SplitMix64 {
    fn from_entropy() -> Self {
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x9E3779B97F4A7C15)
            ^ (std::process::id() as u64) << 32;
        SplitMix64(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<GscPgo>().await;
}
