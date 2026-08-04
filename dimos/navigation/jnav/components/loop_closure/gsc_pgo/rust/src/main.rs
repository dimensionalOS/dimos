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

//! Native Rust PGO module on top of the `dimos_module` framework (config
//! arrives as stdin JSON).
//!
//! Structure:
//! - LCM handlers stash odometry (latest + a sliding interpolation buffer),
//!   pair each lidar scan with the LATEST odometry pose (CloudWithPose), and
//!   buffer LocationConstraints.
//! - A worker loop, woken by the callbacks as scans/constraints arrive, drains
//!   constraints (each becomes its own pose node placed via odometry
//!   interpolated at the constraint's own timestamp), feeds scans into GscPgo
//!   oldest-first, and publishes:
//!   corrected_odometry (offset-corrected pose), correction (map->odom
//!   TFMessage), pose_graph (Graph3D snapshot per keyframe),
//!   loop_closure_event (GraphDelta3D of pre->post iSAM2 deltas when a loop
//!   fired), tf_deformation_nodes (per-keyframe DeformationNode, re-published
//!   only when the optimizer moves a node past an epsilon), and the optional
//!   throttled _global_map debug cloud.
//!
//! GscPgo wraps gtsam, which is thread-unsafe (`!Send`), so the worker is
//! a dedicated OS thread that OWNS the GscPgo; publishes hop back onto the
//! tokio runtime via a captured Handle.

use std::collections::{HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use dimos_gsc_pgo::gsc_pgo::{
    CloudWithPose, Config as PgoConfig, GscPgo, KeyPoseWithCloud, LocationConstraintObs,
    PoseWithTime, GTSAM_THREAD_STACK_BYTES,
};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::msgs::{
    DeformationNode, Graph3D, GraphDelta3D, LocationConstraint, PoseStamped as WirePose,
};
use dimos_gsc_pgo::pointcloud::{self, PointCloud};
use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::tf2_msgs::TFMessage;

mod utils;
use tracing::{error, info, warn};
use utils::{
    build_loop_closure_event, build_odometry, build_pointcloud2, build_pose_graph,
    build_tf_message, extract_xyz, frobenius_diff, interpolate_odom, SplitMix64,
};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Output/map frame (named `map_frame` rather than `frame_id` because the
    /// Python coordinator strips base-config field names — `frame_id` is one —
    /// from the stdin config dict).
    pub map_frame: String,
    /// The corrected edge's child (odom) frame.
    pub child_frame_id: String,
    /// Robot body frame; LocationConstraints must be expressed in it.
    pub body_frame: String,

    // Keyframe detection
    pub keyframe_min_rotation_degrees: f64,
    pub keyframe_min_distance_meters: f64,

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
    pub loop_max_lowe_ratio: f64,
    pub loop_max_yank_rotation_deg: f64,
    pub loop_yank_gate_max_distance_m: f64,
    pub loop_min_id_gap: u64,

    // Long-jump agreement buffer
    pub loop_instant_accept_distance_m: f64,
    pub loop_buffer_agreement_trans_m: f64,
    pub loop_buffer_agreement_rot_deg: f64,
    pub loop_buffer_min_agree: u64,

    /// Transform world-frame scans to body-frame using the paired odometry.
    pub subtract_odom_from_cloud: bool,

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

    // Final batch GNC (TLS) re-solve over the committed closures.
    pub loop_gnc_final: bool,
    pub loop_gnc_var_scale: f64,
    pub loop_gnc_inlier_probability: f64,

    // Location constraints
    pub use_location_constraints: bool,
    pub odom_buffer_window: f64,

    // First-keyframe anchor prior roll/pitch stiffness + optional per-keyframe
    // roll/pitch prior.
    pub anchor_roll_pitch_var: f64,
    pub per_keyframe_roll_pitch_prior: bool,
    pub per_keyframe_roll_pitch_var: f64,

    // Anisotropic odometry between-factor
    pub odom_rot_roll_pitch_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,

    /// Bounded scan FIFO depth (<= 0 = unbounded).
    pub max_scan_queue: i32,

    /// One dial over every false-closure gate, 0 (closure-happy) to 4 (all
    /// gates plus a tight GNC). Redundant with the individual `loop_*` gates
    /// on purpose: when set it overwrites all of them, so set one or the
    /// other. -1 leaves the individual fields alone.
    #[validate(range(min = -1, max = 4))]
    pub loop_conservativeness: i32,
}

impl Config {
    fn pgo(&self) -> PgoConfig {
        let mut config = PgoConfig {
            keyframe_min_rotation_degrees: self.keyframe_min_rotation_degrees,
            keyframe_min_distance_meters: self.keyframe_min_distance_meters,
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
            loop_max_lowe_ratio: self.loop_max_lowe_ratio,
            loop_max_yank_rotation_deg: self.loop_max_yank_rotation_deg,
            loop_yank_gate_max_distance_m: self.loop_yank_gate_max_distance_m,
            loop_min_id_gap: self.loop_min_id_gap,
            loop_instant_accept_distance_m: self.loop_instant_accept_distance_m,
            loop_buffer_agreement_trans_m: self.loop_buffer_agreement_trans_m,
            loop_buffer_agreement_rot_deg: self.loop_buffer_agreement_rot_deg,
            loop_buffer_min_agree: self.loop_buffer_min_agree,
            loop_robust_kernel: self.loop_robust_kernel,
            loop_robust_huber_k: self.loop_robust_huber_k,
            loop_gnc_final: self.loop_gnc_final,
            loop_gnc_var_scale: self.loop_gnc_var_scale,
            loop_gnc_inlier_probability: self.loop_gnc_inlier_probability,
            use_location_constraints: self.use_location_constraints,
            odom_rot_roll_pitch_var: self.odom_rot_roll_pitch_var,
            odom_rot_yaw_var: self.odom_rot_yaw_var,
            odom_trans_xy_var: self.odom_trans_xy_var,
            odom_trans_z_var: self.odom_trans_z_var,
            anchor_roll_pitch_var: self.anchor_roll_pitch_var,
            per_keyframe_roll_pitch_prior: self.per_keyframe_roll_pitch_prior,
            per_keyframe_roll_pitch_var: self.per_keyframe_roll_pitch_var,
            use_scan_context: self.use_scan_context,
            scan_context_num_rings: self.scan_context_num_rings,
            scan_context_num_sectors: self.scan_context_num_sectors,
            scan_context_max_range_m: self.scan_context_max_range_m,
            scan_context_top_k: self.scan_context_top_k,
            scan_context_match_threshold: self.scan_context_match_threshold,
            scan_context_lidar_height_m: self.scan_context_lidar_height_m,
        };
        config.apply_conservativeness(self.loop_conservativeness);
        config
    }
}

// ---- shared state ---------------------------------------------------------------

#[derive(Clone)]
struct OdomSample {
    ts: f64,
    rotation: Mat3,
    translation: Vec3,
    frame_id: String,
}

#[derive(Default)]
struct SharedState {
    latest_odom: Option<OdomSample>,
    /// Reject out-of-order scans.
    last_message_time: f64,
    /// Sliding odometry history for interpolating a constraint's pose at its
    /// own timestamp.
    odom_buffer: VecDeque<OdomSample>,
    /// Bounded scan FIFO.
    cloud_buffer: VecDeque<CloudWithPose>,
    /// Pending LocationConstraints, drained by the worker loop.
    constraints: Vec<(LocationConstraintObs, String)>,
    /// LocationConstraint frames already warned about (once each).
    frame_warned: HashSet<String>,
}

type Shared<T> = Arc<Mutex<T>>;

// ---- module ----------------------------------------------------------------------

#[derive(Module)]
#[module(setup = spawn_worker, teardown = stop_worker)]
struct GscPgoModule {
    #[input(decode = PointCloud2::decode, handler = on_cloud)]
    cloud: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    // Optional decoupled LocationConstraint events; only consumed when
    // config.use_location_constraints is set (otherwise the handler is a
    // no-op).
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
    /// Signaled by callbacks when new work (a scan or constraint) is enqueued,
    /// so the worker wakes immediately instead of polling.
    wake: Arc<Condvar>,
    worker: Option<WorkerHandle>,
}

struct WorkerHandle {
    stop: Arc<AtomicBool>,
    thread: std::thread::JoinHandle<()>,
}

impl GscPgoModule {
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
            wake: Arc::clone(&self.wake),
        };
        // GscPgo (gtsam) is !Send: a dedicated OS thread owns it for the
        // module's whole life instead of a tokio task.
        let thread = std::thread::Builder::new()
            .name("gsc-pgo-worker".into())
            .stack_size(GTSAM_THREAD_STACK_BYTES)
            .spawn(move || worker.run())
            .expect("spawn PGO worker thread");
        self.worker = Some(WorkerHandle { stop, thread });
        info!("PGO native module started (Rust iSAM2 port)");
    }

    async fn stop_worker(&mut self) {
        if let Some(WorkerHandle { stop, thread }) = self.worker.take() {
            stop.store(true, Ordering::Relaxed);
            // Wake the worker out of its Condvar wait so it observes the stop
            // flag promptly instead of sitting until the wait times out.
            self.wake.notify_all();
            let _ = tokio::task::spawn_blocking(move || thread.join()).await;
        }
    }

    /// Handle an incoming odometry message: record it as the latest sample
    /// and push it onto the sliding interpolation buffer.
    async fn on_odometry(&mut self, msg: Odometry) {
        let q = &msg.pose.pose.orientation;
        let rotation = mat3::mat_from_quat(&[q.w, q.x, q.y, q.z]);
        let p = &msg.pose.pose.position;
        let translation = [p.x, p.y, p.z];
        let ts = msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 / 1e9;
        let sample = OdomSample {
            ts,
            rotation,
            translation,
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

    /// Handle an incoming lidar scan: pair it with the latest odometry pose
    /// and enqueue it on the bounded scan FIFO.
    async fn on_cloud(&mut self, msg: PointCloud2) {
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
        let mut pose = PoseWithTime::new(latest.rotation, latest.translation);
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
        drop(state);
        self.wake.notify_one();
    }

    /// Handle an incoming LocationConstraint: validate its frame and buffer
    /// it for the worker loop to drain.
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
            rotation_body_loc: mat3::mat_from_quat(&[qw, qx, qy, qz]),
            translation_body_loc: msg.position,
            covariance: msg.covariance,
            ts: msg.ts,
        };
        state.constraints.push((obs, frame_id));
        drop(state);
        self.wake.notify_one();
    }
}

// ---- worker (the PGO main loop) -----------------------------------------------------

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
    wake: Arc<Condvar>,
}

impl Worker {
    fn publish<T>(&self, out: &Output<T>, msg: &T, what: &str) {
        if let Err(e) = self.rt.block_on(out.publish(msg)) {
            error!(error = %e, "{what} failed to publish");
        }
    }

    fn run(self) {
        let mut pgo = GscPgo::new(self.config.pgo());
        let frame_id = self.config.map_frame.clone();
        let child_frame_id = self.config.child_frame_id.clone();
        let body_frame = self.config.body_frame.clone();

        let publish_global_map = self.config.global_map_publish_rate > 0.0;
        let global_map_interval = if publish_global_map {
            1.0 / self.config.global_map_publish_rate
        } else {
            0.0
        };
        let mut last_global_map_time = 0.0f64;
        // Backstop for the Condvar wait so the stop flag is still observed even
        // if a notify is somehow missed; normal wakeups come from callbacks.
        let wake_timeout = Duration::from_secs(1);
        // While a background GNC solve is in flight during idle, republish the
        // graph at this cadence so downstream consumers (e.g. the eval harness
        // settle heartbeat) know the final verdict is still pending.
        let gnc_wait_publish_interval = Duration::from_secs(2);
        let mut last_gnc_wait_publish = std::time::Instant::now();

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
                    let Some((rotation, translation, interp_frame)) = interp else {
                        warn!(
                            to_id = %constraint.to_id,
                            ts = constraint.ts,
                            "no odometry within the buffer window for constraint; dropping"
                        );
                        continue;
                    };
                    let mut node_pose = PoseWithTime::new(rotation, translation);
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

            // Strict FIFO: process scans oldest-first (backlog bounded at
            // enqueue time). Loops back immediately when more remain, so a
            // burst drains at full speed rather than one-per-tick.
            let cloud_with_pose = {
                let mut state = self.state.lock().expect("state");
                state.cloud_buffer.pop_front()
            };
            let Some(mut cloud_with_pose) = cloud_with_pose else {
                // Idle: fold in any finished background GNC solve now instead
                // of waiting for the next keyframe, so the map snaps into
                // place when the robot pauses (or the stream ends). Adopting
                // the batch solution (poses + rebuilt iSAM2) recovers from
                // incremental divergence; with location constraints in use the
                // rebuild would drop their factors, so fall back to
                // classification-only there.
                let gnc_folded = if self.config.use_location_constraints {
                    let applied = pgo.poll_and_apply_gnc_classification();
                    if applied {
                        pgo.smooth_and_update();
                    }
                    applied
                } else {
                    pgo.poll_and_adopt_gnc_solution()
                };
                if gnc_folded {
                    let last_time = pgo.key_poses().last().map(|kp| kp.time).unwrap_or(0.0);
                    self.publish_graph(
                        &pgo,
                        last_time,
                        &frame_id,
                        deformation_tf_id,
                        &mut deformation_ids,
                        &mut deformation_last,
                        &mut deformation_rng,
                    );
                    continue;
                }
                // Deferred half of adoption: the iSAM2 rebuild blocks this
                // thread for tens of seconds, so it only runs once no newer
                // solve is pending — republishes stay alive until the final
                // solve's poses are already out the door.
                if !pgo.gnc_in_flight() && pgo.rebuild_isam2_if_stale() {
                    let last_time = pgo.key_poses().last().map(|kp| kp.time).unwrap_or(0.0);
                    self.publish_graph(
                        &pgo,
                        last_time,
                        &frame_id,
                        deformation_tf_id,
                        &mut deformation_ids,
                        &mut deformation_last,
                        &mut deformation_rng,
                    );
                    continue;
                }
                if pgo.gnc_in_flight()
                    && last_gnc_wait_publish.elapsed() >= gnc_wait_publish_interval
                {
                    last_gnc_wait_publish = std::time::Instant::now();
                    let last_time = pgo.key_poses().last().map(|kp| kp.time).unwrap_or(0.0);
                    self.publish_graph(
                        &pgo,
                        last_time,
                        &frame_id,
                        deformation_tf_id,
                        &mut deformation_ids,
                        &mut deformation_last,
                        &mut deformation_rng,
                    );
                }
                // Nothing to do: block until a callback signals new work,
                // re-checking under the lock to avoid a lost wakeup.
                let state = self.state.lock().expect("state");
                if state.cloud_buffer.is_empty()
                    && (!self.config.use_location_constraints || state.constraints.is_empty())
                {
                    let _ = self.wake.wait_timeout(state, wake_timeout);
                }
                continue;
            };

            // Optionally transform world-frame scan to body-frame:
            // body = R_odom^T * (world_pts - t_odom).
            if self.config.subtract_odom_from_cloud {
                if let Some(cloud) = cloud_with_pose.cloud.take() {
                    if !cloud.is_empty() {
                        let rotation_inv = mat3::transpose(&cloud_with_pose.pose.rotation);
                        let neg_translation = [
                            -cloud_with_pose.pose.translation[0],
                            -cloud_with_pose.pose.translation[1],
                            -cloud_with_pose.pose.translation[2],
                        ];
                        let translation_body = mat3::mat_vec(&rotation_inv, &neg_translation);
                        cloud_with_pose.cloud = Some(Arc::new(pointcloud::transform_cloud(
                            &cloud,
                            &rotation_inv,
                            &translation_body,
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
                continue;
            }

            // Keyframe added. Snapshot global poses BEFORE search + smooth so
            // we can publish the delta iSAM2 applies if a loop actually fires.
            pgo.search_for_loop_pairs();
            let had_loop = pgo.has_loop();
            let pre_poses: Vec<(Mat3, Vec3)> = if had_loop {
                pgo.key_poses()
                    .iter()
                    .map(|kp| (kp.rotation_global, kp.translation_global))
                    .collect()
            } else {
                Vec::new()
            };

            // Everything below is on the pipeline thread; time it on closures to
            // confirm the batch GNC no longer blocks here (was seconds inline).
            let pipeline_thread_t0 = std::time::Instant::now();

            // Approach B: fold in the newest background GNC classification (drops
            // outlier loops' factors via pending removals) BEFORE the iSAM2 update
            // so the removal takes effect this cycle. iSAM2 stays authoritative for
            // poses — no full-graph pose overwrite on the pipeline thread.
            pgo.poll_and_apply_gnc_classification();
            pgo.smooth_and_update();

            if had_loop {
                // Queue a fresh full-graph GNC classification for the closure just
                // committed. The heavy solve runs off-thread and only feeds back
                // inlier/outlier decisions, so this keyframe never blocks on it.
                pgo.dispatch_gnc_async();
                eprintln!(
                    "gnc-main-thread: closure at {} keyframes handled in {:.1} ms",
                    pgo.key_poses().len(),
                    pipeline_thread_t0.elapsed().as_secs_f64() * 1000.0,
                );
                let msg =
                    build_loop_closure_event(&pre_poses, pgo.key_poses(), cur_time, &frame_id);
                self.publish(&self.loop_closure_event, &msg, "loop_closure_event");
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
            // poses on loop closure). Only GNC-kept loops are published as edges.
            self.publish_graph(
                &pgo,
                cur_time,
                &frame_id,
                deformation_tf_id,
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
                            &kp.rotation_global,
                            &kp.translation_global,
                        ));
                    }
                }
                let filtered =
                    pointcloud::voxel_downsample(&global_cloud, self.config.global_map_voxel_size);
                let msg = build_pointcloud2(&filtered, &frame_id, cur_time);
                self.publish(&self.global_map, &msg, "_global_map");
            }
        }
    }

    /// Publish the pose-graph snapshot (GNC-kept loops only as edges) plus the
    /// per-keyframe deformation-node stream.
    #[allow(clippy::too_many_arguments)]
    fn publish_graph(
        &self,
        pgo: &GscPgo,
        ts: f64,
        frame_id: &str,
        deformation_tf_id: u64,
        deformation_ids: &mut Vec<u64>,
        deformation_last: &mut Vec<(Mat3, Vec3)>,
        deformation_rng: &mut SplitMix64,
    ) {
        let active_pairs: Vec<(usize, usize)> = pgo
            .committed_loops()
            .iter()
            .filter(|committed| committed.active_in_isam2)
            .map(|committed| (committed.target_id, committed.source_id))
            .collect();
        let graph = build_pose_graph(pgo.key_poses(), &active_pairs, ts, frame_id);
        self.publish(&self.pose_graph, &graph, "pose_graph");
        self.publish_deformation_nodes(
            pgo.key_poses(),
            deformation_tf_id,
            frame_id,
            deformation_ids,
            deformation_last,
            deformation_rng,
        );
    }

    fn publish_corrected(
        &self,
        pgo: &GscPgo,
        pose: &PoseWithTime,
        ts: f64,
        frame_id: &str,
        body_frame: &str,
        child_frame_id: &str,
    ) {
        let corrected_rotation = mat3::mat_mul(&pgo.rotation_offset(), &pose.rotation);
        let corrected_translation = mat3::add(
            &mat3::mat_vec(&pgo.rotation_offset(), &pose.translation),
            &pgo.translation_offset(),
        );
        let corrected = build_odometry(
            &corrected_rotation,
            &corrected_translation,
            ts,
            frame_id,
            body_frame,
        );
        self.publish(&self.corrected_odometry, &corrected, "corrected_odometry");

        let tf_msg = build_tf_message(
            &pgo.rotation_offset(),
            &pgo.translation_offset(),
            ts,
            frame_id,
            child_frame_id,
        );
        self.publish(&self.correction, &tf_msg, "correction");
    }

    /// Publish per-keyframe deformation nodes: each keyframe keeps a stable
    /// random id; a node is (re)published only when it's new or the optimizer
    /// moved it past an epsilon.
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
                let (last_rotation, last_translation) = &last_published[i];
                let pos_moved = mat3::norm(&mat3::sub(&kp.translation_global, last_translation));
                let rot_moved = frobenius_diff(&kp.rotation_global, last_rotation);
                if pos_moved <= POS_EPS && rot_moved <= ROT_EPS {
                    continue;
                }
                last_published[i] = (kp.rotation_global, kp.translation_global);
            } else {
                ids.push(rng.next());
                last_published.push((kp.rotation_global, kp.translation_global));
            }
            let q = mat3::quat_from_mat(&kp.rotation_global);
            let node = DeformationNode {
                id: ids[i],
                tf_id,
                pose: WirePose {
                    ts: kp.time,
                    frame_id: frame_id.to_string(),
                    position: kp.translation_global,
                    orientation: [q[1], q[2], q[3], q[0]],
                },
            };
            self.publish(&self.tf_deformation_nodes, &node, "tf_deformation_nodes");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<GscPgoModule>().await;
}
