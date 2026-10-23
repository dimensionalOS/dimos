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
//! - LCM handlers stash the latest odometry and pair each lidar scan with it (CloudWithPose).
//! - A worker thread, woken by the callbacks, feeds scans into GscPgo oldest-first and publishes:
//!   corrected_odometry (offset-corrected pose), tf (the map->odom correction), pose_graph (a
//!   packed record per keyframe), loop_closure_event (a packed record per keyframe a solve moved),
//!   loop_edges (each loop candidate's lifecycle), and the optional debug_global_map cloud. Loop
//!   searches and pose-graph solves run on their own threads; publishes hop back onto the tokio
//!   runtime via a captured Handle.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use dimos_gsc_pgo::gsc_pgo::{
    CloudWithPose, Config as PgoConfig, GscPgo, KeyPoseWithCloud, PoseWithTime,
};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pointcloud::{self, PointCloud};
use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::tf2_msgs::TFMessage;

mod utils;
use tracing::{error, info, warn};
use utils::{
    build_loop_closure_records, build_loop_edge_records, build_odometry, build_pointcloud2,
    build_pose_graph_records, build_tf_message, extract_xyz,
};

// A keyframe pose change smaller than this waits to add up before it is sent.
const POSE_CHANGE_M: f64 = 1e-3;
const POSE_CHANGE_RAD: f64 = 1e-4;
/// How long the pipeline sits without a scan before it adopts a finished background solve: longer than
/// any gap between a replay's scans, so it only fires once the stream pauses or ends.
const SOLVE_IDLE_FINISH: Duration = Duration::from_secs(3);

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Output/map frame (named `map_frame` rather than `frame_id` because the
    /// Python coordinator strips base-config field names — `frame_id` is one —
    /// from the stdin config dict).
    pub map_frame: String,
    /// The corrected edge's child (odom) frame.
    pub child_frame_id: String,
    /// Robot body frame.
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
    pub keyframe_voxel_size: f64,
    pub loop_min_occupancy: i32,
    pub loop_min_degeneracy: f64,
    pub loop_yank_gate: bool,
    pub loop_yank_gate_width: f64,
    pub loop_yank_per_sqrt_gap: bool,
    pub loop_icp_dump_dir: String,
    pub loop_extended_icp_max_keyframes: usize,

    /// Transform world-frame scans to body-frame using the paired odometry.
    pub subtract_odom_from_cloud: bool,

    // Debug global-map publishing (rate <= 0 disables; see module.py).
    pub global_map_voxel_size: f64,
    pub global_map_publish_rate: f64,
    // Where each pose graph snapshot's keyframes go, for offline checks; empty for never.
    pub dump_path: String,

    pub scan_context_num_rings: i32,
    pub scan_context_num_sectors: i32,
    pub scan_context_max_range_m: f64,
    pub scan_context_top_k: i32,
    pub scan_context_match_threshold: f64,
    pub scan_context_lidar_height_m: f64,

    pub loop_robust_huber_k: f64,
    pub loop_icp_max_iterations: u64,
    pub loop_icp_max_correspondence_m: f64,

    pub loop_gnc_var_scale: f64,
    pub loop_gnc_apply_after_fraction: f64,
    pub loop_gnc_deterministic: bool,
    pub loop_gnc_inlier_probability: f64,

    // Anisotropic odometry between-factor
    pub odom_rot_roll_pitch_var: f64,
    pub odom_rot_yaw_var: f64,
    pub odom_trans_xy_var: f64,
    pub odom_trans_z_var: f64,

    /// Bounded scan FIFO depth (<= 0 = unbounded).
    pub max_scan_queue: i32,
}

impl Config {
    fn pgo(&self) -> PgoConfig {
        PgoConfig {
            keyframe_min_rotation_degrees: self.keyframe_min_rotation_degrees,
            keyframe_min_distance_meters: self.keyframe_min_distance_meters,
            loop_search_radius: self.loop_search_radius,
            loop_time_thresh: self.loop_time_thresh,
            loop_score_thresh: self.loop_score_thresh,
            loop_submap_half_range: self.loop_submap_half_range,
            submap_resolution: self.submap_resolution,
            keyframe_voxel_size: self.keyframe_voxel_size,
            loop_min_occupancy: self.loop_min_occupancy,
            loop_min_degeneracy: self.loop_min_degeneracy,
            loop_yank_gate: self.loop_yank_gate,
            loop_yank_gate_width: self.loop_yank_gate_width,
            loop_yank_per_sqrt_gap: self.loop_yank_per_sqrt_gap,
            loop_icp_dump_dir: self.loop_icp_dump_dir.clone(),
            loop_record_strain: false,
            loop_yank_from_strain: false,
            loop_extended_icp_max_keyframes: self.loop_extended_icp_max_keyframes,
            loop_robust_huber_k: self.loop_robust_huber_k,
            loop_icp_max_iterations: self.loop_icp_max_iterations,
            loop_icp_max_correspondence_m: self.loop_icp_max_correspondence_m,
            loop_gnc_var_scale: self.loop_gnc_var_scale,
            loop_gnc_inlier_probability: self.loop_gnc_inlier_probability,
            loop_gnc_apply_after_fraction: self.loop_gnc_apply_after_fraction,
            loop_gnc_deterministic: self.loop_gnc_deterministic,
            odom_rot_roll_pitch_var: self.odom_rot_roll_pitch_var,
            odom_rot_yaw_var: self.odom_rot_yaw_var,
            odom_trans_xy_var: self.odom_trans_xy_var,
            odom_trans_z_var: self.odom_trans_z_var,
            scan_context_num_rings: self.scan_context_num_rings,
            scan_context_num_sectors: self.scan_context_num_sectors,
            scan_context_max_range_m: self.scan_context_max_range_m,
            scan_context_top_k: self.scan_context_top_k,
            scan_context_match_threshold: self.scan_context_match_threshold,
            scan_context_lidar_height_m: self.scan_context_lidar_height_m,
        }
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
    /// Bounded scan FIFO.
    cloud_buffer: VecDeque<CloudWithPose>,
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

    #[output(encode = Odometry::encode)]
    corrected_odometry: Output<Odometry>,

    // The map -> odometry correction.
    #[output(encode = TFMessage::encode)]
    tf: Output<TFMessage>,

    // One packed record per keyframe at its current pose.
    #[output(encode = PointCloud2::encode)]
    pose_graph: Output<PointCloud2>,

    // One packed record per keyframe a solve moved: its pose before, and the delta.
    #[output(encode = PointCloud2::encode)]
    loop_closure_event: Output<PointCloud2>,

    // Each loop candidate's lifecycle as it happens (proposed, gated, GNC votes, accepted,
    // rejected, applied) with both keyframes' current positions, for drawing loop edges.
    #[output(encode = PointCloud2::encode)]
    loop_edges: Output<PointCloud2>,

    // Debug-only, nothing reads it. Publishing is gated on global_map_publish_rate > 0.
    #[output(encode = PointCloud2::encode)]
    debug_global_map: Output<PointCloud2>,

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
            pose_graph: self.pose_graph.clone(),
            loop_closure_event: self.loop_closure_event.clone(),
            loop_edges: self.loop_edges.clone(),
            global_map: self.debug_global_map.clone(),
            tf: self.tf.clone(),
            rt: tokio::runtime::Handle::current(),
            stop: Arc::clone(&stop),
            wake: Arc::clone(&self.wake),
        };
        // a dedicated OS thread owns the GscPgo for the module's whole life
        let thread = std::thread::Builder::new()
            .name("gsc-pgo-worker".into())
            .spawn(move || worker.run())
            .expect("spawn PGO worker thread");
        self.worker = Some(WorkerHandle { stop, thread });
        info!("PGO native module started");
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
        self.state.lock().expect("state").latest_odom = Some(sample);
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
}

// ---- worker (the PGO main loop) -----------------------------------------------------

struct Worker {
    state: Shared<SharedState>,
    config: Config,
    corrected_odometry: Output<Odometry>,
    tf: Output<TFMessage>,
    pose_graph: Output<PointCloud2>,
    loop_closure_event: Output<PointCloud2>,
    loop_edges: Output<PointCloud2>,
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
        // Identity until the first correction, so map -> child resolves from the start.
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0.0, |d| d.as_secs_f64());
        let identity = build_tf_message(
            &mat3::identity(),
            &[0.0; 3],
            now,
            &frame_id,
            &child_frame_id,
        );
        self.publish(&self.tf, &identity, "tf");

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
        let solve_wait_publish_interval = Duration::from_secs(2);
        let mut last_solve_wait_publish = std::time::Instant::now();
        let mut last_scan_at = std::time::Instant::now();

        // Each keyframe's pose as fast-reform consumers last heard it.
        let mut published: Vec<(Mat3, Vec3)> = Vec::new();
        // Loop-edge events already sent on loop_edges.
        let mut edges_sent = 0usize;
        // Search diagnostics already written to <dump_path>.loops.jsonl.
        let mut diagnostics_written = 0usize;
        // The loop search running in the background, if any.
        let mut loop_search: Option<
            std::thread::JoinHandle<dimos_gsc_pgo::gsc_pgo::LoopSearchResult>,
        > = None;

        while !self.stop.load(Ordering::Relaxed) {
            // Not deterministic: adopt a finished solve now, scans or not.
            if !self.config.loop_gnc_deterministic && pgo.solve_pending() && pgo.update(true) {
                let last_time = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
                self.publish_pose_changes(&pgo, &mut published, false, last_time, &frame_id);
            }
            self.publish_loop_edges(&pgo, &mut edges_sent, &frame_id);
            self.dump_loop_diagnostics(&pgo, &mut diagnostics_written);
            // Strict FIFO: process scans oldest-first (backlog bounded at
            // enqueue time). Loops back immediately when more remain, so a
            // burst drains at full speed rather than one-per-tick.
            let cloud_with_pose = {
                let mut state = self.state.lock().expect("state");
                state.cloud_buffer.pop_front()
            };
            let Some(mut cloud_with_pose) = cloud_with_pose else {
                // Idle long enough that the stream has paused or ended: adopt a finished solve
                // and start the next (non-blocking, so the graph keeps republishing while it runs).
                // Short gaps between scans never get here, which keeps a replay's graph deterministic.
                if pgo.solve_pending() && last_scan_at.elapsed() >= SOLVE_IDLE_FINISH {
                    let last_time = pgo.key_poses().last().map(|kp| kp.time).unwrap_or(0.0);
                    let changed = pgo.update(true);
                    if changed {
                        self.publish_pose_changes(
                            &pgo,
                            &mut published,
                            false,
                            last_time,
                            &frame_id,
                        );
                    }
                    if changed || last_solve_wait_publish.elapsed() >= solve_wait_publish_interval {
                        last_solve_wait_publish = std::time::Instant::now();
                        self.publish_graph(&pgo, last_time, &frame_id);
                    }
                    if changed {
                        continue;
                    }
                }
                // Nothing to do: block until a callback signals new work,
                // re-checking under the lock to avoid a lost wakeup.
                let state = self.state.lock().expect("state");
                if state.cloud_buffer.is_empty() {
                    let _ = self.wake.wait_timeout(state, wake_timeout);
                }
                continue;
            };
            last_scan_at = std::time::Instant::now();

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

            // Keyframe added: consumers hold its scans where the current offset put them.
            published.extend(
                pgo.key_poses()[published.len()..]
                    .iter()
                    .map(|kp| (kp.rotation_global, kp.translation_global)),
            );
            // A loop search that finished in the background lands now, so its loop joins this keyframe's update.
            if loop_search
                .as_ref()
                .is_some_and(|search| search.is_finished())
            {
                let search = loop_search.take().expect("checked above");
                pgo.finish_loop_search(search.join().expect("loop search thread panicked"));
            }
            // The ICP half of a loop search runs on its own thread so scans never wait on it; one at a time,
            // and never while scans are waiting (skip it rather than fall behind).
            let behind = !self.state.lock().expect("state").cloud_buffer.is_empty();
            if behind {
                pgo.skip_loop_search_behind();
            } else if loop_search.is_some() {
                pgo.skip_loop_search("search_in_flight");
            } else if let Some(search) = pgo.begin_loop_search() {
                loop_search = Some(
                    std::thread::Builder::new()
                        .name("gsc-loop-search".into())
                        .spawn(move || search.run())
                        .expect("spawn loop search thread"),
                );
            }
            // a finished solve moves the keyframes (on the keyframe schedule when deterministic)
            let moved = pgo.update(!self.config.loop_gnc_deterministic);
            self.publish_pose_changes(&pgo, &mut published, moved, cur_time, &frame_id);

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
            self.publish_graph(&pgo, cur_time, &frame_id);

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
                self.publish(&self.global_map, &msg, "debug_global_map");
            }
        }
    }

    /// Publish every keyframe's current pose (and dump them with `dump_path`).
    fn publish_graph(&self, pgo: &GscPgo, ts: f64, frame_id: &str) {
        let records = build_pose_graph_records(pgo.key_poses(), ts, frame_id);
        self.publish(&self.pose_graph, &records, "pose_graph");
        if !self.config.dump_path.is_empty() {
            if let Err(e) = dump_keyframes(&self.config.dump_path, pgo.key_poses()) {
                warn!(error = %e, "Failed to dump the keyframes.");
            }
        }
    }

    /// Append every loop search since the last call to `<dump_path>.loops.jsonl`: the candidate,
    /// its outcome and every input of the yank / confidence gate, for offline hypothesis tests.
    fn dump_loop_diagnostics(&self, pgo: &GscPgo, written: &mut usize) {
        let diagnostics = pgo.loop_diagnostics();
        if self.config.dump_path.is_empty() || diagnostics.len() <= *written {
            return;
        }
        let number = |v: f64| {
            if v.is_finite() {
                format!("{v}")
            } else {
                "null".to_string()
            }
        };
        let mut lines = String::new();
        for d in &diagnostics[*written..] {
            lines += &format!(
                "{{\"outcome\":\"{}\",\"source_id\":{},\"source_time\":{},\"target_id\":{},\"target_time\":{},\"scan_context_best\":{},\"lowe_ratio\":{},\"candidate_distance\":{},\"degeneracy_min\":{},\"icp_fitness\":{},\"yank_rotation_deg\":{},\"yank_translation\":{},\"yank_drift_scale\":{},\"sigma_rotation_deg\":{},\"sigma_translation_m\":{},\"yank_z\":{},\"scan_context_factor\":{},\"icp_factor\":{},\"degeneracy_factor\":{},\"confidence\":{},\"required_confidence\":{},\"extended_keyframes\":{},\"extended_source_points\":{},\"extended_target_points\":{},\"single_inliers\":{},\"extended_inliers\":{},\"extended_fitness\":{},\"extended_confidence\":{}}}\n",
                d.outcome, d.source_id, number(d.source_time), d.target_id, number(d.target_time),
                number(d.scan_context_best as f64), number(d.lowe_ratio as f64), number(d.candidate_distance),
                number(d.degeneracy_min as f64), number(d.icp_fitness), number(d.yank_rotation_deg),
                number(d.yank_translation), number(d.yank_drift_scale), number(d.sigma_rotation_deg), number(d.sigma_translation_m),
                number(d.yank_z), number(d.scan_context_factor), number(d.icp_factor),
                number(d.degeneracy_factor), number(d.confidence), number(d.required_confidence),
                d.extended_keyframes, d.extended_source_points, d.extended_target_points, d.single_inliers,
                d.extended_inliers, number(d.extended_fitness), number(d.extended_confidence),
            );
        }
        use std::io::Write;
        let appended = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(format!("{}.loops.jsonl", self.config.dump_path))
            .and_then(|mut file| file.write_all(lines.as_bytes()));
        if let Err(e) = appended {
            warn!(error = %e, "Failed to dump the loop diagnostics.");
        }
        *written = diagnostics.len();
    }

    /// Send the loop-edge events since the last call, if any.
    fn publish_loop_edges(&self, pgo: &GscPgo, sent: &mut usize, frame_id: &str) {
        let events = pgo.loop_edge_events();
        if events.len() <= *sent {
            return;
        }
        let ts = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
        let msg = build_loop_edge_records(&events[*sent..], pgo.key_poses(), ts, frame_id);
        self.publish(&self.loop_edges, &msg, "loop_edges");
        if !self.config.dump_path.is_empty() {
            use std::io::Write;
            let appended = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(format!("{}.edges", self.config.dump_path))
                .and_then(|mut file| file.write_all(&msg.data));
            if let Err(e) = appended {
                warn!(error = %e, "Failed to dump the loop edges.");
            }
        }
        *sent = events.len();
    }

    /// Send fast-reform consumers every keyframe pose change since they last
    /// heard: a loop's update, and also iSAM2 relinearizing after earlier
    /// loops or an adopted GNC solve, which move keyframes without a new loop.
    /// Changes too small to matter wait in `published` and add up.
    fn publish_pose_changes(
        &self,
        pgo: &GscPgo,
        published: &mut Vec<(Mat3, Vec3)>,
        loop_closed: bool,
        ts: f64,
        frame_id: &str,
    ) {
        let poses = pgo.key_poses();
        let moved = poses
            .iter()
            .zip(published.iter())
            .any(|(kp, (rotation, translation))| {
                let shift = mat3::sub(&kp.translation_global, translation);
                let turn = mat3::mat_mul(&kp.rotation_global, &mat3::transpose(rotation));
                let cos = ((turn[0][0] + turn[1][1] + turn[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
                mat3::norm(&shift) > POSE_CHANGE_M || cos.acos() > POSE_CHANGE_RAD
            });
        if !loop_closed && !moved {
            return;
        }
        let records = build_loop_closure_records(published, poses, ts, frame_id);
        self.publish(&self.loop_closure_event, &records, "loop_closure_event");
        if !self.config.dump_path.is_empty() {
            if let Err(e) = append_closure(&self.config.dump_path, ts, &records) {
                warn!(error = %e, "Failed to dump the loop closure.");
            }
        }
        *published = poses
            .iter()
            .map(|kp| (kp.rotation_global, kp.translation_global))
            .collect();
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
        self.publish(&self.tf, &tf_msg, "tf");
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<GscPgoModule>().await;
}

/// Each keyframe as little-endian f64s: ts, raw (local) x y z qx qy qz qw, then
/// corrected (global) x y z qx qy qz qw, written atomically.
fn dump_keyframes(path: &str, key_poses: &[KeyPoseWithCloud]) -> std::io::Result<()> {
    let mut bytes = Vec::with_capacity(key_poses.len() * 15 * 8);
    for kp in key_poses {
        let mut values = vec![kp.time];
        for (rotation, translation) in [
            (&kp.rotation_local, &kp.translation_local),
            (&kp.rotation_global, &kp.translation_global),
        ] {
            let q = mat3::quat_from_mat(rotation);
            values.extend_from_slice(translation);
            values.extend_from_slice(&[q[1], q[2], q[3], q[0]]);
        }
        for v in values {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    let tmp = format!("{path}.tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}

/// Append one closure to `<path>.closures`: f64 ts, u32 record count, then the
/// loop_closure_records bytes (68 per keyframe), for offline replays of real closures.
fn append_closure(path: &str, ts: f64, records: &PointCloud2) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(format!("{path}.closures"))?;
    let mut bytes = Vec::with_capacity(12 + records.data.len());
    bytes.extend_from_slice(&ts.to_le_bytes());
    bytes.extend_from_slice(&(records.width as u32).to_le_bytes());
    bytes.extend_from_slice(&records.data);
    file.write_all(&bytes)
}
