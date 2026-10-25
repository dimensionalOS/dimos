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
//!   closure_edges (each closure candidate's lifecycle). Closure
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
use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::tf2_msgs::TFMessage;

mod utils;
use tracing::{error, info, warn};
use utils::{
    build_closure_edge_records, build_loop_closure_records, build_odometry,
    build_pose_graph_records, build_tf_message, extract_xyz,
};

// A keyframe pose change smaller than this waits to add up before it is sent.
const POSE_CHANGE_M: f64 = 1e-3;
const POSE_CHANGE_RAD: f64 = 1e-4;

/// The module's own fields, then every `gsc_pgo::Config` field (passed through by `pgo()`).
macro_rules! config {
    ($($own:ident: $own_type:ty,)* ; $($pgo:ident: $pgo_type:ty,)*) => {
        #[native_config]
        #[derive(Clone)]
        pub struct Config {
            $(pub $own: $own_type,)*
            $(pub $pgo: $pgo_type,)*
        }
        impl Config {
            fn pgo(&self) -> PgoConfig {
                PgoConfig { $($pgo: self.$pgo.clone(),)* }
            }
        }
    };
}

config! {
    // `map_frame`, not `frame_id`: the Python coordinator strips base-config field names from the config
    map_frame: String,
    child_frame_id: String,
    body_frame: String,
    // where each pose graph snapshot's keyframes go, for offline checks; empty for never
    dump_path: String,
    // bounded scan FIFO depth (<= 0 = unbounded)
    max_scan_queue: i32,
    // threads closure search and solves may use, leaving the other cores to the rest of the robot (0 = every core)
    worker_threads: usize,
    ;
    keyframe_min_rotation_degrees: f64,
    keyframe_min_distance_meters: f64,
    closure_search_radius: f64,
    closure_score_thresh: f64,
    closure_submap_half_range: i32,
    submap_resolution: f64,
    closure_min_occupancy: i32,
    closure_min_degeneracy: f64,
    closure_yank_gate_width: f64,
    closure_icp_dump_dir: String,
    closure_extended_icp_max_keyframes: usize,
    closure_robust_huber_k: f64,
    closure_icp_max_iterations: u64,
    closure_icp_max_correspondence_m: f64,
    closure_gnc_var_scale: f64,
    closure_gnc_inlier_probability: f64,
    closure_cooling: bool,
    odom_rot_roll_pitch_var: f64,
    odom_rot_yaw_var: f64,
    odom_trans_xy_var: f64,
    odom_trans_z_var: f64,
    scan_context_num_rings: i32,
    scan_context_num_sectors: i32,
    scan_context_max_range_m: f64,
    scan_context_top_k: i32,
    scan_context_match_threshold: f64,
    scan_context_lidar_height_m: f64,
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
    /// A background pose-graph solve finished and waits to be adopted.
    solve_finished: bool,
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

    // Each closure candidate's lifecycle as it happens (proposed, gated, GNC votes, accepted,
    // rejected, applied) with both keyframes' current positions, for drawing closure edges.
    #[output(encode = PointCloud2::encode)]
    closure_edges: Output<PointCloud2>,

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
            closure_edges: self.closure_edges.clone(),
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
    closure_edges: Output<PointCloud2>,
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
        // ICP and the solver's factorization run on rayon's global pool
        if let Err(e) = rayon::ThreadPoolBuilder::new()
            .num_threads(self.config.worker_threads)
            .thread_name(|i| format!("gsc-worker-{i}"))
            .build_global()
        {
            warn!(error = %e, "could not cap the worker threads");
        }
        let mut pgo = GscPgo::new(self.config.pgo());
        let (state, wake) = (Arc::clone(&self.state), Arc::clone(&self.wake));
        pgo.notify_when_solved(move || {
            state.lock().expect("state").solve_finished = true;
            wake.notify_one();
        });
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

        // Backstop for the Condvar wait so the stop flag is still observed even
        // if a notify is somehow missed; normal wakeups come from callbacks.
        let wake_timeout = Duration::from_secs(1);

        // Each keyframe's pose as fast-reform consumers last heard it.
        let mut published: Vec<(Mat3, Vec3)> = Vec::new();
        // Closure-edge events already sent on closure_edges.
        let mut edges_sent = 0usize;
        // Search diagnostics already written to <dump_path>.closures.jsonl.
        let mut diagnostics_written = 0usize;
        // The closure search running in the background, if any.
        let mut closure_search: Option<
            std::thread::JoinHandle<dimos_gsc_pgo::gsc_pgo::ClosureSearchResult>,
        > = None;

        while !self.stop.load(Ordering::Relaxed) {
            // adopt a finished solve now, scans or not
            let solve_finished =
                std::mem::take(&mut self.state.lock().expect("state").solve_finished);
            if solve_finished && pgo.wait_for_solve() {
                let last_time = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
                self.publish_pose_changes(&pgo, &mut published, false, last_time, &frame_id);
            }
            self.publish_closure_edges(&pgo, &mut edges_sent, &frame_id);
            self.dump_closure_diagnostics(&pgo, &mut diagnostics_written);
            // Strict FIFO: process scans oldest-first (backlog bounded at
            // enqueue time). Loops back immediately when more remain, so a
            // burst drains at full speed rather than one-per-tick.
            let cloud_with_pose = {
                let mut state = self.state.lock().expect("state");
                state.cloud_buffer.pop_front()
            };
            let Some(cloud_with_pose) = cloud_with_pose else {
                // Nothing to do: block until a callback signals new work,
                // re-checking under the lock to avoid a lost wakeup.
                let state = self.state.lock().expect("state");
                if state.cloud_buffer.is_empty() && !state.solve_finished {
                    let _ = self.wake.wait_timeout(state, wake_timeout);
                }
                continue;
            };

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
            // A closure search that finished in the background lands now, so its closure joins this keyframe's update.
            if closure_search
                .as_ref()
                .is_some_and(|search| search.is_finished())
            {
                let search = closure_search.take().expect("checked above");
                pgo.finish_closure_search(search.join().expect("closure search thread panicked"));
            }
            // Stages 1-2 run here on every keyframe; the best waiting proposal goes to the search thread when it is
            // free and no scans are waiting.
            pgo.propose_closure();
            let behind = !self.state.lock().expect("state").cloud_buffer.is_empty();
            if !behind && closure_search.is_none() {
                if let Some(search) = pgo.begin_closure_search() {
                    closure_search = Some(
                        std::thread::Builder::new()
                            .name("gsc-closure-search".into())
                            .spawn(move || search.run())
                            .expect("spawn closure search thread"),
                    );
                }
            }
            // a finished solve moves the keyframes
            let moved = pgo.update();
            self.publish_pose_changes(&pgo, &mut published, moved, cur_time, &frame_id);

            self.publish_corrected(
                &pgo,
                &cloud_with_pose.pose,
                cur_time,
                &frame_id,
                &body_frame,
                &child_frame_id,
            );

            // every keyframe's current pose, each keyframe
            self.publish_graph(&pgo, cur_time, &frame_id);
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

    /// Append every closure search since the last call to `<dump_path>.closures.jsonl`: the candidate,
    /// its outcome and every input of the yank / confidence gate, for offline hypothesis tests.
    fn dump_closure_diagnostics(&self, pgo: &GscPgo, written: &mut usize) {
        let diagnostics = pgo.closure_diagnostics();
        if self.config.dump_path.is_empty() || diagnostics.len() <= *written {
            return;
        }
        let lines: String = diagnostics[*written..]
            .iter()
            .filter_map(|d| serde_json::to_string(d).ok())
            .map(|line| line + "\n")
            .collect();
        use std::io::Write;
        let appended = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(format!("{}.closures.jsonl", self.config.dump_path))
            .and_then(|mut file| file.write_all(lines.as_bytes()));
        if let Err(e) = appended {
            warn!(error = %e, "Failed to dump the closure diagnostics.");
        }
        *written = diagnostics.len();
    }

    /// Send the closure-edge events since the last call, if any.
    fn publish_closure_edges(&self, pgo: &GscPgo, sent: &mut usize, frame_id: &str) {
        let events = pgo.closure_edge_events();
        if events.len() <= *sent {
            return;
        }
        let ts = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
        let msg = build_closure_edge_records(&events[*sent..], pgo.key_poses(), ts, frame_id);
        self.publish(&self.closure_edges, &msg, "closure_edges");
        if !self.config.dump_path.is_empty() {
            use std::io::Write;
            let appended = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open(format!("{}.edges", self.config.dump_path))
                .and_then(|mut file| file.write_all(&msg.data));
            if let Err(e) = appended {
                warn!(error = %e, "Failed to dump the closure edges.");
            }
        }
        *sent = events.len();
    }

    /// Send fast-reform consumers every keyframe pose change since they last heard (an adopted solve).
    /// Changes too small to matter wait in `published` and add up.
    fn publish_pose_changes(
        &self,
        pgo: &GscPgo,
        published: &mut Vec<(Mat3, Vec3)>,
        closure_closed: bool,
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
        if !closure_closed && !moved {
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
