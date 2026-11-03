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
//!   closure_edges (each closure candidate's lifecycle), debug_log (JSON debugging numbers). Closure
//!   searches and pose-graph solves run on their own threads; publishes hop back onto the tokio
//!   runtime via a captured Handle.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config as PgoConfig, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::records::{
    append_pose_changes, build_closure_edge_records, build_loop_closure_records, build_odometry,
    build_pose_graph_records, extract_xyz, write_keyframes, HeardPoses,
};
use dimos_module::nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};
use dimos_module::{native_config, run_with_transport, Input, Module, Output, Tf, Transform};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::std_msgs;
use tracing::{error, info, warn};

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
    // bounded scan FIFO depth for spikes; overflow drops all but the newest scan (<= 0 = unbounded)
    max_scan_queue: i32,
    // threads closure search and solves may use, leaving the other cores to the rest of the robot (0 = every core)
    worker_threads: usize,
    ;
    mode: dimos_gsc_pgo::gsc_pgo::Mode,
    deterministic_config: dimos_gsc_pgo::gsc_pgo::DeterministicConfig,
    overshoot_warning_interval_s: f64,
    keyframe_min_rotation_degrees: f64,
    keyframe_min_distance_meters: f64,
    closure_search_radius: f64,
    closure_icp_inlier_distance_m: f64,
    closure_icp_max_fitness_m: f64,
    closure_icp_point_to_plane: bool,
    closure_icp_target_resolution_m: f64,
    closure_icp_early_stop: bool,
    closure_alternative_places_min_pressure: f64,
    drop_reseen_points: bool,
    auto_aggregate_cloud_size: usize,
    closure_target_extra_fraction: f64,
    closure_target_extra_keyframes: usize,
    submap_resolution: f64,
    closure_min_occupancy: i32,
    closure_min_degeneracy: f64,
    closure_yank_gate_width: f64,
    closure_yank_drift_range: f64,
    closure_extension_skip_unreachable: bool,
    closure_yank_pre_check: bool,
    closure_extension_stop_when_flat: bool,
    closure_extension_stop_on_swing: bool,
    barely_fail_uncertainty_range: f64,
    barely_pass_uncertainty_range: f64,
    decisive_pressure_rate: f64,
    extension_cloud_spacing: f64,
    compounding_agreement_sensitivity: f64,
    graph_change_heat_enabled: bool,
    graph_change_heat_threshold: f64,
    graph_change_heat_floor: f64,
    graph_change_heat_decay: f64,
    fast_match_workers: usize,
    sc_buffer_size: usize,
    verification_workers: usize,
    verification_buffer_slots: usize,
    solve_speed_window_poses: usize,
    closure_icp_dump_dir: String,
    closure_extended_icp_max_keyframes: usize,
    closure_robust_huber_k: f64,
    closure_icp_max_iterations: u64,
    closure_icp_max_correspondence_m: f64,
    closure_gnc_var_scale: f64,
    closure_gnc_inlier_probability: f64,
    closure_informative_by_graph_covariance: bool,
    closure_uninformative_gate: bool,
    odometry_error_estimate_xy_per_sqrt_m: f64,
    odometry_error_estimate_xy_per_sqrt_rad: f64,
    odometry_error_estimate_z_per_sqrt_m: f64,
    odometry_error_estimate_z_per_sqrt_rad: f64,
    odometry_error_estimate_yaw_per_sqrt_m: f64,
    odometry_error_estimate_yaw_per_sqrt_rad: f64,
    odometry_error_estimate_tilt_per_sqrt_m: f64,
    odometry_error_estimate_tilt_per_sqrt_rad: f64,
    odometry_error_estimate_from_lidar_range: bool,
    lidar_range: dimos_gsc_pgo::lidar_range::LidarRangeConfig,
    learned_odometry_error_estimate_prior_ln_sd_xy: f64,
    learned_odometry_error_estimate_prior_ln_sd_z: f64,
    learned_odometry_error_estimate_prior_ln_sd_yaw: f64,
    learned_odometry_error_estimate_prior_ln_sd_tilt: f64,
    learned_odometry_error_estimate_stiffness: f64,
    scan_context_num_rings: i32,
    scan_context_num_sectors: i32,
    scan_context_max_range_m: f64,
    scan_context_top_k: i32,
    scan_context_match_threshold: f64,
    scan_context_lidar_height_m: f64,
    closure_sharpness_half: f64,
    closure_sharpness_nudge_m: f64,
    closure_sharpness_nudge_deg: f64,
    closure_likelihood_confidence: bool,
    closure_alternative_places: bool,
    closure_likelihood_visibility: bool,
    learned_odometry_error_estimate_from_closures: bool,
    odometry_error_estimate_evidence_core: bool,
    pressure_check_frequency: usize,
    soft_pressure_half_log10_hazard: f64,
    soft_pressure_decades: f64,
}

// ---- shared state ---------------------------------------------------------------

#[derive(Clone)]
struct OdomSample {
    ts: f64,
    rotation: Mat3,
    translation: Vec3,
}

/// A landmark sighting in the body frame, waiting for the worker (`GscPgo::add_landmark_factor`).
struct LandmarkFix {
    ts: f64,
    landmark: u64,
    rotation: Mat3,
    position: Vec3,
    information: [[f64; 6]; 6],
}

/// A sensor's measured position in a fixed world frame, waiting for the worker (`GscPgo::add_coordinate_factor`).
struct WorldFix {
    ts: f64,
    frame: u32,
    lever: Vec3,
    position: Vec3,
    covariance: Mat3,
}

#[derive(Default)]
struct SharedState {
    /// World-position fixes not yet handed to the pose graph.
    world_fixes: Vec<WorldFix>,
    /// Each world frame's name, its index the frame id the pose graph knows it by.
    world_frames: Vec<String>,
    /// Landmark sightings not yet handed to the pose graph, and each landmark's name (its index the id).
    landmark_factors: Vec<LandmarkFix>,
    landmark_names: Vec<String>,
    latest_odom: Option<OdomSample>,
    /// Reject out-of-order scans.
    last_message_time: f64,
    /// Bounded scan FIFO.
    cloud_buffer: VecDeque<CloudWithPose>,
    /// A background pose-graph solve finished and waits to be adopted.
    solve_finished: bool,
    /// Whether the scans' frame was already found to be a fixed frame (warned once).
    fixed_frame_warned: bool,
}

type Shared<T> = Arc<Mutex<T>>;

/// A world-fixed frame name, which a lidar scan's frame should not be.
fn is_fixed_frame(frame_id: &str) -> bool {
    matches!(frame_id, "odom" | "map" | "world")
}

// ---- module ----------------------------------------------------------------------

#[derive(Module)]
#[module(setup = spawn_worker, teardown = stop_worker)]
struct GscPgoModule {
    #[input(decode = PointCloud2::decode, handler = on_cloud)]
    cloud: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    // A sensor's position in a fixed world frame (GPS, mocap, ...): header.frame_id names the world frame,
    // child_frame_id the sensor's mount (its offset from body_frame comes from tf), pose.covariance's xyz block (m²)
    // its uncertainty. Orientation and twist are ignored.
    #[input(decode = Odometry::decode, handler = on_coordinate_factor)]
    coordinate_factor: Input<Odometry>,

    // A landmark seen by a sensor (a camera's bearing with a rough range, a reflector, another robot's landmark):
    // header.frame_id is the sensor (placed on the body through tf), child_frame_id names the landmark (every sighting
    // with that name is of the same fixed pose), pose.pose is its pose in the sensor's frame and pose.covariance that
    // pose's uncertainty there; a dimension whose variance is not a positive finite number (ROS's -1, a huge value works
    // too) is unknown, so a reflector or a ranged bearing is a sighting whose rotation is unknown, and a zero
    // quaternion leaves the rotation unknown too. Twist is ignored.
    #[input(decode = Odometry::decode, handler = on_landmark_factor)]
    landmark_factor: Input<Odometry>,

    #[output(encode = Odometry::encode)]
    corrected_odometry: Output<Odometry>,

    // The map -> odometry correction.
    #[tf]
    tf: Tf,

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

    // Debugging numbers as JSON lines tagged with "kind" and "ts": "pressure" (every pressure_check_frequency-th scan),
    // "closure" (every closure search), "keyframe" (queue, threads, stage seconds), "solve" (each adopted solve). Pressure: {"ts", "unresolved_loop_probability", "expected_overlaps",
    // "most_likely_keyframe", "most_likely_probability"} (see gsc_pgo::loop_overlap).
    #[output(encode = std_msgs::String::encode)]
    debug_log: Output<std_msgs::String>,

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
            debug_log: self.debug_log.clone(),
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

    /// Handle an incoming odometry message: record it as the latest sample.
    async fn on_odometry(&mut self, msg: Odometry) {
        let q = &msg.pose.pose.orientation;
        let rotation = mat3::mat_from_quat(&[q.w, q.x, q.y, q.z]);
        let p = &msg.pose.pose.position;
        let translation = [p.x, p.y, p.z];
        let ts = msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 / 1e9;
        self.state.lock().expect("state").latest_odom = Some(OdomSample {
            ts,
            rotation,
            translation,
        });
    }

    async fn on_coordinate_factor(&mut self, msg: Odometry) {
        let ts = msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 / 1e9;
        let mount = &msg.child_frame_id;
        let body = &self.config.body_frame;
        let lever = if mount.is_empty() || mount == body {
            [0.0; 3]
        } else {
            // a fix placed through a guessed mount would bias the map, so one without its mount is dropped
            let Some(t) = self.tf.lookup(body, mount).at(ts).get() else {
                return;
            };
            let v = t.translation();
            [v.x, v.y, v.z]
        };
        let c = &msg.pose.covariance;
        let covariance = [
            [c[0], c[1], c[2]],
            [c[6], c[7], c[8]],
            [c[12], c[13], c[14]],
        ];
        let p = &msg.pose.pose.position;
        let position = [p.x, p.y, p.z];
        if !position
            .iter()
            .chain(covariance.iter().flatten())
            .all(|v| v.is_finite())
        {
            warn!(frame = %msg.header.frame_id, "world position not finite; dropped");
            return;
        }
        let mut state = self.state.lock().expect("state");
        let frame = match state
            .world_frames
            .iter()
            .position(|f| *f == msg.header.frame_id)
        {
            Some(i) => i,
            None => {
                state.world_frames.push(msg.header.frame_id.clone());
                state.world_frames.len() - 1
            }
        } as u32;
        state.world_fixes.push(WorldFix {
            ts,
            frame,
            lever,
            position,
            covariance,
        });
        drop(state);
        self.wake.notify_one();
    }

    async fn on_landmark_factor(&mut self, msg: Odometry) {
        let ts = msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 / 1e9;
        let sensor = &msg.header.frame_id;
        let body = &self.config.body_frame;
        let p = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        let c = &msg.pose.covariance;
        // ROS orders the covariance (x, y, z, rotation about x, y, z) with rotations about the sensor's axes; the pose
        // graph orders it (rotation about the landmark's own axes, position) and takes information, zero where unknown
        let ros = [3, 4, 5, 0, 1, 2];
        let mut information: [[f64; 6]; 6] =
            dimos_gsc_pgo::landmarks::known_inverse(&std::array::from_fn(|i| {
                std::array::from_fn(|j| c[6 * ros[i] + ros[j]])
            }));
        let norm = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
        let seen: Mat3 = if norm > 0.0 {
            mat3::mat_from_quat(&[q.w / norm, q.x / norm, q.y / norm, q.z / norm])
        } else {
            // no orientation: the rotation is unknown
            for (i, row) in information.iter_mut().enumerate() {
                for (j, v) in row.iter_mut().enumerate() {
                    if i < 3 || j < 3 {
                        *v = 0.0;
                    }
                }
            }
            mat3::identity()
        };
        let mut mount = mat3::identity();
        let (mut rotation, mut position) = (seen, [p.x, p.y, p.z]);
        if !sensor.is_empty() && sensor != body {
            // a sighting placed through a guessed mount would bias the map, so one without its mount is dropped
            let Some(t) = self.tf.lookup(body, sensor).at(ts).get() else {
                return;
            };
            let r = t.rotation().to_rotation_matrix();
            mount = std::array::from_fn(|i| std::array::from_fn(|j| r[(i, j)]));
            let v = t.translation();
            rotation = mat3::mat_mul(&mount, &seen);
            position = mat3::add(&mat3::mat_vec(&mount, &position), &[v.x, v.y, v.z]);
        }
        // the residual in the pose graph's terms is A times the message's, A = diag(seen^T, mount), so I <- A I A^T
        let seen_t = mat3::transpose(&seen);
        let a = |i: usize, k: usize| match (i < 3, k < 3) {
            (true, true) => seen_t[i][k],
            (false, false) => mount[i - 3][k - 3],
            _ => 0.0,
        };
        let information: [[f64; 6]; 6] = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                (0..6)
                    .flat_map(|k| (0..6).map(move |l| (k, l)))
                    .map(|(k, l)| a(i, k) * information[k][l] * a(j, l))
                    .sum()
            })
        });
        if !position
            .iter()
            .chain(rotation.iter().flatten())
            .chain(information.iter().flatten())
            .all(|v| v.is_finite())
        {
            warn!(landmark = %msg.child_frame_id, "landmark sighting not finite; dropped");
            return;
        }
        let mut state = self.state.lock().expect("state");
        let landmark = match state
            .landmark_names
            .iter()
            .position(|n| *n == msg.child_frame_id)
        {
            Some(i) => i,
            None => {
                state.landmark_names.push(msg.child_frame_id.clone());
                state.landmark_names.len() - 1
            }
        } as u64;
        state.landmark_factors.push(LandmarkFix {
            ts,
            landmark,
            rotation,
            position,
            information,
        });
        drop(state);
        self.wake.notify_one();
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

        // the lidar range (odometry_error_estimate_from_lidar_range) is measured from the scan's frame origin
        if !state.fixed_frame_warned && is_fixed_frame(&msg.header.frame_id) {
            state.fixed_frame_warned = true;
            warn!(frame = %msg.header.frame_id, "lidar scans should be in the sensor (or body) frame, not a fixed frame like odom, map or world");
        }
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
        });
        let cap = self.config.max_scan_queue;
        if cap > 0 && state.cloud_buffer.len() as i32 > cap {
            // the queue only absorbs spikes: once it overflows, keep just the newest scan so latency stays low
            let newest = state.cloud_buffer.pop_back();
            state.cloud_buffer.clear();
            state.cloud_buffer.extend(newest);
        }
        drop(state);
        self.wake.notify_one();
    }
}

// ---- the graph's publisher: pose_graph, loop_closure_event and the keyframe dumps, off the scan thread ----------------

/// What the publisher sends next: a snapshot of the graph version (O(1), `store::KeyFrames`), whether a closure closed
/// since the last one it sent (or-ed over any it skipped), and the data time.
struct Publish {
    keys: dimos_gsc_pgo::store::KeyFrames,
    closure_closed: bool,
    ts: f64,
}

/// The newest snapshot waiting to publish and whether the publisher should stop, with its wakeup.
type PendingPublish = Arc<(Mutex<(Option<Publish>, bool)>, Condvar)>;

/// The thread that publishes each keyframe's graph (every keyframe's pose: O(map)) and the pose changes consumers have
/// not heard (`HeardPoses`: O(map)), so the scan thread only hands it a snapshot; a newer snapshot replaces one not yet
/// sent, so a slow publish never queues stale graphs.
struct GraphPublisher {
    pending: PendingPublish,
}

impl GraphPublisher {
    fn spawn(worker: &Worker, frame_id: String) -> GraphPublisher {
        let pending: PendingPublish = Arc::default();
        let (shared, stop) = (Arc::clone(&pending), Arc::clone(&worker.stop));
        let (pose_graph, loop_closure_event) = (worker.pose_graph.clone(), worker.loop_closure_event.clone());
        let (rt, dump_path) = (worker.rt.clone(), worker.config.dump_path.clone());
        std::thread::Builder::new()
            .name("gsc-publish".into())
            .spawn(move || {
                let mut heard = HeardPoses::default();
                let publish = |out: &dyn Fn() -> Result<(), String>, what: &str| {
                    if let Err(e) = out() {
                        error!(error = %e, "{what} failed to publish");
                    }
                };
                loop {
                    let next = {
                        let (lock, ready) = &*shared;
                        let mut state = lock.lock().expect("publisher");
                        while state.0.is_none() && !state.1 {
                            let (guard, _) = ready
                                .wait_timeout(state, Duration::from_secs(1))
                                .expect("publisher");
                            state = guard;
                            if stop.load(Ordering::Relaxed) && state.0.is_none() {
                                return;
                            }
                        }
                        match state.0.take() {
                            Some(next) => next,
                            None => return,
                        }
                    };
                    let Publish { keys, closure_closed, ts } = next;
                    heard.add_new(&keys);
                    if let Some(changes) = heard.take_changes(&keys, closure_closed) {
                        let records = build_loop_closure_records(changes, ts, &frame_id);
                        publish(
                            &|| rt.block_on(loop_closure_event.publish(&records)).map_err(|e| e.to_string()),
                            "loop_closure_event",
                        );
                        if !dump_path.is_empty() {
                            let appended = std::fs::OpenOptions::new()
                                .create(true)
                                .append(true)
                                .open(format!("{dump_path}.closures"))
                                .and_then(|mut file| append_pose_changes(&mut file, ts, &records.data));
                            if let Err(e) = appended {
                                warn!(error = %e, "Failed to dump the loop closure.");
                            }
                        }
                    }
                    let records = build_pose_graph_records(&keys, ts, &frame_id);
                    publish(
                        &|| rt.block_on(pose_graph.publish(&records)).map_err(|e| e.to_string()),
                        "pose_graph",
                    );
                    if !dump_path.is_empty() {
                        if let Err(e) = write_keyframes(dump_path.as_ref(), &keys) {
                            warn!(error = %e, "Failed to dump the keyframes.");
                        }
                    }
                }
            })
            .expect("spawn graph publisher thread");
        GraphPublisher { pending }
    }
    /// Hand the publisher the graph as it is now (O(1)); an unsent older one is replaced, its closure flag kept.
    fn send(&self, pgo: &GscPgo, closure_closed: bool, ts: f64) {
        let (lock, ready) = &*self.pending;
        let mut state = lock.lock().expect("publisher");
        let closed = closure_closed || state.0.as_ref().is_some_and(|p| p.closure_closed);
        state.0 = Some(Publish {
            keys: pgo.key_poses().clone(),
            closure_closed: closed,
            ts,
        });
        ready.notify_one();
    }
}

impl Drop for GraphPublisher {
    fn drop(&mut self) {
        let (lock, ready) = &*self.pending;
        lock.lock().expect("publisher").1 = true;
        ready.notify_all();
    }
}

// ---- worker (the PGO main loop) -----------------------------------------------------

struct Worker {
    state: Shared<SharedState>,
    config: Config,
    corrected_odometry: Output<Odometry>,
    tf: Tf,
    pose_graph: Output<PointCloud2>,
    loop_closure_event: Output<PointCloud2>,
    closure_edges: Output<PointCloud2>,
    debug_log: Output<std_msgs::String>,
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
        self.publish_tf(
            &mat3::identity(),
            &[0.0; 3],
            now,
            &frame_id,
            &child_frame_id,
        );

        // Backstop for the Condvar wait so the stop flag is still observed even
        // if a notify is somehow missed; normal wakeups come from callbacks.
        let wake_timeout = Duration::from_secs(1);

        // the graph's publishing (pose_graph, loop_closure_event, the keyframe dumps) runs on its own thread off O(1)
        // snapshots of the graph version (`GraphPublisher`), the newest pending one replacing an unsent older one
        let publisher = GraphPublisher::spawn(&self, frame_id.clone());
        // The scan count of the last loop-overlap pressure sent on debug_log, and the closure searches sent there.
        let mut pressure_sent = 0usize;
        let mut closures_logged = 0usize;
        // Closure-edge events already sent on closure_edges.
        let mut edges_sent = 0usize;

        while !self.stop.load(Ordering::Relaxed) {
            // adopt a finished solve now, scans or not, once its hold has passed (`solve_hold_enabled`; else each scan's update)
            let solve_finished =
                std::mem::take(&mut self.state.lock().expect("state").solve_finished);
            if solve_finished && pgo.update() {
                let last_time = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
                publisher.send(&pgo, false, last_time);
                self.log_solve(&pgo, last_time);
            }
            // world-position fixes and landmark sightings join the graph as they arrive; one alone starts a solve
            let (fixes, sightings) = {
                let mut state = self.state.lock().expect("state");
                (
                    std::mem::take(&mut state.world_fixes),
                    std::mem::take(&mut state.landmark_factors),
                )
            };
            if !fixes.is_empty() || !sightings.is_empty() {
                for f in fixes {
                    pgo.add_coordinate_factor(f.ts, f.frame, f.lever, f.position, f.covariance);
                }
                for s in sightings {
                    pgo.add_landmark_factor(
                        s.ts,
                        s.landmark,
                        s.rotation,
                        s.position,
                        s.information,
                    );
                }
                if pgo.update() {
                    let last_time = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
                    publisher.send(&pgo, false, last_time);
                    self.log_solve(&pgo, last_time);
                }
            }
            self.publish_closure_edges(&pgo, &mut edges_sent, &frame_id);
            self.log_closures(&pgo, &mut closures_logged);
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
                if state.cloud_buffer.is_empty()
                    && !state.solve_finished
                    && state.world_fixes.is_empty()
                    && state.landmark_factors.is_empty()
                {
                    let _ = self.wake.wait_timeout(state, wake_timeout);
                }
                continue;
            };

            let cur_time = cloud_with_pose.pose.second;

            let keyframe = pgo.add_key_pose(&cloud_with_pose);
            // the loop-overlap pressure, each time it is recomputed (every pressure_check_frequency-th scan)
            if let Some((scan, overlap)) = pgo.pressure() {
                if scan != pressure_sent {
                    pressure_sent = scan;
                    self.log(
                        "pressure",
                        cur_time,
                        serde_json::to_value(overlap).unwrap_or_default(),
                    );
                }
            }
            if !keyframe {
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

            // the closure search's jobs that may land do (deterministic: their holds passed; fast: finished), so a closure
            // joins this keyframe's update, and this keyframe's match joins the pipeline
            pgo.step_closure_search(false);
            // a finished solve moves the keyframes
            let moved = pgo.update();
            // every keyframe's current pose and the pose changes since consumers last heard, published off this thread
            publisher.send(&pgo, moved, cur_time);
            if moved {
                self.log_solve(&pgo, cur_time);
            }

            self.publish_corrected(
                &pgo,
                &cloud_with_pose.pose,
                cur_time,
                &frame_id,
                &body_frame,
                &child_frame_id,
            );

            let scan_queue = self.state.lock().expect("state").cloud_buffer.len();
            self.log(
                "keyframe",
                cur_time,
                serde_json::json!({
                    "keyframes": pgo.key_poses().len(),
                    "scan_queue": scan_queue,
                    "closure_search_running": pgo.closure_search_running(),
                    "solve_running": pgo.solve_started().is_some(),
                    "stage_seconds": pgo.stage_seconds().iter().map(|(k, v)| (k.to_string(), *v)).collect::<std::collections::BTreeMap<_, _>>(),
                }),
            );
        }
        // stopping: settle the graph and report the odometry error estimate its kept loops measured, once
        pgo.finish();
        pgo.overshoot_report();
        let last_time = pgo.key_poses().last().map_or(0.0, |kp| kp.time);
        self.log_odometry_drift(&pgo, last_time);
    }

    /// One debug_log line: `fields` plus its `kind` and `ts`, as JSON.
    fn log(&self, kind: &str, ts: f64, mut fields: serde_json::Value) {
        fields["kind"] = kind.into();
        fields["ts"] = ts.into();
        let msg = std_msgs::String {
            data: fields.to_string(),
        };
        self.publish(&self.debug_log, &msg, "debug_log");
    }

    /// Every closure search since the last call on debug_log (`kind` "closure": the candidate, every gate input and the
    /// outcome, as `ClosureCandidateDiag`).
    fn log_closures(&self, pgo: &GscPgo, logged: &mut usize) {
        for diag in &pgo.closure_diagnostics()[*logged..] {
            self.log(
                "closure",
                diag.source_time,
                serde_json::to_value(diag).unwrap_or_default(),
            );
        }
        *logged = pgo.closure_diagnostics().len();
    }

    /// An adopted solve on debug_log (`kind` "solve"): the closures it kept and judged, and the graph's error.
    fn log_solve(&self, pgo: &GscPgo, ts: f64) {
        // a solve that measured the learned odometry error estimate (after a longer loop than any before) reports it
        if pgo.learned_odometry_error_estimate_measured() {
            self.log_odometry_drift(pgo, ts);
        }
        let closures = pgo.committed_closures();
        let judged = closures.iter().filter(|c| !c.gnc_weight.is_nan()).count();
        let kept = closures.iter().filter(|c| c.gnc_weight > 0.5).count();
        self.log(
            "solve",
            ts,
            serde_json::json!({
                "committed": closures.len(),
                "judged": judged,
                "kept": kept,
                "graph_error": pgo.graph_error(),
                "keyframes": pgo.key_poses().len(),
                // each landmark (its index among `landmark_factor`'s names) and each world frame, where the solve put it
                "landmarks": pgo.landmarks().iter().map(|(id, _, p)| serde_json::json!([id, p])).collect::<Vec<_>>(),
                "world_alignment": pgo.world_alignment().iter().map(|(f, yaw, t)| serde_json::json!([f, yaw, t])).collect::<Vec<_>>(),
            }),
        );
    }

    /// The odometry error estimate the kept loops measured, as the config inputs for the next run (`kind`
    /// "odometry_drift"): self-calibration, a source no kept loop measured repeating the configured drift. Measured once,
    /// when the module stops (`GscPgo::finish`).
    fn log_odometry_drift(&self, pgo: &GscPgo, ts: f64) {
        let kept = pgo
            .committed_closures()
            .iter()
            .filter(|c| c.gnc_weight > 0.5)
            .count();
        if kept > 0 {
            let (drift, measured) = pgo.odometry_error_estimate_for_next_run();
            let mut fields = serde_json::Map::new();
            for (k, source) in ["xy", "z", "yaw", "tilt"].iter().enumerate() {
                fields.insert(
                    format!("odometry_error_estimate_{source}_per_sqrt_m"),
                    drift[k][0].into(),
                );
                fields.insert(
                    format!("odometry_error_estimate_{source}_per_sqrt_rad"),
                    drift[k][1].into(),
                );
                fields.insert(format!("{source}_measured"), measured[k].into());
            }
            // beside it, the deterministic_config this run measured (`GscPgo::suggested_deterministic_config`), the exact
            // object to paste into the config, and each LET's measured / estimated (diagnostics only)
            if let Ok(suggested) = serde_json::to_value(pgo.suggested_deterministic_config()) {
                fields.insert("suggested_deterministic_config".into(), suggested);
            }
            let mut ratios = serde_json::Map::new();
            for (name, ratio, jobs, _) in pgo.runtime_profile() {
                ratios.insert(name.into(), serde_json::json!({ "measured_over_estimated_p99": ratio, "jobs": jobs }));
            }
            fields.insert("let_measured".into(), serde_json::Value::Object(ratios));
            self.log("odometry_drift", ts, serde_json::Value::Object(fields));
        }
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
        *sent = events.len();
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

        self.publish_tf(
            &pgo.rotation_offset(),
            &pgo.translation_offset(),
            ts,
            frame_id,
            child_frame_id,
        );
    }

    /// The map -> odometry correction on tf.
    fn publish_tf(&self, rotation: &Mat3, translation: &Vec3, ts: f64, parent: &str, child: &str) {
        let iso = Isometry3::from_parts(
            Translation3::new(translation[0], translation[1], translation[2]),
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(
                Matrix3::from_fn(|i, j| rotation[i][j]),
            )),
        );
        if let Err(e) = self
            .rt
            .block_on(self.tf.publish(&[Transform::new(parent, child, ts, iso)]))
        {
            error!(error = %e, "tf failed to publish");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<GscPgoModule>().await;
}
