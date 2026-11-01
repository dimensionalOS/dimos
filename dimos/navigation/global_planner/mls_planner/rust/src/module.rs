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

use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::mls_planner::{ms_since, Config, Planner, RegionBounds, DEBUG_LOG_PERIOD};
use crate::reform::Warp;
use crate::region_viz::{pack_cell, Cell, RegionUpdate, RegionViz, Segment};
use crate::voxel::{surface_point_xyz, voxelize, VoxelKey};
use dimos_module::time::now;
use dimos_module::{debug_throttled, error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, PointStamped, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::mpsc;
use tokio::sync::Notify;
use tracing::{debug, info, warn};

/// How often the planner logs its memory and container sizes.
const MEMORY_LOG_PERIOD: Duration = Duration::from_secs(10);

/// A point in the planner's world frame.
type Xyz = (f32, f32, f32);
type Xyzi = (f32, f32, f32, f32);

/// State shared between the handle loop and the worker.
pub(crate) type Shared<T> = Arc<Mutex<Option<T>>>;

/// A map input handed from the handle loop to the worker, its points already
/// extracted so the worker only mutates the graph. Only the newest is kept,
/// so a dropped intermediate frame is harmless.
enum MapUpdate {
    Region {
        points: Vec<Xyz>,
        bounds: PoseStamped,
        received: Instant,
    },
    Global {
        points: Vec<Xyz>,
        stamp: f64,
    },
}

/// One region of a seeded map, as the ray tracer hands them on.
struct SeedRegion {
    cloud: PointCloud2,
    bounds: PoseStamped,
}

/// A seed region with its points extracted, queued for the worker. They
/// queue, since every one must land.
struct QueuedSeed {
    points: Vec<Xyz>,
    bounds: PoseStamped,
}

/// One viz tick's content, read by the worker and published off it.
struct VizBatch {
    regions: Vec<(Cell, RegionUpdate)>,
    node_points: Vec<Xyz>,
}

/// Viz batches the publisher may hold while the worker moves on. With no
/// slot free the tick is skipped untaken, so its cells stay due.
const VIZ_QUEUE_CAPACITY: usize = 2;

/// How long half a seed region waits for its counterpart before it is dropped.
const SEED_PAIR_TIMEOUT: Duration = Duration::from_secs(10);

/// A seed queue quiet this long is taken as a finished load.
const SEED_SETTLE: Duration = Duration::from_secs(2);

/// How long queued seed regions may go unapplied, with live work filling
/// every pass, before the worker says so.
const SEED_STARVED_WARN_AFTER: Duration = Duration::from_secs(2);

/// The seed regions applied since the queue was last quiet, for the log.
#[derive(Default)]
struct SeedProgress {
    started: Option<Instant>,
    last_at: Option<Instant>,
    /// When regions were first seen queued with none applied yet.
    waiting_since: Option<Instant>,
    applied: usize,
    points: usize,
    max_region_ms: f64,
    sum_region_ms: f64,
    max_live_wait_ms: f64,
}

impl SeedProgress {
    fn in_flight(&self) -> bool {
        self.started.is_some()
    }

    fn settled(&self) -> bool {
        self.last_at.is_some_and(|at| at.elapsed() >= SEED_SETTLE)
    }

    fn mean_region_ms(&self) -> f64 {
        self.sum_region_ms / self.applied.max(1) as f64
    }

    /// Whether queued regions have gone unapplied for SEED_STARVED_WARN_AFTER,
    /// counted from the last one applied or from the first seen queued.
    fn starved(&self) -> bool {
        self.last_at
            .or(self.waiting_since)
            .is_some_and(|at| at.elapsed() >= SEED_STARVED_WARN_AFTER)
    }

    /// Regions are queued and none has been applied yet.
    fn note_waiting(&mut self) {
        if self.last_at.is_none() {
            self.waiting_since.get_or_insert(Instant::now());
        }
    }

    /// Count one region by the points it applied.
    fn record(&mut self, points: usize, region_ms: f64) {
        let now = Instant::now();
        self.started.get_or_insert(now);
        self.last_at = Some(now);
        self.applied += 1;
        self.points += points;
        self.max_region_ms = self.max_region_ms.max(region_ms);
        self.sum_region_ms += region_ms;
    }

    /// Track the longest a live region waited while seed regions load.
    fn record_live_wait(&mut self, wait_ms: f64) {
        if self.in_flight() {
            self.max_live_wait_ms = self.max_live_wait_ms.max(wait_ms);
        }
    }

    fn log_progress(&self, queued: usize) {
        debug_throttled!(
            Duration::from_millis(500),
            regions_done = self.applied,
            queued,
            points = self.points,
            max_region_ms = self.max_region_ms,
            mean_region_ms = self.mean_region_ms(),
            max_live_wait_ms = self.max_live_wait_ms,
            "Premap load in progress."
        );
    }

    /// Log the load's summary and start over.
    fn finish(&mut self) {
        if let (Some(started), Some(last_at)) = (self.started, self.last_at) {
            info!(
                regions = self.applied,
                points = self.points,
                load_s = last_at.duration_since(started).as_secs_f64(),
                max_region_ms = self.max_region_ms,
                mean_region_ms = self.mean_region_ms(),
                max_live_wait_ms = self.max_live_wait_ms,
                "Premap load finished."
            );
        }
        *self = Self::default();
    }
}

/// Seed clouds and bounds waiting for their counterpart, keyed by the region
/// number in their header seq, since every region of a seed shares one stamp.
#[derive(Default)]
struct SeedPairs {
    clouds: HashMap<i32, (PointCloud2, Instant)>,
    bounds: HashMap<i32, (PoseStamped, Instant)>,
}

impl SeedPairs {
    fn pair_cloud(&mut self, msg: PointCloud2) -> Option<SeedRegion> {
        self.expire_before(Instant::now());
        match self.bounds.remove(&msg.header.seq) {
            Some((bounds, _)) => Some(SeedRegion { cloud: msg, bounds }),
            None => {
                self.clouds.insert(msg.header.seq, (msg, Instant::now()));
                None
            }
        }
    }

    fn pair_bounds(&mut self, msg: PoseStamped) -> Option<SeedRegion> {
        self.expire_before(Instant::now());
        match self.clouds.remove(&msg.header.seq) {
            Some((cloud, _)) => Some(SeedRegion { cloud, bounds: msg }),
            None => {
                self.bounds.insert(msg.header.seq, (msg, Instant::now()));
                None
            }
        }
    }

    /// Drop halves that waited longer than the timeout as of `now`. A half
    /// without a counterpart means the other message was lost on the way.
    fn expire_before(&mut self, now: Instant) {
        let fresh = |at: &Instant| now.duration_since(*at) <= SEED_PAIR_TIMEOUT;
        let before = self.clouds.len() + self.bounds.len();
        self.clouds.retain(|_, (_, at)| fresh(at));
        self.bounds.retain(|_, (_, at)| fresh(at));
        let dropped = before - self.clouds.len() - self.bounds.len();
        if dropped > 0 {
            warn!(
                dropped,
                "Seed regions never paired, the other half was lost."
            );
        }
    }
}

#[derive(Module)]
#[module(name = "mls_planner", setup = spawn_worker, teardown = stop_worker)]
pub struct MlsPlanner {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = PoseStamped::decode, handler = on_region_bounds)]
    region_bounds: Input<PoseStamped>,

    // A seeded map's regions as the ray tracer lands them, applied through
    // the region pipeline between live updates. Live updates keep priority.
    #[input(decode = PointCloud2::decode, handler = on_seed_map)]
    seed_map: Input<PointCloud2>,

    #[input(decode = PoseStamped::decode, handler = on_seed_bounds)]
    seed_bounds: Input<PoseStamped>,

    #[input(decode = PointStamped::decode, handler = on_goal)]
    goal: Input<PointStamped>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    surface_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    nodes: Output<PointCloud2>,

    // The wire payload is a Path. dimos names the channel LineSegments3D.
    #[output(encode = Path::encode, msg = "LineSegments3D")]
    node_edges: Output<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    #[config]
    config: Config,

    intake: Intake,
    tasks: WorkerTasks,
}

impl MlsPlanner {
    async fn spawn_worker(&mut self) {
        let outputs = WorkerOutputs {
            surface_map: self.surface_map.clone(),
            nodes: self.nodes.clone(),
            node_edges: self.node_edges.clone(),
            path: self.path.clone(),
        };
        self.tasks = self
            .intake
            .spawn(self.tf.clone(), self.config.clone(), outputs, false, None);
    }

    async fn stop_worker(&mut self) {
        self.tasks.stop();
    }

    async fn on_global_map(&mut self, msg: PointCloud2) {
        self.intake.global_map(msg);
    }

    async fn on_local_map(&mut self, msg: PointCloud2) {
        self.intake.local_map(msg);
    }

    async fn on_region_bounds(&mut self, msg: PoseStamped) {
        self.intake.region_bounds(msg);
    }

    async fn on_seed_map(&mut self, msg: PointCloud2) {
        self.intake.seed_map(msg);
    }

    async fn on_seed_bounds(&mut self, msg: PoseStamped) {
        self.intake.seed_bounds(msg);
    }

    async fn on_goal(&mut self, msg: PointStamped) {
        self.intake.goal(msg);
    }
}

/// The outputs the worker and its viz publisher write.
pub(crate) struct WorkerOutputs {
    pub(crate) surface_map: Output<PointCloud2>,
    pub(crate) nodes: Output<PointCloud2>,
    pub(crate) node_edges: Output<Path>,
    pub(crate) path: Output<Path>,
}

/// The worker and viz publisher tasks a module runs.
#[derive(Default)]
pub(crate) struct WorkerTasks {
    worker: Option<tokio::task::JoinHandle<()>>,
    viz_publisher: Option<tokio::task::JoinHandle<()>>,
}

impl WorkerTasks {
    pub(crate) fn stop(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
        if let Some(handle) = self.viz_publisher.take() {
            handle.abort();
        }
    }
}

/// The handle loop's side of a planner module: inputs held until they pair,
/// and the handoffs to the worker, so the loop never blocks on map work.
#[derive(Default)]
pub(crate) struct Intake {
    // Held on the handle loop until stamps match, then handed off paired.
    pending_local: Option<PointCloud2>,
    pending_bounds: Option<PoseStamped>,
    pending_seeds: SeedPairs,

    // Written by the handle loop, read by the worker. Seed regions and warps
    // queue in arrival order, since every one must land.
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<QueuedSeed>>>,
    warps: Arc<Mutex<VecDeque<Warp>>>,
    active_goal: Shared<Xyz>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,
}

impl Intake {
    /// Start the worker and its viz publisher. `track_voxel_times` keeps when
    /// each voxel landed, which warps read.
    pub(crate) fn spawn(
        &self,
        tf: Tf,
        config: Config,
        outputs: WorkerOutputs,
        track_voxel_times: bool,
        dump_path: Option<String>,
    ) -> WorkerTasks {
        let (viz_tx, viz_rx) = mpsc::channel(VIZ_QUEUE_CAPACITY);
        let publisher = VizPublisher {
            batches: viz_rx,
            voxel_size: config.voxel_size,
            frame: config.world_frame.clone(),
            surface_map: outputs.surface_map,
            nodes: outputs.nodes,
            node_edges: outputs.node_edges,
        };
        let worker = Worker {
            pending: Arc::clone(&self.pending),
            seed_regions: Arc::clone(&self.seed_regions),
            warps: Arc::clone(&self.warps),
            active_goal: Arc::clone(&self.active_goal),
            goal_changed: Arc::clone(&self.goal_changed),
            wake: Arc::clone(&self.wake),
            tf,
            config,
            viz: viz_tx,
            path: outputs.path,
            track_voxel_times,
            dump_path,
        };
        WorkerTasks {
            viz_publisher: Some(tokio::spawn(publisher.run())),
            worker: Some(tokio::spawn(worker.run())),
        }
    }

    pub(crate) fn global_map(&mut self, msg: PointCloud2) {
        if let Some(points) = cloud_points(&msg, "global_map") {
            let stamp = seconds(&msg.header.stamp);
            self.hand_off(MapUpdate::Global { points, stamp });
        }
    }

    pub(crate) fn local_map(&mut self, msg: PointCloud2) {
        self.pending_local = Some(msg);
        self.try_pair();
    }

    pub(crate) fn region_bounds(&mut self, msg: PoseStamped) {
        self.pending_bounds = Some(msg);
        self.try_pair();
    }

    pub(crate) fn seed_map(&mut self, msg: PointCloud2) {
        if let Some(region) = self.pending_seeds.pair_cloud(msg) {
            self.queue_seed(region);
        }
    }

    pub(crate) fn seed_bounds(&mut self, msg: PoseStamped) {
        if let Some(region) = self.pending_seeds.pair_bounds(msg) {
            self.queue_seed(region);
        }
    }

    /// Queue a warp of the whole graph, applied before any other work.
    pub(crate) fn queue_warp(&self, warp: Warp) {
        self.warps.lock().expect("warp mutex").push_back(warp);
        self.wake.notify_one();
    }

    /// Hand off the local map and bounds once their stamps match.
    fn try_pair(&mut self) {
        if !stamps_paired(self.pending_bounds.as_ref(), self.pending_local.as_ref()) {
            return;
        }
        let bounds = self.pending_bounds.take().expect("checked above");
        let cloud = self.pending_local.take().expect("checked above");
        if let Some(points) = cloud_points(&cloud, "local_map") {
            let received = Instant::now();
            self.hand_off(MapUpdate::Region {
                points,
                bounds,
                received,
            });
        }
    }

    fn queue_seed(&self, region: SeedRegion) {
        let Some(points) = cloud_points(&region.cloud, "seed_map") else {
            return;
        };
        self.seed_regions
            .lock()
            .expect("seed mutex")
            .push_back(QueuedSeed {
                points,
                bounds: region.bounds,
            });
        self.wake.notify_one();
    }

    fn hand_off(&self, update: MapUpdate) {
        let replaced = self.pending.lock().expect("pending mutex").replace(update);
        match replaced {
            Some(MapUpdate::Region { received, .. }) => warn!(
                waited_ms = ms_since(received),
                "A live cloud was replaced before the worker took it."
            ),
            Some(MapUpdate::Global { .. }) => {
                warn!("A global map was replaced before the worker took it.")
            }
            None => {}
        }
        self.wake.notify_one();
    }

    /// Set or cancel the active goal from a click, then wake the worker.
    pub(crate) fn goal(&mut self, msg: PointStamped) {
        *self.active_goal.lock().expect("goal mutex") = goal_position(&msg.point);
        self.goal_changed.store(true, Ordering::SeqCst);
        self.wake.notify_one();
    }
}

fn seconds(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
}

/// True when bounds and a local cloud are both present with matching stamps.
fn stamps_paired(bounds: Option<&PoseStamped>, cloud: Option<&PointCloud2>) -> bool {
    match (bounds, cloud) {
        (Some(b), Some(c)) => same_stamp(&b.header.stamp, &c.header.stamp),
        _ => false,
    }
}

/// The goal position, or None when any coordinate is non-finite, which is the
/// cancel signal.
fn goal_position(p: &Point) -> Option<Xyz> {
    let goal = (p.x as f32, p.y as f32, p.z as f32);
    (goal.0.is_finite() && goal.1.is_finite() && goal.2.is_finite()).then_some(goal)
}

/// Owns the planner graph and does map mutation, publishing, and replanning
/// off the handle loop. Woken by the handlers. Each pass takes a live update
/// or goal first, else one seed region, else waits.
struct Worker {
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<QueuedSeed>>>,
    warps: Arc<Mutex<VecDeque<Warp>>>,
    active_goal: Shared<Xyz>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,
    tf: Tf,
    config: Config,
    viz: mpsc::Sender<VizBatch>,
    path: Output<Path>,
    track_voxel_times: bool,
    // Where the map state goes every MEMORY_LOG_PERIOD, for offline checks.
    dump_path: Option<String>,
}

/// Builds and publishes viz messages from the worker's ticks, so the worker
/// only reads the graph and goes back to its queue.
struct VizPublisher {
    batches: mpsc::Receiver<VizBatch>,
    voxel_size: f32,
    frame: String,
    surface_map: Output<PointCloud2>,
    nodes: Output<PointCloud2>,
    node_edges: Output<Path>,
}

impl VizPublisher {
    async fn run(mut self) {
        while let Some(batch) = self.batches.recv().await {
            let stamp = now();
            for (cell, update) in batch.regions {
                if let Some(points) = update.surface {
                    let surface = tokio::task::block_in_place(|| {
                        surface_message(cell, points, self.voxel_size, &self.frame, stamp.clone())
                    });
                    publish_cloud(&self.surface_map, &surface).await;
                }
                if let Some(segments) = update.segments {
                    let edges = tokio::task::block_in_place(|| {
                        edges_message(cell, segments, self.voxel_size, &self.frame, stamp.clone())
                    });
                    publish_path(&self.node_edges, &edges).await;
                }
            }
            let node_cloud = build_pc2_xyz(&batch.node_points, &self.frame, stamp);
            publish_cloud(&self.nodes, &node_cloud).await;
        }
    }
}

impl Worker {
    async fn run(self) {
        let mut planner = Planner::new(self.config.worker_threads);
        if self.track_voxel_times {
            planner.track_voxel_times();
        }
        let mut last_path_at: Option<Instant> = None;
        let mut last_viz_at: Option<Instant> = None;
        let mut viz = RegionViz::new(
            (self.config.viz_cell_m / self.config.voxel_size).round() as i32,
            self.config.viz_reach_cells(),
            self.config.viz_sweep_cells as usize,
        );
        let mut seed_progress = SeedProgress::default();
        let mut last_memory_log = Instant::now();
        loop {
            if self.track_voxel_times && last_memory_log.elapsed() >= MEMORY_LOG_PERIOD {
                last_memory_log = Instant::now();
                self.log_memory(&planner);
            }
            // A warp moves the frame everything else is in, so it goes first.
            let warp = self.warps.lock().expect("warp mutex").pop_front();
            if let Some(warp) = warp {
                tokio::task::block_in_place(|| self.apply_warp(&mut planner, &warp, &mut viz));
                self.publish_viz_if_due(&planner, &mut viz, &mut last_viz_at)
                    .await;
                self.maybe_replan(&mut planner, &mut last_path_at).await;
                continue;
            }
            let goal_changed = self.goal_changed.swap(false, Ordering::SeqCst);
            let update = self.pending.lock().expect("pending mutex").take();
            if update.is_some() || goal_changed {
                if let Some(MapUpdate::Region { received, .. }) = &update {
                    seed_progress.record_live_wait(ms_since(*received));
                }
                let live_update = match update {
                    Some(update) => {
                        self.apply_update(&mut planner, update, &mut viz, &mut last_viz_at)
                            .await
                    }
                    None => false,
                };
                if replan_due(goal_changed, live_update) {
                    self.maybe_replan(&mut planner, &mut last_path_at).await;
                }
                self.warn_if_seeds_starved(&mut seed_progress);
                continue;
            }

            let (seed, queued) = {
                let mut queue = self.seed_regions.lock().expect("seed mutex");
                (queue.pop_front(), queue.len())
            };
            if let Some(seed) = seed {
                if !seed_progress.in_flight() {
                    info!("Premap load started.");
                }
                let points = seed.points.len();
                let region_start = Instant::now();
                tokio::task::block_in_place(|| self.ingest_seed(&mut planner, seed, &mut viz));
                let region_ms = ms_since(region_start);
                seed_progress.record(points, region_ms);
                seed_progress.log_progress(queued);
                self.publish_viz_if_due(&planner, &mut viz, &mut last_viz_at)
                    .await;
                tokio::task::yield_now().await;
                continue;
            }

            // A seed whose queue stays quiet has finished.
            if seed_progress.in_flight() {
                let _ = tokio::time::timeout(SEED_SETTLE, self.wake.notified()).await;
                if seed_progress.settled() {
                    seed_progress.finish();
                }
            } else {
                // Wake at least once a period, so the memory log and dump keep up when idle.
                let _ = tokio::time::timeout(MEMORY_LOG_PERIOD, self.wake.notified()).await;
            }
        }
    }

    /// Carry the graph and the active goal through a warp.
    fn apply_warp(&self, planner: &mut Planner, warp: &Warp, viz: &mut RegionViz) {
        let started = Instant::now();
        // The lock is not held through the warp, so a click lands meanwhile.
        // A goal set during the warp is already in the new frame and stays.
        let before = *self.active_goal.lock().expect("goal mutex");
        let mut held: Vec<Xyz> = before.iter().copied().collect();
        let report = planner.apply_warp(warp, &mut held, &self.config);
        let mut goal = self.active_goal.lock().expect("goal mutex");
        if *goal == before {
            *goal = held.first().copied();
        }
        drop(goal);
        viz.mark_all();
        let (median_shift_m, max_shift_m) = warp.keyframe_shift_m();
        warn!(
            total_ms = ms_since(started),
            warp_ms = report.warp_ms,
            move_ms = report.move_ms,
            keyframes = warp.keyframes(),
            keyframes_moved = warp.moved_keyframes(),
            keyframes_blended = warp.blended_keyframes(),
            median_shift_m,
            max_shift_m,
            voxels_warped = report.voxels_warped,
            cells_warped = report.cells_warped,
            nodes = report.nodes,
            "Loop closure applied in place."
        );
        if let Some((from, to)) = report.worst_implausible {
            warn!(
                held_in_place = report.implausible,
                limit_m = warp.max_plausible_move_m(),
                worst_from = ?from,
                worst_to = ?to,
                "A warp sent points somewhere non-finite or implausibly far; they stayed put."
            );
        }
    }

    /// Resident memory and the containers that hold it, every MEMORY_LOG_PERIOD.
    fn log_memory(&self, planner: &Planner) {
        let sizes = planner.sizes();
        warn!(
            rss_mb = crate::rss::rss_mb().unwrap_or(f64::NAN),
            voxels = sizes.voxels,
            column_tiles = sizes.column_tiles,
            surface_cells = sizes.surface_cells,
            surface_slots = sizes.surface_slots,
            surface_edges = sizes.surface_edges,
            nodes = sizes.nodes,
            node_edges = sizes.node_edges,
            dijkstra_slots = sizes.dijkstra_slots,
            voxel_times = sizes.voxel_times,
            queued_warps = self.warps.lock().expect("warp mutex").len(),
            queued_seeds = self.seed_regions.lock().expect("seed mutex").len(),
            "Planner memory."
        );
        if let Some(path) = &self.dump_path {
            let (voxels, surface) = planner.dump_centers(self.config.voxel_size);
            for (suffix, values) in [("voxels", voxels), ("surface", surface)] {
                if let Err(e) = crate::rss::dump_f32(&format!("{path}.{suffix}"), &values) {
                    warn!(error = %e, path, "Failed to dump the planner state.");
                }
            }
        }
    }

    /// Live work has kept seed regions queued and unapplied for too long.
    fn warn_if_seeds_starved(&self, progress: &mut SeedProgress) {
        let queued = self.seed_regions.lock().expect("seed mutex").len();
        if queued == 0 {
            return;
        }
        progress.note_waiting();
        if progress.starved() {
            warn_throttled!(
                SEED_STARVED_WARN_AFTER,
                queued,
                regions_done = progress.applied,
                "Premap load is starved: live updates have kept the worker busy.",
            );
        }
    }

    /// Apply one live update and refresh the viz artifacts.
    async fn apply_update(
        &self,
        planner: &mut Planner,
        update: MapUpdate,
        viz: &mut RegionViz,
        last_viz_at: &mut Option<Instant>,
    ) -> bool {
        let applied = tokio::task::block_in_place(|| self.ingest(planner, update, viz));
        if applied {
            self.publish_viz_if_due(planner, viz, last_viz_at).await;
        }
        applied
    }

    /// Publish the nodes and the surface and edge cells due this tick, at
    /// most viz_publish_hz.
    async fn publish_viz_if_due(
        &self,
        planner: &Planner,
        viz: &mut RegionViz,
        last_viz_at: &mut Option<Instant>,
    ) {
        if self.config.viz_publish_hz <= 0.0 {
            return;
        }
        let tick_at = Instant::now();
        let viz_interval = Duration::from_secs_f32(1.0 / self.config.viz_publish_hz);
        if last_viz_at.is_some_and(|t| tick_at.duration_since(t) < viz_interval) {
            return;
        }
        let Ok(permit) = self.viz.try_reserve() else {
            debug_throttled!(
                DEBUG_LOG_PERIOD,
                "viz tick skipped, the publisher is behind"
            );
            return;
        };
        let near = voxelize(
            self.base_position().unwrap_or_default(),
            self.config.voxel_size,
        );
        let (regions, node_points) = tokio::task::block_in_place(|| {
            let regions = viz.tick(
                planner.surface_clearance_iter(),
                planner.edge_segment_iter(),
                near,
            );
            let node_points: Vec<Xyz> = planner.graph().nodes.iter().map(|n| n.pos).collect();
            (regions, node_points)
        });
        *last_viz_at = Some(tick_at);
        permit.send(VizBatch {
            regions,
            node_points,
        });
    }

    /// Mutate the graph from a map update. False when a region has no base
    /// pose on tf or the global map is empty.
    fn ingest(&self, planner: &mut Planner, update: MapUpdate, viz: &mut RegionViz) -> bool {
        match update {
            MapUpdate::Region {
                points,
                bounds,
                received,
            } => {
                let Some((_, _, sensor_z)) = self.base_position() else {
                    warn!(
                        world_frame = %self.config.world_frame,
                        base_frame = %self.config.base_frame,
                        "No base pose on tf, dropped a region update.",
                    );
                    return false;
                };
                let region = region_bounds(&bounds).capped_at(sensor_z, self.config.max_overhead_m);
                planner.set_clock(seconds(&bounds.header.stamp));
                let process_start = Instant::now();
                if let Some(window) = planner.update_region(&points, &region, &self.config) {
                    viz.mark_window(window);
                }
                debug_throttled!(
                    DEBUG_LOG_PERIOD,
                    process_ms = ms_since(process_start),
                    wait_ms = process_start.duration_since(received).as_secs_f64() * 1e3,
                    points = points.len(),
                    "local region processed"
                );
                true
            }
            MapUpdate::Global { points, stamp } => {
                if points.is_empty() {
                    return false;
                }
                planner.set_clock(stamp);
                planner.update_global_map(&points, &self.config);
                viz.mark_all();
                debug!(global_map_points = points.len(), "global_map processed");
                true
            }
        }
    }

    /// Apply one seed region through the region pipeline. Its bounds are the
    /// premap's own, so no sensor ceiling applies.
    fn ingest_seed(&self, planner: &mut Planner, seed: QueuedSeed, viz: &mut RegionViz) {
        let region = region_bounds(&seed.bounds);
        planner.set_clock(seconds(&seed.bounds.header.stamp));
        if let Some(window) = planner.update_seed_region(&seed.points, &region, &self.config) {
            viz.mark_window(window);
        }
    }

    /// The base frame position in the world frame, from the latest tf.
    fn base_position(&self) -> Option<Xyz> {
        let t = self
            .tf
            .get_latest(&self.config.world_frame, &self.config.base_frame)?
            .translation();
        Some((t.x as f32, t.y as f32, t.z as f32))
    }

    /// Gate and publish a replan. The planning itself lives in Planner::plan.
    async fn maybe_replan(&self, planner: &mut Planner, last_path_at: &mut Option<Instant>) {
        let Some(start) = self.base_position() else {
            return;
        };
        let start = (start.0, start.1, start.2 - self.config.start_z_offset_m);
        let goal = {
            let mut guard = self.active_goal.lock().expect("goal mutex");
            let Some(goal) = *guard else {
                return;
            };
            if is_at_goal(start, goal, self.config.goal_tolerance) {
                *guard = None;
                return;
            }
            goal
        };

        let plan_start = Instant::now();
        let waypoints =
            tokio::task::block_in_place(|| planner.plan_or_truncate(start, goal, &self.config));
        if waypoints.is_empty() {
            // No full path and nothing safe ahead on the cached path, so stop.
            publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
            return;
        }
        let plan_ms = ms_since(plan_start);
        let produced = Instant::now();
        let since_last_ms = last_path_at.map_or(-1.0, |t| (produced - t).as_secs_f64() * 1e3);
        *last_path_at = Some(produced);

        let stamp = now();
        let path_msg = build_path_from_waypoints(&waypoints, &self.config.world_frame, stamp);
        debug_throttled!(
            DEBUG_LOG_PERIOD,
            waypoints = waypoints.len(),
            plan_ms,
            since_last_ms,
            "path planned"
        );
        publish_path(&self.path, &path_msg).await;
    }
}

/// Whether a worker pass replans. Seed regions never trigger one, so a pass
/// that only applied a seed region does not.
fn replan_due(goal_changed: bool, live_update_applied: bool) -> bool {
    goal_changed || live_update_applied
}

/// The region a bounds message describes: position is the center, orientation
/// carries radius, z_min and z_max.
fn region_bounds(msg: &PoseStamped) -> RegionBounds {
    RegionBounds {
        origin_x: msg.pose.position.x as f32,
        origin_y: msg.pose.position.y as f32,
        radius: msg.pose.orientation.x as f32,
        z_min: msg.pose.orientation.y as f32,
        z_max: msg.pose.orientation.z as f32,
    }
}

/// One cell's surface as a message, the cell in its header seq.
fn surface_message(
    cell: Cell,
    surface: Vec<(VoxelKey, f32)>,
    voxel_size: f32,
    frame: &str,
    stamp: Time,
) -> PointCloud2 {
    let points: Vec<Xyzi> = surface
        .into_iter()
        .map(|((ix, iy, iz), clearance)| {
            let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
            (x, y, z, clearance)
        })
        .collect();
    let mut msg = build_pc2_xyzi(&points, frame, stamp);
    msg.header.seq = pack_cell(cell);
    msg
}

/// One cell's edge corridors as a message, the cell in its header seq.
fn edges_message(
    cell: Cell,
    segments: Vec<Segment>,
    voxel_size: f32,
    frame: &str,
    stamp: Time,
) -> Path {
    let mut msg = build_segments_path(segments, voxel_size, frame, stamp);
    msg.header.seq = pack_cell(cell);
    msg
}

/// True if within tolerance of the goal on the ground plane.
fn is_at_goal(start: Xyz, goal: Xyz, tol: f32) -> bool {
    (start.0 - goal.0).hypot(start.1 - goal.1) < tol
}

fn same_stamp(a: &Time, b: &Time) -> bool {
    a.sec == b.sec && a.nsec == b.nsec
}

async fn publish_cloud(out: &Output<PointCloud2>, cloud: &PointCloud2) {
    if let Err(e) = out.publish(cloud).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "Cloud failed to publish",
        );
    }
}

async fn publish_path(out: &Output<Path>, msg: &Path) {
    if let Err(e) = out.publish(msg).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "Path failed to publish",
        );
    }
}

fn header(frame_id: &str, stamp: Time) -> Header {
    Header {
        seq: 0,
        stamp,
        frame_id: frame_id.into(),
    }
}

fn pose_at(xyz: (f32, f32, f32), orient_w: f64) -> Pose {
    Pose {
        position: Point {
            x: xyz.0 as f64,
            y: xyz.1 as f64,
            z: xyz.2 as f64,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: orient_w,
        },
    }
}

fn pose_stamped(xyz: (f32, f32, f32), orient_w: f64, frame_id: &str, stamp: Time) -> PoseStamped {
    PoseStamped {
        header: header(frame_id, stamp),
        pose: pose_at(xyz, orient_w),
    }
}

fn empty_path(frame_id: &str, stamp: Time) -> Path {
    Path {
        header: header(frame_id, stamp),
        poses: Vec::new(),
    }
}

fn build_path_from_waypoints(waypoints: &[(f32, f32, f32)], frame_id: &str, stamp: Time) -> Path {
    let poses: Vec<PoseStamped> = waypoints
        .iter()
        .map(|&w| pose_stamped(w, 1.0, frame_id, stamp.clone()))
        .collect();
    Path {
        header: header(frame_id, stamp),
        poses,
    }
}

/// Emit edges as alternating PoseStamped pairs with orientation.w carrying
/// the per-edge cost.
fn build_segments_path(
    segments: Vec<(VoxelKey, VoxelKey, f32)>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> Path {
    let mut poses: Vec<PoseStamped> = Vec::with_capacity(segments.len() * 2);
    for (a, b, cost) in segments {
        let pa = surface_point_xyz(a.0, a.1, a.2, voxel_size);
        let pb = surface_point_xyz(b.0, b.1, b.2, voxel_size);
        poses.push(pose_stamped(pa, cost as f64, frame_id, stamp.clone()));
        poses.push(pose_stamped(pb, cost as f64, frame_id, stamp.clone()));
    }
    Path {
        header: header(frame_id, stamp),
        poses,
    }
}

/// Like `build_pc2_xyz` plus an `intensity` float carrying the cell's wall clearance.
fn build_pc2_xyzi(points: &[Xyzi], frame_id: &str, stamp: Time) -> PointCloud2 {
    let n = points.len() as i32;
    let mut data = Vec::with_capacity(points.len() * 16);
    for &(x, y, z, i) in points {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&i.to_le_bytes());
    }
    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: header(frame_id, stamp),
        height: 1,
        width: n,
        fields: vec![
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: true,
    }
}

fn build_pc2_xyz(points: &[(f32, f32, f32)], frame_id: &str, stamp: Time) -> PointCloud2 {
    let n = points.len() as i32;
    let mut data = Vec::with_capacity(points.len() * 12);
    for &(x, y, z) in points {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
    }
    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    PointCloud2 {
        header: header(frame_id, stamp),
        height: 1,
        width: n,
        fields: vec![make_field("x", 0), make_field("y", 4), make_field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * n,
        data,
        is_dense: true,
    }
}

struct ExtractError(&'static str);
impl std::fmt::Display for ExtractError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

/// The xyz points of a cloud, or None with a warning when it cannot be read.
fn cloud_points(msg: &PointCloud2, topic: &'static str) -> Option<Vec<Xyz>> {
    match extract_xyz(msg) {
        Ok(points) => Some(points),
        Err(e) => {
            warn_throttled!(
                Duration::from_secs(1),
                error = %e,
                topic,
                "Failed to extract cloud points, dropped it.",
            );
            None
        }
    }
}

fn extract_xyz(msg: &PointCloud2) -> Result<Vec<(f32, f32, f32)>, ExtractError> {
    let mut x_off: Option<usize> = None;
    let mut y_off: Option<usize> = None;
    let mut z_off: Option<usize> = None;
    for f in &msg.fields {
        if f.datatype != PointField::FLOAT32 as u8 {
            continue;
        }
        match f.name.as_str() {
            "x" => x_off = Some(f.offset as usize),
            "y" => y_off = Some(f.offset as usize),
            "z" => z_off = Some(f.offset as usize),
            _ => {}
        }
    }
    let xo = x_off.ok_or(ExtractError("missing float32 x field"))?;
    let yo = y_off.ok_or(ExtractError("missing float32 y field"))?;
    let zo = z_off.ok_or(ExtractError("missing float32 z field"))?;

    let n = (msg.width as usize) * (msg.height as usize);
    let step = msg.point_step as usize;
    if step == 0 {
        return Err(ExtractError("point_step is 0"));
    }
    if msg.data.len() < n * step {
        return Err(ExtractError(
            "data buffer shorter than width*height*point_step",
        ));
    }
    if xo + 4 > step || yo + 4 > step || zo + 4 > step {
        return Err(ExtractError(
            "xyz field offsets do not fit within point_step",
        ));
    }
    if msg.is_bigendian {
        return Err(ExtractError("big-endian point data not supported"));
    }

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let base = i * step;
        let x = read_f32_le(&msg.data, base + xo);
        let y = read_f32_le(&msg.data, base + yo);
        let z = read_f32_le(&msg.data, base + zo);
        if x.is_finite() && y.is_finite() && z.is_finite() {
            out.push((x, y, z));
        }
    }
    Ok(out)
}

#[inline]
fn read_f32_le(buf: &[u8], off: usize) -> f32 {
    let bytes: [u8; 4] = buf[off..off + 4]
        .try_into()
        .expect("bounds checked by caller");
    f32::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_at_goal_respects_tolerance_and_ignores_z() {
        assert!(is_at_goal((0.0, 0.0, 0.0), (0.05, 0.0, 9.0), 0.1));
        assert!(!is_at_goal((0.0, 0.0, 0.0), (0.2, 0.0, 0.0), 0.1));
    }

    fn stamped(stamp: Time) -> Header {
        Header {
            stamp,
            ..Default::default()
        }
    }

    fn bounds_at(stamp: Time) -> PoseStamped {
        PoseStamped {
            header: stamped(stamp),
            ..Default::default()
        }
    }

    fn cloud_at(stamp: Time) -> PointCloud2 {
        PointCloud2 {
            header: stamped(stamp),
            ..Default::default()
        }
    }

    #[test]
    fn stamps_paired_only_when_both_present_and_stamps_match() {
        let s = Time { sec: 2, nsec: 3 };
        let b = bounds_at(s.clone());
        let c = cloud_at(s);
        assert!(stamps_paired(Some(&b), Some(&c)));

        let other = cloud_at(Time { sec: 2, nsec: 4 });
        assert!(!stamps_paired(Some(&b), Some(&other)));

        assert!(!stamps_paired(Some(&b), None));
        assert!(!stamps_paired(None, Some(&c)));
        assert!(!stamps_paired(None, None));
    }

    #[test]
    fn seed_pairs_match_on_seq_whichever_side_lands_first() {
        let stamp = Time { sec: 2, nsec: 3 };
        let mut pairs = SeedPairs::default();
        let mut b1 = bounds_at(stamp.clone());
        b1.header.seq = 1;
        let mut b2 = bounds_at(stamp.clone());
        b2.header.seq = 2;
        let mut c1 = cloud_at(stamp.clone());
        c1.header.seq = 1;
        let mut c2 = cloud_at(stamp);
        c2.header.seq = 2;

        assert!(pairs.pair_bounds(b1).is_none());
        assert!(pairs.pair_bounds(b2).is_none());
        let first = pairs
            .pair_cloud(c1)
            .expect("region 1 pairs with its own bounds");
        assert_eq!(first.bounds.header.seq, 1);
        let second = pairs
            .pair_cloud(c2)
            .expect("region 2 pairs with its own bounds");
        assert_eq!(second.bounds.header.seq, 2);
        assert!(pairs.clouds.is_empty() && pairs.bounds.is_empty());
    }

    #[test]
    fn seed_pairs_match_cloud_first_and_interleaved() {
        let stamp = Time { sec: 2, nsec: 3 };
        let mut pairs = SeedPairs::default();
        let mut c1 = cloud_at(stamp.clone());
        c1.header.seq = 1;
        let mut b1 = bounds_at(stamp.clone());
        b1.header.seq = 1;
        assert!(pairs.pair_cloud(c1).is_none());
        assert_eq!(
            pairs
                .pair_bounds(b1)
                .expect("cloud first pairs")
                .cloud
                .header
                .seq,
            1
        );

        // Interleaved: cloud 2, bounds 3, bounds 2, cloud 3.
        let mut c2 = cloud_at(stamp.clone());
        c2.header.seq = 2;
        let mut b3 = bounds_at(stamp.clone());
        b3.header.seq = 3;
        let mut b2 = bounds_at(stamp.clone());
        b2.header.seq = 2;
        let mut c3 = cloud_at(stamp);
        c3.header.seq = 3;
        assert!(pairs.pair_cloud(c2).is_none());
        assert!(pairs.pair_bounds(b3).is_none());
        assert_eq!(pairs.pair_bounds(b2).expect("region 2").cloud.header.seq, 2);
        assert_eq!(pairs.pair_cloud(c3).expect("region 3").bounds.header.seq, 3);
        assert!(pairs.clouds.is_empty() && pairs.bounds.is_empty());
    }

    #[test]
    fn region_messages_carry_the_cell_in_their_headers_even_when_empty() {
        let cell = (1, -2);
        let surface = surface_message(cell, vec![((1, 2, 0), 0.5)], 0.1, "odom", Time::default());
        let segments = vec![((1, 2, 0), (2, 2, 0), 1.5)];
        let edges = edges_message(cell, segments, 0.1, "odom", Time::default());
        assert_eq!(surface.header.seq, pack_cell(cell));
        assert_eq!(edges.header.seq, pack_cell(cell));
        assert_eq!((surface.width, edges.poses.len()), (1, 2));

        let surface = surface_message(cell, Vec::new(), 0.1, "odom", Time::default());
        let edges = edges_message(cell, Vec::new(), 0.1, "odom", Time::default());
        assert_eq!(surface.header.seq, pack_cell(cell));
        assert_eq!(edges.header.seq, pack_cell(cell));
        assert_eq!((surface.width, edges.poses.len()), (0, 0));
    }

    #[test]
    fn an_unpaired_seed_half_expires_after_the_timeout() {
        let stamp = Time { sec: 2, nsec: 3 };
        let mut pairs = SeedPairs::default();
        let mut b1 = bounds_at(stamp.clone());
        b1.header.seq = 1;
        assert!(pairs.pair_bounds(b1).is_none());
        pairs.expire_before(Instant::now() + SEED_PAIR_TIMEOUT / 2);
        assert_eq!(pairs.bounds.len(), 1);
        pairs.expire_before(Instant::now() + SEED_PAIR_TIMEOUT * 2);
        assert!(pairs.bounds.is_empty());
        let mut c1 = cloud_at(stamp);
        c1.header.seq = 1;
        assert!(
            pairs.pair_cloud(c1).is_none(),
            "the expired bounds no longer pair"
        );
    }

    #[test]
    fn seed_progress_counts_applied_regions() {
        const REGIONS: usize = 20;
        let mut progress = SeedProgress::default();
        assert!(!progress.in_flight());
        progress.record(100, 1.0);
        assert!(progress.in_flight());
        for i in 2..REGIONS {
            progress.record(100, i as f64);
        }
        progress.record(100, 0.5);
        assert_eq!(progress.applied, REGIONS);
        assert_eq!(progress.points, 100 * REGIONS);
        assert_eq!(progress.max_region_ms, (REGIONS - 1) as f64);
        let sum: f64 = (1..REGIONS).map(|i| i as f64).sum::<f64>() + 0.5;
        assert!((progress.mean_region_ms() - sum / REGIONS as f64).abs() < 1e-9);

        progress.finish();
        assert!(!progress.in_flight());
        assert_eq!(progress.applied, 0);
    }

    #[test]
    fn seed_progress_is_starved_from_the_first_queued_region_or_the_last_applied() {
        let old = Instant::now() - SEED_STARVED_WARN_AFTER;
        let mut progress = SeedProgress::default();
        assert!(!progress.starved());

        progress.note_waiting();
        assert!(!progress.starved());
        progress.waiting_since = Some(old);
        assert!(progress.starved());

        progress.record(100, 1.0);
        assert!(!progress.starved());
        progress.last_at = Some(old);
        assert!(progress.starved());

        progress.finish();
        assert!(!progress.starved());
    }

    #[test]
    fn a_pass_replans_on_a_goal_change_or_a_live_update_only() {
        // goal changed, live update applied, replans
        let passes = [
            (true, true, true),
            (true, false, true),
            (false, true, true),
            // A pass that only applied a seed region, or an idle wake.
            (false, false, false),
        ];
        for (goal_changed, live_update_applied, replans) in passes {
            assert_eq!(
                replan_due(goal_changed, live_update_applied),
                replans,
                "goal_changed {goal_changed}, live_update_applied {live_update_applied}"
            );
        }
    }

    fn point(x: f64, y: f64, z: f64) -> Point {
        Point { x, y, z }
    }

    #[test]
    fn goal_position_passes_finite_and_cancels_on_non_finite() {
        assert_eq!(goal_position(&point(1.0, 2.0, 3.0)), Some((1.0, 2.0, 3.0)));
        assert_eq!(goal_position(&point(f64::NAN, 0.0, 0.0)), None);
        assert_eq!(goal_position(&point(0.0, f64::INFINITY, 0.0)), None);
        assert_eq!(goal_position(&point(0.0, 0.0, f64::NEG_INFINITY)), None);
    }
}
