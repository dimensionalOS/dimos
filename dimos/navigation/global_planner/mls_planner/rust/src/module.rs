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

use crate::mls_planner::{Config, Plan, Planner, RegionBounds};
use crate::region_viz::{pack_cell, Cell, RegionContent, RegionViz};
use crate::voxel::{surface_point_xyz, VoxelKey, Xyz};
use dimos_module::time::now;
use dimos_module::{debug_throttled, error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::actionlib_msgs::{GoalID, GoalStatus};
use lcm_msgs::geometry_msgs::{Point, PointStamped, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::Notify;
use tracing::{debug, info, warn};

type Xyzi = (f32, f32, f32, f32);

/// State shared between the handle loop and the worker.
type Shared<T> = Arc<Mutex<Option<T>>>;

/// The last status reported. Held across each publish, so a heartbeat can
/// never send an older goal's status after a newer one.
type LatestStatus = Arc<tokio::sync::Mutex<Option<Report>>>;

/// A map input handed from the handle loop to the worker. Only the newest is
/// kept, so a dropped intermediate frame is harmless.
enum MapUpdate {
    Region {
        cloud: PointCloud2,
        bounds: PoseStamped,
        received: Instant,
    },
    Global {
        cloud: PointCloud2,
    },
}

/// One region of a seeded map, as the ray tracer hands them on. They queue,
/// since every one must land.
struct SeedRegion {
    cloud: PointCloud2,
    bounds: PoseStamped,
}

/// How often the latest goal status repeats between transitions.
const STATUS_HEARTBEAT: Duration = Duration::from_secs(1);

/// Names a goal in its status reports.
#[derive(Clone, Copy, Debug, PartialEq)]
struct GoalId {
    /// Stamp of the message that set the goal. Senders match reports on it.
    sec: i32,
    nsec: i32,
    /// Arrival order, which tells a displaced goal from the one after it.
    arrival: u64,
}

impl GoalId {
    fn same_stamp(&self, other: &GoalId) -> bool {
        (self.sec, self.nsec) == (other.sec, other.nsec)
    }
}

/// The active goal and the id its status reports carry.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Goal {
    id: GoalId,
    position: Xyz,
}

/// How long half a seed region waits for its counterpart before it is dropped.
const SEED_PAIR_TIMEOUT: Duration = Duration::from_secs(10);

/// A seed queue quiet this long is taken as a finished load.
const SEED_SETTLE: Duration = Duration::from_secs(2);

/// The seed regions applied since the queue was last quiet, for the log.
#[derive(Default)]
struct SeedProgress {
    started: Option<Instant>,
    last_at: Option<Instant>,
    applied: usize,
    unusable: usize,
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

    /// Count one region by the points it applied.
    fn record(&mut self, applied: Option<usize>, region_ms: f64) {
        let now = Instant::now();
        self.started.get_or_insert(now);
        self.last_at = Some(now);
        let Some(points) = applied else {
            self.unusable += 1;
            return;
        };
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
            unusable = self.unusable,
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
                unusable = self.unusable,
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

    #[output(encode = GoalStatus::encode)]
    nav_status: Output<GoalStatus>,

    #[config]
    config: Config,

    // Held on the handle loop until stamps match, then handed off paired.
    pending_local: Option<PointCloud2>,
    pending_bounds: Option<PoseStamped>,
    pending_seeds: SeedPairs,

    // Written by the handle loop, read by the worker, so the loop never blocks
    // on map processing. Seed regions queue in arrival order.
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<SeedRegion>>>,
    active_goal: Shared<Goal>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,
    latest_status: LatestStatus,

    // Counts goal messages in arrival order.
    goal_count: u64,

    worker: Option<tokio::task::JoinHandle<()>>,
    heartbeat: Option<tokio::task::JoinHandle<()>>,
}

impl MlsPlanner {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            pending: Arc::clone(&self.pending),
            seed_regions: Arc::clone(&self.seed_regions),
            active_goal: Arc::clone(&self.active_goal),
            goal_changed: Arc::clone(&self.goal_changed),
            wake: Arc::clone(&self.wake),
            tf: self.tf.clone(),
            config: self.config.clone(),
            surface_map: self.surface_map.clone(),
            nodes: self.nodes.clone(),
            node_edges: self.node_edges.clone(),
            path: self.path.clone(),
            status: self.status(),
        };
        self.worker = Some(tokio::spawn(worker.run()));
        self.heartbeat = Some(tokio::spawn(self.status().heartbeat()));
    }

    async fn stop_worker(&mut self) {
        for handle in [self.worker.take(), self.heartbeat.take()]
            .into_iter()
            .flatten()
        {
            handle.abort();
        }
    }

    fn status(&self) -> StatusReporter {
        StatusReporter {
            out: self.nav_status.clone(),
            latest: Arc::clone(&self.latest_status),
        }
    }

    async fn on_global_map(&mut self, msg: PointCloud2) {
        self.hand_off(MapUpdate::Global { cloud: msg });
    }

    async fn on_local_map(&mut self, msg: PointCloud2) {
        self.pending_local = Some(msg);
        self.try_pair();
    }

    async fn on_region_bounds(&mut self, msg: PoseStamped) {
        self.pending_bounds = Some(msg);
        self.try_pair();
    }

    async fn on_seed_map(&mut self, msg: PointCloud2) {
        if let Some(region) = self.pending_seeds.pair_cloud(msg) {
            self.queue_seed(region);
        }
    }

    async fn on_seed_bounds(&mut self, msg: PoseStamped) {
        if let Some(region) = self.pending_seeds.pair_bounds(msg) {
            self.queue_seed(region);
        }
    }

    /// Hand off the local map and bounds once their stamps match.
    fn try_pair(&mut self) {
        if !stamps_paired(self.pending_bounds.as_ref(), self.pending_local.as_ref()) {
            return;
        }
        let bounds = self.pending_bounds.take().expect("checked above");
        let cloud = self.pending_local.take().expect("checked above");
        self.hand_off(MapUpdate::Region {
            cloud,
            bounds,
            received: Instant::now(),
        });
    }

    fn queue_seed(&self, region: SeedRegion) {
        self.seed_regions
            .lock()
            .expect("seed mutex")
            .push_back(region);
        self.wake.notify_one();
    }

    fn hand_off(&self, update: MapUpdate) {
        *self.pending.lock().expect("pending mutex") = Some(update);
        self.wake.notify_one();
    }

    /// Set, cancel or reject the active goal, then wake the worker. A goal
    /// this displaces is reported preempted under its own id.
    async fn on_goal(&mut self, msg: PointStamped) {
        match goal_request(&msg.point) {
            GoalRequest::Set(position) => {
                let id = self.next_goal_id(&msg.header.stamp);
                let held = *self.active_goal.lock().expect("goal mutex");
                // A goal sent again keeps its id, so it is not preempted.
                if let Some(held) = held.filter(|held| !held.id.same_stamp(&id)) {
                    self.preempt(held, "replaced").await;
                }
                self.status()
                    .report(Report::new(id, GoalStatus::PENDING, "planning"))
                    .await;
                self.set_active_goal(Some(Goal { id, position }));
            }
            GoalRequest::Cancel => {
                if let Some(held) = self.set_active_goal(None) {
                    self.preempt(held, "canceled").await;
                }
            }
            GoalRequest::Invalid => {
                if let Some(held) = self.set_active_goal(None) {
                    self.preempt(held, "replaced").await;
                }
                let id = self.next_goal_id(&msg.header.stamp);
                self.status()
                    .report(Report::new(id, GoalStatus::REJECTED, "goal is not finite"))
                    .await;
            }
        }
        self.goal_changed.store(true, Ordering::SeqCst);
        self.wake.notify_one();
    }

    async fn preempt(&self, goal: Goal, reason: &'static str) {
        self.status()
            .report(Report::new(goal.id, GoalStatus::PREEMPTED, reason))
            .await;
    }

    fn next_goal_id(&mut self, stamp: &Time) -> GoalId {
        self.goal_count += 1;
        GoalId {
            sec: stamp.sec,
            nsec: stamp.nsec,
            arrival: self.goal_count,
        }
    }

    /// Replace the active goal, returning the one it displaced.
    fn set_active_goal(&self, goal: Option<Goal>) -> Option<Goal> {
        std::mem::replace(&mut *self.active_goal.lock().expect("goal mutex"), goal)
    }
}

/// One goal's state as nav_status carries it.
#[derive(Clone, Debug, PartialEq)]
struct Report {
    id: GoalId,
    status: u8,
    reason: &'static str,
    remaining_m: Option<f32>,
}

impl Report {
    fn new(id: GoalId, status: i8, reason: &'static str) -> Self {
        Self {
            id,
            status: status as u8,
            reason,
            remaining_m: None,
        }
    }

    fn with_remaining(mut self, remaining_m: f32) -> Self {
        self.remaining_m = Some(remaining_m);
        self
    }

    /// Whether this differs from the last report by more than the distance.
    fn is_transition_from(&self, last: Option<&Report>) -> bool {
        last.is_none_or(|last| {
            (last.id, last.status, last.reason) != (self.id, self.status, self.reason)
        })
    }

    /// Whether the goal is over. An aborted goal is still retried.
    fn is_terminal(&self) -> bool {
        [
            GoalStatus::SUCCEEDED,
            GoalStatus::PREEMPTED,
            GoalStatus::REJECTED,
        ]
        .contains(&(self.status as i8))
    }

    fn message(&self) -> GoalStatus {
        let text = match self.remaining_m {
            Some(remaining_m) => format!("{}, {remaining_m:.2} m from the goal", self.reason),
            None => self.reason.to_string(),
        };
        GoalStatus {
            goal_id: GoalID {
                stamp: Time {
                    sec: self.id.sec,
                    nsec: self.id.nsec,
                },
                id: format!("{}.{:09}", self.id.sec, self.id.nsec),
            },
            status: self.status,
            text,
        }
    }
}

/// Record a report as the latest. True when it is a transition to publish
/// now. A report for a goal older than the latest is dropped, and so is one
/// for a goal that is already over.
fn record(latest: &mut Option<Report>, report: &Report) -> bool {
    let stale = latest.as_ref().is_some_and(|last| {
        last.id.arrival > report.id.arrival
            || (last.id.arrival == report.id.arrival && last.is_terminal())
    });
    if stale {
        return false;
    }
    let transition = report.is_transition_from(latest.as_ref());
    *latest = Some(report.clone());
    transition
}

/// Publishes goal status on every transition and on the heartbeat.
#[derive(Clone)]
struct StatusReporter {
    out: Output<GoalStatus>,
    latest: LatestStatus,
}

impl StatusReporter {
    async fn report(&self, report: Report) {
        let mut latest = self.latest.lock().await;
        if record(&mut latest, &report) {
            self.publish(&report).await;
        }
    }

    async fn heartbeat(self) {
        let mut tick = tokio::time::interval(STATUS_HEARTBEAT);
        loop {
            tick.tick().await;
            let latest = self.latest.lock().await;
            if let Some(report) = latest.as_ref() {
                self.publish(report).await;
            }
        }
    }

    async fn publish(&self, report: &Report) {
        if let Err(e) = self.out.publish(&report.message()).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                topic = %self.out.topic,
                "Goal status failed to publish",
            );
        }
    }
}

/// True when bounds and a local cloud are both present with matching stamps.
fn stamps_paired(bounds: Option<&PoseStamped>, cloud: Option<&PointCloud2>) -> bool {
    match (bounds, cloud) {
        (Some(b), Some(c)) => same_stamp(&b.header.stamp, &c.header.stamp),
        _ => false,
    }
}

/// What a goal message asks for.
#[derive(Debug, PartialEq)]
enum GoalRequest {
    Set(Xyz),
    Cancel,
    Invalid,
}

/// A finite point sets a goal and an all-NaN point cancels. Any other
/// non-finite point is invalid.
fn goal_request(p: &Point) -> GoalRequest {
    let goal = (p.x as f32, p.y as f32, p.z as f32);
    if goal.0.is_finite() && goal.1.is_finite() && goal.2.is_finite() {
        GoalRequest::Set(goal)
    } else if p.x.is_nan() && p.y.is_nan() && p.z.is_nan() {
        GoalRequest::Cancel
    } else {
        GoalRequest::Invalid
    }
}

/// Owns the planner graph and does map mutation, publishing, and replanning
/// off the handle loop. Woken by the handlers.
struct Worker {
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<SeedRegion>>>,
    active_goal: Shared<Goal>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,
    tf: Tf,
    config: Config,
    surface_map: Output<PointCloud2>,
    nodes: Output<PointCloud2>,
    node_edges: Output<Path>,
    path: Output<Path>,
    status: StatusReporter,
}

impl Worker {
    async fn run(self) {
        let mut planner = Planner::new(self.config.worker_threads);
        let mut last_path_at: Option<Instant> = None;
        let mut last_viz_at: Option<Instant> = None;
        let mut viz = RegionViz::new(
            (self.config.viz_region_m / self.config.voxel_size).round() as i32,
            self.config.viz_reach_cells(),
            self.config.viz_sweep_regions as usize,
        );
        let mut seed_progress = SeedProgress::default();
        loop {
            if seed_progress.in_flight() {
                let woke = tokio::time::timeout(SEED_SETTLE, self.wake.notified()).await;
                if seed_progress.settled() {
                    seed_progress.finish();
                }
                if woke.is_err() {
                    continue;
                }
            } else {
                self.wake.notified().await;
            }
            loop {
                let goal_changed = self.goal_changed.swap(false, Ordering::SeqCst);
                let update = self.pending.lock().expect("pending mutex").take();
                if let Some(MapUpdate::Region { received, .. }) = &update {
                    seed_progress.record_live_wait(received.elapsed().as_secs_f64() * 1e3);
                }
                let live_update = match update {
                    Some(update) => {
                        self.apply_update(&mut planner, update, &mut viz, &mut last_viz_at)
                            .await
                    }
                    None => false,
                };
                let goal = *self.active_goal.lock().expect("goal mutex");
                if stop_due(goal_changed, goal) {
                    publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
                }
                if replan_due(goal_changed, live_update) {
                    self.maybe_replan(&mut planner, &mut last_path_at).await;
                }
                // Live updates apply first, then one seed region per pass.
                let (seed, queued) = {
                    let mut queue = self.seed_regions.lock().expect("seed mutex");
                    (queue.pop_front(), queue.len())
                };
                let Some(seed) = seed else {
                    break;
                };
                if !seed_progress.in_flight() {
                    info!("Premap load started.");
                }
                let region_start = Instant::now();
                let applied =
                    tokio::task::block_in_place(|| self.ingest_seed(&mut planner, seed, &mut viz));
                let region_ms = region_start.elapsed().as_secs_f64() * 1e3;
                seed_progress.record(applied, region_ms);
                seed_progress.log_progress(queued);
                if applied.is_some() {
                    self.publish_viz_if_due(&planner, &mut viz, &mut last_viz_at)
                        .await;
                }
                tokio::task::yield_now().await;
            }
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
        let (due_regions, node_cloud) = tokio::task::block_in_place(|| {
            let due_regions = viz.tick(
                planner.surface_clearance_iter(),
                planner.edge_segment_iter(),
            );
            let node_points: Vec<Xyz> = planner.graph().nodes.iter().map(|n| n.pos).collect();
            (
                due_regions,
                build_pc2_xyz(&node_points, &self.config.world_frame, now()),
            )
        });
        *last_viz_at = Some(tick_at);
        let (voxel_size, frame) = (self.config.voxel_size, self.config.world_frame.as_str());
        let stamp = now();
        for (cell, content) in due_regions {
            let (surface, edges) = tokio::task::block_in_place(|| {
                region_messages(cell, content, voxel_size, frame, stamp.clone())
            });
            publish_cloud(&self.surface_map, &surface).await;
            publish_path(&self.node_edges, &edges).await;
        }
        publish_cloud(&self.nodes, &node_cloud).await;
    }

    /// Mutate the graph from a map update. False if the cloud was unusable.
    fn ingest(&self, planner: &mut Planner, update: MapUpdate, viz: &mut RegionViz) -> bool {
        match update {
            MapUpdate::Region {
                cloud,
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
                let process_start = Instant::now();
                let applied =
                    self.apply_region(planner, &cloud, &region, viz, "local region processed");
                if let Some(points) = applied {
                    debug_throttled!(
                        Duration::from_secs(5),
                        process_ms = process_start.elapsed().as_secs_f64() * 1e3,
                        wait_ms = process_start.duration_since(received).as_secs_f64() * 1e3,
                        points,
                        "local region processed"
                    );
                }
                applied.is_some()
            }
            MapUpdate::Global { cloud } => {
                let points = match extract_xyz(&cloud) {
                    Ok(p) => p,
                    Err(e) => {
                        warn_throttled!(
                            Duration::from_secs(1),
                            error = %e,
                            "Failed to extract lidar points, dropped a cloud.",
                        );
                        return false;
                    }
                };
                if points.is_empty() {
                    return false;
                }
                planner.update_global_map(&points, &self.config);
                viz.mark_all();
                debug!(global_map_points = points.len(), "global_map processed");
                true
            }
        }
    }

    /// Apply one seed region through the region pipeline. Its bounds are the
    /// premap's own, so no sensor ceiling applies. The points applied, or
    /// None if unusable.
    fn ingest_seed(
        &self,
        planner: &mut Planner,
        seed: SeedRegion,
        viz: &mut RegionViz,
    ) -> Option<usize> {
        let region = region_bounds(&seed.bounds);
        self.apply_region(planner, &seed.cloud, &region, viz, "seed region processed")
    }

    /// Replace the voxels in a region and repair the graph around them,
    /// marking the rewritten window for the viz. The points applied, or None
    /// if the cloud was unusable.
    fn apply_region(
        &self,
        planner: &mut Planner,
        cloud: &PointCloud2,
        region: &RegionBounds,
        viz: &mut RegionViz,
        label: &'static str,
    ) -> Option<usize> {
        let points = match extract_xyz(cloud) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    label,
                    "Failed to extract region points, dropped it.",
                );
                return None;
            }
        };
        if let Some(window) = planner.update_region(&points, region, &self.config) {
            viz.mark_window(window);
        }
        Some(points.len())
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
        let Some(goal) = *self.active_goal.lock().expect("goal mutex") else {
            return;
        };

        let plan_start = Instant::now();
        let outcome =
            tokio::task::block_in_place(|| replan(planner, start, goal.position, &self.config));
        let (status, reason) = outcome.status();
        let report =
            Report::new(goal.id, status, reason).with_remaining(distance(start, goal.position));
        let waypoints = match outcome {
            Replan::Reached => {
                self.clear_goal(goal);
                self.status.report(report).await;
                return;
            }
            Replan::Planned(Plan::Blocked) => {
                publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
                self.status.report(report).await;
                return;
            }
            Replan::Planned(Plan::Full(waypoints) | Plan::Truncated(waypoints)) => waypoints,
        };
        let plan_ms = plan_start.elapsed().as_secs_f64() * 1e3;
        let produced = Instant::now();
        let since_last_ms = last_path_at.map_or(-1.0, |t| (produced - t).as_secs_f64() * 1e3);
        *last_path_at = Some(produced);

        let stamp = now();
        let path_msg = build_path_from_waypoints(&waypoints, &self.config.world_frame, stamp);
        debug_throttled!(
            Duration::from_secs(5),
            waypoints = waypoints.len(),
            plan_ms,
            since_last_ms,
            "path planned"
        );
        publish_path(&self.path, &path_msg).await;
        self.status.report(report).await;
    }

    /// Clear the active goal unless a newer one replaced it meanwhile.
    fn clear_goal(&self, goal: Goal) {
        let mut guard = self.active_goal.lock().expect("goal mutex");
        if *guard == Some(goal) {
            *guard = None;
        }
    }
}

/// What a replan pass decided for the active goal.
#[derive(Debug, PartialEq)]
enum Replan {
    Reached,
    Planned(Plan),
}

impl Replan {
    /// The goal status this outcome reports, with its reason.
    fn status(&self) -> (i8, &'static str) {
        match self {
            Replan::Reached => (GoalStatus::SUCCEEDED, "reached"),
            Replan::Planned(Plan::Full(_)) => (GoalStatus::ACTIVE, "following the path"),
            Replan::Planned(Plan::Truncated(_)) => (
                GoalStatus::ACTIVE,
                "path blocked ahead, following it while safe",
            ),
            Replan::Planned(Plan::Blocked) => (GoalStatus::ABORTED, "no safe path"),
        }
    }
}

/// Decide a replan pass. A goal already within tolerance needs no path.
fn replan(planner: &mut Planner, start: Xyz, goal: Xyz, config: &Config) -> Replan {
    if is_at_goal(start, goal, config.goal_tolerance, config.goal_z_tolerance) {
        return Replan::Reached;
    }
    Replan::Planned(planner.plan_or_truncate(start, goal, config))
}

fn distance(a: Xyz, b: Xyz) -> f32 {
    ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt()
}

/// Whether a worker pass stops the follower. A goal message that leaves no
/// goal is a cancel, and the follower is still on the last path.
fn stop_due(goal_changed: bool, goal: Option<Goal>) -> bool {
    goal_changed && goal.is_none()
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

/// One cell's surface and edge messages, its cell in the header seq.
fn region_messages(
    cell: Cell,
    content: RegionContent,
    voxel_size: f32,
    frame: &str,
    stamp: Time,
) -> (PointCloud2, Path) {
    let surface_points: Vec<Xyzi> = content
        .surface
        .into_iter()
        .map(|((ix, iy, iz), clearance)| {
            let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
            (x, y, z, clearance)
        })
        .collect();
    let mut surface = build_pc2_xyzi(&surface_points, frame, stamp.clone());
    surface.header.seq = pack_cell(cell);
    let mut edges = build_segments_path(content.segments, voxel_size, frame, stamp);
    edges.header.seq = pack_cell(cell);
    (surface, edges)
}

/// True if within tolerance of the goal on the ground plane and vertically.
fn is_at_goal(start: Xyz, goal: Xyz, tol: f32, z_tol: f32) -> bool {
    (start.0 - goal.0).hypot(start.1 - goal.1) < tol && (start.2 - goal.2).abs() < z_tol
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
    fn is_at_goal_respects_the_planar_and_vertical_tolerances() {
        assert!(is_at_goal((0.0, 0.0, 0.0), (0.05, 0.0, 0.4), 0.1, 0.5));
        assert!(!is_at_goal((0.0, 0.0, 0.0), (0.05, 0.0, 9.0), 0.1, 0.5));
        assert!(!is_at_goal((0.0, 0.0, 0.0), (0.2, 0.0, 0.0), 0.1, 0.5));
    }

    fn test_config() -> Config {
        Config {
            world_frame: "odom".into(),
            base_frame: "base_link".into(),
            voxel_size: 0.1,
            robot_height: 0.5,
            start_z_offset_m: 0.0,
            max_overhead_m: 2.0,
            surface_closing_radius: 0.3,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.0,
            wall_buffer_m: 0.3,
            wall_buffer_weight: 1.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 0.0,
            goal_tolerance: 0.3,
            goal_z_tolerance: 0.5,
            viz_publish_hz: 2.0,
            viz_region_m: 4.0,
            viz_sweep_regions: 0,
            worker_threads: 1,
        }
    }

    #[test]
    fn a_goal_directly_above_the_start_is_not_reached() {
        let mut planner = Planner::new(1);
        let outcome = replan(
            &mut planner,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 3.0),
            &test_config(),
        );
        assert_eq!(outcome, Replan::Planned(Plan::Blocked));
    }

    #[test]
    fn a_goal_within_tolerance_is_reached_without_a_path() {
        let mut planner = Planner::new(1);
        let outcome = replan(
            &mut planner,
            (0.0, 0.0, 0.0),
            (0.1, 0.0, 0.0),
            &test_config(),
        );
        assert_eq!(outcome, Replan::Reached);
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
    fn region_messages_carry_the_cell_in_both_headers_even_when_empty() {
        let cell = (1, -2);
        let content = RegionContent {
            surface: vec![((1, 2, 0), 0.5)],
            segments: vec![((1, 2, 0), (2, 2, 0), 1.5)],
        };
        let (surface, edges) = region_messages(cell, content, 0.1, "odom", Time::default());
        assert_eq!(surface.header.seq, pack_cell(cell));
        assert_eq!(edges.header.seq, pack_cell(cell));
        assert_eq!((surface.width, edges.poses.len()), (1, 2));

        let (surface, edges) =
            region_messages(cell, RegionContent::default(), 0.1, "odom", Time::default());
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
        progress.record(None, 1.0);
        assert!(progress.in_flight());
        for i in 1..REGIONS {
            progress.record(Some(100), i as f64);
        }
        progress.record(Some(100), 0.5);
        assert_eq!(progress.applied, REGIONS);
        assert_eq!(progress.unusable, 1);
        assert_eq!(progress.points, 100 * REGIONS);
        assert_eq!(progress.max_region_ms, (REGIONS - 1) as f64);
        let sum: f64 = (1..REGIONS).map(|i| i as f64).sum::<f64>() + 0.5;
        assert!((progress.mean_region_ms() - sum / REGIONS as f64).abs() < 1e-9);

        progress.finish();
        assert!(!progress.in_flight());
        assert_eq!(progress.applied, 0);
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

    #[test]
    fn a_pass_stops_the_follower_only_when_a_goal_message_leaves_no_goal() {
        let goal = Goal {
            id: goal_id(1),
            position: (1.0, 0.0, 0.0),
        };
        assert!(stop_due(true, None));
        assert!(!stop_due(true, Some(goal)));
        // The worker clearing a reached goal is not a goal message.
        assert!(!stop_due(false, None));
    }

    /// The id of the goal that arrived in this order, stamped at that second.
    fn goal_id(arrival: u64) -> GoalId {
        GoalId {
            sec: arrival as i32,
            nsec: 5,
            arrival,
        }
    }

    fn point(x: f64, y: f64, z: f64) -> Point {
        Point { x, y, z }
    }

    #[test]
    fn goal_request_sets_on_finite_cancels_on_all_nan_and_rejects_the_rest() {
        assert_eq!(
            goal_request(&point(1.0, 2.0, 3.0)),
            GoalRequest::Set((1.0, 2.0, 3.0))
        );
        assert_eq!(
            goal_request(&point(f64::NAN, f64::NAN, f64::NAN)),
            GoalRequest::Cancel
        );
        assert_eq!(
            goal_request(&point(f64::NAN, 0.0, 0.0)),
            GoalRequest::Invalid
        );
        assert_eq!(
            goal_request(&point(0.0, f64::INFINITY, 0.0)),
            GoalRequest::Invalid
        );
    }

    #[test]
    fn each_replan_outcome_maps_to_its_goal_status() {
        let waypoints = || vec![(0.0, 0.0, 0.0)];
        let full = Replan::Planned(Plan::Full(waypoints()));
        let truncated = Replan::Planned(Plan::Truncated(waypoints()));
        assert_eq!(Replan::Reached.status().0, GoalStatus::SUCCEEDED);
        assert_eq!(full.status().0, GoalStatus::ACTIVE);
        assert_eq!(truncated.status().0, GoalStatus::ACTIVE);
        assert_ne!(full.status().1, truncated.status().1);
        assert_eq!(
            Replan::Planned(Plan::Blocked).status().0,
            GoalStatus::ABORTED
        );
    }

    #[test]
    fn a_report_publishes_on_a_transition_and_not_on_a_distance_change() {
        let mut latest = None;
        let active = Report::new(goal_id(1), GoalStatus::ACTIVE, "following the path");
        assert!(record(&mut latest, &active.clone().with_remaining(4.0)));
        assert!(!record(&mut latest, &active.clone().with_remaining(3.0)));
        assert_eq!(latest.as_ref().and_then(|r| r.remaining_m), Some(3.0));
        assert!(record(
            &mut latest,
            &Report::new(goal_id(1), GoalStatus::SUCCEEDED, "reached")
        ));
    }

    #[test]
    fn a_report_for_an_older_goal_is_dropped() {
        let mut latest = None;
        let pending = Report::new(goal_id(2), GoalStatus::PENDING, "planning");
        assert!(record(&mut latest, &pending));
        assert!(!record(
            &mut latest,
            &Report::new(goal_id(1), GoalStatus::ACTIVE, "following the path")
        ));
        assert_eq!(latest, Some(pending));
    }

    #[test]
    fn a_report_for_a_goal_that_is_over_is_dropped() {
        let mut latest = None;
        let preempted = Report::new(goal_id(1), GoalStatus::PREEMPTED, "canceled");
        assert!(record(&mut latest, &preempted));
        assert!(!record(
            &mut latest,
            &Report::new(goal_id(1), GoalStatus::ACTIVE, "following the path")
        ));
        assert_eq!(latest, Some(preempted));
    }

    #[test]
    fn an_aborted_goal_can_still_report() {
        let mut latest = None;
        assert!(record(
            &mut latest,
            &Report::new(goal_id(1), GoalStatus::ABORTED, "no safe path")
        ));
        assert!(record(
            &mut latest,
            &Report::new(goal_id(1), GoalStatus::ACTIVE, "following the path")
        ));
    }

    #[test]
    fn a_report_message_carries_the_id_status_reason_and_distance() {
        let msg = Report::new(goal_id(7), GoalStatus::ABORTED, "no safe path")
            .with_remaining(1.5)
            .message();
        assert_eq!(msg.goal_id.id, "7.000000005");
        assert_eq!((msg.goal_id.stamp.sec, msg.goal_id.stamp.nsec), (7, 5));
        assert_eq!(msg.status, GoalStatus::ABORTED as u8);
        assert_eq!(msg.text, "no safe path, 1.50 m from the goal");
    }
}
