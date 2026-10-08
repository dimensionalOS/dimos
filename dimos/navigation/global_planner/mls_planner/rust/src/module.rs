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

use crate::mls_planner::{Config, Planner, RegionBounds};
use crate::region_viz::{pack_cell, Cell, RegionContent, RegionViz};
use crate::voxel::{surface_point_xyz, VoxelKey};
use dimos_module::time::now;
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, PointStamped, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::Notify;
use tracing::{debug, info, warn};

/// A point in the planner's world frame.
type Xyz = (f32, f32, f32);
type Xyzi = (f32, f32, f32, f32);

/// State shared between the handle loop and the worker.
type Shared<T> = Arc<Mutex<Option<T>>>;

/// A map input handed from the handle loop to the worker. Only the newest is
/// kept, so a dropped intermediate frame is harmless.
enum MapUpdate {
    Region {
        cloud: PointCloud2,
        bounds: PoseStamped,
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

/// How long half a seed region waits for its counterpart before it is dropped.
const SEED_PAIR_TIMEOUT: Duration = Duration::from_secs(10);

/// Seed regions between progress lines.
const SEED_PROGRESS_REGIONS: usize = 20;

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
    applied: usize,
    unusable: usize,
    points: usize,
    max_region_ms: f64,
    sum_region_ms: f64,
}

impl SeedProgress {
    fn in_flight(&self) -> bool {
        self.started.is_some()
    }

    fn mean_region_ms(&self) -> f64 {
        self.sum_region_ms / self.applied.max(1) as f64
    }

    /// Whether no region has been applied for SEED_STARVED_WARN_AFTER.
    fn starved(&self) -> bool {
        self.last_at
            .is_some_and(|at| at.elapsed() >= SEED_STARVED_WARN_AFTER)
    }

    /// Count one region by the points it applied. True when a progress line
    /// is due.
    fn record(&mut self, applied: Option<usize>, region_ms: f64) -> bool {
        let now = Instant::now();
        self.started.get_or_insert(now);
        self.last_at = Some(now);
        let Some(points) = applied else {
            self.unusable += 1;
            return false;
        };
        self.applied += 1;
        self.points += points;
        self.max_region_ms = self.max_region_ms.max(region_ms);
        self.sum_region_ms += region_ms;
        self.applied.is_multiple_of(SEED_PROGRESS_REGIONS)
    }

    fn log_progress(&self, queued: usize) {
        info!(
            regions_done = self.applied,
            queued,
            unusable = self.unusable,
            points = self.points,
            max_region_ms = self.max_region_ms,
            mean_region_ms = self.mean_region_ms(),
            "Seed regions in progress."
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
                "Applied the seed regions to the graph."
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

    // Held on the handle loop until stamps match, then handed off paired.
    pending_local: Option<PointCloud2>,
    pending_bounds: Option<PoseStamped>,
    pending_seeds: SeedPairs,

    // Written by the handle loop, read by the worker, so the loop never blocks
    // on map processing. Seed regions queue in arrival order.
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<SeedRegion>>>,
    active_goal: Shared<Xyz>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,

    worker: Option<tokio::task::JoinHandle<()>>,
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
        };
        self.worker = Some(tokio::spawn(worker.run()));
    }

    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
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
        self.hand_off(MapUpdate::Region { cloud, bounds });
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

    /// Set or cancel the active goal from a click, then wake the worker.
    async fn on_goal(&mut self, msg: PointStamped) {
        *self.active_goal.lock().expect("goal mutex") = goal_position(&msg.point);
        self.goal_changed.store(true, Ordering::SeqCst);
        self.wake.notify_one();
    }
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
/// off the handle loop. Woken by the handlers.
struct Worker {
    pending: Shared<MapUpdate>,
    seed_regions: Arc<Mutex<VecDeque<SeedRegion>>>,
    active_goal: Shared<Xyz>,
    goal_changed: Arc<AtomicBool>,
    wake: Arc<Notify>,
    tf: Tf,
    config: Config,
    surface_map: Output<PointCloud2>,
    nodes: Output<PointCloud2>,
    node_edges: Output<Path>,
    path: Output<Path>,
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
        // One unit of work per pass: a live update or goal first, else one
        // seed region, else wait.
        loop {
            let goal_changed = self.goal_changed.swap(false, Ordering::SeqCst);
            let update = self.pending.lock().expect("pending mutex").take();
            if update.is_some() || goal_changed {
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
                self.warn_if_seeds_starved(&seed_progress);
                continue;
            }

            let (seed, queued) = {
                let mut queue = self.seed_regions.lock().expect("seed mutex");
                (queue.pop_front(), queue.len())
            };
            if let Some(seed) = seed {
                let region_start = Instant::now();
                let applied =
                    tokio::task::block_in_place(|| self.ingest_seed(&mut planner, seed, &mut viz));
                let region_ms = region_start.elapsed().as_secs_f64() * 1e3;
                if seed_progress.record(applied, region_ms) {
                    seed_progress.log_progress(queued);
                }
                if applied.is_some() {
                    self.publish_viz_if_due(&planner, &mut viz, &mut last_viz_at)
                        .await;
                }
                tokio::task::yield_now().await;
                continue;
            }

            // A seed whose queue stays quiet has finished.
            if seed_progress.in_flight() {
                let woke = tokio::time::timeout(SEED_SETTLE, self.wake.notified()).await;
                if woke.is_err() {
                    seed_progress.finish();
                }
            } else {
                self.wake.notified().await;
            }
        }
    }

    /// Live work has kept seed regions queued and unapplied for too long.
    fn warn_if_seeds_starved(&self, progress: &SeedProgress) {
        if !progress.starved() {
            return;
        }
        let queued = self.seed_regions.lock().expect("seed mutex").len();
        if queued > 0 {
            warn_throttled!(
                SEED_STARVED_WARN_AFTER,
                queued,
                regions_done = progress.applied,
                "Seed regions are starved: live updates have kept the worker busy.",
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
        let tick_ms = tick_at.elapsed().as_secs_f64() * 1e3;
        *last_viz_at = Some(tick_at);
        let (voxel_size, frame) = (self.config.voxel_size, self.config.world_frame.as_str());
        let stamp = now();
        let regions = due_regions.len();
        let (mut surface_bytes, mut edge_segments) = (0usize, 0usize);
        for (cell, content) in due_regions {
            edge_segments += content.segments.len();
            let (surface, edges) = tokio::task::block_in_place(|| {
                region_messages(cell, content, voxel_size, frame, stamp.clone())
            });
            surface_bytes += surface.data.len();
            publish_cloud(&self.surface_map, &surface).await;
            publish_path(&self.node_edges, &edges).await;
        }
        publish_cloud(&self.nodes, &node_cloud).await;
        debug!(
            regions,
            tick_ms, surface_bytes, edge_segments, "viz published"
        );
    }

    /// Mutate the graph from a map update. False if the cloud was unusable.
    fn ingest(&self, planner: &mut Planner, update: MapUpdate, viz: &mut RegionViz) -> bool {
        match update {
            MapUpdate::Region { cloud, bounds } => {
                let Some((_, _, sensor_z)) = self.base_position() else {
                    warn!(
                        world_frame = %self.config.world_frame,
                        base_frame = %self.config.base_frame,
                        "No base pose on tf, dropped a region update.",
                    );
                    return false;
                };
                let region = region_bounds(&bounds).capped_at(sensor_z, self.config.max_overhead_m);
                self.apply_region(planner, &cloud, &region, viz, "local region processed")
                    .is_some()
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
        let update_start = Instant::now();
        if let Some(window) = planner.update_region(&points, region, &self.config) {
            viz.mark_window(window);
        }
        debug!(
            update_ms = update_start.elapsed().as_secs_f64() * 1e3,
            points = points.len(),
            "{label}"
        );
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
        let plan_ms = plan_start.elapsed().as_secs_f64() * 1e3;
        let produced = Instant::now();
        let since_last_ms = last_path_at.map_or(-1.0, |t| (produced - t).as_secs_f64() * 1e3);
        *last_path_at = Some(produced);

        let stamp = now();
        let path_msg = build_path_from_waypoints(&waypoints, &self.config.world_frame, stamp);
        debug!(
            waypoints = waypoints.len(),
            plan_ms, since_last_ms, "path planned"
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
    fn seed_progress_counts_applied_regions_and_is_due_every_batch() {
        let mut progress = SeedProgress::default();
        assert!(!progress.in_flight());
        assert!(
            !progress.record(None, 1.0),
            "unusable regions never trigger a line"
        );
        assert!(progress.in_flight());
        for i in 1..SEED_PROGRESS_REGIONS {
            assert!(!progress.record(Some(100), i as f64), "region {i}");
        }
        assert!(
            progress.record(Some(100), 0.5),
            "the batch's last region is due"
        );
        assert_eq!(progress.applied, SEED_PROGRESS_REGIONS);
        assert_eq!(progress.unusable, 1);
        assert_eq!(progress.points, 100 * SEED_PROGRESS_REGIONS);
        assert_eq!(progress.max_region_ms, (SEED_PROGRESS_REGIONS - 1) as f64);
        let sum: f64 = (1..SEED_PROGRESS_REGIONS).map(|i| i as f64).sum::<f64>() + 0.5;
        assert!((progress.mean_region_ms() - sum / SEED_PROGRESS_REGIONS as f64).abs() < 1e-9);

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
