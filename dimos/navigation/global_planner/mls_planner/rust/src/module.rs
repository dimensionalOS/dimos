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
use tracing::{debug, warn};

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

/// One region of a seeded map, as the ray tracer hands them on. Every one
/// must land, so they queue rather than replace each other.
struct SeedRegion {
    cloud: PointCloud2,
    bounds: PoseStamped,
}

/// Seed clouds and bounds waiting for their counterpart, keyed by the region
/// number in their header seq. Every region of a seed shares one stamp and
/// the two topics can interleave, so a newest-wins slot would mispair them.
#[derive(Default)]
struct SeedPairs {
    clouds: HashMap<i32, PointCloud2>,
    bounds: HashMap<i32, PoseStamped>,
}

impl SeedPairs {
    fn cloud(&mut self, msg: PointCloud2) -> Option<SeedRegion> {
        match self.bounds.remove(&msg.header.seq) {
            Some(bounds) => Some(SeedRegion { cloud: msg, bounds }),
            None => {
                self.clouds.insert(msg.header.seq, msg);
                None
            }
        }
    }

    fn bounds(&mut self, msg: PoseStamped) -> Option<SeedRegion> {
        match self.clouds.remove(&msg.header.seq) {
            Some(cloud) => Some(SeedRegion { cloud, bounds: msg }),
            None => {
                self.bounds.insert(msg.header.seq, msg);
                None
            }
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
        if let Some(region) = self.pending_seeds.cloud(msg) {
            self.queue_seed(region);
        }
    }

    async fn on_seed_bounds(&mut self, msg: PoseStamped) {
        if let Some(region) = self.pending_seeds.bounds(msg) {
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
            self.config.viz_sweep_regions as usize,
        );
        loop {
            self.wake.notified().await;
            loop {
                let goal_changed = self.goal_changed.swap(false, Ordering::SeqCst);
                let update = self.pending.lock().expect("pending mutex").take();
                let live_update = match update {
                    Some(update) => {
                        self.apply_update(&mut planner, update, &mut viz, &mut last_viz_at)
                            .await
                    }
                    None => false,
                };
                if goal_changed || live_update {
                    self.maybe_replan(&mut planner, &mut last_path_at).await;
                }
                // Live updates apply first, then one seed region per pass, so a
                // seed never holds up the map around the robot. Seed regions
                // alone never replan, so a seed cannot flood the path topic.
                let seed = self.seed_regions.lock().expect("seed mutex").pop_front();
                let Some(seed) = seed else {
                    break;
                };
                if tokio::task::block_in_place(|| self.ingest_seed(&mut planner, seed)) {
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
        let applied = tokio::task::block_in_place(|| self.ingest(planner, update));
        if applied {
            self.publish_viz_if_due(planner, viz, last_viz_at).await;
        }
        applied
    }

    /// Publish the nodes and the surface and edge cells due this tick, rate
    /// capped to viz_publish_hz since building them is costly and unread by
    /// planning.
    async fn publish_viz_if_due(
        &self,
        planner: &Planner,
        viz: &mut RegionViz,
        last_viz_at: &mut Option<Instant>,
    ) {
        let now = Instant::now();
        let due = self.config.viz_publish_hz > 0.0 && {
            let viz_interval = Duration::from_secs_f32(1.0 / self.config.viz_publish_hz);
            last_viz_at.is_none_or(|t| now.duration_since(t) >= viz_interval)
        };
        if !due {
            return;
        }
        let (regions, node_cloud) =
            tokio::task::block_in_place(|| self.build_graph_messages(planner, viz));
        for (surface, edges) in &regions {
            publish_cloud(&self.surface_map, surface).await;
            publish_path(&self.node_edges, edges).await;
        }
        publish_cloud(&self.nodes, &node_cloud).await;
        debug!(regions = regions.len(), "viz published");
        *last_viz_at = Some(now);
    }

    /// Mutate the graph from a map update. False if the cloud was unusable.
    fn ingest(&self, planner: &mut Planner, update: MapUpdate) -> bool {
        match update {
            MapUpdate::Region { cloud, bounds } => {
                let points = match extract_xyz(&cloud) {
                    Ok(p) => p,
                    Err(e) => {
                        warn_throttled!(
                            Duration::from_secs(1),
                            error = %e,
                            "Failed to extract local map points, dropped a region update.",
                        );
                        return false;
                    }
                };
                let z_max = bounds.pose.orientation.z as f32;
                let Some((_, _, sensor_z)) = self.base_position() else {
                    warn!(
                        world_frame = %self.config.world_frame,
                        base_frame = %self.config.base_frame,
                        "No base pose on tf, dropped a region update.",
                    );
                    return false;
                };
                let region = RegionBounds::capped(
                    bounds.pose.position.x as f32,
                    bounds.pose.position.y as f32,
                    bounds.pose.orientation.x as f32,
                    bounds.pose.orientation.y as f32,
                    z_max,
                    sensor_z,
                    self.config.max_overhead_m,
                );

                let update_start = Instant::now();
                planner.update_region(&points, &region, &self.config);
                debug!(
                    update_ms = update_start.elapsed().as_secs_f64() * 1e3,
                    local_points = points.len(),
                    "local region processed"
                );
                true
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
                debug!(global_map_points = points.len(), "global_map processed");
                true
            }
        }
    }

    /// Apply one seed region through the region pipeline. Its bounds are the
    /// premap's own, so no sensor ceiling applies. False if unusable.
    fn ingest_seed(&self, planner: &mut Planner, seed: SeedRegion) -> bool {
        let points = match extract_xyz(&seed.cloud) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to extract seed region points, dropped a region.",
                );
                return false;
            }
        };
        let b = &seed.bounds.pose;
        let region = RegionBounds {
            origin_x: b.position.x as f32,
            origin_y: b.position.y as f32,
            radius: b.orientation.x as f32,
            z_min: b.orientation.y as f32,
            z_max: b.orientation.z as f32,
        };
        let update_start = Instant::now();
        planner.update_region(&points, &region, &self.config);
        debug!(
            update_ms = update_start.elapsed().as_secs_f64() * 1e3,
            seed_points = points.len(),
            "seed region processed"
        );
        true
    }

    /// The surface and edge messages of every cell due this tick, each with
    /// its cell in the header seq, plus the whole node cloud.
    fn build_graph_messages(
        &self,
        planner: &Planner,
        viz: &mut RegionViz,
    ) -> (Vec<(PointCloud2, Path)>, PointCloud2) {
        let frame = &self.config.world_frame;
        let stamp = now();
        let due = viz.tick(planner.surface_clearance(), planner.edge_segments());
        let regions = due
            .into_iter()
            .map(|(cell, content)| self.build_region_messages(cell, content, stamp.clone()))
            .collect();

        let node_points: Vec<Xyz> = planner.graph().nodes.iter().map(|n| n.pos).collect();
        let node_cloud = build_pc2_xyz(&node_points, frame, stamp);
        (regions, node_cloud)
    }

    fn build_region_messages(
        &self,
        cell: Cell,
        content: RegionContent,
        stamp: Time,
    ) -> (PointCloud2, Path) {
        let voxel_size = self.config.voxel_size;
        let frame = &self.config.world_frame;
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

        assert!(pairs.bounds(b1).is_none());
        assert!(pairs.bounds(b2).is_none());
        let first = pairs.cloud(c1).expect("region 1 pairs with its own bounds");
        assert_eq!(first.bounds.header.seq, 1);
        let second = pairs.cloud(c2).expect("region 2 pairs with its own bounds");
        assert_eq!(second.bounds.header.seq, 2);
        assert!(pairs.clouds.is_empty() && pairs.bounds.is_empty());
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
