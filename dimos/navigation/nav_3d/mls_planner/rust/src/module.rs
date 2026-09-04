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

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::mls_planner::{partition_cloud, CloudPartition, Config, MapTile, Planner, RegionBounds};
use crate::voxel::{surface_point_xyz, VoxelKey};
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, PointStamped, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::Notify;
use tracing::{debug, info, warn};

/// Grid spacing of full-map load tiles, sized so one tile applies quickly
/// between live updates.
const FULL_MAP_TILE_M: f32 = 4.0;

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

/// What `ingest` applied, so the load tracker can react to it.
enum AppliedUpdate {
    Region(RegionBounds),
    Global,
}

/// An in-progress tiled full-map load, advanced one tile per worker cycle.
struct MapLoad {
    tiles: Vec<MapTile>,
    next: usize,
    /// Regions applied since the load started. Tiles overlapping them are
    /// skipped, since the live content is fresher.
    regions: Vec<RegionBounds>,
}

impl MapLoad {
    /// Index of the next tile to apply, consuming skipped tiles.
    fn next_tile_index(&mut self) -> Option<usize> {
        while self.next < self.tiles.len() {
            let i = self.next;
            self.next += 1;
            let overlapped = self
                .regions
                .iter()
                .any(|r| cylinders_overlap(&self.tiles[i].bounds, r));
            if !overlapped {
                return Some(i);
            }
        }
        None
    }

    fn finished(&self) -> bool {
        self.next >= self.tiles.len()
    }
}

/// Extract and partition a full-map cloud. None when unusable or empty.
fn extract_and_partition(msg: &PointCloud2, voxel_size: f32) -> Option<CloudPartition> {
    let points = match extract_xyz(msg) {
        Ok(p) => p,
        Err(e) => {
            warn_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Failed to extract full map points, dropped a load.",
            );
            return None;
        }
    };
    if points.is_empty() {
        return None;
    }
    Some(partition_cloud(&points, FULL_MAP_TILE_M, voxel_size))
}

/// Whether two region cylinders intersect.
fn cylinders_overlap(a: &RegionBounds, b: &RegionBounds) -> bool {
    if a.z_max < b.z_min || b.z_max < a.z_min {
        return false;
    }
    let dx = a.origin_x - b.origin_x;
    let dy = a.origin_y - b.origin_y;
    let r = a.radius + b.radius;
    dx * dx + dy * dy <= r * r
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

    // Whole-map snapshot loaded tile by tile through the region pipeline,
    // between live updates. Live updates keep priority.
    #[input(decode = PointCloud2::decode, handler = on_full_map)]
    full_map: Input<PointCloud2>,

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

    // Written by the handle loop, read by the worker, so the loop never blocks
    // on map processing. The full-map partition has its own slot, so a live
    // update arriving first cannot clobber it.
    pending: Shared<MapUpdate>,
    pending_full_map: Shared<CloudPartition>,
    active_goal: Shared<Xyz>,
    wake: Arc<Notify>,

    worker: Option<tokio::task::JoinHandle<()>>,
}

impl MlsPlanner {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            pending: Arc::clone(&self.pending),
            pending_full_map: Arc::clone(&self.pending_full_map),
            active_goal: Arc::clone(&self.active_goal),
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

    /// Partition the cloud on a blocking thread, so neither the handle loop
    /// nor the worker stalls on a building-scale message.
    async fn on_full_map(&mut self, msg: PointCloud2) {
        let slot = Arc::clone(&self.pending_full_map);
        let wake = Arc::clone(&self.wake);
        let voxel_size = self.config.voxel_size;
        tokio::task::spawn_blocking(move || {
            let Some(part) = extract_and_partition(&msg, voxel_size) else {
                return;
            };
            *slot.lock().expect("full map mutex") = Some(part);
            wake.notify_one();
        });
    }

    async fn on_local_map(&mut self, msg: PointCloud2) {
        self.pending_local = Some(msg);
        self.try_pair();
    }

    async fn on_region_bounds(&mut self, msg: PoseStamped) {
        self.pending_bounds = Some(msg);
        self.try_pair();
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

    fn hand_off(&self, update: MapUpdate) {
        *self.pending.lock().expect("pending mutex") = Some(update);
        self.wake.notify_one();
    }

    /// Set or cancel the active goal from a click, then wake the worker.
    async fn on_goal(&mut self, msg: PointStamped) {
        *self.active_goal.lock().expect("goal mutex") = goal_position(&msg.point);
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
    pending_full_map: Shared<CloudPartition>,
    active_goal: Shared<Xyz>,
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
        let mut load: Option<MapLoad> = None;
        let mut last_path_at: Option<Instant> = None;
        let mut last_viz_at: Option<Instant> = None;
        loop {
            // An in-progress load keeps cycling, one tile per pass, checking
            // the mailboxes and goal between tiles. Live updates apply first.
            if load.is_none() {
                self.wake.notified().await;
            } else {
                tokio::task::yield_now().await;
            }
            let update = self.pending.lock().expect("pending mutex").take();
            if let Some(update) = update {
                match self
                    .apply_update(&mut planner, update, &mut last_viz_at)
                    .await
                {
                    Some(AppliedUpdate::Region(bounds)) => {
                        if let Some(l) = load.as_mut() {
                            l.regions.push(bounds);
                        }
                    }
                    // A full rebuild replaces everything a load would add.
                    Some(AppliedUpdate::Global) => load = None,
                    None => {}
                }
            }
            let full = self.pending_full_map.lock().expect("full map mutex").take();
            if let Some(part) = full {
                load = Some(self.start_load(&planner, part));
            }
            if let Some(l) = load.as_mut() {
                if let Some(i) = l.next_tile_index() {
                    let tile = &l.tiles[i];
                    let tile_start = Instant::now();
                    tokio::task::block_in_place(|| {
                        planner.update_region(&tile.points, &tile.bounds, &self.config)
                    });
                    debug!(
                        tile = i,
                        tile_ms = tile_start.elapsed().as_secs_f64() * 1e3,
                        "full map tile applied"
                    );
                    self.publish_viz_if_due(&planner, &mut last_viz_at).await;
                }
                if l.finished() {
                    info!("full map load finished");
                    load = None;
                }
            }
            self.maybe_replan(&mut planner, &mut last_path_at).await;
        }
    }

    /// Order the partitioned cloud against the current map into a tiled
    /// load, nearest the robot first.
    fn start_load(&self, planner: &Planner, part: CloudPartition) -> MapLoad {
        let center = self.base_position().map_or((0.0, 0.0), |(x, y, _)| (x, y));
        let tiles =
            tokio::task::block_in_place(|| planner.finish_partition(part, center, &self.config));
        info!(tiles = tiles.len(), "full map load started");
        MapLoad {
            tiles,
            next: 0,
            regions: Vec::new(),
        }
    }

    /// Apply one live update and refresh the viz artifacts.
    async fn apply_update(
        &self,
        planner: &mut Planner,
        update: MapUpdate,
        last_viz_at: &mut Option<Instant>,
    ) -> Option<AppliedUpdate> {
        let applied = tokio::task::block_in_place(|| self.ingest(planner, update));
        if applied.is_some() {
            self.publish_viz_if_due(planner, last_viz_at).await;
        }
        applied
    }

    /// Publish the surface, node, and edge viz artifacts, rate-capped to
    /// viz_publish_hz since building those clouds is costly and unread by
    /// planning.
    async fn publish_viz_if_due(&self, planner: &Planner, last_viz_at: &mut Option<Instant>) {
        let now = Instant::now();
        let due = self.config.viz_publish_hz > 0.0 && {
            let viz_interval = Duration::from_secs_f32(1.0 / self.config.viz_publish_hz);
            last_viz_at.is_none_or(|t| now.duration_since(t) >= viz_interval)
        };
        if !due {
            return;
        }
        let (surface, node_cloud, edges) =
            tokio::task::block_in_place(|| self.build_graph_messages(planner));
        publish_cloud(&self.surface_map, &surface).await;
        publish_cloud(&self.nodes, &node_cloud).await;
        publish_path(&self.node_edges, &edges).await;
        *last_viz_at = Some(now);
    }

    /// Mutate the graph from a map update. None if the cloud was unusable.
    fn ingest(&self, planner: &mut Planner, update: MapUpdate) -> Option<AppliedUpdate> {
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
                        return None;
                    }
                };
                let z_max = bounds.pose.orientation.z as f32;
                let Some((_, _, sensor_z)) = self.base_position() else {
                    warn!(
                        world_frame = %self.config.world_frame,
                        base_frame = %self.config.base_frame,
                        "No base pose on tf, dropped a region update.",
                    );
                    return None;
                };
                let bounds = RegionBounds::capped(
                    bounds.pose.position.x as f32,
                    bounds.pose.position.y as f32,
                    bounds.pose.orientation.x as f32,
                    bounds.pose.orientation.y as f32,
                    z_max,
                    sensor_z,
                    self.config.max_overhead_m,
                );

                let update_start = Instant::now();
                planner.update_region(&points, &bounds, &self.config);
                debug!(
                    update_ms = update_start.elapsed().as_secs_f64() * 1e3,
                    local_points = points.len(),
                    "local region processed"
                );
                Some(AppliedUpdate::Region(bounds))
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
                        return None;
                    }
                };
                if points.is_empty() {
                    return None;
                }
                planner.update_global_map(&points, &self.config);
                debug!(global_map_points = points.len(), "global_map processed");
                Some(AppliedUpdate::Global)
            }
        }
    }

    fn build_graph_messages(&self, planner: &Planner) -> (PointCloud2, PointCloud2, Path) {
        let voxel_size = self.config.voxel_size;
        let frame = &self.config.world_frame;
        let graph = planner.graph();

        let surface_points: Vec<Xyzi> = planner
            .surface_clearance()
            .into_iter()
            .map(|((ix, iy, iz), clearance)| {
                let (x, y, z) = surface_point_xyz(ix, iy, iz, voxel_size);
                (x, y, z, clearance)
            })
            .collect();
        let surface = build_pc2_xyzi(&surface_points, frame, now());

        let node_points: Vec<Xyz> = graph.nodes.iter().map(|n| n.pos).collect();
        let node_cloud = build_pc2_xyz(&node_points, frame, now());

        let edges = build_segments_path(planner.edge_segments(), voxel_size, frame, now());
        (surface, node_cloud, edges)
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

fn now() -> Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    Time {
        sec: dur.as_secs().min(i32::MAX as u64) as i32,
        nsec: dur.subsec_nanos() as i32,
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

    fn point(x: f64, y: f64, z: f64) -> Point {
        Point { x, y, z }
    }

    fn cyl(x: f32, y: f32, r: f32) -> RegionBounds {
        RegionBounds {
            origin_x: x,
            origin_y: y,
            radius: r,
            z_min: -1.0,
            z_max: 1.0,
        }
    }

    #[test]
    fn cylinders_overlap_by_distance_and_z() {
        assert!(cylinders_overlap(&cyl(0.0, 0.0, 1.0), &cyl(1.5, 0.0, 1.0)));
        assert!(!cylinders_overlap(&cyl(0.0, 0.0, 1.0), &cyl(3.0, 0.0, 1.0)));
        let mut high = cyl(0.0, 0.0, 1.0);
        high.z_min = 2.0;
        high.z_max = 3.0;
        assert!(!cylinders_overlap(&cyl(0.0, 0.0, 1.0), &high));
    }

    #[test]
    fn map_load_skips_tiles_overlapping_applied_regions() {
        let tile = |x: f32| MapTile {
            bounds: cyl(x, 0.0, 1.0),
            points: Vec::new(),
        };
        let mut load = MapLoad {
            tiles: vec![tile(0.0), tile(2.0), tile(4.0)],
            next: 0,
            regions: Vec::new(),
        };
        assert_eq!(load.next_tile_index(), Some(0));
        // A live region lands on the second tile before it applies.
        load.regions.push(cyl(2.0, 0.0, 0.5));
        assert_eq!(load.next_tile_index(), Some(2), "overlapped tile skipped");
        assert!(load.finished());
        assert_eq!(load.next_tile_index(), None);
    }

    /// The full-map slot is separate from the live-update slot, so a region
    /// frame arriving before the worker wakes cannot clobber a pending load.
    #[test]
    fn full_map_slot_survives_a_region_hand_off() {
        let pending: Shared<MapUpdate> = Arc::new(Mutex::new(None));
        let pending_full_map: Shared<CloudPartition> = Arc::new(Mutex::new(None));

        *pending_full_map.lock().unwrap() = Some(partition_cloud(&[(0.5, 0.5, 0.5)], 4.0, 0.1));
        *pending.lock().unwrap() = Some(MapUpdate::Region {
            cloud: PointCloud2::default(),
            bounds: PoseStamped::default(),
        });

        assert!(pending.lock().unwrap().take().is_some());
        assert!(
            pending_full_map.lock().unwrap().take().is_some(),
            "pending load must survive the region hand-off"
        );
    }

    #[test]
    fn goal_position_passes_finite_and_cancels_on_non_finite() {
        assert_eq!(goal_position(&point(1.0, 2.0, 3.0)), Some((1.0, 2.0, 3.0)));
        assert_eq!(goal_position(&point(f64::NAN, 0.0, 0.0)), None);
        assert_eq!(goal_position(&point(0.0, f64::INFINITY, 0.0)), None);
        assert_eq!(goal_position(&point(0.0, 0.0, f64::NEG_INFINITY)), None);
    }
}
