// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_mls_planner::edges::{edges_to_segments, PlannerGraph};
use dimos_mls_planner::mls_planner::{Config, Planner, RegionBounds};
use dimos_mls_planner::voxel::surface_point_xyz;
use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tokio::sync::Notify;
use tracing::debug;

/// A point in the planner's world frame.
type Xyz = (f32, f32, f32);

/// State shared between the handle loop and the worker. The handle loop writes
/// the newest value, the worker reads it.
type Shared<T> = Arc<Mutex<Option<T>>>;

/// A coalesced map input handed from the handle loop to the worker. Only the
/// newest is kept, so a dropped intermediate frame is harmless.
enum MapUpdate {
    Region {
        cloud: PointCloud2,
        bounds: PoseStamped,
    },
    Global {
        cloud: PointCloud2,
    },
}

#[derive(Module)]
#[module(setup = spawn_worker)]
struct MlsPlanner {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    // Incremental path: a local map slice paired by stamp with the region
    // bounds it covers, published by the ray tracer alongside local_map.
    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = PoseStamped::decode, handler = on_region_bounds)]
    region_bounds: Input<PoseStamped>,

    #[input(decode = PoseStamped::decode, handler = on_start_pose)]
    start_pose: Input<PoseStamped>,

    #[input(decode = PoseStamped::decode, handler = on_goal_pose)]
    goal_pose: Input<PoseStamped>,

    #[output(encode = PointCloud2::encode)]
    surface_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    nodes: Output<PointCloud2>,

    #[output(encode = Path::encode)]
    node_edges: Output<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    #[config]
    config: Config,

    // Handle-loop-local: pair a local map with its stamp-matching bounds before
    // handing the matched snapshot to the worker.
    pending_local: Option<PointCloud2>,
    pending_bounds: Option<PoseStamped>,

    // Shared with the worker task. The handle loop only writes these and wakes
    // the worker, so it never blocks on the heavy map processing.
    pending: Shared<MapUpdate>,
    latest_start: Shared<Xyz>,
    active_goal: Shared<Xyz>,
    wake: Arc<Notify>,
}

impl MlsPlanner {
    /// Spawn the worker that owns the planner graph and does all heavy map
    /// processing, so the handle loop stays free to drain its inputs.
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            pending: Arc::clone(&self.pending),
            latest_start: Arc::clone(&self.latest_start),
            active_goal: Arc::clone(&self.active_goal),
            wake: Arc::clone(&self.wake),
            config: self.config.clone(),
            surface_map: self.surface_map.clone(),
            nodes: self.nodes.clone(),
            node_edges: self.node_edges.clone(),
            path: self.path.clone(),
        };
        tokio::spawn(worker.run());
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

    /// Hand a local map and its stamp-matching bounds to the worker once both
    /// are in hand. Cheap stamp compare. Runs on the handle loop.
    fn try_pair(&mut self) {
        let (Some(bounds_msg), Some(cloud)) = (&self.pending_bounds, &self.pending_local) else {
            return;
        };
        if !same_stamp(&bounds_msg.header.stamp, &cloud.header.stamp) {
            return;
        }
        let bounds = self.pending_bounds.take().expect("checked above");
        let cloud = self.pending_local.take().expect("checked above");
        self.hand_off(MapUpdate::Region { cloud, bounds });
    }

    /// Store the newest map input (coalescing over any unprocessed one) and
    /// wake the worker.
    fn hand_off(&self, update: MapUpdate) {
        *self.pending.lock().expect("pending mutex") = Some(update);
        self.wake.notify_one();
    }

    /// Store-only: record the latest start pose. The worker reads it when it
    /// replans. No wake here, so odometry never drives replanning.
    async fn on_start_pose(&mut self, msg: PoseStamped) {
        let p = &msg.pose.position;
        *self.latest_start.lock().expect("start mutex") =
            Some((p.x as f32, p.y as f32, p.z as f32));
    }

    /// Arm the active goal, or clear it on a non-finite goal (the cancel
    /// signal), then wake the worker so a fresh click replans immediately
    /// against the current graph. A goal arrives once per click, so this is not
    /// the odometry-rate external trigger the refactor removed.
    async fn on_goal_pose(&mut self, msg: PoseStamped) {
        let p = &msg.pose.position;
        let goal = (p.x as f32, p.y as f32, p.z as f32);
        *self.active_goal.lock().expect("goal mutex") =
            (goal.0.is_finite() && goal.1.is_finite() && goal.2.is_finite()).then_some(goal);
        self.wake.notify_one();
    }
}

/// Owns the planner graph and does every map mutation, graph publish, and
/// replan off the handle loop. Woken by the handlers via `wake`.
struct Worker {
    pending: Shared<MapUpdate>,
    latest_start: Shared<Xyz>,
    active_goal: Shared<Xyz>,
    wake: Arc<Notify>,
    config: Config,
    surface_map: Output<PointCloud2>,
    nodes: Output<PointCloud2>,
    node_edges: Output<Path>,
    path: Output<Path>,
}

impl Worker {
    async fn run(self) {
        let mut planner = Planner::default();
        let mut last_path_at: Option<Instant> = None;
        let mut last_viz_at: Option<Instant> = None;
        loop {
            self.wake.notified().await;
            // Coalesced: take the newest map input, dropping any intermediates.
            let update = self.pending.lock().expect("pending mutex").take();
            if let Some(update) = update {
                self.apply_update(&mut planner, update, &mut last_viz_at)
                    .await;
            }
            self.maybe_replan(&mut planner, &mut last_path_at).await;
        }
    }

    /// Mutate the graph from a map update, then publish the viz artifacts at no
    /// more than `viz_publish_hz`. The graph itself is always updated; only the
    /// surface_map / nodes / node_edges rebuild + publish is rate-capped, since
    /// rebuilding the whole graph every cycle is the dominant per-cycle cost and
    /// nothing on the planning path consumes those outputs. The CPU-bound
    /// section runs under `block_in_place` so the runtime can still schedule the
    /// handle loop on another thread.
    async fn apply_update(
        &self,
        planner: &mut Planner,
        update: MapUpdate,
        last_viz_at: &mut Option<Instant>,
    ) {
        let now = Instant::now();
        let viz_due = self.config.viz_publish_hz > 0.0 && {
            let viz_interval = Duration::from_secs_f32(1.0 / self.config.viz_publish_hz);
            last_viz_at.is_none_or(|t| now.duration_since(t) >= viz_interval)
        };

        let messages = tokio::task::block_in_place(|| {
            let updated = self.ingest(planner, update);
            (updated && viz_due).then(|| self.build_graph_messages(planner))
        });

        if let Some((surface, node_cloud, edges)) = messages {
            publish_cloud(&self.surface_map, &surface).await;
            publish_cloud(&self.nodes, &node_cloud).await;
            publish_path(&self.node_edges, &edges).await;
            *last_viz_at = Some(now);
        }
    }

    /// Pure-CPU graph mutation from a map update. Returns whether the graph was
    /// updated (false if the cloud was unusable).
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
                let bounds = RegionBounds {
                    origin_x: bounds.pose.position.x as f32,
                    origin_y: bounds.pose.position.y as f32,
                    radius: bounds.pose.orientation.x as f32,
                    z_min: bounds.pose.orientation.y as f32,
                    z_max: bounds.pose.orientation.z as f32,
                };

                let update_start = Instant::now();
                planner.update_region(&points, &bounds, &self.config);
                debug!(
                    update_ms = update_start.elapsed().as_secs_f64() * 1e3,
                    local_points = points.len(),
                    "local region processed"
                );
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
            }
        }
        true
    }

    fn build_graph_messages(&self, planner: &Planner) -> (PointCloud2, PointCloud2, Path) {
        let voxel_size = self.config.voxel_size;
        let frame = &self.config.world_frame;
        let graph = planner.graph();

        let surface_points: Vec<Xyz> = planner
            .surface()
            .map(|(ix, iy, iz)| surface_point_xyz(ix, iy, iz, voxel_size))
            .collect();
        let surface = build_pc2_xyz(&surface_points, frame, now());

        let node_points: Vec<Xyz> = graph.nodes.iter().map(|n| n.pos).collect();
        let node_cloud = build_pc2_xyz(&node_points, frame, now());

        let edges = build_segments_path(graph, voxel_size, frame, now());
        (surface, node_cloud, edges)
    }

    /// Replan from the latest start to the active goal. Pure glue: it gates and
    /// does IO, all planning lives in `Planner::plan`.
    async fn maybe_replan(&self, planner: &mut Planner, last_path_at: &mut Option<Instant>) {
        let start = *self.latest_start.lock().expect("start mutex");
        let goal = *self.active_goal.lock().expect("goal mutex");
        let (Some(start), Some(goal)) = (start, goal) else {
            return;
        };
        if is_at_goal(start, goal, self.config.goal_tolerance) {
            *self.active_goal.lock().expect("goal mutex") = None;
            return;
        }

        let plan_start = Instant::now();
        let waypoints = tokio::task::block_in_place(|| planner.plan(start, goal, &self.config));
        let waypoints = match waypoints {
            Some(wp) => wp,
            None => {
                tracing::warn!(?start, ?goal, "no path between start and goal");
                publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
                return;
            }
        };
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

/// True when start is within `tol` of goal in the ground plane.
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
fn build_segments_path(plg: &PlannerGraph, voxel_size: f32, frame_id: &str, stamp: Time) -> Path {
    let segments = edges_to_segments(&plg.cells, &plg.cell_state, &plg.node_edges);
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

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<MlsPlanner, _>(transport).await;
}
