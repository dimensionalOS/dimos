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

use crate::mls_planner::{Config, Planner, RegionBounds};
use crate::voxel::{surface_point_xyz, VoxelKey};
use dimos_generated_messages::builtin_interfaces::msg::Time;
use dimos_generated_messages::dimos_msgs::msg::{LineSegment3D, LineSegments3D};
use dimos_generated_messages::geometry_msgs::msg::{
    Point, PointStamped, Pose, PoseStamped, Quaternion,
};
use dimos_generated_messages::nav_msgs::msg::Path;
use dimos_generated_messages::sensor_msgs::msg::{PointCloud2, PointField};
use dimos_generated_messages::std_msgs::msg::Header;
use dimos_module::cdr;
use dimos_module::pointcloud::xyz_points as extract_xyz;
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
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

#[derive(Module)]
#[module(name = "mls_planner", setup = spawn_worker, teardown = stop_worker)]
pub struct MlsPlanner {
    #[input(decode = cdr::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = cdr::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = cdr::decode, handler = on_region_bounds)]
    region_bounds: Input<PoseStamped>,

    #[input(decode = cdr::decode, handler = on_goal)]
    goal: Input<PointStamped>,

    #[tf]
    tf: Tf,

    #[output(encode = cdr::encode)]
    surface_map: Output<PointCloud2>,

    #[output(encode = cdr::encode)]
    nodes: Output<PointCloud2>,

    #[output(encode = cdr::encode)]
    node_edges: Output<LineSegments3D>,

    #[output(encode = cdr::encode)]
    path: Output<Path>,

    #[config]
    config: Config,

    // Held on the handle loop until stamps match, then handed off paired.
    pending_local: Option<PointCloud2>,
    pending_bounds: Option<PoseStamped>,

    // Written by the handle loop, read by the worker, so the loop never blocks
    // on map processing.
    pending: Shared<MapUpdate>,
    active_goal: Shared<Xyz>,
    wake: Arc<Notify>,

    worker: Option<tokio::task::JoinHandle<()>>,
}

impl MlsPlanner {
    async fn spawn_worker(&mut self) {
        let worker = Worker {
            pending: Arc::clone(&self.pending),
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
    active_goal: Shared<Xyz>,
    wake: Arc<Notify>,
    tf: Tf,
    config: Config,
    surface_map: Output<PointCloud2>,
    nodes: Output<PointCloud2>,
    node_edges: Output<LineSegments3D>,
    path: Output<Path>,
}

impl Worker {
    async fn run(self) {
        let mut planner = Planner::new(self.config.worker_threads);
        let mut last_path_at: Option<Instant> = None;
        let mut last_viz_at: Option<Instant> = None;
        loop {
            self.wake.notified().await;
            let update = self.pending.lock().expect("pending mutex").take();
            if let Some(update) = update {
                self.apply_update(&mut planner, update, &mut last_viz_at)
                    .await;
            }
            self.maybe_replan(&mut planner, &mut last_path_at).await;
        }
    }

    /// Update the graph every cycle, but rate-cap viz publishing to
    /// viz_publish_hz since building those clouds is costly and unread by planning.
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
            if let Err(error) = self.node_edges.publish(&edges).await {
                warn!(%error, "edge segments failed to publish");
            }
            *last_viz_at = Some(now);
        }
    }

    /// Mutate the graph from a map update. Returns false if the cloud was
    /// unusable.
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

    fn build_graph_messages(
        &self,
        planner: &Planner,
    ) -> (PointCloud2, PointCloud2, LineSegments3D) {
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

        let edges = build_segments(planner.edge_segments(), voxel_size, frame, now());
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
    a.sec == b.sec && a.nanosec == b.nanosec
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
        nanosec: dur.subsec_nanos(),
    }
}

fn header(frame_id: &str, stamp: Time) -> Header {
    Header {
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

/// Emit explicit endpoints and weights in the dedicated custom message.
fn build_segments(
    segments: Vec<(VoxelKey, VoxelKey, f32)>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> LineSegments3D {
    let point = |key: VoxelKey| {
        let (x, y, z) = surface_point_xyz(key.0, key.1, key.2, voxel_size);
        Point {
            x: x as f64,
            y: y as f64,
            z: z as f64,
        }
    };
    LineSegments3D {
        header: header(frame_id, stamp),
        segments: segments
            .into_iter()
            .map(|(a, b, cost)| LineSegment3D {
                start: point(a),
                end: point(b),
                weight: cost as f64,
            })
            .collect(),
    }
}

/// Like `build_pc2_xyz` plus an `intensity` float carrying the cell's wall clearance.
fn build_pc2_xyzi(points: &[Xyzi], frame_id: &str, stamp: Time) -> PointCloud2 {
    let n = points.len() as u32;
    let mut data = Vec::with_capacity(points.len() * 16);
    for &(x, y, z, i) in points {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&i.to_le_bytes());
    }
    let make_field = |name: &str, off: u32| PointField {
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
    let n = points.len() as u32;
    let mut data = Vec::with_capacity(points.len() * 12);
    for &(x, y, z) in points {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
    }
    let make_field = |name: &str, off: u32| PointField {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn graph_edges_encode_explicit_endpoints_and_cost() {
        let stamp = Time {
            sec: 1700000000,
            nanosec: 123456789,
        };
        let message = build_segments(vec![((0, 1, 2), (2, 3, 4), 3.5)], 0.5, "map", stamp.clone());
        let decoded: LineSegments3D = cdr::decode(&cdr::encode(&message).unwrap()).unwrap();
        assert_eq!(decoded.header.frame_id, "map");
        assert_eq!(decoded.header.stamp, stamp);
        assert_eq!(decoded.segments.len(), 1);
        assert_eq!(
            decoded.segments[0].start,
            Point {
                x: 0.25,
                y: 0.75,
                z: 1.5
            }
        );
        assert_eq!(
            decoded.segments[0].end,
            Point {
                x: 1.25,
                y: 1.75,
                z: 2.5
            }
        );
        assert_eq!(decoded.segments[0].weight, 3.5);
    }

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
        let s = Time { sec: 2, nanosec: 3 };
        let b = bounds_at(s.clone());
        let c = cloud_at(s);
        assert!(stamps_paired(Some(&b), Some(&c)));

        let other = cloud_at(Time { sec: 2, nanosec: 4 });
        assert!(!stamps_paired(Some(&b), Some(&other)));

        assert!(!stamps_paired(Some(&b), None));
        assert!(!stamps_paired(None, Some(&c)));
        assert!(!stamps_paired(None, None));
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
