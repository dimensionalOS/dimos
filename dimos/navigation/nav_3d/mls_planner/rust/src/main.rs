// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

mod adjacency;
mod dijkstra;
mod edges;
mod nodes;
mod planner;
mod surfaces;
mod voxel;

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use serde::Deserialize;
use tracing::info;
use validator::Validate;

use ahash::AHashSet;

use crate::adjacency::{build_surface_cells, build_surface_lookup};
use crate::edges::{build_node_edges, edges_to_segments, PlannerGraph};
use crate::nodes::place_nodes;
use crate::surfaces::{extract_surfaces, ColumnIz};
use crate::voxel::{surface_point_xyz, voxelize, VoxelKey};

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
struct Config {
    world_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    voxel_size: f32,
    #[validate(range(exclusive_min = 0.0))]
    robot_height: f32,
    #[validate(range(min = 0))]
    surface_dilation_passes: u32,
    #[validate(range(min = 0))]
    surface_erosion_passes: u32,
    #[validate(range(exclusive_min = 0.0))]
    node_spacing_m: f32,
    #[validate(range(min = 0.0))]
    node_wall_buffer_m: f32,
    #[validate(range(min = 0.0))]
    node_step_threshold_m: f32,
}

#[derive(Module)]
#[module(setup = setup)]
struct MlsPlanner {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

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

    clearance_cells: i32,
    step_cells: i32,
    planner_graph: Option<PlannerGraph>,
    latest_start: Option<(f32, f32, f32)>,

    voxel_map_buf: AHashSet<VoxelKey>,
    by_col_buf: ColumnIz,
    surface_buf: Vec<VoxelKey>,
}

impl MlsPlanner {
    async fn setup(&mut self) {
        let cfg = &self.config;
        self.clearance_cells = (cfg.robot_height / cfg.voxel_size).ceil() as i32;
        self.step_cells = (cfg.node_step_threshold_m / cfg.voxel_size).floor() as i32;

        info!(
            world_frame = %cfg.world_frame,
            voxel_size = cfg.voxel_size,
            robot_height = cfg.robot_height,
            clearance_cells = self.clearance_cells,
            step_cells = self.step_cells,
            "mls_planner ready",
        );
    }

    async fn on_global_map(&mut self, msg: PointCloud2) {
        let points = match extract_xyz(&msg) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to extract lidar points, dropped a cloud.",
                );
                return;
            }
        };
        if points.is_empty() {
            return;
        }

        let voxel_size = self.config.voxel_size;
        let step_cells = self.step_cells;
        let clearance_cells = self.clearance_cells;
        let dil = self.config.surface_dilation_passes;
        let ero = self.config.surface_erosion_passes;
        let spacing = self.config.node_spacing_m;
        let wall_buf = self.config.node_wall_buffer_m;
        let frame = self.config.world_frame.clone();

        let t_surface = Instant::now();
        self.voxel_map_buf.clear();
        for &p in &points {
            self.voxel_map_buf.insert(voxelize(p, voxel_size));
        }
        let voxels_count = self.voxel_map_buf.len();

        extract_surfaces(
            &self.voxel_map_buf,
            clearance_cells,
            dil,
            ero,
            &mut self.by_col_buf,
            &mut self.surface_buf,
        );
        let surface_count = self.surface_buf.len();

        let plg = self.planner_graph.get_or_insert_with(PlannerGraph::new);
        build_surface_lookup(&self.surface_buf, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, voxel_size, step_cells);
        let surface_ms = ms(t_surface.elapsed());

        let surface_points: Vec<(f32, f32, f32)> = self
            .surface_buf
            .iter()
            .map(|&(ix, iy, iz)| surface_point_xyz(ix, iy, iz, voxel_size))
            .collect();
        publish_cloud(&self.surface_map, &surface_points, &frame, now()).await;

        let t_nodes = Instant::now();
        let plg = self.planner_graph.as_mut().expect("just inserted");
        place_nodes(
            &mut plg.cells,
            voxel_size,
            spacing,
            wall_buf,
            &mut plg.cell_state,
            &mut plg.nodes,
        );
        let nodes_ms = ms(t_nodes.elapsed());
        let nodes_count = plg.nodes.len();

        let node_points: Vec<(f32, f32, f32)> = plg.nodes.iter().map(|n| n.pos).collect();
        publish_cloud(&self.nodes, &node_points, &frame, now()).await;

        let t_edges = Instant::now();
        let plg = self.planner_graph.as_mut().expect("just inserted");
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );
        let edges_ms = ms(t_edges.elapsed());
        let edges_count = plg.node_edges.len();

        let edges_path = build_segments_path(plg, voxel_size, &frame, now());
        publish_path(&self.node_edges, &edges_path).await;

        info!(
            global_map_points = points.len(),
            voxels = voxels_count,
            surface_cells = surface_count,
            nodes = nodes_count,
            edges = edges_count,
            surface_ms,
            nodes_ms,
            edges_ms,
            "global_map processed",
        );
    }

    async fn on_start_pose(&mut self, msg: PoseStamped) {
        let p = &msg.pose.position;
        self.latest_start = Some((p.x as f32, p.y as f32, p.z as f32));
        // Drop any previous plan so the visualizer doesn't show a stale path
        // rooted at the old start.
        publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
    }

    async fn on_goal_pose(&mut self, msg: PoseStamped) {
        let Some(start) = self.latest_start else {
            tracing::warn!("MLSPlanner received goal before start; skipping");
            return;
        };
        let Some(plg) = self.planner_graph.as_ref() else {
            tracing::warn!("MLSPlanner received goal before graph was built; skipping");
            return;
        };
        if plg.nodes.is_empty() {
            tracing::warn!("MLSPlanner received goal before graph had nodes; skipping");
            return;
        }

        let p = &msg.pose.position;
        let goal = (p.x as f32, p.y as f32, p.z as f32);

        let t_plan = Instant::now();
        let waypoints = match planner::plan(
            plg,
            start,
            goal,
            self.config.voxel_size,
            self.config.robot_height,
        ) {
            Some(wp) => wp,
            None => {
                tracing::warn!(?start, ?goal, "no path between start and goal");
                publish_path(&self.path, &empty_path(&self.config.world_frame, now())).await;
                return;
            }
        };
        let plan_ms = ms(t_plan.elapsed());

        let stamp = now();
        let path_msg = build_path_from_waypoints(&waypoints, &self.config.world_frame, stamp);
        info!(waypoints = waypoints.len(), plan_ms, "path planned");
        publish_path(&self.path, &path_msg).await;
    }
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

async fn publish_cloud(
    out: &Output<PointCloud2>,
    points: &[(f32, f32, f32)],
    frame_id: &str,
    stamp: Time,
) {
    let cloud = build_pc2_xyz(points, frame_id, stamp);
    if let Err(e) = out.publish(&cloud).await {
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
