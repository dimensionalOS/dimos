// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::cmp::Ordering;
use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use dimos_module::{run, Input, LcmTransport, Module, Output};
use glam::{Mat4, Quat, Vec3};
use lcm_msgs::geometry_msgs::PoseStamped;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::Header;
use rayon::prelude::*;
use serde::Deserialize;

mod entity;
use entity::{raycast as raycast_entity, Entity, EntityStateBatch};

const LEAF_TRIANGLES: usize = 16;
const RAY_EPSILON: f32 = 1.0e-6;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    scene_metadata_path: String,
    collision_path: Option<String>,
    hz: f32,
    horizontal_samples: usize,
    vertical_samples: usize,
    elevation_min_deg: f32,
    elevation_max_deg: f32,
    max_range: f32,
    sensor_x: f32,
    sensor_y: f32,
    sensor_z: f32,
    yaw_offset_deg: f32,
    output_voxel_size: f32,
}

#[derive(Debug, Deserialize)]
struct SceneMeta {
    alignment: Alignment,
    artifacts: Artifacts,
}

#[derive(Debug, Deserialize)]
struct Artifacts {
    browser_collision: Option<String>,
}

#[derive(Debug, Deserialize)]
struct Alignment {
    scale: f32,
    rotation_zyx_deg: [f32; 3],
    translation: [f32; 3],
    y_up: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct Triangle {
    a: Vec3,
    b: Vec3,
    c: Vec3,
    min: Vec3,
    max: Vec3,
    centroid: Vec3,
}

impl Triangle {
    fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let min = a.min(b).min(c);
        let max = a.max(b).max(c);
        let centroid = (a + b + c) / 3.0;
        Self {
            a,
            b,
            c,
            min,
            max,
            centroid,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct BvhNode {
    min: Vec3,
    max: Vec3,
    left: Option<usize>,
    right: Option<usize>,
    start: usize,
    len: usize,
}

#[derive(Debug, Default)]
struct Bvh {
    nodes: Vec<BvhNode>,
    indices: Vec<usize>,
}

impl Bvh {
    fn build(triangles: &[Triangle]) -> Self {
        let mut bvh = Self {
            nodes: Vec::with_capacity(triangles.len().saturating_mul(2)),
            indices: (0..triangles.len()).collect(),
        };
        if !triangles.is_empty() {
            bvh.build_node(0, triangles.len(), triangles);
        }
        bvh
    }

    fn build_node(&mut self, start: usize, len: usize, triangles: &[Triangle]) -> usize {
        let node_index = self.nodes.len();
        let (min, max) = self.bounds(start, len, triangles);
        self.nodes.push(BvhNode {
            min,
            max,
            start,
            len,
            ..BvhNode::default()
        });

        if len <= LEAF_TRIANGLES {
            return node_index;
        }

        let (centroid_min, centroid_max) = self.centroid_bounds(start, len, triangles);
        let extent = centroid_max - centroid_min;
        let axis = if extent.x >= extent.y && extent.x >= extent.z {
            0
        } else if extent.y >= extent.z {
            1
        } else {
            2
        };

        self.indices[start..start + len].sort_by(|left, right| {
            let a = triangles[*left].centroid[axis];
            let b = triangles[*right].centroid[axis];
            a.partial_cmp(&b).unwrap_or(Ordering::Equal)
        });

        let mid = start + len / 2;
        if mid == start || mid == start + len {
            return node_index;
        }

        let left = self.build_node(start, mid - start, triangles);
        let right = self.build_node(mid, start + len - mid, triangles);
        self.nodes[node_index].left = Some(left);
        self.nodes[node_index].right = Some(right);
        self.nodes[node_index].len = 0;
        node_index
    }

    fn bounds(&self, start: usize, len: usize, triangles: &[Triangle]) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for &tri_index in &self.indices[start..start + len] {
            let tri = triangles[tri_index];
            min = min.min(tri.min);
            max = max.max(tri.max);
        }
        (min, max)
    }

    fn centroid_bounds(&self, start: usize, len: usize, triangles: &[Triangle]) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for &tri_index in &self.indices[start..start + len] {
            let c = triangles[tri_index].centroid;
            min = min.min(c);
            max = max.max(c);
        }
        (min, max)
    }

    fn raycast(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_range: f32,
        triangles: &[Triangle],
    ) -> Option<(Vec3, f32)> {
        if self.nodes.is_empty() {
            return None;
        }

        let mut closest = max_range;
        let mut hit = None;
        let mut stack = vec![0usize];

        while let Some(node_index) = stack.pop() {
            let node = self.nodes[node_index];
            if !intersect_aabb(origin, direction, node.min, node.max, closest) {
                continue;
            }

            if let (Some(left), Some(right)) = (node.left, node.right) {
                stack.push(left);
                stack.push(right);
                continue;
            }

            for &tri_index in &self.indices[node.start..node.start + node.len] {
                if let Some(t) = intersect_triangle(origin, direction, triangles[tri_index]) {
                    if t > 0.0 && t < closest {
                        closest = t;
                        hit = Some(origin + direction * t);
                    }
                }
            }
        }

        hit.map(|p| (p, closest))
    }
}

#[derive(Debug, Default)]
struct SceneAccel {
    triangles: Vec<Triangle>,
    bvh: Bvh,
}

impl SceneAccel {
    fn load(config: &Config) -> Self {
        let metadata_path = PathBuf::from(&config.scene_metadata_path);
        let meta_text = std::fs::read_to_string(&metadata_path).unwrap_or_else(|e| {
            panic!(
                "scene_lidar: failed to read scene metadata {}: {e}",
                metadata_path.display()
            )
        });
        let meta: SceneMeta = serde_json::from_str(&meta_text).unwrap_or_else(|e| {
            panic!(
                "scene_lidar: failed to parse scene metadata {}: {e}",
                metadata_path.display()
            )
        });

        let collision_path = resolve_collision_path(config, &meta, &metadata_path);
        let triangles = load_gltf_triangles(&collision_path, &meta.alignment);
        if triangles.is_empty() {
            panic!(
                "scene_lidar: collision mesh has no triangles: {}",
                collision_path.display()
            );
        }

        let bvh = Bvh::build(&triangles);
        eprintln!(
            "scene_lidar: loaded {} triangles, {} bvh nodes from {}",
            triangles.len(),
            bvh.nodes.len(),
            collision_path.display()
        );
        Self { triangles, bvh }
    }

    fn raycast(&self, origin: Vec3, direction: Vec3, max_range: f32) -> Option<(Vec3, f32)> {
        self.bvh
            .raycast(origin, direction, max_range, &self.triangles)
    }
}

#[derive(Module)]
#[module(setup = setup)]
struct SceneLidar {
    #[input(decode = PoseStamped::decode, handler = on_pose)]
    pose: Input<PoseStamped>,

    #[input(decode = EntityStateBatch::decode, handler = on_entities)]
    entity_states: Input<EntityStateBatch>,

    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[config]
    config: Config,

    scene: SceneAccel,
    directions: Vec<Vec3>,
    last_scan: Option<Instant>,
    entities: Vec<Entity>,
}

impl SceneLidar {
    async fn setup(&mut self) {
        validate_config(&self.config);
        self.scene = SceneAccel::load(&self.config);
        self.directions = lidar_directions(&self.config);
        eprintln!(
            "scene_lidar: configured {} rays at {:.1} Hz, max_range {:.2} m",
            self.directions.len(),
            self.config.hz,
            self.config.max_range
        );
    }

    async fn on_pose(&mut self, msg: PoseStamped) {
        let now = Instant::now();
        let interval = Duration::from_secs_f32(1.0 / self.config.hz);
        if self
            .last_scan
            .is_some_and(|last_scan| now.duration_since(last_scan) < interval)
        {
            return;
        }
        self.last_scan = Some(now);

        let orientation = pose_quat(&msg);
        let sensor_offset = Vec3::new(
            self.config.sensor_x,
            self.config.sensor_y,
            self.config.sensor_z,
        );
        let origin = Vec3::new(
            msg.pose.position.x as f32,
            msg.pose.position.y as f32,
            msg.pose.position.z as f32,
        ) + orientation * sensor_offset;

        let yaw_offset = Quat::from_rotation_z(self.config.yaw_offset_deg.to_radians());
        let max_range = self.config.max_range;
        let entities: &[Entity] = &self.entities;
        let hits: Vec<(Vec3, f32)> = self
            .directions
            .par_iter()
            .filter_map(|direction| {
                let world_direction = (orientation * yaw_offset * *direction).normalize();
                let mut best = self.scene.raycast(origin, world_direction, max_range);
                let mut best_dist = best.map(|(_, d)| d).unwrap_or(max_range);
                for entity in entities {
                    if let Some((hit, dist)) =
                        raycast_entity(entity, origin, world_direction, best_dist)
                    {
                        if dist < best_dist {
                            best_dist = dist;
                            best = Some((hit, dist));
                        }
                    }
                }
                best
            })
            .collect();

        let cloud = build_pointcloud(
            hits,
            &msg.header.frame_id,
            msg.header.stamp,
            self.config.output_voxel_size,
        );
        if let Err(e) = self.lidar.publish(&cloud).await {
            eprintln!("scene_lidar: publish failed: {e}");
        }
    }

    async fn on_entities(&mut self, msg: EntityStateBatch) {
        // Whole batch replaces the table — Python republishes every
        // browser physics tick (~30 Hz), so we always have a fresh
        // snapshot. Despawned entities drop out by simply not appearing
        // in the next batch.
        self.entities = msg.entries;
    }
}

fn validate_config(config: &Config) {
    if config.hz <= 0.0 || !config.hz.is_finite() {
        panic!("scene_lidar: hz must be > 0, got {}", config.hz);
    }
    if config.horizontal_samples == 0 {
        panic!("scene_lidar: horizontal_samples must be > 0");
    }
    if config.vertical_samples == 0 {
        panic!("scene_lidar: vertical_samples must be > 0");
    }
    if config.max_range <= 0.0 || !config.max_range.is_finite() {
        panic!(
            "scene_lidar: max_range must be finite and > 0, got {}",
            config.max_range
        );
    }
    if config.output_voxel_size < 0.0 || !config.output_voxel_size.is_finite() {
        panic!(
            "scene_lidar: output_voxel_size must be finite and >= 0, got {}",
            config.output_voxel_size
        );
    }
}

fn resolve_collision_path(config: &Config, meta: &SceneMeta, metadata_path: &Path) -> PathBuf {
    let raw = config
        .collision_path
        .as_ref()
        .or(meta.artifacts.browser_collision.as_ref())
        .unwrap_or_else(|| panic!("scene_lidar: scene package has no browser_collision artifact"));
    let path = PathBuf::from(raw);
    if path.is_absolute() {
        return path;
    }
    metadata_path
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .join(path)
}

fn load_gltf_triangles(path: &Path, alignment: &Alignment) -> Vec<Triangle> {
    let (document, buffers, _) = gltf::import(path)
        .unwrap_or_else(|e| panic!("scene_lidar: failed to import {}: {e}", path.display()));
    let transform = alignment_transform(alignment);
    let mut triangles = Vec::new();
    for scene in document.scenes() {
        for node in scene.nodes() {
            collect_node_triangles(node, Mat4::IDENTITY, transform, &buffers, &mut triangles);
        }
    }
    triangles
}

fn collect_node_triangles(
    node: gltf::Node<'_>,
    parent_transform: Mat4,
    alignment_transform: Mat4,
    buffers: &[gltf::buffer::Data],
    out: &mut Vec<Triangle>,
) {
    let local_transform = node_transform(&node);
    let node_transform = parent_transform * local_transform;
    if let Some(mesh) = node.mesh() {
        for primitive in mesh.primitives() {
            let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()].0));
            let Some(positions_iter) = reader.read_positions() else {
                continue;
            };
            let positions: Vec<Vec3> = positions_iter.map(Vec3::from_array).collect();
            if positions.len() < 3 {
                continue;
            }
            let indices: Vec<usize> = match reader.read_indices() {
                Some(iter) => iter.into_u32().map(|i| i as usize).collect(),
                None => (0..positions.len()).collect(),
            };
            for tri in indices.chunks_exact(3) {
                let a = transform_vertex(positions[tri[0]], node_transform, alignment_transform);
                let b = transform_vertex(positions[tri[1]], node_transform, alignment_transform);
                let c = transform_vertex(positions[tri[2]], node_transform, alignment_transform);
                if (b - a).cross(c - a).length_squared() > RAY_EPSILON {
                    out.push(Triangle::new(a, b, c));
                }
            }
        }
    }
    for child in node.children() {
        collect_node_triangles(child, node_transform, alignment_transform, buffers, out);
    }
}

fn node_transform(node: &gltf::Node<'_>) -> Mat4 {
    let (translation, rotation, scale) = node.transform().decomposed();
    Mat4::from_scale_rotation_translation(
        Vec3::from_array(scale),
        Quat::from_xyzw(rotation[0], rotation[1], rotation[2], rotation[3]),
        Vec3::from_array(translation),
    )
}

fn alignment_transform(alignment: &Alignment) -> Mat4 {
    let yaw = alignment.rotation_zyx_deg[0].to_radians();
    let pitch = alignment.rotation_zyx_deg[1].to_radians();
    let roll = alignment.rotation_zyx_deg[2].to_radians();
    let euler =
        Quat::from_rotation_z(yaw) * Quat::from_rotation_y(pitch) * Quat::from_rotation_x(roll);
    let y_to_z = if alignment.y_up {
        Mat4::from_cols_array_2d(&[
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    } else {
        Mat4::IDENTITY
    };
    Mat4::from_translation(Vec3::from_array(alignment.translation))
        * Mat4::from_quat(euler)
        * y_to_z
        * Mat4::from_scale(Vec3::splat(alignment.scale))
}

fn transform_vertex(vertex: Vec3, node_transform: Mat4, alignment_transform: Mat4) -> Vec3 {
    alignment_transform.transform_point3(node_transform.transform_point3(vertex))
}

fn lidar_directions(config: &Config) -> Vec<Vec3> {
    let mut directions = Vec::with_capacity(config.horizontal_samples * config.vertical_samples);
    let min_elev = config.elevation_min_deg.to_radians();
    let max_elev = config.elevation_max_deg.to_radians();
    for elev_index in 0..config.vertical_samples {
        let elev_t = if config.vertical_samples == 1 {
            0.5
        } else {
            elev_index as f32 / (config.vertical_samples - 1) as f32
        };
        let elev = min_elev + (max_elev - min_elev) * elev_t;
        let cos_elev = elev.cos();
        for az_index in 0..config.horizontal_samples {
            let az = std::f32::consts::TAU * az_index as f32 / config.horizontal_samples as f32;
            directions.push(Vec3::new(
                cos_elev * az.cos(),
                cos_elev * az.sin(),
                elev.sin(),
            ));
        }
    }
    directions
}

fn pose_quat(msg: &PoseStamped) -> Quat {
    let q = Quat::from_xyzw(
        msg.pose.orientation.x as f32,
        msg.pose.orientation.y as f32,
        msg.pose.orientation.z as f32,
        msg.pose.orientation.w as f32,
    );
    if q.length_squared() > 0.0 {
        q.normalize()
    } else {
        Quat::IDENTITY
    }
}

fn intersect_aabb(origin: Vec3, direction: Vec3, min: Vec3, max: Vec3, max_t: f32) -> bool {
    let mut t_min = 0.0;
    let mut t_max = max_t;
    for axis in 0..3 {
        let o = origin[axis];
        let d = direction[axis];
        let min_axis = min[axis];
        let max_axis = max[axis];
        if d.abs() < RAY_EPSILON {
            if o < min_axis || o > max_axis {
                return false;
            }
            continue;
        }
        let inv = 1.0 / d;
        let mut t0 = (min_axis - o) * inv;
        let mut t1 = (max_axis - o) * inv;
        if t0 > t1 {
            std::mem::swap(&mut t0, &mut t1);
        }
        t_min = f32::max(t_min, t0);
        t_max = f32::min(t_max, t1);
        if t_max < t_min {
            return false;
        }
    }
    true
}

fn intersect_triangle(origin: Vec3, direction: Vec3, tri: Triangle) -> Option<f32> {
    let edge1 = tri.b - tri.a;
    let edge2 = tri.c - tri.a;
    let h = direction.cross(edge2);
    let det = edge1.dot(h);
    if det.abs() < RAY_EPSILON {
        return None;
    }
    let inv_det = 1.0 / det;
    let s = origin - tri.a;
    let u = inv_det * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(edge1);
    let v = inv_det * direction.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = inv_det * edge2.dot(q);
    (t > RAY_EPSILON).then_some(t)
}

fn build_pointcloud(
    hits: Vec<(Vec3, f32)>,
    frame_id: &str,
    stamp: lcm_msgs::std_msgs::Time,
    voxel_size: f32,
) -> PointCloud2 {
    let mut seen = HashSet::new();
    let mut data = Vec::with_capacity(hits.len() * 16);
    let mut count = 0_i32;
    for (point, distance) in hits {
        if voxel_size > 0.0 {
            let inv = 1.0 / voxel_size;
            let key = (
                (point.x * inv).floor() as i32,
                (point.y * inv).floor() as i32,
                (point.z * inv).floor() as i32,
            );
            if !seen.insert(key) {
                continue;
            }
        }
        data.extend_from_slice(&point.x.to_le_bytes());
        data.extend_from_slice(&point.y.to_le_bytes());
        data.extend_from_slice(&point.z.to_le_bytes());
        data.extend_from_slice(&distance.to_le_bytes());
        count += 1;
    }

    let make_field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };

    PointCloud2 {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.into(),
        },
        height: 1,
        width: count,
        fields: vec![
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * count,
        data,
        is_dense: true,
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<SceneLidar, _>(transport)
        .await
        .expect("scene_lidar run failed");
}
