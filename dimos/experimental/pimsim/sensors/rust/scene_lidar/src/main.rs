// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

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
use validator::Validate;

mod accel;
mod entity;
use accel::{Bvh, Triangle, RAY_EPSILON};
use entity::{raycast as raycast_entity, Entity, EntityStateBatch, MeshCache};

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
struct Config {
    scene_metadata_path: String,
    collision_path: Option<String>,
    #[serde(default = "default_scan_model")]
    scan_model: String,
    #[serde(default = "default_frame_id")]
    frame_id: String,
    #[serde(default)]
    publish_sensor_frame: bool,
    hz: f32,
    #[serde(default = "default_point_rate")]
    point_rate: usize,
    horizontal_samples: usize,
    vertical_samples: usize,
    elevation_min_deg: f32,
    elevation_max_deg: f32,
    #[serde(default)]
    min_range: f32,
    max_range: f32,
    sensor_x: f32,
    sensor_y: f32,
    sensor_z: f32,
    #[serde(default)]
    sensor_roll_deg: f32,
    #[serde(default)]
    sensor_pitch_deg: f32,
    #[serde(default)]
    sensor_yaw_deg: f32,
    yaw_offset_deg: f32,
    output_voxel_size: f32,
    #[serde(default)]
    support_floor: bool,
    #[serde(default)]
    support_floor_z: f32,
    #[serde(default)]
    support_floor_size: f32,
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
            bvh.node_count(),
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
    entity_mesh_cache: MeshCache,
    last_entity_count: usize,
    scan_index: u64,
}

fn default_scan_model() -> String {
    "uniform".into()
}

fn default_frame_id() -> String {
    "lidar_link".into()
}

fn default_point_rate() -> usize {
    200_000
}

// Per-scan azimuth advance for the non-repetitive sweep: TAU * golden-ratio
// conjugate (~222.5 deg). Irrational and incommensurate with the scan pattern's
// own golden-angle spacing, so successive scans land in the gaps rather than
// replaying the same rays — letting the accumulating mapper fill coverage over
// time, like a real non-repetitive Mid-360.
const SCAN_SPIN_RADIANS: f32 = std::f32::consts::TAU * 0.618_034;

impl SceneLidar {
    async fn setup(&mut self) {
        validate_config(&self.config);
        self.scene = SceneAccel::load(&self.config);
        self.entity_mesh_cache =
            MeshCache::new(entity_asset_root(&self.config.scene_metadata_path));
        self.directions = lidar_directions(&self.config);
        eprintln!(
            "scene_lidar: configured {} {} rays at {:.1} Hz, frame {}, max_range {:.2} m",
            self.directions.len(),
            self.config.scan_model,
            self.config.hz,
            if self.config.publish_sensor_frame {
                self.config.frame_id.as_str()
            } else {
                "pose"
            },
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

        let base_orientation = pose_quat(&msg);
        // Spin the fixed ray pattern about the sensor's up-axis by an irrational
        // azimuth each scan so it sweeps new points over time (non-repetitive,
        // like a real Mid-360). Carving stays intact for dynamic objects; the
        // static map just accumulates instead of replaying one frozen scan.
        let scan_spin = Quat::from_rotation_z(self.scan_index as f32 * SCAN_SPIN_RADIANS);
        self.scan_index = self.scan_index.wrapping_add(1);
        let sensor_orientation = base_orientation * sensor_mount_rotation(&self.config) * scan_spin;
        let sensor_offset = Vec3::new(
            self.config.sensor_x,
            self.config.sensor_y,
            self.config.sensor_z,
        );
        let origin = Vec3::new(
            msg.pose.position.x as f32,
            msg.pose.position.y as f32,
            msg.pose.position.z as f32,
        ) + base_orientation * sensor_offset;

        let max_range = self.config.max_range;
        let min_range = self.config.min_range;
        let entities: &[Entity] = &self.entities;
        let hits: Vec<(Vec3, f32)> = self
            .directions
            .par_iter()
            .filter_map(|direction| {
                let world_direction = (sensor_orientation * *direction).normalize();
                let mut best = self.scene.raycast(origin, world_direction, max_range);
                let mut best_dist = best.map(|(_, d)| d).unwrap_or(max_range);
                if let Some((hit, dist)) =
                    raycast_support_floor(&self.config, origin, world_direction, best_dist)
                {
                    best_dist = dist;
                    best = Some((hit, dist));
                }
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
                let (world_point, distance) = best?;
                if distance < min_range {
                    return None;
                }
                let point = if self.config.publish_sensor_frame {
                    *direction * distance
                } else {
                    world_point
                };
                Some((point, intensity_for_distance(distance, max_range)))
            })
            .collect();

        let frame_id = if self.config.publish_sensor_frame {
            self.config.frame_id.as_str()
        } else {
            msg.header.frame_id.as_str()
        };
        let cloud = build_pointcloud(
            hits,
            frame_id,
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
        if msg.entries.len() != self.last_entity_count {
            eprintln!(
                "scene_lidar: entity table now {} entries",
                msg.entries.len()
            );
            self.last_entity_count = msg.entries.len();
        }
        let mut entries = msg.entries;
        self.entity_mesh_cache.resolve_entities(&mut entries);
        self.entities = entries;
    }
}

fn entity_asset_root(scene_metadata_path: &str) -> PathBuf {
    PathBuf::from(scene_metadata_path)
        .parent()
        .unwrap_or_else(|| Path::new("."))
        .to_path_buf()
}

fn validate_config(config: &Config) {
    if config.hz <= 0.0 || !config.hz.is_finite() {
        panic!("scene_lidar: hz must be > 0, got {}", config.hz);
    }
    if !matches!(config.scan_model.as_str(), "uniform" | "mid360") {
        panic!(
            "scene_lidar: scan_model must be 'uniform' or 'mid360', got {}",
            config.scan_model
        );
    }
    if config.publish_sensor_frame && config.frame_id.trim().is_empty() {
        panic!("scene_lidar: frame_id must be non-empty when publish_sensor_frame is true");
    }
    if config.scan_model == "mid360" && config.point_rate == 0 {
        panic!("scene_lidar: point_rate must be > 0 for mid360 scan_model");
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
    if config.min_range < 0.0 || !config.min_range.is_finite() {
        panic!(
            "scene_lidar: min_range must be finite and >= 0, got {}",
            config.min_range
        );
    }
    if config.min_range >= config.max_range {
        panic!(
            "scene_lidar: min_range ({}) must be < max_range ({})",
            config.min_range, config.max_range
        );
    }
    for (name, angle) in [
        ("sensor_roll_deg", config.sensor_roll_deg),
        ("sensor_pitch_deg", config.sensor_pitch_deg),
        ("sensor_yaw_deg", config.sensor_yaw_deg),
        ("yaw_offset_deg", config.yaw_offset_deg),
    ] {
        if !angle.is_finite() {
            panic!("scene_lidar: {name} must be finite, got {angle}");
        }
    }
    if config.output_voxel_size < 0.0 || !config.output_voxel_size.is_finite() {
        panic!(
            "scene_lidar: output_voxel_size must be finite and >= 0, got {}",
            config.output_voxel_size
        );
    }
    if !config.support_floor_z.is_finite() {
        panic!(
            "scene_lidar: support_floor_z must be finite, got {}",
            config.support_floor_z
        );
    }
    if config.support_floor_size < 0.0 || !config.support_floor_size.is_finite() {
        panic!(
            "scene_lidar: support_floor_size must be finite and >= 0, got {}",
            config.support_floor_size
        );
    }
}

fn raycast_support_floor(
    config: &Config,
    origin: Vec3,
    direction: Vec3,
    max_range: f32,
) -> Option<(Vec3, f32)> {
    if !config.support_floor || direction.z >= -RAY_EPSILON {
        return None;
    }
    let distance = (config.support_floor_z - origin.z) / direction.z;
    if distance <= RAY_EPSILON || distance >= max_range {
        return None;
    }
    let hit = origin + direction * distance;
    if config.support_floor_size > 0.0 {
        let half = config.support_floor_size * 0.5;
        if hit.x.abs() > half || hit.y.abs() > half {
            return None;
        }
    }
    Some((hit, distance))
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

fn sensor_mount_rotation(config: &Config) -> Quat {
    Quat::from_rotation_z((config.sensor_yaw_deg + config.yaw_offset_deg).to_radians())
        * Quat::from_rotation_y(config.sensor_pitch_deg.to_radians())
        * Quat::from_rotation_x(config.sensor_roll_deg.to_radians())
}

fn intensity_for_distance(distance: f32, max_range: f32) -> f32 {
    (1.0 - distance / max_range).clamp(0.0, 1.0)
}

fn lidar_directions(config: &Config) -> Vec<Vec3> {
    match config.scan_model.as_str() {
        "mid360" => mid360_directions(config),
        _ => uniform_directions(config),
    }
}

fn uniform_directions(config: &Config) -> Vec<Vec3> {
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
        for az_index in 0..config.horizontal_samples {
            let az = std::f32::consts::TAU * az_index as f32 / config.horizontal_samples as f32;
            directions.push(direction_from_az_elev(az, elev));
        }
    }
    directions
}

fn mid360_directions(config: &Config) -> Vec<Vec3> {
    // Even, dense, band-limited coverage via a Fibonacci (golden-angle) spiral:
    // azimuth advances by the golden angle each step while elevation sweeps the
    // band with equal-area spacing (uniform in sin(elev)). The old approach
    // interleaved two *independent* 1-D low-discrepancy sequences (golden ratio
    // in az, sqrt2-1 in elev), which lattice together into Lissajous-style
    // curves with gaps between them. A single golden-angle spiral has no such
    // structure: it fills the band uniformly, like the Livox Mid-360's
    // non-repetitive pattern.
    let rays_per_scan = ((config.point_rate as f32 / config.hz).round() as usize).max(1);
    let min_elev = config.elevation_min_deg.to_radians();
    let max_elev = config.elevation_max_deg.to_radians();
    let sin_min = min_elev.sin();
    let sin_max = max_elev.sin();
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0_f32.sqrt());
    let mut directions = Vec::with_capacity(rays_per_scan);
    for i in 0..rays_per_scan {
        let frac = (i as f32 + 0.5) / rays_per_scan as f32;
        let sin_elev = sin_min + (sin_max - sin_min) * frac;
        let elev = sin_elev.clamp(-1.0, 1.0).asin();
        let az = (i as f32 * golden_angle).rem_euclid(std::f32::consts::TAU);
        directions.push(direction_from_az_elev(az, elev));
    }
    directions
}

fn direction_from_az_elev(az: f32, elev: f32) -> Vec3 {
    let cos_elev = elev.cos();
    Vec3::new(cos_elev * az.cos(), cos_elev * az.sin(), elev.sin()).normalize()
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
    run::<SceneLidar, _>(transport).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> Config {
        Config {
            scene_metadata_path: "scene.meta.json".into(),
            collision_path: None,
            scan_model: "uniform".into(),
            frame_id: "lidar_link".into(),
            publish_sensor_frame: false,
            hz: 10.0,
            point_rate: 200_000,
            horizontal_samples: 1,
            vertical_samples: 1,
            elevation_min_deg: 0.0,
            elevation_max_deg: 0.0,
            min_range: 0.0,
            max_range: 10.0,
            sensor_x: 0.0,
            sensor_y: 0.0,
            sensor_z: 0.0,
            sensor_roll_deg: 0.0,
            sensor_pitch_deg: 0.0,
            sensor_yaw_deg: 0.0,
            yaw_offset_deg: 0.0,
            output_voxel_size: 0.0,
            support_floor: true,
            support_floor_z: 0.0,
            support_floor_size: 0.0,
        }
    }

    #[test]
    fn support_floor_hits_downward_ray() {
        let config = test_config();
        let hit = raycast_support_floor(
            &config,
            Vec3::new(1.0, 2.0, 2.0),
            Vec3::new(0.0, 0.0, -1.0),
            10.0,
        )
        .expect("floor should intersect");
        assert_eq!(hit.0, Vec3::new(1.0, 2.0, 0.0));
        assert_eq!(hit.1, 2.0);
    }

    #[test]
    fn support_floor_respects_bounds() {
        let mut config = test_config();
        config.support_floor_size = 2.0;
        assert!(raycast_support_floor(
            &config,
            Vec3::new(2.0, 0.0, 1.0),
            Vec3::new(0.0, 0.0, -1.0),
            10.0,
        )
        .is_none());
    }
}
