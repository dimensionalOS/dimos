// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::time::Duration;

use ahash::{AHashMap, AHashSet};
use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use serde::Deserialize;
use validator::{Validate, ValidationError};

type VoxelKey = (i32, i32, i32);

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
#[validate(schema(function = "validate_health_range"))]
struct Config {
    #[validate(range(exclusive_min = 0.0))]
    voxel_size: f32,
    #[validate(range(min = 0.0))]
    max_range: f32,
    #[validate(range(min = 1))]
    ray_subsample: u32,
    #[validate(range(min = 0.0))]
    shadow_depth: f32,
    #[validate(range(min = 0.0))]
    grace_depth: f32,
    min_health: i32,
    #[validate(range(min = 1))]
    max_health: i32,
}

fn validate_health_range(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    Ok(())
}

#[derive(Default)]
struct VoxelMap {
    // Save health of each voxel
    voxels: AHashMap<VoxelKey, i32>,
}

struct LocalBounds {
    origin_x: f32,
    origin_y: f32,
    r_xy_max_sq: f32,
    z_min: f32,
    z_max: f32,
}

#[derive(Module)]
struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map: Output<PointCloud2>,

    #[config]
    config: Config,

    map: VoxelMap,
    last_origin: Option<(f32, f32, f32)>,
}

impl RayTracingVoxelMap {
    async fn on_odometry(&mut self, msg: Odometry) {
        self.last_origin = Some((
            msg.pose.pose.position.x as f32,
            msg.pose.pose.position.y as f32,
            msg.pose.pose.position.z as f32,
        ));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        let Some(origin) = self.last_origin else {
            // Need at least one odometry sample before we can raycast.
            return;
        };

        let voxel_size = self.config.voxel_size;

        let points = match extract_xyz(&msg) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to get lidar points, dropped a cloud.",
                );
                return;
            }
        };
        if points.is_empty() {
            return;
        }

        let inv = 1.0_f32 / voxel_size;
        let half = voxel_size * 0.5;
        let mut live: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        let mut r_xy_max_sq = 0.0_f32;
        for &(x, y, z) in &points {
            let key = world_to_voxel(x, y, z, inv);
            live.insert(key);
            let cx = key.0 as f32 * voxel_size + half;
            let cy = key.1 as f32 * voxel_size + half;
            let cz = key.2 as f32 * voxel_size + half;
            z_min = z_min.min(cz);
            z_max = z_max.max(cz);
            let dx = cx - origin.0;
            let dy = cy - origin.1;
            r_xy_max_sq = r_xy_max_sq.max(dx * dx + dy * dy);
        }
        let cylinder = LocalBounds {
            origin_x: origin.0,
            origin_y: origin.1,
            r_xy_max_sq,
            z_min,
            z_max,
        };

        update_map(&mut self.map, origin, &points, &self.config);

        let (global_cloud, local_cloud) = build_pointclouds(
            &self.map,
            &live,
            voxel_size,
            &cylinder,
            &msg.header.frame_id,
            msg.header.stamp,
        );
        if let Err(e) = self.global_map.publish(&global_cloud).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Updated global voxel map failed to publish",
            );
        }
        if let Err(e) = self.local_map.publish(&local_cloud).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Updated local voxel map failed to publish",
            );
        }
    }
}

fn update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) {
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    let mut hits: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
    for &(x, y, z) in points {
        hits.insert(world_to_voxel(x, y, z, inv));
    }

    let mut misses: AHashSet<VoxelKey> = AHashSet::new();
    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let step = cfg.ray_subsample as usize;
    for (i, &p) in points.iter().enumerate() {
        if i % step != 0 {
            continue;
        }
        let dx = p.0 - origin.0;
        let dy = p.1 - origin.1;
        let dz = p.2 - origin.2;
        if dx * dx + dy * dy + dz * dz > max_range_sq {
            continue;
        }
        let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
        find_misses_along_ray(
            &mut misses,
            &map.voxels,
            origin,
            p,
            cfg.voxel_size,
            cfg.shadow_depth,
            cfg.grace_depth,
            origin_voxel,
            endpoint,
        );
    }

    // add new hits
    for v in &hits {
        let h = map.voxels.entry(*v).or_insert(cfg.min_health);
        *h = (*h + 1).min(cfg.max_health);
    }

    // each miss is only checked once
    for v in misses.difference(&hits) {
        if let Some(h) = map.voxels.get_mut(v) {
            *h -= 1;
            if *h <= cfg.min_health {
                map.voxels.remove(v);
            }
        }
    }
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Amanatides & Woo 3d DDA. Records voxels on ray in between the end of the shadow region
/// and origin if it is in the map. Voxels within grace region of the endpoint are spared from being marked as misses.
#[allow(clippy::too_many_arguments)]
fn find_misses_along_ray(
    misses: &mut AHashSet<VoxelKey>,
    map_voxels: &AHashMap<VoxelKey, i32>,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
    shadow_depth: f32,
    grace_depth: f32,
    origin_voxel: VoxelKey,
    endpoint: VoxelKey,
) {
    if origin_voxel == endpoint {
        return;
    }

    let (ox, oy, oz) = origin;
    let dx = end.0 - ox;
    let dy = end.1 - oy;
    let dz = end.2 - oz;

    let (mut x, mut y, mut z) = origin_voxel;

    let step_x = dx.signum() as i32;
    let step_y = dy.signum() as i32;
    let step_z = dz.signum() as i32;

    let t_max_init = |p: f32, d: f32, vox: i32, step: i32| -> f32 {
        if step == 0 {
            return f32::INFINITY;
        }
        let next_boundary = if step > 0 {
            (vox + 1) as f32 * voxel_size
        } else {
            vox as f32 * voxel_size
        };
        (next_boundary - p) / d
    };

    let mut tx = t_max_init(ox, dx, x, step_x);
    let mut ty = t_max_init(oy, dy, y, step_y);
    let mut tz = t_max_init(oz, dz, z, step_z);

    let dt_x = if step_x == 0 {
        f32::INFINITY
    } else {
        voxel_size / dx.abs()
    };
    let dt_y = if step_y == 0 {
        f32::INFINITY
    } else {
        voxel_size / dy.abs()
    };
    let dt_z = if step_z == 0 {
        f32::INFINITY
    } else {
        voxel_size / dz.abs()
    };

    let half = voxel_size * 0.5;
    let endpoint_center = (
        endpoint.0 as f32 * voxel_size + half,
        endpoint.1 as f32 * voxel_size + half,
        endpoint.2 as f32 * voxel_size + half,
    );
    let shadow_sq = shadow_depth.powi(2);
    let grace_sq = grace_depth.powi(2);

    let ray_len = (dx * dx + dy * dy + dz * dz).sqrt();
    let t_max = 1.0 + shadow_depth / ray_len.max(f32::EPSILON);

    let mut past_endpoint = false;
    loop {
        let t_enter = tx.min(ty).min(tz);
        if t_enter > t_max {
            return;
        }
        if t_enter >= 1.0 {
            past_endpoint = true;
        }

        if tx < ty {
            if tx < tz {
                x += step_x;
                tx += dt_x;
            } else {
                z += step_z;
                tz += dt_z;
            }
        } else if ty < tz {
            y += step_y;
            ty += dt_y;
        } else {
            z += step_z;
            tz += dt_z;
        }

        if (x, y, z) == endpoint {
            past_endpoint = true;
            continue;
        }

        // don't remove points in the same xy plane as the hit, unless the plane only walks that plane
        // we do this to preserve floors, which is more important than some missed points
        if origin_voxel.2 != endpoint.2 && z == endpoint.2 {
            continue;
        }

        let cx = x as f32 * voxel_size + half;
        let cy = y as f32 * voxel_size + half;
        let cz = z as f32 * voxel_size + half;
        let ddx = cx - endpoint_center.0;
        let ddy = cy - endpoint_center.1;
        let ddz = cz - endpoint_center.2;
        let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;

        if past_endpoint {
            // continue past the endpoint and in to the shadow realm
            if dist_sq > shadow_sq {
                return;
            }
        } else if dist_sq < grace_sq {
            // too close to the endpoint to safely mark as miss because we might be clipping other voxel's rays
            continue;
        }

        if map_voxels.contains_key(&(x, y, z)) {
            misses.insert((x, y, z));
        }
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

fn build_pointclouds(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    cylinder: &LocalBounds,
    frame_id: &str,
    stamp: Time,
) -> (PointCloud2, PointCloud2) {
    let half = voxel_size * 0.5;
    let mut global_data = Vec::with_capacity((map.voxels.len() + live.len()) * 16);
    let mut local_data = Vec::with_capacity(live.len() * 2 * 16);
    let mut global_n: i32 = 0;
    let mut local_n: i32 = 0;

    let write_point = |data: &mut Vec<u8>, n: &mut i32, x: f32, y: f32, z: f32| {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&0.0_f32.to_le_bytes());
        *n += 1;
    };

    // if point is within local map
    let in_cylinder = |x: f32, y: f32, z: f32| -> bool {
        if z < cylinder.z_min || z > cylinder.z_max {
            return false;
        }
        let dx = x - cylinder.origin_x;
        let dy = y - cylinder.origin_y;
        dx * dx + dy * dy <= cylinder.r_xy_max_sq
    };

    // add healthy voxels to global, and local if necessary
    for (&(kx, ky, kz), &health) in &map.voxels {
        if health <= 0 {
            continue;
        }
        let x = kx as f32 * voxel_size + half;
        let y = ky as f32 * voxel_size + half;
        let z = kz as f32 * voxel_size + half;
        write_point(&mut global_data, &mut global_n, x, y, z);
        if in_cylinder(x, y, z) {
            write_point(&mut local_data, &mut local_n, x, y, z);
        }
    }

    // add live voxels to both if they aren't already there
    for &(kx, ky, kz) in live {
        if matches!(map.voxels.get(&(kx, ky, kz)), Some(h) if *h > 0) {
            continue;
        }
        let x = kx as f32 * voxel_size + half;
        let y = ky as f32 * voxel_size + half;
        let z = kz as f32 * voxel_size + half;
        write_point(&mut global_data, &mut global_n, x, y, z);
        write_point(&mut local_data, &mut local_n, x, y, z);
    }

    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let fields = vec![
        make_field("x", 0),
        make_field("y", 4),
        make_field("z", 8),
        make_field("intensity", 12),
    ];

    let global_cloud = PointCloud2 {
        header: Header {
            seq: 0,
            stamp: stamp.clone(),
            frame_id: frame_id.into(),
        },
        height: 1,
        width: global_n,
        fields: fields.clone(),
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * global_n,
        data: global_data,
        is_dense: true,
    };
    let local_cloud = PointCloud2 {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.into(),
        },
        height: 1,
        width: local_n,
        fields,
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * local_n,
        data: local_data,
        is_dense: true,
    };

    (global_cloud, local_cloud)
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<RayTracingVoxelMap, _>(transport).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basic_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
        }
    }

    #[test]
    fn find_misses_along_ray_hits_correct_voxels_1() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (5.5, 0.5, 0.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let expected: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (2, 0, 0),
            (3, 0, 0),
            (4, 0, 0),
            (6, 0, 0),
            (7, 0, 0),
        ]
        .into_iter()
        .collect();
        let mut map_voxels: AHashMap<VoxelKey, i32> = AHashMap::new();
        for v in &expected {
            map_voxels.insert(*v, 1);
        }

        let mut misses: AHashSet<VoxelKey> = AHashSet::new();
        find_misses_along_ray(
            &mut misses,
            &map_voxels,
            origin,
            end,
            voxel_size,
            shadow_depth,
            0.0,
            origin_voxel,
            endpoint,
        );

        assert_eq!(misses, expected);
    }

    #[test]
    fn find_misses_along_ray_hits_correct_voxels_2() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (3.5, 2.5, 1.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let walked: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (1, 1, 0),
            (1, 1, 1),
            (2, 1, 1),
            (2, 2, 1),
            (4, 2, 1),
            (4, 3, 1),
            (4, 3, 2),
        ]
        .into_iter()
        .collect();
        let mut map_voxels: AHashMap<VoxelKey, i32> = AHashMap::new();
        for v in &walked {
            map_voxels.insert(*v, 1);
        }

        let mut misses: AHashSet<VoxelKey> = AHashSet::new();
        find_misses_along_ray(
            &mut misses,
            &map_voxels,
            origin,
            end,
            voxel_size,
            shadow_depth,
            0.0,
            origin_voxel,
            endpoint,
        );

        // z-slab protection skips voxels in the endpoint's z-slab (z=1) when the
        // ray crosses z-slabs.
        let expected: AHashSet<VoxelKey> = walked
            .iter()
            .filter(|v| v.2 != endpoint.2)
            .copied()
            .collect();
        assert_eq!(misses, expected);
    }

    #[test]
    fn hits_insert_voxels() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(5.5, 0.5, 0.5), (0.5, 5.5, 0.5)],
            &cfg,
        );
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(0, 5, 0)), Some(&1));
        assert_eq!(map.voxels.len(), 2);
    }

    #[test]
    fn voxels_on_ray_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // make sure the initial point got cleared by the new update
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_not_on_ray_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 5, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 5, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_within_shadow_region_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((6, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // point within the shadow is no longer included, new point is included
        assert!(!map.voxels.contains_key(&(6, 0, 0)));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_beyond_shadow_region_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((8, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(8, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn hit_caught_by_other_ray_is_not_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        update_map(
            &mut map,
            (0.0, 0.0, 0.0),
            &[(3.5, 0.5, 0.5), (5.5, 0.5, 0.5)],
            &cfg,
        );
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn point_beyond_max_range_does_not_clear() {
        let cfg = Config {
            max_range: 3.0,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));
    }

    #[test]
    fn two_hits_needed_when_min_health_is_negative() {
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&0));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    fn cloud_points(c: &PointCloud2) -> AHashSet<(u32, u32, u32)> {
        let mut out = AHashSet::new();
        let step = c.point_step as usize;
        for i in 0..c.width as usize {
            let base = i * step;
            let x = f32::from_le_bytes(c.data[base..base + 4].try_into().unwrap());
            let y = f32::from_le_bytes(c.data[base + 4..base + 8].try_into().unwrap());
            let z = f32::from_le_bytes(c.data[base + 8..base + 12].try_into().unwrap());
            out.insert((x.to_bits(), y.to_bits(), z.to_bits()));
        }
        out
    }

    fn voxel_center(kx: i32, ky: i32, kz: i32) -> (u32, u32, u32) {
        (
            (kx as f32 + 0.5).to_bits(),
            (ky as f32 + 0.5).to_bits(),
            (kz as f32 + 0.5).to_bits(),
        )
    }

    #[test]
    fn local_map_includes_voxel_inside_cylinder() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 0), 1);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let (global, local) =
            build_pointclouds(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 0)));
        assert!(cloud_points(&local).contains(&voxel_center(0, 0, 0)));
    }

    #[test]
    fn local_map_excludes_voxel_outside_radius() {
        let mut map = VoxelMap::default();
        map.voxels.insert((5, 0, 0), 1);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: -10.0,
            z_max: 10.0,
        };
        let (global, local) =
            build_pointclouds(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(5, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(5, 0, 0)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn local_map_excludes_voxel_outside_z_range() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 5), 1);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 100.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let (global, local) =
            build_pointclouds(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 5)));
        assert!(!cloud_points(&local).contains(&voxel_center(0, 0, 5)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn local_map_always_includes_live_voxels() {
        let map = VoxelMap::default();
        let mut live: AHashSet<VoxelKey> = AHashSet::new();
        live.insert((10, 10, 10));
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 0.0,
            z_min: 0.0,
            z_max: 0.0,
        };
        let (global, local) =
            build_pointclouds(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(10, 10, 10)));
        assert!(cloud_points(&local).contains(&voxel_center(10, 10, 10)));
    }

    /// Test how bad the planar ray clipping is.
    /// For example, points on floors can be counted as misses because they are close to the same ray as the hit.
    #[test]
    fn ground_clipping_single_ray() {
        let voxel_size = 0.1_f32;
        let lidar_height = 1.0_f32;
        let cfg = Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
        };
        let inv = 1.0 / voxel_size;

        // Cover the full range we will probe, plus a little for shadow.
        let max_x = 25.0_f32;
        let n_ground = (max_x / voxel_size).ceil() as i32;

        let ranges: Vec<f32> = (1..=20).map(|i| i as f32).collect();
        let mut table = format!(
            "voxel_size={voxel_size} lidar_height={lidar_height} grace={} shadow={}\n\
             range_m  ground_voxels_in_row  clipped  clipped_pct\n",
            cfg.grace_depth, cfg.shadow_depth
        );
        let mut total_clipped = 0usize;
        for &range in &ranges {
            let mut map = VoxelMap::default();
            for i in 0..n_ground {
                let x = (i as f32) * voxel_size + voxel_size * 0.5;
                let key = world_to_voxel(x, 0.0, 0.0, inv);
                map.voxels.insert(key, cfg.max_health);
            }
            let n_before = map.voxels.len();

            let origin = (0.0_f32, 0.0_f32, lidar_height);
            let hits = vec![(range, 0.0_f32, 0.0_f32)];
            update_map(&mut map, origin, &hits, &cfg);

            let n_after_ground: usize = (0..n_ground)
                .filter(|i| {
                    let x = (*i as f32) * voxel_size + voxel_size * 0.5;
                    let key = world_to_voxel(x, 0.0, 0.0, inv);
                    map.voxels.contains_key(&key)
                })
                .count();
            let clipped = n_before - n_after_ground;
            let pct = 100.0 * clipped as f32 / n_before as f32;
            table.push_str(&format!(
                "{range:>6.1}  {n_before:>20}  {clipped:>7}  {pct:>10.1}\n"
            ));
            total_clipped += clipped;
        }
        eprint!("{table}");
        assert!(
            total_clipped == 0,
            "planar grace regressed, ground voxels clipped:\n{table}"
        );
    }

    #[test]
    fn two_misses_needed_when_max_health_is_two() {
        let cfg = Config {
            max_health: 2,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&2));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
    }
}
