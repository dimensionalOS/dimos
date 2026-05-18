// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use ahash::{AHashMap, AHashSet};
use dimos_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use serde::Deserialize;

type VoxelKey = (i32, i32, i32);

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    voxel_size: f32,
    max_range: f32,
    ray_subsample: u32,
    shadow_depth: f32,
    grace_depth: f32,
    min_health: i32,
    max_health: i32,
}

#[derive(Default)]
struct VoxelMap {
    // Save health of each voxel
    voxels: AHashMap<VoxelKey, i32>,
}

#[derive(Module)]
struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

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
        if voxel_size <= 0.0 {
            eprintln!("voxel_ray_tracing: voxel_size must be > 0, got {voxel_size}");
            return;
        }

        let points = match extract_xyz(&msg) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("voxel_ray_tracing: bad cloud, dropped: {e}");
                return;
            }
        };
        if points.is_empty() {
            return;
        }

        let inv = 1.0_f32 / voxel_size;
        let mut live: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
        for &(x, y, z) in &points {
            live.insert(world_to_voxel(x, y, z, inv));
        }

        update_map(&mut self.map, origin, &points, &self.config);

        // Echo the input cloud's frame; the global map lives in the same
        // world frame as the upstream lidar/odometry.
        let cloud = build_pointcloud(
            &self.map,
            &live,
            voxel_size,
            &msg.header.frame_id,
            msg.header.stamp,
        );
        if let Err(e) = self.global_map.publish(&cloud).await {
            eprintln!("voxel_ray_tracing: publish failed: {e}");
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
    let step = cfg.ray_subsample.max(1) as usize;
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

/// Amanatides & Woo 3-D DDA. Records voxels on ray in between the end of the shadow region
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
    let shadow_sq = shadow_depth.max(0.0).powi(2);
    let grace_sq = grace_depth.max(0.0).powi(2);

    let mut past_endpoint = false;
    loop {
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

fn build_pointcloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> PointCloud2 {
    let half = voxel_size * 0.5;
    let mut data = Vec::with_capacity((map.voxels.len() + live.len()) * 16);
    let mut n: i32 = 0;

    // helper just to inline the data handling, makes it a little faster
    let mut add_to_cloud = |kx: i32, ky: i32, kz: i32| {
        let x = kx as f32 * voxel_size + half;
        let y = ky as f32 * voxel_size + half;
        let z = kz as f32 * voxel_size + half;
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&0.0_f32.to_le_bytes());
        n += 1;
    };

    // add the healthy voxels
    for (&(kx, ky, kz), &health) in &map.voxels {
        if health > 0 {
            add_to_cloud(kx, ky, kz);
        }
    }

    // add in the live voxels if they aren't already there
    for &(kx, ky, kz) in live {
        if !matches!(map.voxels.get(&(kx, ky, kz)), Some(h) if *h > 0) {
            add_to_cloud(kx, ky, kz);
        }
    }

    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };

    // assemble the final cloud
    PointCloud2 {
        header: Header {
            seq: 0,
            stamp,
            frame_id: frame_id.into(),
        },
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

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<RayTracingVoxelMap, _>(transport)
        .await
        .expect("voxel_ray_tracing run failed");
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

        let expected: AHashSet<VoxelKey> = [
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
