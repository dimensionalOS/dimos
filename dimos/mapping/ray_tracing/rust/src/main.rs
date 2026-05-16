// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Native Rust voxel-map module with raycast clearing.
//
// Algorithm (v1):
//   * Insert the voxel of every point into the global hash set.
//   * For every point, walk the 3D-DDA ray from the sensor origin
//     (latest odometry pose) toward the point, removing every
//     intermediate voxel from the map.  The endpoint voxel itself
//     is kept (it just got inserted as a hit).
//
// Inputs (LCM topics, set by the dimos NativeModule coordinator):
//   * `lidar`    : sensor_msgs::PointCloud2  (world frame)
//   * `odometry` : nav_msgs::Odometry        (world frame)
//
// Output:
//   * `global_map` : sensor_msgs::PointCloud2  (world frame)
//
// PointCloud2 input is expected in the standard FastLio2 layout
// (xyz at offsets 0/4/8 as little-endian f32, point_step >= 12).

use ahash::AHashSet;
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
}

#[derive(Default)]
struct VoxelMap {
    voxels: AHashSet<VoxelKey>,
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

        update_map(&mut self.map, origin, &points, &self.config);

        // Echo the input cloud's frame; the global map lives in the same
        // world frame as the upstream lidar/odometry.
        let cloud = build_pointcloud(
            &self.map,
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

    for &(x, y, z) in points {
        map.voxels.insert(world_to_voxel(x, y, z, inv));
    }

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
        walk_ray(map, origin, p, cfg.voxel_size, origin_voxel, endpoint);
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

/// Amanatides & Woo 3-D DDA. Removes every voxel strictly between
/// `origin_voxel` and `endpoint` from the map. Endpoint is kept.
fn walk_ray(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
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

    // FIXME: I don't know if we really need this
    let max_iter = 4096;
    for _ in 0..max_iter {
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
            return;
        }

        // don't need any fancy logic, it removes it if it's there
        map.voxels.remove(&(x, y, z));
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

fn build_pointcloud(map: &VoxelMap, voxel_size: f32, frame_id: &str, stamp: Time) -> PointCloud2 {
    let n = map.voxels.len();
    let half = voxel_size * 0.5;
    let mut data = Vec::with_capacity(n * 16);
    for &(kx, ky, kz) in &map.voxels {
        let x = kx as f32 * voxel_size + half;
        let y = ky as f32 * voxel_size + half;
        let z = kz as f32 * voxel_size + half;
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&0.0_f32.to_le_bytes());
    }

    let make_field = |name: &str, off: i32| PointField {
        name: name.into(),
        offset: off,
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
        width: n as i32,
        fields: vec![
            make_field("x", 0),
            make_field("y", 4),
            make_field("z", 8),
            make_field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n as i32,
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

    fn cfg(voxel_size: f32) -> Config {
        Config {
            voxel_size,
            max_range: 100.0,
            ray_subsample: 1,
        }
    }

    #[test]
    fn insert_creates_voxel_at_point() {
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(1.5, 0.0, 0.0)], &cfg(1.0));
        assert!(map.voxels.contains(&(1, 0, 0)));
    }

    #[test]
    fn raycast_clears_voxel_between_origin_and_hit() {
        let mut map = VoxelMap::default();
        // Pre-existing voxel at x=2 that the ray should sweep through.
        map.voxels.insert((2, 0, 0));
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.0, 0.0)], &cfg(1.0));
        // Endpoint kept...
        assert!(map.voxels.contains(&(5, 0, 0)));
        // ...intermediate voxel cleared.
        assert!(!map.voxels.contains(&(2, 0, 0)));
    }

    #[test]
    fn raycast_does_not_clear_endpoint_voxel() {
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.0, 0.0)], &cfg(1.0));
        assert!(map.voxels.contains(&(3, 0, 0)));
    }

    #[test]
    fn diagonal_ray_clears_traversed_voxels() {
        let mut map = VoxelMap::default();
        // Dynamic obstacle that has moved; should be carved out.
        map.voxels.insert((1, 1, 0));
        update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 3.5, 0.0)], &cfg(1.0));
        assert!(!map.voxels.contains(&(1, 1, 0)));
        assert!(map.voxels.contains(&(3, 3, 0)));
    }

    #[test]
    fn point_inside_origin_voxel_no_raycast() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 0));
        update_map(&mut map, (0.5, 0.5, 0.5), &[(0.6, 0.5, 0.5)], &cfg(1.0));
        assert!(map.voxels.contains(&(0, 0, 0)));
    }

    #[test]
    fn build_pointcloud_writes_xyz_at_voxel_centers() {
        let mut map = VoxelMap::default();
        map.voxels.insert((1, 0, 0));
        let cloud = build_pointcloud(&map, 1.0, "world", Time::default());
        assert_eq!(cloud.width, 1);
        assert_eq!(cloud.point_step, 16);
        let x = read_f32_le(&cloud.data, 0);
        let y = read_f32_le(&cloud.data, 4);
        let z = read_f32_le(&cloud.data, 8);
        assert!((x - 1.5).abs() < 1e-6);
        assert!((y - 0.5).abs() < 1e-6);
        assert!((z - 0.5).abs() < 1e-6);
    }
}
