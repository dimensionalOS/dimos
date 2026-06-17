// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Native Rust voxel-map module with raycast clearing.
//
// Inputs (LCM topics, set by the dimos NativeModule coordinator):
//   * `lidar`    : sensor_msgs::PointCloud2  (world frame)
//   * `odometry` : nav_msgs::Odometry        (world frame)
//
// Outputs:
//   * `global_map` : nav_msgs::DynamicCloud  (world frame, sparse voxel keys)
//   * `local_map`  : sensor_msgs::PointCloud2 (world frame, cylinder around robot)

mod dynamic_cloud;

use std::time::Duration;

use ahash::AHashSet;
use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{
    iter_global_points, update_map, Config, LocalBounds, VoxelKey, VoxelMap,
};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

use dynamic_cloud::DynamicCloud;

#[derive(Module)]
struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[output(encode = encode_dynamic_cloud)]
    global_map: Output<DynamicCloud>,

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

        let live = update_map(&mut self.map, origin, &points, &self.config);

        let half = voxel_size * 0.5;
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        let mut r_xy_max_sq = 0.0_f32;
        for &(kx, ky, kz) in &live {
            let cx = kx as f32 * voxel_size + half;
            let cy = ky as f32 * voxel_size + half;
            let cz = kz as f32 * voxel_size + half;
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

        let global_cloud = build_global_cloud(
            &self.map,
            &live,
            voxel_size,
            &msg.header.frame_id,
            msg.header.stamp.clone(),
        );
        let local_cloud = build_local_cloud(
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

fn encode_dynamic_cloud(msg: &DynamicCloud) -> Vec<u8> {
    msg.encode()
        .expect("DynamicCloud::encode: frame_id exceeds 65535 bytes")
}

fn stamp_to_nanos(stamp: &Time) -> u64 {
    (stamp.sec as i64 as u64)
        .wrapping_mul(1_000_000_000)
        .wrapping_add(stamp.nsec.max(0) as u64)
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

/// Sparse voxel-key cloud of the full global map. Includes every healthy
/// voxel (health > 0) plus any voxel hit this scan (live), even if still
/// uncertain. `quantity` carries per-voxel health; the event log is empty
/// because this map has no per-voxel timestamps to report.
fn build_global_cloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> DynamicCloud {
    let mut voxels = Vec::with_capacity(map.voxels.len() + live.len());
    let mut quantity = Vec::with_capacity(map.voxels.len() + live.len());

    // healthy voxels
    for (&key, &health) in &map.voxels {
        if health > 0 {
            voxels.push(key);
            quantity.push(health as u32);
        }
    }
    // live voxels that aren't already healthy
    for &key in live {
        if matches!(map.voxels.get(&key), Some(h) if *h > 0) {
            continue;
        }
        voxels.push(key);
        let health = map.voxels.get(&key).copied().unwrap_or(0);
        quantity.push(health.max(0) as u32);
    }

    DynamicCloud {
        timestamp_nanos: stamp_to_nanos(&stamp),
        voxel_size,
        frame_id: frame_id.to_string(),
        voxels,
        quantity,
        event_indices: Vec::new(),
        event_timestamps: Vec::new(),
    }
}

/// Dense XYZ point cloud of the voxels inside the cylinder around the robot,
/// always including this scan's live voxels.
fn build_local_cloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    cylinder: &LocalBounds,
    frame_id: &str,
    stamp: Time,
) -> PointCloud2 {
    let half = voxel_size * 0.5;
    let mut local_data = Vec::with_capacity(live.len() * 2 * 16);
    let mut local_n: i32 = 0;

    let write_point = |data: &mut Vec<u8>, n: &mut i32, x: f32, y: f32, z: f32| {
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&0.0_f32.to_le_bytes());
        *n += 1;
    };

    // healthy voxels within the cylinder
    for (x, y, z) in iter_global_points(map, voxel_size) {
        if cylinder.contains(x, y, z) {
            write_point(&mut local_data, &mut local_n, x, y, z);
        }
    }

    // live voxels that aren't already healthy
    for &(kx, ky, kz) in live {
        if matches!(map.voxels.get(&(kx, ky, kz)), Some(c) if c.health > 0) {
            continue;
        }
        let x = kx as f32 * voxel_size + half;
        let y = ky as f32 * voxel_size + half;
        let z = kz as f32 * voxel_size + half;
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

    PointCloud2 {
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
    }
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
    use dimos_voxel_ray_tracing::voxel_ray_tracer::Voxel;

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
    fn global_map_includes_healthy_voxel() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 0), 1);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        assert!(global.voxels.contains(&(0, 0, 0)));
    }

    #[test]
    fn local_map_includes_voxel_inside_cylinder() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 0), Voxel::with_health(1));
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&local).contains(&voxel_center(0, 0, 0)));
    }

    #[test]
    fn local_map_excludes_voxel_outside_radius() {
        let mut map = VoxelMap::default();
        map.voxels.insert((5, 0, 0), Voxel::with_health(1));
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: -10.0,
            z_max: 10.0,
        };
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(global.voxels.contains(&(5, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(5, 0, 0)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn local_map_excludes_voxel_outside_z_range() {
        let mut map = VoxelMap::default();
        map.voxels.insert((0, 0, 5), Voxel::with_health(1));
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 100.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(global.voxels.contains(&(0, 0, 5)));
        assert!(!cloud_points(&local).contains(&voxel_center(0, 0, 5)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn maps_always_include_live_voxels() {
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
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(global.voxels.contains(&(10, 10, 10)));
        assert!(cloud_points(&local).contains(&voxel_center(10, 10, 10)));
    }
}
