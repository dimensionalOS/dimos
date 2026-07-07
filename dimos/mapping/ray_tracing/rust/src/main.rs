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

use std::time::Duration;

use ahash::AHashSet;
use dimos_module::{error_throttled, run, warn_throttled, Input, LcmTransport, Module, Output};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{
    batch_local_bounds, iter_global_points, update_map, Config, LocalBounds, VoxelKey, VoxelMap,
};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use nalgebra::{UnitQuaternion, Vector3};

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

    // Cylinder bounds of the local map. Position is the center, orientation holds
    // radius, z_min, z_max. Stamped like local_map so consumers pair them.
    #[output(encode = PoseStamped::encode)]
    region_bounds: Output<PoseStamped>,

    #[config]
    config: Config,

    map: VoxelMap,
    last_pose: Option<(Vector3<f32>, UnitQuaternion<f32>)>,
    frame_count: u32,
    batch_points: Vec<(f32, f32, f32)>,
    batch_origins: Vec<(f32, f32, f32)>,
}

impl RayTracingVoxelMap {
    async fn on_odometry(&mut self, msg: Odometry) {
        let p = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        self.last_pose = Some((
            Vector3::new(p.x as f32, p.y as f32, p.z as f32),
            UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                q.w as f32, q.x as f32, q.y as f32, q.z as f32,
            )),
        ));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        let Some((translation, rotation)) = self.last_pose else {
            // Need an odometry sample before we can raycast.
            return;
        };
        let origin = (translation.x, translation.y, translation.z);

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

        // Transform sensor-frame points into the world by the odom pose.
        let rot = rotation.to_rotation_matrix();
        let points: Vec<(f32, f32, f32)> = points
            .iter()
            .map(|&(x, y, z)| {
                let p = rot * Vector3::new(x, y, z) + translation;
                (p.x, p.y, p.z)
            })
            .collect();

        let out_frame_id = "world";

        let live = update_map(&mut self.map, origin, &points, &self.config);

        // The batch only feeds the local region bounds, so skip it when the local
        // map is disabled.
        if self.config.emit_every > 0 {
            self.batch_points.extend_from_slice(&points);
            self.batch_origins.push(origin);
        }

        self.frame_count += 1;
        let local_due = emit_due(self.frame_count, self.config.emit_every);

        let cylinder = if local_due {
            let margin = self.config.shadow_depth + voxel_size;
            let (cx, cy, radius, z_min, z_max) = batch_local_bounds(
                &self.batch_points,
                &self.batch_origins,
                self.config.region_percentile,
                margin,
            );
            self.batch_points.clear();
            self.batch_origins.clear();

            let bounds_msg = PoseStamped {
                header: Header {
                    seq: 0,
                    stamp: msg.header.stamp.clone(),
                    frame_id: out_frame_id.to_string(),
                },
                pose: Pose {
                    position: Point {
                        x: cx as f64,
                        y: cy as f64,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        x: radius as f64,
                        y: z_min as f64,
                        z: z_max as f64,
                        w: 0.0,
                    },
                },
            };
            if let Err(e) = self.region_bounds.publish(&bounds_msg).await {
                error_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Region bounds failed to publish",
                );
            }
            Some(LocalBounds {
                origin_x: cx,
                origin_y: cy,
                r_xy_max_sq: radius * radius,
                z_min,
                z_max,
            })
        } else {
            None
        };

        let global_due = emit_due(self.frame_count, self.config.global_emit_every);

        let stamp = msg.header.stamp;
        if let Some(cyl) = &cylinder {
            if global_due {
                let (global, local) =
                    build_global_and_local(&self.map, &live, voxel_size, cyl, out_frame_id, stamp);
                publish_cloud(&self.global_map, &global).await;
                publish_cloud(&self.local_map, &local).await;
            } else {
                let local =
                    build_local_cloud(&self.map, &live, voxel_size, cyl, out_frame_id, stamp);
                publish_cloud(&self.local_map, &local).await;
            }
        } else if global_due {
            let global = build_global_cloud(&self.map, &live, voxel_size, out_frame_id, stamp);
            publish_cloud(&self.global_map, &global).await;
        }
    }
}

/// Whether the Nth-frame output fires this frame. Zero disables it.
fn emit_due(frame_count: u32, every: u32) -> bool {
    every != 0 && frame_count.is_multiple_of(every)
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

fn write_point(data: &mut Vec<u8>, n: &mut i32, x: f32, y: f32, z: f32) {
    data.extend_from_slice(&x.to_le_bytes());
    data.extend_from_slice(&y.to_le_bytes());
    data.extend_from_slice(&z.to_le_bytes());
    data.extend_from_slice(&0.0_f32.to_le_bytes());
    *n += 1;
}

/// Centers of live voxels not yet healthy enough to appear in the map.
fn unhealthy_live_centers<'a>(
    map: &'a VoxelMap,
    live: &'a AHashSet<VoxelKey>,
    voxel_size: f32,
) -> impl Iterator<Item = (f32, f32, f32)> + 'a {
    let half = voxel_size * 0.5;
    live.iter().filter_map(move |&(kx, ky, kz)| {
        if matches!(map.voxels.get(&(kx, ky, kz)), Some(c) if c.health > 0) {
            return None;
        }
        Some((
            kx as f32 * voxel_size + half,
            ky as f32 * voxel_size + half,
            kz as f32 * voxel_size + half,
        ))
    })
}

fn make_cloud(data: Vec<u8>, n: i32, frame_id: &str, stamp: Time) -> PointCloud2 {
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

/// All healthy voxels plus this frame's live voxels.
fn build_global_cloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    frame_id: &str,
    stamp: Time,
) -> PointCloud2 {
    let mut data = Vec::with_capacity((map.voxels.len() + live.len()) * 16);
    let mut n: i32 = 0;
    for (x, y, z) in iter_global_points(map, voxel_size) {
        write_point(&mut data, &mut n, x, y, z);
    }
    for (x, y, z) in unhealthy_live_centers(map, live, voxel_size) {
        write_point(&mut data, &mut n, x, y, z);
    }
    make_cloud(data, n, frame_id, stamp)
}

/// Healthy voxels and this frame's live voxels, all inside the cylinder.
fn build_local_cloud(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    cylinder: &LocalBounds,
    frame_id: &str,
    stamp: Time,
) -> PointCloud2 {
    let mut data = Vec::with_capacity(live.len() * 2 * 16);
    let mut n: i32 = 0;
    for (x, y, z) in iter_global_points(map, voxel_size) {
        if cylinder.contains(x, y, z) {
            write_point(&mut data, &mut n, x, y, z);
        }
    }
    for (x, y, z) in unhealthy_live_centers(map, live, voxel_size) {
        if cylinder.contains(x, y, z) {
            write_point(&mut data, &mut n, x, y, z);
        }
    }
    make_cloud(data, n, frame_id, stamp)
}

/// Build the global and local clouds in one pass over the map.
fn build_global_and_local(
    map: &VoxelMap,
    live: &AHashSet<VoxelKey>,
    voxel_size: f32,
    cylinder: &LocalBounds,
    frame_id: &str,
    stamp: Time,
) -> (PointCloud2, PointCloud2) {
    let mut g_data = Vec::with_capacity((map.voxels.len() + live.len()) * 16);
    let mut g_n: i32 = 0;
    let mut l_data = Vec::with_capacity(live.len() * 2 * 16);
    let mut l_n: i32 = 0;
    let mut push = |x: f32, y: f32, z: f32| {
        write_point(&mut g_data, &mut g_n, x, y, z);
        if cylinder.contains(x, y, z) {
            write_point(&mut l_data, &mut l_n, x, y, z);
        }
    };
    for (x, y, z) in iter_global_points(map, voxel_size) {
        push(x, y, z);
    }
    for (x, y, z) in unhealthy_live_centers(map, live, voxel_size) {
        push(x, y, z);
    }
    (
        make_cloud(g_data, g_n, frame_id, stamp.clone()),
        make_cloud(l_data, l_n, frame_id, stamp),
    )
}

async fn publish_cloud(out: &Output<PointCloud2>, cloud: &PointCloud2) {
    if let Err(e) = out.publish(cloud).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "Voxel map failed to publish",
        );
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
    fn emit_due_fires_every_nth_frame_and_zero_disables() {
        assert!(emit_due(1, 1));
        assert!(emit_due(2, 1));
        assert!(!emit_due(1, 2));
        assert!(emit_due(2, 2));
        assert!(!emit_due(5, 3));
        assert!(emit_due(6, 3));
        for n in 1..10 {
            assert!(!emit_due(n, 0));
        }
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
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 0)));
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
        assert!(cloud_points(&global).contains(&voxel_center(5, 0, 0)));
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
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 5)));
        assert!(!cloud_points(&local).contains(&voxel_center(0, 0, 5)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn live_voxels_follow_the_cylinder_in_local_map() {
        let map = VoxelMap::default();
        let mut live: AHashSet<VoxelKey> = AHashSet::new();
        live.insert((1, 0, 0));
        live.insert((10, 10, 10));
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = build_global_cloud(&map, &live, 1.0, "world", Time::default());
        let local = build_local_cloud(&map, &live, 1.0, &cylinder, "world", Time::default());
        assert!(cloud_points(&global).contains(&voxel_center(1, 0, 0)));
        assert!(cloud_points(&global).contains(&voxel_center(10, 10, 10)));
        assert!(cloud_points(&local).contains(&voxel_center(1, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(10, 10, 10)));
    }
}
