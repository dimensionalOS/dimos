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

use crate::mapper::{Mapper, Pose};
use crate::voxel_ray_tracer::Config;
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, Pose as PoseMsg, PoseStamped, Quaternion};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};

#[derive(Module)]
#[module(name = "ray_tracing", setup = init_mapper)]
pub struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    global_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    local_map_fine: Output<PointCloud2>,

    // Cylinder bounds of the local map. Position is the center, orientation holds
    // radius, z_min, z_max. Stamped like local_map so consumers pair them.
    #[output(encode = PoseStamped::encode)]
    region_bounds: Output<PoseStamped>,

    #[config]
    config: Config,

    // Built once at startup by init_mapper. All mapping state lives inside.
    mapper: Option<Mapper>,
}

impl RayTracingVoxelMap {
    async fn init_mapper(&mut self) {
        self.mapper = Some(Mapper::new(self.config.clone()));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        // Register with the transform nearest the cloud stamp, waiting briefly
        // for one still in flight rather than dropping the cloud.
        let stamp = time_secs(&msg.header.stamp);
        let Some(tf_pose) = self
            .tf
            .lookup(&self.config.world_frame, &msg.header.frame_id)
            .at(stamp)
            .tolerance(TF_MATCH_TOLERANCE_S)
            .within(TF_WAIT_TIMEOUT)
            .await
        else {
            warn_throttled!(
                Duration::from_secs(1),
                world_frame = %self.config.world_frame,
                cloud_frame = %msg.header.frame_id,
                "No transform within tolerance of the cloud stamp, dropped a cloud.",
            );
            return;
        };
        let translation = tf_pose.translation().cast::<f32>();
        let rotation = tf_pose.rotation().cast::<f32>();
        let pose = Pose {
            position: (translation.x, translation.y, translation.z),
            orientation: (
                rotation.coords.x,
                rotation.coords.y,
                rotation.coords.z,
                rotation.coords.w,
            ),
        };

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

        let mapper = self.mapper.as_mut().expect("built in setup");
        mapper.add_frame(points, pose);

        let region = mapper.local_due().then(|| mapper.take_local_bounds());
        let cylinder = region.map(|c| c.bounds());

        let global_points = mapper.global_due().then(|| mapper.global_points());
        let local_points = cylinder.as_ref().map(|cyl| mapper.local_points(cyl));
        let fine_points = cylinder.as_ref().and_then(|cyl| mapper.fine_points(cyl));

        let out_frame_id = self.config.world_frame.as_str();
        let stamp = msg.header.stamp;

        // Bounds pair with local_map by stamp, so publish them on its cadence.
        if let Some(c) = region {
            let bounds_msg = PoseStamped {
                header: Header {
                    seq: 0,
                    stamp: stamp.clone(),
                    frame_id: out_frame_id.to_string(),
                },
                pose: PoseMsg {
                    position: Point {
                        x: c.cx as f64,
                        y: c.cy as f64,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        x: c.radius as f64,
                        y: c.z_min as f64,
                        z: c.z_max as f64,
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
        }

        if let Some(points) = global_points {
            let global = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.global_map, &global).await;
        }
        if let Some(points) = local_points {
            let local = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.local_map, &local).await;
        }
        if let Some(points) = fine_points {
            let fine = points_to_cloud(&points, out_frame_id, stamp);
            publish_cloud(&self.local_map_fine, &fine).await;
        }
    }
}

/// Max stamp gap between a cloud and the transform used to register it (s), one
/// period of the 30 Hz odometry.
const TF_MATCH_TOLERANCE_S: f64 = 1.0 / 30.0;

/// How long to wait for a late transform before dropping a cloud.
const TF_WAIT_TIMEOUT: Duration = Duration::from_millis(50);

fn time_secs(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
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

/// Pack flat (x, y, z) triples into an LCM cloud message.
fn points_to_cloud(points: &[f32], frame_id: &str, stamp: Time) -> PointCloud2 {
    let mut data = Vec::with_capacity((points.len() / 3) * 16);
    let mut n: i32 = 0;
    for p in points.as_chunks::<3>().0 {
        write_point(&mut data, &mut n, p[0], p[1], p[2]);
    }
    make_cloud(data, n, frame_id, stamp)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::voxel_ray_tracer::{emit_points, update_map, LocalBounds, VoxelKey, VoxelMap};
    use ahash::AHashSet;

    /// Build a map whose listed voxels are healthy, through the public API.
    fn map_with_healthy(keys: &[VoxelKey]) -> VoxelMap {
        let cfg = Config {
            voxel_size: 1.0,
            fine_divisor: 0,
            max_range: 1000.0,
            ray_subsample: 1,
            shadow_depth: 0.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            support_min: 0,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
            world_frame: "world".to_string(),
            worker_threads: 4,
        };
        let mut map = VoxelMap::default();
        let pts: Vec<(f32, f32, f32)> = keys
            .iter()
            .map(|&(x, y, z)| (x as f32 + 0.5, y as f32 + 0.5, z as f32 + 0.5))
            .collect();
        update_map(&mut map, (0.25, 0.25, 0.25), &pts, &cfg);
        map
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
        let map = map_with_healthy(&[(0, 0, 0)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, 0, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), 0, &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(0, 0, 0)));
        assert!(cloud_points(&local).contains(&voxel_center(0, 0, 0)));
    }

    #[test]
    fn local_map_excludes_voxel_outside_radius() {
        let map = map_with_healthy(&[(5, 0, 0)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 4.0,
            z_min: -10.0,
            z_max: 10.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, 0, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), 0, &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(5, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(5, 0, 0)));
        assert_eq!(local.width, 0);
    }

    #[test]
    fn local_map_excludes_voxel_outside_z_range() {
        let map = map_with_healthy(&[(0, 0, 5)]);
        let live: AHashSet<VoxelKey> = AHashSet::new();
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 100.0,
            z_min: 0.0,
            z_max: 1.0,
        };
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, 0, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), 0, &live),
            "world",
            Time::default(),
        );
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
        let global = points_to_cloud(
            &emit_points(&map, 1.0, None, 0, &live),
            "world",
            Time::default(),
        );
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), 0, &live),
            "world",
            Time::default(),
        );
        assert!(cloud_points(&global).contains(&voxel_center(1, 0, 0)));
        assert!(cloud_points(&global).contains(&voxel_center(10, 10, 10)));
        assert!(cloud_points(&local).contains(&voxel_center(1, 0, 0)));
        assert!(!cloud_points(&local).contains(&voxel_center(10, 10, 10)));
    }

    #[test]
    fn local_map_applies_support_min() {
        // The live local cloud must honor support_min, so an isolated healthy
        // voxel is dropped while a dense patch survives. Live voxels bypass it.
        let mut keys: Vec<VoxelKey> = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                keys.push((x, y, 0));
            }
        }
        keys.push((20, 0, 0));
        let map = map_with_healthy(&keys);
        let mut live: AHashSet<VoxelKey> = AHashSet::new();
        live.insert((25, 0, 0));
        let cylinder = LocalBounds {
            origin_x: 0.0,
            origin_y: 0.0,
            r_xy_max_sq: 1e6,
            z_min: -10.0,
            z_max: 10.0,
        };
        let local = points_to_cloud(
            &emit_points(&map, 1.0, Some(&cylinder), 3, &live),
            "world",
            Time::default(),
        );
        let pts = cloud_points(&local);
        assert!(pts.contains(&voxel_center(1, 1, 0)), "dense patch kept");
        assert!(
            !pts.contains(&voxel_center(20, 0, 0)),
            "isolated healthy voxel dropped by support_min"
        );
        assert!(
            pts.contains(&voxel_center(25, 0, 0)),
            "live voxel bypasses support_min"
        );
    }
}
