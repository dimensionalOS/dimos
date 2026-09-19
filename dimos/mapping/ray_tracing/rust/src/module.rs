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

use std::collections::HashMap;
use std::time::Duration;

use crate::mapper::{Mapper, Pose};
use crate::voxel_ray_tracer::Config;
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, Pose as PoseMsg, PoseStamped, Quaternion};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use tracing::warn;

#[derive(Module)]
#[module(name = "ray_tracing", setup = init_mapper)]
pub struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    // World-frame points a sensor knows to be empty. Their voxels are deleted
    // outright, reaching space ray tracing cannot clear.
    #[input(decode = PointCloud2::decode, handler = on_voxel_clear_mask)]
    voxel_clear_mask: Input<PointCloud2>,

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

    // Stamp of the last applied clear mask, so a late one cannot erase voxels a
    // newer mask already accounted for.
    last_clear_mask_stamp: f64,

    /// Cloud stamp of the last global map published, so a slow input cannot
    /// stretch the global map's cadence without bound. Zero until the first
    /// one, which makes that first publish due immediately.
    last_global_stamp: f64,

    // Stamp of the last cloud registered from each source frame, for
    // max_cloud_rate_hz. Keyed by frame_id so a fast sensor cannot starve a
    // slow one sharing the port.
    last_registered: HashMap<String, f64>,
}

impl RayTracingVoxelMap {
    async fn init_mapper(&mut self) {
        self.mapper = Some(Mapper::new(self.config.clone()));
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        // Register with the transform nearest the cloud stamp, waiting briefly
        // for one still in flight rather than dropping the cloud.
        let stamp = time_secs(&msg.header.stamp);
        if rate_limited(
            &mut self.last_registered,
            self.config.max_cloud_rate_hz,
            &msg.header.frame_id,
            stamp,
        ) {
            return;
        }
        let Some(tf_pose) = self
            .tf
            .lookup(&self.config.world_frame, &msg.header.frame_id)
            .at(stamp)
            .tolerance(self.config.tf_match_tolerance_s)
            .within(TF_WAIT_TIMEOUT)
            .await
        else {
            warn!(
                stamp,
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

        let global_due = global_map_due(
            mapper.global_due(),
            stamp,
            self.last_global_stamp,
            self.config.global_max_interval_s,
        );
        if global_due {
            self.last_global_stamp = stamp;
        }
        let global_points = global_due.then(|| mapper.global_points());
        let local_points = cylinder.as_ref().map(|cyl| mapper.local_points(cyl));
        let fine_points = self
            .config
            .emit_fine
            .then(|| cylinder.as_ref().and_then(|cyl| mapper.fine_points(cyl)))
            .flatten();

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

    /// Delete the voxels covering a cloud of world-frame points a sensor knows
    /// to be empty.
    ///
    /// Ray tracing only clears what a ray passes through, so a sensor that
    /// occludes itself - a wrist camera staring past its own arm - can never
    /// clear the volume its arm hides. It deposits voxels of itself there and
    /// walls itself in. A publisher that knows those points are free says so
    /// here.
    async fn on_voxel_clear_mask(&mut self, msg: PointCloud2) {
        let stamp = time_secs(&msg.header.stamp);
        if stamp < self.last_clear_mask_stamp {
            warn_throttled!(
                Duration::from_secs(1),
                stamp,
                last_stamp = self.last_clear_mask_stamp,
                "Out-of-order voxel clear mask dropped",
            );
            return;
        }
        // The keys are metric positions, so the cloud must already be in the
        // world frame. Registering it would need a pose this node has no reason
        // to trust for a mask.
        if msg.header.frame_id != self.config.world_frame {
            warn_throttled!(
                Duration::from_secs(1),
                frame = %msg.header.frame_id,
                expected = %self.config.world_frame,
                "Voxel clear mask is not in the world frame, dropped a mask.",
            );
            return;
        }
        let points = match extract_xyz(&msg) {
            Ok(p) => p,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to get clear mask points, dropped a mask.",
                );
                return;
            }
        };
        self.last_clear_mask_stamp = stamp;
        if points.is_empty() {
            return;
        }
        let mapper = self.mapper.as_mut().expect("built in setup");
        mapper.clear_metric(points);
    }
}

/// How long to wait for a late transform before dropping a cloud.
const TF_WAIT_TIMEOUT: Duration = Duration::from_millis(50);

/// Whether a cloud arrives too soon after the last one registered from its
/// frame, recording it in `last_registered` when it does not.
///
/// Gating on the cloud stamp rather than the clock keeps a replay thinning the
/// same frames the live run did. A stamp that goes backwards -- a replay
/// looping, or a sensor resetting its clock -- restarts the window rather than
/// blocking every cloud until the old stamp is passed again.
fn rate_limited(
    last_registered: &mut HashMap<String, f64>,
    max_cloud_rate_hz: f32,
    frame_id: &str,
    stamp: f64,
) -> bool {
    if max_cloud_rate_hz <= 0.0 {
        return false;
    }
    let min_period = 1.0 / max_cloud_rate_hz as f64;
    if let Some(&last) = last_registered.get(frame_id) {
        let elapsed = stamp - last;
        if elapsed >= 0.0 && elapsed < min_period {
            return true;
        }
    }
    last_registered.insert(frame_id.to_string(), stamp);
    false
}

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

/// Whether the global map is due: on the frame count, or because too long has
/// passed in cloud time.
///
/// The count alone is what made the global map look dead on the 2026-09-16 R1
/// run. It is due every Nth *accepted cloud*, so anything that slows the input
/// slows the map by the same factor -- at 4 clouds a second, `50` is every
/// 12.5 s; at a quarter of a cloud a second it is every 200 s. That run's input
/// fell to 1.5 Hz, its 63rd global publish came due at frame 3150, and the run
/// ended at frame 3117. Nothing had failed and nothing was ever going to
/// publish again.
///
/// *max_interval_s* is measured on the cloud's own stamp, not the wall clock,
/// so a replay emits on the same cadence as the run it is replaying. Zero
/// leaves the count as the only trigger, which is the default: where the input
/// is slow *because* the global emit is expensive -- it is unbounded and scans
/// the whole map -- forcing it more often makes that worse.
fn global_map_due(count_due: bool, stamp: f64, last_stamp: f64, max_interval_s: f32) -> bool {
    count_due || (max_interval_s > 0.0 && stamp - last_stamp >= max_interval_s as f64)
}

#[cfg(test)]

mod tests {
    #[test]
    fn a_slow_input_cannot_stretch_the_global_map_without_bound() {
        // The 2026-09-16 R1 run, in miniature. Clouds arrive every four
        // seconds and the count is not due; without an interval the global map
        // is simply never published again.
        let mut last = 1000.0;
        let mut published = 0;
        for step in 1..=10 {
            let stamp = 1000.0 + 4.0 * step as f64;
            if global_map_due(false, stamp, last, 0.0) {
                published += 1;
                last = stamp;
            }
        }
        assert_eq!(published, 0, "the count alone publishes nothing here");

        // With a 20 s ceiling the same clouds get a map every 20 s.
        let mut last = 1000.0;
        let mut published = 0;
        for step in 1..=10 {
            let stamp = 1000.0 + 4.0 * step as f64;
            if global_map_due(false, stamp, last, 20.0) {
                published += 1;
                last = stamp;
            }
        }
        assert_eq!(published, 2, "40 s of clouds at a 20 s ceiling");
    }

    #[test]
    fn the_count_still_wins_when_the_input_is_healthy() {
        // The ceiling must not *add* publishes to a fast input -- that is the
        // case where the global emit is already the expensive thing.
        assert!(global_map_due(true, 100.0, 99.9, 0.0));
        assert!(global_map_due(true, 100.0, 99.9, 20.0));
        assert!(!global_map_due(false, 100.0, 99.9, 20.0));
    }

    #[test]
    fn a_zero_interval_leaves_the_count_as_the_only_trigger() {
        // The default, and what every robot that has not measured itself gets.
        assert!(!global_map_due(false, 1e9, 0.0, 0.0));
    }

    use super::*;
    use crate::voxel_ray_tracer::{
        emit_points, metric_voxel_keys, update_map, LocalBounds, VoxelKey, VoxelMap,
    };
    use ahash::AHashSet;

    /// Build a map whose listed voxels are healthy, through the public API.
    fn map_with_healthy(keys: &[VoxelKey]) -> VoxelMap {
        let cfg = Config {
            voxel_size: 1.0,
            fine_divisor: 0,
            emit_fine: false,
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
            global_max_interval_s: 0.0,
            region_percentile: 95.0,
            world_frame: "world".to_string(),
            tf_match_tolerance_s: 0.1,
            worker_threads: 4,
            max_cloud_rate_hz: 0.0,
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

    /// The clear-mask handler names voxels by decoding a cloud and quantizing
    /// it. Both halves have to agree with how returns were quantized on the way
    /// in, or a mask silently clears nothing.
    #[test]
    fn clear_mask_cloud_round_trips_to_the_voxels_it_covers() {
        let map = map_with_healthy(&[(3, -2, 1)]);
        let occupied: Vec<VoxelKey> = map.voxels.keys().copied().collect();
        assert_eq!(occupied, vec![(3, -2, 1)]);

        // A mask cloud naming that voxel's center, encoded and decoded exactly
        // as the port would.
        let cloud = points_to_cloud(&[3.5, -1.5, 1.5], "world", Time::default());
        let Ok(points) = extract_xyz(&cloud) else {
            panic!("clear mask cloud must decode");
        };
        let keys: Vec<VoxelKey> = metric_voxel_keys(points, 1.0).collect();

        assert_eq!(keys, occupied);
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

    /// A gate at `hz`, and its state.
    fn gate(hz: f32) -> (HashMap<String, f64>, f32) {
        (HashMap::new(), hz)
    }

    #[test]
    fn zero_rate_accepts_every_cloud() {
        let (mut last, hz) = gate(0.0);
        for i in 0..10 {
            assert!(!rate_limited(
                &mut last,
                hz,
                "lidar",
                100.0 + i as f64 * 0.001
            ));
        }
    }

    #[test]
    fn a_cloud_inside_the_window_is_skipped_and_one_past_it_is_not() {
        let (mut last, hz) = gate(5.0);
        assert!(
            !rate_limited(&mut last, hz, "lidar", 100.0),
            "first is always taken"
        );
        assert!(
            rate_limited(&mut last, hz, "lidar", 100.1),
            "0.1s < the 0.2s window"
        );
        assert!(
            !rate_limited(&mut last, hz, "lidar", 100.2),
            "0.2s reaches the window"
        );
        // The window runs from the cloud that was taken, not the one skipped:
        // 100.3 is 0.1 s after 100.2.
        assert!(rate_limited(&mut last, hz, "lidar", 100.3));
    }

    #[test]
    fn a_fast_frame_cannot_starve_a_slow_one_sharing_the_port() {
        // The whole reason the cap is per frame_id: on the R1 Pro a 10 Hz lidar
        // and a 4.5 Hz stereo cloud arrive on the same input.
        let (mut last, hz) = gate(5.0);
        assert!(!rate_limited(&mut last, hz, "lidar", 100.0));
        assert!(
            !rate_limited(&mut last, hz, "camera", 100.01),
            "different source, own window"
        );
        assert!(rate_limited(&mut last, hz, "lidar", 100.05));
        assert!(rate_limited(&mut last, hz, "camera", 100.06));
    }

    #[test]
    fn a_stamp_that_goes_backwards_restarts_the_window() {
        // A replay looping, or a sensor resetting its clock. Holding the old
        // stamp would block every cloud until the recording caught up again.
        let (mut last, hz) = gate(5.0);
        assert!(!rate_limited(&mut last, hz, "lidar", 1000.0));
        assert!(
            !rate_limited(&mut last, hz, "lidar", 100.0),
            "earlier stamp is taken"
        );
        assert!(
            rate_limited(&mut last, hz, "lidar", 100.1),
            "and becomes the new window"
        );
    }
}
