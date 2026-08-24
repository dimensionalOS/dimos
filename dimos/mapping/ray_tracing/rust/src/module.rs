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

use crate::voxel_ray_tracer::{
    batch_local_bounds, emit_points, update_map, Config, LocalBounds, VoxelMap,
};
use dimos_module::{error_throttled, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseStamped, Quaternion};
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use nalgebra::{Isometry3, Translation3, Vector3};

#[derive(Module)]
#[module(name = "ray_tracing")]
pub struct RayTracingVoxelMap {
    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

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

    #[tf]
    tf: Tf,

    map: VoxelMap,
    frame_count: u32,
    batch_points: Vec<(f32, f32, f32)>,
    batch_origins: Vec<(f32, f32, f32)>,
}

impl RayTracingVoxelMap {
    async fn on_lidar(&mut self, msg: PointCloud2) {
        let stamp = time_secs(&msg.header.stamp);
        // The tf graph fills from the transport, not this loop, so this await cannot
        // deadlock, and it returns early once every tf edge has passed the stamp.
        let Some(transform) = self
            .tf
            .lookup(&self.config.output_frame, &msg.header.frame_id)
            .at(stamp)
            .tolerance(self.config.tf_match_tolerance_s)
            .within(Duration::from_secs_f64(self.config.tf_wait_timeout_s))
            .await
        else {
            warn_throttled!(
                Duration::from_secs(1),
                output_frame = %self.config.output_frame,
                frame_id = %msg.header.frame_id,
                "No tf arrived in time to place a cloud, dropped it.",
            );
            return;
        };
        let sensor_pose: Isometry3<f32> = Isometry3::from_parts(
            Translation3::from(transform.translation()),
            transform.rotation(),
        )
        .cast::<f32>();

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

        let rot = sensor_pose.rotation.to_rotation_matrix();
        let trans = sensor_pose.translation.vector;
        let origin = (trans.x, trans.y, trans.z);
        let points: Vec<(f32, f32, f32)> = points
            .iter()
            .map(|&(x, y, z)| {
                let p = rot * Vector3::new(x, y, z) + trans;
                (p.x, p.y, p.z)
            })
            .collect();

        let out_frame_id = self.config.output_frame.clone();

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
                    frame_id: out_frame_id.clone(),
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
        let support_min = self.config.support_min;
        if global_due {
            let points = emit_points(&self.map, voxel_size, None, 0, &live);
            let global = points_to_cloud(&points, &out_frame_id, stamp.clone());
            publish_cloud(&self.global_map, &global).await;
        }
        if let Some(cyl) = &cylinder {
            let points = emit_points(&self.map, voxel_size, Some(cyl), support_min, &live);
            let local = points_to_cloud(&points, &out_frame_id, stamp);
            publish_cloud(&self.local_map, &local).await;
        }
    }
}

/// Whether the Nth-frame output fires this frame. Zero disables it.
fn emit_due(frame_count: u32, every: u32) -> bool {
    every != 0 && frame_count.is_multiple_of(every)
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

/// Pack selected points into an LCM cloud message.
fn points_to_cloud(points: &[(f32, f32, f32)], frame_id: &str, stamp: Time) -> PointCloud2 {
    let mut data = Vec::with_capacity(points.len() * 16);
    let mut n: i32 = 0;
    for &(x, y, z) in points {
        write_point(&mut data, &mut n, x, y, z);
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
    use crate::voxel_ray_tracer::{Voxel, VoxelKey};
    use ahash::AHashSet;
    use dimos_module::run;
    use dimos_module::transport::{Dispatch, Transport};
    use lcm_msgs::geometry_msgs::{
        Transform as TfTransform, TransformStamped, Vector3 as TfVector3,
    };
    use lcm_msgs::tf2_msgs::TFMessage;
    use std::collections::{HashMap, VecDeque};
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};
    use tokio::sync::Notify;

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
        let mut map = VoxelMap::default();
        for x in 0..3 {
            for y in 0..3 {
                map.voxels.insert((x, y, 0), Voxel::with_health(1));
            }
        }
        map.voxels.insert((20, 0, 0), Voxel::with_health(1));
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

    const LIDAR_TOPIC: &str = "/lidar";
    const TF_TOPIC: &str = "/tf";
    const GLOBAL_TOPIC: &str = "/global_map";
    const WAIT_TIMEOUT_S: f64 = 3.0;
    const TOLERANCE_S: f64 = 0.1;

    type Inbound = Arc<Mutex<VecDeque<(String, Vec<u8>)>>>;

    #[derive(Clone)]
    struct Bus {
        inbound: Inbound,
        arrived: Arc<Notify>,
        subscriptions: Arc<Mutex<HashMap<String, Vec<Dispatch>>>>,
        published: Arc<Mutex<Vec<String>>>,
        delivering: Arc<AtomicBool>,
    }

    impl Bus {
        fn new() -> Self {
            Self {
                inbound: Arc::new(Mutex::new(VecDeque::new())),
                arrived: Arc::new(Notify::new()),
                subscriptions: Arc::new(Mutex::new(HashMap::new())),
                published: Arc::new(Mutex::new(Vec::new())),
                delivering: Arc::new(AtomicBool::new(false)),
            }
        }

        fn send(&self, channel: &str, data: Vec<u8>) {
            self.inbound
                .lock()
                .unwrap()
                .push_back((channel.to_string(), data));
            self.arrived.notify_one();
        }

        fn publish_count(&self, channel: &str) -> usize {
            self.published
                .lock()
                .unwrap()
                .iter()
                .filter(|c| c.as_str() == channel)
                .count()
        }

        fn subscribed(&self, channel: &str) -> bool {
            self.subscriptions.lock().unwrap().contains_key(channel)
        }

        fn spawn_delivery(&self) {
            let inbound = Arc::clone(&self.inbound);
            let arrived = Arc::clone(&self.arrived);
            let subscriptions = Arc::clone(&self.subscriptions);
            tokio::spawn(async move {
                loop {
                    let next = inbound.lock().unwrap().pop_front();
                    let Some((channel, data)) = next else {
                        arrived.notified().await;
                        continue;
                    };
                    let callbacks = subscriptions.lock().unwrap().get(&channel).cloned();
                    for callback in callbacks.iter().flatten() {
                        callback(&data);
                    }
                }
            });
        }
    }

    impl Transport for Bus {
        async fn publish(&self, channel: &str, _data: Vec<u8>) -> std::io::Result<()> {
            self.published.lock().unwrap().push(channel.to_string());
            Ok(())
        }

        async fn subscribe(&self, channel: &str, on_msg: Dispatch) -> std::io::Result<()> {
            self.subscriptions
                .lock()
                .unwrap()
                .entry(channel.to_string())
                .or_default()
                .push(on_msg);
            if !self.delivering.swap(true, Ordering::SeqCst) {
                self.spawn_delivery();
            }
            Ok(())
        }
    }

    fn tf_test_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.5,
            support_min: 0,
            emit_every: 0,
            global_emit_every: 1,
            region_percentile: 95.0,
            output_frame: "map".into(),
            tf_match_tolerance_s: TOLERANCE_S,
            tf_wait_timeout_s: WAIT_TIMEOUT_S,
        }
    }

    fn stamp_at(ts: f64) -> Time {
        let sec = ts.floor();
        Time {
            sec: sec as i32,
            nsec: ((ts - sec) * 1e9).round() as i32,
        }
    }

    fn tf_bytes(edges: &[(&str, &str, f64)]) -> Vec<u8> {
        let transforms = edges
            .iter()
            .map(|(parent, child, ts)| TransformStamped {
                header: Header {
                    seq: 0,
                    stamp: stamp_at(*ts),
                    frame_id: (*parent).into(),
                },
                child_frame_id: (*child).into(),
                transform: TfTransform {
                    translation: TfVector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    rotation: Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
            })
            .collect();
        TFMessage { transforms }.encode()
    }

    fn cloud_bytes(ts: f64) -> Vec<u8> {
        points_to_cloud(&[(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], "lidar", stamp_at(ts)).encode()
    }

    async fn start(config: Config) -> Bus {
        let bus = Bus::new();
        let launch = serde_json::json!({
            "topics": {
                "lidar": LIDAR_TOPIC,
                "tf": TF_TOPIC,
                "global_map": GLOBAL_TOPIC,
                "local_map": "/local_map",
                "region_bounds": "/region_bounds",
            },
            "config": serde_json::to_value(&config).expect("config serializes"),
        });
        let running = bus.clone();
        tokio::spawn(async move { run::<RayTracingVoxelMap, Bus>(running, launch).await });
        poll_until(Duration::from_secs(5), || {
            bus.subscribed(LIDAR_TOPIC) && bus.subscribed(TF_TOPIC)
        })
        .await;
        bus
    }

    async fn poll_until(within: Duration, mut done: impl FnMut() -> bool) -> bool {
        let deadline = tokio::time::Instant::now() + within;
        while tokio::time::Instant::now() < deadline {
            if done() {
                return true;
            }
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
        done()
    }

    // The wait exists for the normal case: the cloud beats its transform to the
    // module. Removing it would drop every such cloud.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_cloud_ahead_of_the_tf_stream_waits_for_the_late_transform() {
        let bus = start(tf_test_config()).await;
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1000.0), ("odom", "lidar", 1000.0)]),
        );
        bus.send(LIDAR_TOPIC, cloud_bytes(1000.4));
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(
            bus.publish_count(GLOBAL_TOPIC),
            0,
            "no transform within tolerance yet, so nothing may be placed"
        );
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1000.4), ("odom", "lidar", 1000.4)]),
        );
        assert!(
            poll_until(Duration::from_millis(500), || bus
                .publish_count(GLOBAL_TOPIC)
                >= 1)
            .await,
            "the late transform must still place the waiting cloud"
        );
    }

    // A composed transform carries its stalest edge's stamp, so an edge that has
    // not reached the cloud yet keeps the composed latest behind the cloud and
    // the full wait alive. The zero wait cannot fire while any edge is in flight.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn an_edge_behind_the_cloud_holds_the_composed_latest_back() {
        let bus = start(tf_test_config()).await;
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1000.5), ("map", "odom", 1001.0)]),
        );
        bus.send(TF_TOPIC, tf_bytes(&[("odom", "lidar", 999.0)]));
        bus.send(LIDAR_TOPIC, cloud_bytes(1000.5));
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(
            bus.publish_count(GLOBAL_TOPIC),
            0,
            "the lidar edge has no sample near the cloud yet"
        );
        bus.send(TF_TOPIC, tf_bytes(&[("odom", "lidar", 1000.5)]));
        assert!(
            poll_until(Duration::from_millis(500), || bus
                .publish_count(GLOBAL_TOPIC)
                >= 1)
            .await,
            "a fresher sibling edge must not shorten the wait on the stale edge"
        );
    }

    // Every edge past the stamp means no sample within tolerance can still
    // arrive, so waiting only delays the clouds queued behind this one.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_stamp_every_edge_has_passed_is_dropped_without_delaying_the_next_cloud() {
        let bus = start(tf_test_config()).await;
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1000.0), ("odom", "lidar", 1000.0)]),
        );
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1001.0), ("odom", "lidar", 1001.0)]),
        );
        bus.send(LIDAR_TOPIC, cloud_bytes(1000.5));
        bus.send(LIDAR_TOPIC, cloud_bytes(1001.0));
        assert!(
            poll_until(Duration::from_millis(500), || bus
                .publish_count(GLOBAL_TOPIC)
                >= 1)
            .await,
            "the placeable cloud must not sit behind a {WAIT_TIMEOUT_S}s wait it can never win"
        );
        assert_eq!(
            bus.publish_count(GLOBAL_TOPIC),
            1,
            "the passed-over cloud has no transform within tolerance and must be dropped"
        );
    }

    // Same drop rule, but the edges that pass the stamp land only after the wait
    // has already begun: the give-up must fire from inside the wait, not just on
    // entry.
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn a_cloud_older_than_the_whole_tf_stream_does_not_delay_the_next_cloud() {
        let bus = start(tf_test_config()).await;
        bus.send(LIDAR_TOPIC, cloud_bytes(1000.0));
        tokio::time::sleep(Duration::from_millis(30)).await;
        bus.send(
            TF_TOPIC,
            tf_bytes(&[("map", "odom", 1001.0), ("odom", "lidar", 1001.0)]),
        );
        bus.send(LIDAR_TOPIC, cloud_bytes(1001.0));
        assert!(
            poll_until(Duration::from_millis(500), || bus
                .publish_count(GLOBAL_TOPIC)
                >= 1)
            .await,
            "the placeable cloud must not wait out the first cloud's {WAIT_TIMEOUT_S}s timeout"
        );
        assert_eq!(
            bus.publish_count(GLOBAL_TOPIC),
            1,
            "the pre-stream cloud can never match and must be dropped"
        );
    }
}
