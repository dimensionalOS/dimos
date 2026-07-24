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

use std::collections::VecDeque;
use std::time::Duration;

use dimos_module::{error_throttled, run_with_transport, warn_throttled, Input, Module, Output};
use dimos_voxel_ray_tracing::voxel_ray_tracer::{
    batch_local_bounds, emit_points, update_map, Config, LocalBounds, VoxelMap,
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
    poses: VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
    pending_clouds: PendingClouds,
    odometry_watermark: f64,
    watermark_expiries: u64,
    capacity_evictions: u64,
    frame_count: u32,
    batch_points: Vec<(f32, f32, f32)>,
    batch_origins: Vec<(f32, f32, f32)>,
}

impl RayTracingVoxelMap {
    async fn on_odometry(&mut self, msg: Odometry) {
        let p = &msg.pose.pose.position;
        let q = &msg.pose.pose.orientation;
        let stamp = time_secs(&msg.header.stamp);
        self.odometry_watermark = self.odometry_watermark.max(stamp);
        push_pose(
            &mut self.poses,
            (
                stamp,
                Vector3::new(p.x as f32, p.y as f32, p.z as f32),
                UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    q.w as f32, q.x as f32, q.y as f32, q.z as f32,
                )),
            ),
        );
        self.process_pending_clouds().await;
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        if let Some(evicted) = self.pending_clouds.push(msg) {
            self.capacity_evictions += 1;
            let diagnostic = capacity_eviction_diagnostic(
                &evicted,
                &self.poses,
                self.odometry_watermark,
                self.config.pose_match_tolerance_s as f64,
            );
            let SyncDiagnostic::CapacityEviction {
                cloud_stamp,
                watermark,
                tolerance,
                capacity,
                best_pose_gap,
            } = diagnostic
            else {
                unreachable!("capacity diagnostic has the capacity variant");
            };
            warn_throttled!(
                Duration::from_secs(1),
                cloud_stamp,
                watermark,
                tolerance,
                capacity,
                best_pose_gap,
                "Ray tracing cloud evicted at pending capacity",
            );
        }
        self.process_pending_clouds().await;
    }

    async fn process_pending_clouds(&mut self) {
        let tolerance = self.config.pose_match_tolerance_s as f64;
        let decisions =
            self.pending_clouds
                .take_ready(&self.poses, self.odometry_watermark, tolerance);
        for decision in decisions {
            match decision {
                PendingDecision::Ready(msg, (translation, rotation)) => {
                    self.process_cloud(msg, translation, rotation).await;
                }
                PendingDecision::Expired {
                    cloud_stamp,
                    watermark,
                    tolerance,
                    capacity,
                    best_pose_gap,
                } => {
                    self.watermark_expiries += 1;
                    let diagnostic = SyncDiagnostic::WatermarkExpiry {
                        cloud_stamp,
                        watermark,
                        tolerance,
                        capacity,
                        best_pose_gap,
                    };
                    let SyncDiagnostic::WatermarkExpiry {
                        cloud_stamp,
                        watermark,
                        tolerance,
                        capacity,
                        best_pose_gap,
                    } = diagnostic
                    else {
                        unreachable!("expiry diagnostic has the expiry variant");
                    };
                    warn_throttled!(
                        Duration::from_secs(1),
                        cloud_stamp,
                        watermark,
                        tolerance,
                        capacity,
                        best_pose_gap,
                        "Ray tracing cloud expired at odometry watermark",
                    );
                }
            }
        }
    }

    async fn process_cloud(
        &mut self,
        msg: PointCloud2,
        translation: Vector3<f32>,
        rotation: UnitQuaternion<f32>,
    ) {
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
        let support_min = self.config.support_min;
        if global_due {
            let points = emit_points(&self.map, voxel_size, None, 0, &live);
            let global = points_to_cloud(&points, out_frame_id, stamp.clone());
            publish_cloud(&self.global_map, &global).await;
        }
        if let Some(cyl) = &cylinder {
            let points = emit_points(&self.map, voxel_size, Some(cyl), support_min, &live);
            let local = points_to_cloud(&points, out_frame_id, stamp);
            publish_cloud(&self.local_map, &local).await;
        }
    }
}

/// Whether the Nth-frame output fires this frame. Zero disables it.
fn emit_due(frame_count: u32, every: u32) -> bool {
    every != 0 && frame_count.is_multiple_of(every)
}

/// Odometry samples kept for cloud-stamp matching.
const POSE_BUFFER_LEN: usize = 256;

/// Fixed implementation bound for clouds waiting for odometry.
const PENDING_CLOUD_CAPACITY: usize = 32;

fn time_secs(t: &Time) -> f64 {
    t.sec as f64 + t.nsec as f64 * 1e-9
}

/// Append a pose sample, evicting the oldest to keep the buffer bounded.
fn push_pose(
    poses: &mut VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
    sample: (f64, Vector3<f32>, UnitQuaternion<f32>),
) {
    poses.push_back(sample);
    if poses.len() > POSE_BUFFER_LEN {
        poses.pop_front();
    }
}

/// Select the nearest pose on either side of the cloud, once the watermark has
/// reached the cloud. In particular, the preceding pose is never selected
/// while a later odometry sample could still arrive.
fn nearest_pose(
    poses: &VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
    stamp: f64,
    tolerance: f64,
) -> Option<(Vector3<f32>, UnitQuaternion<f32>)> {
    nearest_pose_with_gap(poses, stamp, tolerance).map(|(pose, _)| pose)
}

fn nearest_pose_with_gap(
    poses: &VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
    stamp: f64,
    tolerance: f64,
) -> Option<((Vector3<f32>, UnitQuaternion<f32>), f64)> {
    let preceding = poses
        .iter()
        .filter(|(t, _, _)| *t <= stamp)
        .max_by(|a, b| a.0.total_cmp(&b.0));
    let following = poses
        .iter()
        .filter(|(t, _, _)| *t >= stamp)
        .min_by(|a, b| a.0.total_cmp(&b.0));
    let best = [preceding, following]
        .into_iter()
        .flatten()
        .min_by(|a, b| (a.0 - stamp).abs().total_cmp(&(b.0 - stamp).abs()));
    let best_gap = best.map_or(f64::INFINITY, |(t, _, _)| (t - stamp).abs());
    let best = best.map(|(_, v, q)| ((*v, *q), best_gap));
    if best_gap <= tolerance {
        best
    } else {
        None
    }
}

fn best_pose_gap(poses: &VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>, stamp: f64) -> f64 {
    poses
        .iter()
        .map(|(t, _, _)| (t - stamp).abs())
        .fold(f64::INFINITY, f64::min)
}

#[derive(Default)]
struct PendingClouds {
    clouds: VecDeque<PointCloud2>,
}

enum PendingDecision {
    Ready(PointCloud2, (Vector3<f32>, UnitQuaternion<f32>)),
    Expired {
        cloud_stamp: f64,
        watermark: f64,
        tolerance: f64,
        capacity: usize,
        best_pose_gap: f64,
    },
}

enum SyncDiagnostic {
    WatermarkExpiry {
        cloud_stamp: f64,
        watermark: f64,
        tolerance: f64,
        capacity: usize,
        best_pose_gap: f64,
    },
    CapacityEviction {
        cloud_stamp: f64,
        watermark: f64,
        tolerance: f64,
        capacity: usize,
        best_pose_gap: f64,
    },
}

fn capacity_eviction_diagnostic(
    cloud: &PointCloud2,
    poses: &VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
    watermark: f64,
    tolerance: f64,
) -> SyncDiagnostic {
    let cloud_stamp = time_secs(&cloud.header.stamp);
    SyncDiagnostic::CapacityEviction {
        cloud_stamp,
        watermark,
        tolerance,
        capacity: PENDING_CLOUD_CAPACITY,
        best_pose_gap: best_pose_gap(poses, cloud_stamp),
    }
}

impl PendingClouds {
    fn push(&mut self, cloud: PointCloud2) -> Option<PointCloud2> {
        let evicted = if self.clouds.len() >= PENDING_CLOUD_CAPACITY {
            self.clouds.pop_front()
        } else {
            None
        };
        self.clouds.push_back(cloud);
        evicted
    }

    fn take_ready(
        &mut self,
        poses: &VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)>,
        watermark: f64,
        tolerance: f64,
    ) -> Vec<PendingDecision> {
        let mut decisions = Vec::new();
        let count = self.clouds.len();
        for _ in 0..count {
            let cloud = self.clouds.pop_front().expect("counted pending cloud");
            let stamp = time_secs(&cloud.header.stamp);
            if watermark < stamp {
                self.clouds.push_back(cloud);
            } else if let Some((pose, _)) = nearest_pose_with_gap(poses, stamp, tolerance) {
                decisions.push(PendingDecision::Ready(cloud, pose));
            } else if watermark < stamp + tolerance {
                self.clouds.push_back(cloud);
            } else {
                decisions.push(PendingDecision::Expired {
                    cloud_stamp: stamp,
                    watermark,
                    tolerance,
                    capacity: PENDING_CLOUD_CAPACITY,
                    best_pose_gap: best_pose_gap(poses, stamp),
                });
            }
        }
        decisions
    }
}

#[derive(Debug)]
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

#[tokio::main]
async fn main() {
    run_with_transport::<RayTracingVoxelMap>().await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use ahash::AHashSet;
    use dimos_voxel_ray_tracing::voxel_ray_tracer::{Voxel, VoxelKey};

    #[test]
    fn nearest_pose_picks_by_stamp_and_gates_on_tolerance() {
        let mut poses: VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)> = VecDeque::new();
        for (t, x) in [(1.0, 1.0f32), (2.0, 2.0), (3.0, 3.0)] {
            poses.push_back((t, Vector3::new(x, 0.0, 0.0), UnitQuaternion::identity()));
        }
        let (v, _) = nearest_pose(&poses, 2.04, 0.1).expect("within tolerance");
        assert_eq!(v.x, 2.0, "nearest stamp wins, not the latest");
        assert!(
            nearest_pose(&poses, 3.5, 0.1).is_none(),
            "stale poses must not register a cloud"
        );
        assert!(nearest_pose(&VecDeque::new(), 1.0, 0.1).is_none());
    }

    fn stamped_cloud(stamp: f64) -> PointCloud2 {
        let sec = stamp.floor() as i32;
        make_cloud(
            Vec::new(),
            0,
            "sensor",
            Time {
                sec,
                nsec: ((stamp - sec as f64) * 1e9) as i32,
            },
        )
    }

    fn point_cloud(stamp: f64) -> PointCloud2 {
        let mut data = Vec::new();
        let mut count = 0;
        write_point(&mut data, &mut count, 1.5, 0.5, 0.5);
        let sec = stamp.floor() as i32;
        make_cloud(
            data,
            count,
            "sensor",
            Time {
                sec,
                nsec: ((stamp - sec as f64) * 1e9) as i32,
            },
        )
    }

    fn sync_test_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 0.1,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
            graze_cos: 0.7,
            support_min: 0,
            emit_every: 1,
            global_emit_every: 1,
            region_percentile: 95.0,
            pose_match_tolerance_s: 0.1,
        }
    }

    fn test_pose(stamp: f64, x: f32) -> (f64, Vector3<f32>, UnitQuaternion<f32>) {
        (stamp, Vector3::new(x, 0.0, 0.0), UnitQuaternion::identity())
    }

    #[test]
    fn pending_cloud_waits_for_newer_pose_and_processes_once() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        let mut poses = VecDeque::from([test_pose(0.950, 1.0)]);
        pending.push(stamped_cloud(1.0));
        assert!(pending.take_ready(&poses, 0.950, 0.1).is_empty());
        poses.push_back(test_pose(1.003, 2.0));
        let ready = pending.take_ready(&poses, 1.003, 0.1);
        assert_eq!(ready.len(), 1);
        let PendingDecision::Ready(_, (translation, _)) = &ready[0] else {
            panic!("cloud should be ready");
        };
        assert_eq!(translation.x, 2.0);
        let PendingDecision::Ready(cloud, _) = &ready[0] else {
            panic!("cloud should be ready");
        };
        assert_eq!(time_secs(&cloud.header.stamp), 1.0);
        assert!(pending.take_ready(&poses, 1.003, 0.1).is_empty());
    }

    #[test]
    fn asynchronous_callback_flow_updates_once_and_preserves_source_stamp() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        let mut poses = VecDeque::new();
        let mut map = VoxelMap::default();
        let config = sync_test_config();
        pending.push(point_cloud(1.0));
        assert!(pending.take_ready(&poses, 0.0, 0.1).is_empty());

        poses.push_back(test_pose(1.003, 0.0));
        let ready = pending.take_ready(&poses, 1.003, 0.1);
        assert_eq!(ready.len(), 1);
        let PendingDecision::Ready(cloud, (translation, rotation)) =
            ready.into_iter().next().unwrap()
        else {
            panic!("later odometry should make cloud ready");
        };
        let points = extract_xyz(&cloud).unwrap();
        let transformed: Vec<_> = points
            .iter()
            .map(|&(x, y, z)| {
                let p = rotation * Vector3::new(x, y, z) + translation;
                (p.x, p.y, p.z)
            })
            .collect();
        update_map(
            &mut map,
            (translation.x, translation.y, translation.z),
            &transformed,
            &config,
        );
        let output = points_to_cloud(
            &emit_points(&map, 1.0, None, 0, &AHashSet::new()),
            "world",
            cloud.header.stamp.clone(),
        );
        assert_eq!(map.healthy_count(), 1);
        assert_eq!(time_secs(&output.header.stamp), 1.0);
        assert!(pending.take_ready(&poses, 1.003, 0.1).is_empty());
    }

    #[test]
    fn pending_cloud_expires_after_watermark() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        pending.push(stamped_cloud(1.0));
        assert!(matches!(
            pending.take_ready(&VecDeque::new(), 1.101, 0.1).as_slice(),
            [PendingDecision::Expired { .. }]
        ));
        assert!(pending.clouds.is_empty());
    }

    #[test]
    fn pending_cloud_expires_at_exact_tolerance_boundary_with_context() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        pending.push(stamped_cloud(1.0));
        let decisions = pending.take_ready(&VecDeque::new(), 1.1, 0.1);
        assert!(matches!(
            decisions.as_slice(),
            [PendingDecision::Expired {
                cloud_stamp,
                watermark,
                tolerance,
                capacity,
                best_pose_gap
            }] if (*cloud_stamp - 1.0).abs() < 1e-9
                && (*watermark - 1.1).abs() < 1e-9
                && (*tolerance - 0.1).abs() < 1e-9
                && *capacity == PENDING_CLOUD_CAPACITY
                && best_pose_gap.is_infinite()
        ));
    }

    #[test]
    fn pending_cloud_capacity_is_bounded() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        for stamp in 0..PENDING_CLOUD_CAPACITY + 5 {
            pending.push(stamped_cloud(stamp as f64));
        }
        assert_eq!(pending.clouds.len(), PENDING_CLOUD_CAPACITY);
        assert_eq!(
            time_secs(&pending.clouds.front().unwrap().header.stamp),
            5.0
        );
        let decisions = pending.take_ready(&VecDeque::new(), 100.0, 0.1);
        assert_eq!(decisions.len(), PENDING_CLOUD_CAPACITY);
        assert!(decisions
            .iter()
            .all(|decision| matches!(decision, PendingDecision::Expired { .. })));
    }

    #[test]
    fn capacity_eviction_diagnostic_is_distinct_and_contextual() {
        let poses = VecDeque::from([test_pose(2.0, 0.0)]);
        let diagnostic = capacity_eviction_diagnostic(&stamped_cloud(1.0), &poses, 1.5, 0.1);
        assert!(matches!(
            &diagnostic,
            SyncDiagnostic::CapacityEviction {
                cloud_stamp,
                watermark,
                tolerance,
                capacity,
                best_pose_gap
            } if *cloud_stamp == 1.0
                && *watermark == 1.5
                && *tolerance == 0.1
                && *capacity == PENDING_CLOUD_CAPACITY
                && *best_pose_gap == 1.0
        ));
        assert!(!matches!(
            &diagnostic,
            SyncDiagnostic::WatermarkExpiry { .. }
        ));
    }

    #[test]
    fn expired_and_evicted_clouds_never_update_or_emit() {
        let mut pending = PendingClouds {
            clouds: VecDeque::new(),
        };
        let evicted_stamp = 1.0;
        pending.push(point_cloud(evicted_stamp));
        for stamp in 2..=PENDING_CLOUD_CAPACITY + 1 {
            pending.push(point_cloud(stamp as f64));
        }
        let decisions = pending.take_ready(&VecDeque::new(), 100.0, 0.1);
        let mut updates = 0;
        let mut outputs = 0;
        for decision in &decisions {
            if matches!(decision, PendingDecision::Ready(..)) {
                updates += 1;
                outputs += 1;
            }
        }
        assert_eq!(decisions.len(), PENDING_CLOUD_CAPACITY);
        assert_eq!(updates, 0);
        assert_eq!(outputs, 0);
        assert!(decisions
            .iter()
            .all(|decision| matches!(decision, PendingDecision::Expired { .. })));
        assert!(!decisions.iter().any(|decision| match decision {
            PendingDecision::Ready(cloud, _) => time_secs(&cloud.header.stamp) == evicted_stamp,
            PendingDecision::Expired { cloud_stamp, .. } => *cloud_stamp == evicted_stamp,
        }));
        assert!(pending.clouds.is_empty());
    }

    #[test]
    fn push_pose_evicts_oldest_beyond_capacity() {
        let mut poses: VecDeque<(f64, Vector3<f32>, UnitQuaternion<f32>)> = VecDeque::new();
        for i in 0..(POSE_BUFFER_LEN + 10) {
            push_pose(
                &mut poses,
                (i as f64, Vector3::zeros(), UnitQuaternion::identity()),
            );
        }
        assert_eq!(
            poses.len(),
            POSE_BUFFER_LEN,
            "buffer capped at POSE_BUFFER_LEN"
        );
        assert_eq!(poses.front().unwrap().0, 10.0, "oldest 10 evicted");
        assert_eq!(poses.back().unwrap().0, (POSE_BUFFER_LEN + 9) as f64);
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
}
