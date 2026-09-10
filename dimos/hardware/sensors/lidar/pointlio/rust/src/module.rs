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

//! Point-LIO native module: PointCloud2 + Imu in, odometry + body cloud + tf out.

use dimos_livox::pipeline::GRAVITY_MS2;
use dimos_module::nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use dimos_module::{native_config, Input, Module, Output, Tf, Transform};
use lcm_msgs::geometry_msgs::{Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance};
use lcm_msgs::geometry_msgs::{Quaternion as QuatMsg, Vector3};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use pointlio_core::{LivoxPoint, PointLio, PointXYZI};
use std::time::Duration;
use validator::ValidationError;

/// The Python `PointLioTuning` fields, 1:1. The tuning block is handed to
/// `pointlio_core::Config` by a JSON round trip, so it stays name-compatible
/// with the C++ config without a hand-written conversion.
#[native_config]
#[derive(Clone)]
#[validate(schema(function = core_config_parses))]
pub struct Config {
    frame_id: String,
    sensor_frame_id: String,
    #[validate(range(exclusive_min = 0.0))]
    pointcloud_freq: f64,
    #[validate(range(exclusive_min = 0.0))]
    odom_freq: f64,
    msr_freq: f64,
    main_freq: f64,
    con_frame: bool,
    con_frame_num: i32,
    cut_frame: bool,
    cut_frame_time_interval: f64,
    time_lag_imu_to_lidar: f64,
    scan_line: i32,
    scan_rate: i32,
    blind: f64,
    point_filter_num: i32,
    use_imu_as_input: bool,
    prop_at_freq_of_imu: bool,
    check_satu: bool,
    init_map_size: i32,
    space_down_sample: bool,
    satu_acc: f64,
    satu_gyro: f64,
    acc_norm: f64,
    plane_thr: f64,
    filter_size_surf: f64,
    filter_size_map: f64,
    ivox_grid_resolution: f64,
    ivox_nearby_type: String,
    cube_side_length: f64,
    det_range: f64,
    fov_degree: f64,
    imu_en: bool,
    start_in_aggressive_motion: bool,
    extrinsic_est_en: bool,
    imu_time_inte: f64,
    lidar_meas_cov: f64,
    acc_cov_input: f64,
    vel_cov: f64,
    gyr_cov_input: f64,
    gyr_cov_output: f64,
    acc_cov_output: f64,
    b_gyr_cov: f64,
    b_acc_cov: f64,
    imu_meas_acc_cov: f64,
    imu_meas_omg_cov: f64,
    match_s: f64,
    gravity_align: bool,
    gravity: Vec<f64>,
    gravity_init: Vec<f64>,
    extrinsic_t: Vec<f64>,
    extrinsic_r: Vec<f64>,
    publish_odometry_without_downsample: bool,
    odom_only: bool,
}

impl Config {
    fn core(&self) -> serde_json::Result<pointlio_core::Config> {
        serde_json::from_value(serde_json::to_value(self)?)
    }
}

fn core_config_parses(cfg: &Config) -> Result<(), ValidationError> {
    cfg.core().map(|_| ()).map_err(|e| {
        let mut err = ValidationError::new("pointlio_core");
        err.message = Some(e.to_string().into());
        err
    })
}

#[derive(Module)]
#[module(name = "pointlio", setup = start)]
pub struct PointLioModule {
    #[input(decode = PointCloud2::decode)]
    lidar_raw: Input<PointCloud2>,

    #[input(decode = Imu::decode)]
    imu: Input<Imu>,

    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,

    #[tf]
    tf: Tf,

    #[config]
    config: Config,

    // Handlers run one at a time on the dispatch loop, so the estimator is
    // fed in arrival order without a lock.
    lio: Option<PointLio>,
    last_cloud_ts: Option<f64>,
    last_odom_ts: Option<f64>,
}

impl PointLioModule {
    async fn start(&mut self) {
        let cfg = self.config.core().expect("validated at launch");
        self.lio = Some(PointLio::new(&cfg));
        tracing::info!(
            frame_id = %self.config.frame_id,
            sensor_frame_id = %self.config.sensor_frame_id,
            odom_freq = self.config.odom_freq,
            pointcloud_freq = self.config.pointcloud_freq,
            "pointlio started"
        );
    }

    async fn handle_imu(&mut self, msg: Imu) {
        let ts = stamp_secs(&msg.header.stamp);
        let g = &msg.angular_velocity;
        let a = &msg.linear_acceleration;
        self.lio.as_mut().expect("setup ran").feed_imu(
            ts,
            [g.x, g.y, g.z],
            [a.x / GRAVITY_MS2, a.y / GRAVITY_MS2, a.z / GRAVITY_MS2],
        );
        self.drain().await;
    }

    async fn handle_lidar_raw(&mut self, msg: PointCloud2) {
        let points = match livox_points(&msg) {
            Ok(points) => points,
            Err(err) => {
                dimos_module::warn_throttled!(Duration::from_secs(5), %err, "dropping cloud");
                return;
            }
        };
        let start_ns = stamp_ns(&msg.header.stamp);
        self.lio
            .as_mut()
            .expect("setup ran")
            .feed_lidar(start_ns, &points);
        self.drain().await;
    }

    /// Replay drain rule: `process()` until the lidar buffer stops shrinking.
    async fn drain(&mut self) {
        loop {
            let lio = self.lio.as_mut().expect("setup ran");
            let before = lio.lidar_buffer_len();
            let fresh = lio.process();
            let after = lio.lidar_buffer_len();
            if fresh {
                self.publish().await;
            }
            if after == before {
                break;
            }
        }
    }

    async fn publish(&mut self) {
        let lio = self.lio.as_ref().expect("setup ran");
        let o = lio.odometry();
        if due(&mut self.last_odom_ts, o.ts, self.config.odom_freq) {
            let msg = odometry_message(&self.config, &o);
            let _ = self.odometry.publish(&msg).await;
            let iso = Isometry3::from_parts(
                Translation3::new(o.pos[0], o.pos[1], o.pos[2]),
                UnitQuaternion::from_quaternion(Quaternion::new(
                    o.quat[3], o.quat[0], o.quat[1], o.quat[2],
                )),
            );
            let t = Transform::new(
                &self.config.frame_id,
                &self.config.sensor_frame_id,
                o.ts,
                iso,
            );
            let _ = self.tf.publish(&[t]).await;
        }
        if due(&mut self.last_cloud_ts, o.ts, self.config.pointcloud_freq) {
            let cloud = lio.body_cloud();
            if !cloud.is_empty() {
                let msg = cloud_message(&self.config.sensor_frame_id, o.ts, &cloud);
                let _ = self.lidar.publish(&msg).await;
            }
        }
    }
}

/// Rate limit on the estimator clock; 10 % slack, `lidar_end_time` jitters a few ms.
fn due(last: &mut Option<f64>, ts: f64, hz: f64) -> bool {
    if last.is_some_and(|l| ts - l < 0.9 / hz) {
        return false;
    }
    *last = Some(ts);
    true
}

fn stamp_secs(t: &Time) -> f64 {
    f64::from(t.sec) + f64::from(t.nsec) * 1e-9
}

fn stamp_ns(t: &Time) -> u64 {
    t.sec as u64 * 1_000_000_000 + t.nsec as u64
}

fn header(frame_id: &str, ts: f64) -> Header {
    let sec = ts.floor();
    Header {
        seq: 0,
        stamp: Time {
            sec: sec as i32,
            nsec: (((ts - sec) * 1e9).round() as i32).min(999_999_999),
        },
        frame_id: frame_id.to_string(),
    }
}

/// Byte offsets of the driver's `minimal`/`full` fields, found by name.
struct Layout {
    x: usize,
    y: usize,
    z: usize,
    offset_time: usize,
    intensity: Option<usize>,
    tag: Option<usize>,
}

fn layout(cloud: &PointCloud2) -> Result<Layout, String> {
    let find = |name: &str, datatype: i8| -> Result<Option<usize>, String> {
        let Some(f) = cloud.fields.iter().find(|f| f.name == name) else {
            return Ok(None);
        };
        if f.datatype != datatype as u8 {
            return Err(format!(
                "field {name} has datatype {}, want {datatype}",
                f.datatype
            ));
        }
        Ok(Some(f.offset as usize))
    };
    let need = |name: &str, datatype: i8| {
        find(name, datatype)?.ok_or_else(|| format!("cloud has no {name} field"))
    };
    if cloud.is_bigendian {
        return Err("big-endian clouds are not supported".into());
    }
    Ok(Layout {
        x: need("x", PointField::FLOAT32)?,
        y: need("y", PointField::FLOAT32)?,
        z: need("z", PointField::FLOAT32)?,
        offset_time: need("offset_time", PointField::UINT32)?,
        intensity: find("intensity", PointField::FLOAT32)?,
        tag: find("tag", PointField::UINT8)?,
    })
}

/// Same point conversion as the replay harness: reflectivity = lround(intensity * 255), line 0.
fn livox_points(cloud: &PointCloud2) -> Result<Vec<LivoxPoint>, String> {
    let l = layout(cloud)?;
    let step = cloud.point_step as usize;
    let n = cloud.width as usize * cloud.height as usize;
    if step == 0 || cloud.data.len() < n * step {
        return Err(format!(
            "data has {} bytes for {n} points of {step}",
            cloud.data.len()
        ));
    }
    let f32_at = |p: &[u8], off: usize| f32::from_le_bytes(p[off..off + 4].try_into().unwrap());
    let u32_at = |p: &[u8], off: usize| u32::from_le_bytes(p[off..off + 4].try_into().unwrap());
    Ok(cloud
        .data
        .chunks_exact(step)
        .take(n)
        .map(|p| LivoxPoint {
            x: f32_at(p, l.x),
            y: f32_at(p, l.y),
            z: f32_at(p, l.z),
            reflectivity: l
                .intensity
                .map_or(0, |off| (f32_at(p, off) * 255.0).round() as u16),
            tag: l.tag.map_or(0, |off| p[off]),
            line: 0,
            offset_time: u64::from(u32_at(p, l.offset_time)),
        })
        .collect())
}

fn odometry_message(cfg: &Config, o: &pointlio_core::Odom) -> Odometry {
    let v = |a: [f64; 3]| Vector3 {
        x: a[0],
        y: a[1],
        z: a[2],
    };
    Odometry {
        header: header(&cfg.frame_id, o.ts),
        child_frame_id: cfg.sensor_frame_id.clone(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: o.pos[0],
                    y: o.pos[1],
                    z: o.pos[2],
                },
                orientation: QuatMsg {
                    x: o.quat[0],
                    y: o.quat[1],
                    z: o.quat[2],
                    w: o.quat[3],
                },
            },
            covariance: [0.0; 36],
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: v(o.vel),
                angular: v(o.omg),
            },
            covariance: [0.0; 36],
        },
    }
}

/// Body cloud as xyzi, the layout the C++ module publishes.
fn cloud_message(frame_id: &str, ts: f64, cloud: &[PointXYZI]) -> PointCloud2 {
    const STEP: usize = 16;
    let mut data = Vec::with_capacity(cloud.len() * STEP);
    for p in cloud {
        for v in [p.x, p.y, p.z, p.intensity] {
            data.extend_from_slice(&v.to_le_bytes());
        }
    }
    let fields = ["x", "y", "z", "intensity"]
        .iter()
        .enumerate()
        .map(|(i, name)| PointField {
            name: (*name).into(),
            offset: (i * 4) as i32,
            datatype: PointField::FLOAT32 as u8,
            count: 1,
        })
        .collect();
    let n = cloud.len() as i32;
    PointCloud2 {
        header: header(frame_id, ts),
        height: 1,
        width: n,
        fields,
        is_bigendian: false,
        point_step: STEP as i32,
        row_step: STEP as i32 * n,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(name: &str, offset: usize, datatype: i8) -> PointField {
        PointField {
            name: name.into(),
            offset: offset as i32,
            datatype: datatype as u8,
            count: 1,
        }
    }

    fn cloud(fields: Vec<PointField>, step: usize, data: Vec<u8>) -> PointCloud2 {
        PointCloud2 {
            header: header("lidar_link", 1.5),
            height: 1,
            width: (data.len() / step) as i32,
            fields,
            is_bigendian: false,
            point_step: step as i32,
            row_step: data.len() as i32,
            data,
            is_dense: true,
        }
    }

    #[test]
    fn full_layout_is_read_by_name() {
        // Driver `full`: x y z intensity offset_time tag line, 22 B.
        let mut data = Vec::new();
        for v in [1.0f32, 2.0, 3.0, 0.5] {
            data.extend_from_slice(&v.to_le_bytes());
        }
        data.extend_from_slice(&42u32.to_le_bytes());
        data.extend_from_slice(&[7u8, 0]);
        let c = cloud(
            vec![
                field("x", 0, PointField::FLOAT32),
                field("y", 4, PointField::FLOAT32),
                field("z", 8, PointField::FLOAT32),
                field("intensity", 12, PointField::FLOAT32),
                field("offset_time", 16, PointField::UINT32),
                field("tag", 20, PointField::UINT8),
                field("line", 21, PointField::UINT8),
            ],
            22,
            data,
        );
        let pts = livox_points(&c).unwrap();
        assert_eq!(pts.len(), 1);
        assert_eq!((pts[0].x, pts[0].y, pts[0].z), (1.0, 2.0, 3.0));
        assert_eq!(pts[0].reflectivity, 128);
        assert_eq!(pts[0].tag, 7);
        assert_eq!(pts[0].offset_time, 42);
        assert_eq!(stamp_ns(&c.header.stamp), 1_500_000_000);
    }

    #[test]
    fn minimal_layout_has_no_intensity() {
        let mut data = Vec::new();
        for v in [1.0f32, 2.0, 3.0] {
            data.extend_from_slice(&v.to_le_bytes());
        }
        data.extend_from_slice(&9u32.to_le_bytes());
        let c = cloud(
            vec![
                field("x", 0, PointField::FLOAT32),
                field("y", 4, PointField::FLOAT32),
                field("z", 8, PointField::FLOAT32),
                field("offset_time", 12, PointField::UINT32),
            ],
            16,
            data,
        );
        let pts = livox_points(&c).unwrap();
        assert_eq!(pts[0].reflectivity, 0);
        assert_eq!(pts[0].offset_time, 9);
        // Legacy xyzi has no offset_time and is refused.
        let mut legacy = c.clone();
        legacy.fields[3] = field("intensity", 12, PointField::FLOAT32);
        assert!(livox_points(&legacy).is_err());
    }

    #[test]
    fn rate_limit_runs_on_estimator_clock() {
        let mut last = None;
        assert!(due(&mut last, 10.0, 10.0));
        assert!(!due(&mut last, 10.05, 10.0));
        // 0.095 s later: inside the slack, so a jittery 10 Hz frame is not skipped.
        assert!(due(&mut last, 10.095, 10.0));
        assert!(!due(&mut last, 10.18, 10.0));
    }

    #[test]
    fn config_round_trips_into_core_and_rejects_bad_stencil() {
        let cfg: Config = serde_json::from_str(SAMPLE).unwrap();
        let core = cfg.core().unwrap();
        assert_eq!(core.ivox_nearby_type, pointlio_core::NearbyType::Nearby18);
        assert_eq!(core.blind, 0.7);
        use validator::Validate;
        cfg.validate().unwrap();
        let mut bad: Config = serde_json::from_str(SAMPLE).unwrap();
        bad.ivox_nearby_type = "x".into();
        assert!(bad.validate().is_err());
    }

    const SAMPLE: &str = r#"{
        "frame_id":"odom","sensor_frame_id":"mid360_link","pointcloud_freq":10.0,"odom_freq":30.0,
        "msr_freq":50.0,"main_freq":5000.0,"con_frame":false,"con_frame_num":1,"cut_frame":false,
        "cut_frame_time_interval":0.1,"time_lag_imu_to_lidar":0.0,"scan_line":4,"scan_rate":10,
        "blind":0.7,"point_filter_num":3,"use_imu_as_input":false,"prop_at_freq_of_imu":true,
        "check_satu":true,"init_map_size":10,"space_down_sample":true,"satu_acc":3.0,
        "satu_gyro":35.0,"acc_norm":1.0,"plane_thr":0.1,"filter_size_surf":0.2,
        "filter_size_map":0.5,"ivox_grid_resolution":2.0,"ivox_nearby_type":"nearby18",
        "cube_side_length":1000.0,"det_range":100.0,"fov_degree":360.0,"imu_en":true,
        "start_in_aggressive_motion":false,"extrinsic_est_en":false,"imu_time_inte":0.005,
        "lidar_meas_cov":0.01,"acc_cov_input":0.1,"vel_cov":20.0,"gyr_cov_input":0.01,
        "gyr_cov_output":1000.0,"acc_cov_output":500.0,"b_gyr_cov":0.0001,"b_acc_cov":0.0001,
        "imu_meas_acc_cov":0.01,"imu_meas_omg_cov":0.01,"match_s":81.0,"gravity_align":true,
        "gravity":[0.0,0.0,-9.81],"gravity_init":[0.0,0.0,-9.81],
        "extrinsic_t":[-0.011,-0.02329,0.04412],"extrinsic_r":[1,0,0,0,1,0,0,0,1],
        "publish_odometry_without_downsample":false,"odom_only":false
    }"#;
}
