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

//! RoboSense Airy native module: robosense-airy's sensor -> PointCloud2/Imu.

use dimos_module::{native_config, Module, Output};
use lcm_msgs::geometry_msgs::{Quaternion, Vector3};
use lcm_msgs::sensor_msgs::{Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use robosense_airy::{
    DecoderConfig, FilterConfig, ImuSample, Scan, Sensor, SensorConfig, SplitMode, StopHandle,
};
use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::sync::atomic::Ordering;
use tokio::runtime::Handle;

/// Python's `None`, sent as a JSON null under a key that is always present (a missing key stays an error).
#[derive(Debug, Clone, Serialize)]
#[serde(transparent)]
pub struct Nullable<T>(Option<T>);

impl<'de, T: serde::de::DeserializeOwned> Deserialize<'de> for Nullable<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = serde_json::Value::deserialize(deserializer)?;
        if value.is_null() {
            return Ok(Nullable(None));
        }
        serde_json::from_value(value)
            .map(|inner| Nullable(Some(inner)))
            .map_err(serde::de::Error::custom)
    }
}

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Multicast group the lidar sends to; null listens for unicast.
    group: Nullable<String>,
    /// Address of the local NIC on the lidar's network (joins the group there); null = any.
    host_ip: Nullable<String>,
    /// Only logged here; the Python side derives host_ip from it.
    lidar_ip: Nullable<String>,
    msop_port: u16,
    difop_port: u16,
    imu_port: u16,
    enable_imu: bool,
    point_format: PointFormat,
    frame_id: String,
    imu_frame_id: String,
    imu_in_lidar_frame: bool,
    #[validate(range(min = 0.0))]
    min_distance: f32,
    #[validate(range(min = 0.0))]
    max_distance: f32,
    use_lidar_clock: bool,
    start_angle: f32,
    end_angle: f32,
    split_angle: f32,
    #[validate(range(min = 0.0))]
    voxel: f32,
    #[validate(range(min = 1))]
    decimate_points: usize,
    #[validate(range(min = 1))]
    decimate_scans: usize,
    #[validate(range(min = 1))]
    scan_queue: usize,
    #[validate(range(min = 1))]
    imu_queue: usize,
    recv_buffer_bytes: Nullable<usize>,
}

// Per-point wire layouts. The Python AiryConfig comment is the reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
enum PointFormat {
    Full,
    Minimal,
    Legacy,
}

impl Config {
    fn sensor_config(&self) -> SensorConfig {
        SensorConfig {
            group: self
                .group
                .0
                .as_deref()
                .map(|g| g.parse().expect("invalid group")),
            interface: self
                .host_ip
                .0
                .as_deref()
                .map_or(Ipv4Addr::UNSPECIFIED, |ip| {
                    ip.parse().expect("invalid host_ip")
                }),
            msop_port: self.msop_port,
            difop_port: self.difop_port,
            imu_port: self.enable_imu.then_some(self.imu_port),
            frame_id: self.frame_id.clone(),
            decoder: DecoderConfig {
                min_distance: self.min_distance,
                max_distance: self.max_distance,
                use_lidar_clock: self.use_lidar_clock,
                dense_points: true,
                // stamped with the first point, so every offset_time is >= 0
                ts_first_point: true,
                wait_for_difop: true,
                start_angle: self.start_angle,
                end_angle: self.end_angle,
                split: SplitMode::Angle(self.split_angle),
                transform: None,
            },
            filter: FilterConfig {
                voxel: self.voxel,
                decimate_points: self.decimate_points,
                decimate_scans: self.decimate_scans,
            },
            scan_queue: self.scan_queue,
            imu_queue: self.imu_queue,
            imu_in_lidar_frame: self.imu_in_lidar_frame,
            recv_buffer_bytes: self.recv_buffer_bytes.0,
        }
    }
}

#[derive(Module)]
#[module(name = "airy", setup = start, handle = wait, teardown = stop)]
pub struct Airy {
    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = Imu::encode)]
    imu: Output<Imu>,

    #[config]
    config: Config,

    stopper: Option<StopHandle>,
    ended: std::sync::Arc<tokio::sync::Notify>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Airy {
    async fn start(&mut self) {
        let mut sensor = Sensor::open(self.config.sensor_config())
            .unwrap_or_else(|err| panic!("failed to open the Airy's sockets: {err}"));
        self.stopper = Some(sensor.stop_handle().expect("clone sockets"));
        tracing::info!(group = ?self.config.group.0, host_ip = ?self.config.host_ip.0, lidar_ip = ?self.config.lidar_ip.0, msop = self.config.msop_port,
            frame_id = %self.config.frame_id, "airy listening");
        let handle = Handle::current();
        let imu_rx = sensor.take_imu();
        {
            let (imu, frame_id, handle) = (
                self.imu.clone(),
                self.config.imu_frame_id.clone(),
                handle.clone(),
            );
            self.threads.push(std::thread::spawn(move || {
                for sample in imu_rx.iter() {
                    let _ = handle.block_on(imu.publish(&imu_message(&frame_id, &sample)));
                }
            }));
        }
        let (lidar, config, ended) = (self.lidar.clone(), self.config.clone(), self.ended.clone());
        self.threads.push(std::thread::spawn(move || {
            let stats = sensor.stats_handle();
            for framed in sensor.scans().iter() {
                let msg = cloud_message(config.point_format, &config.frame_id, &framed.scan);
                let _ = handle.block_on(lidar.publish(&msg));
            }
            let get = |c: &std::sync::atomic::AtomicU64| c.load(Ordering::Relaxed);
            tracing::info!(
                scans = get(&stats.scans),
                dropped = get(&stats.scans_dropped),
                imu = get(&stats.imu_samples),
                imu_dropped = get(&stats.imu_dropped),
                decode_errors = get(&stats.decode_errors),
                "airy stopped"
            );
            ended.notify_one();
        }));
    }

    /// Ends the module if the receive side ends on its own.
    async fn wait(&mut self) {
        self.ended.notified().await;
    }

    async fn stop(&mut self) {
        if let Some(stopper) = self.stopper.take() {
            stopper.stop();
        }
        // joined off the runtime: a publisher thread may be parked in a publish that needs this worker
        let threads = std::mem::take(&mut self.threads);
        let _ = tokio::task::spawn_blocking(move || {
            for t in threads {
                let _ = t.join();
            }
        })
        .await;
    }
}

fn stamp(seconds: f64) -> Time {
    let ns = (seconds * 1e9).round().max(0.0) as u64;
    Time {
        sec: (ns / 1_000_000_000) as i32,
        nsec: (ns % 1_000_000_000) as i32,
    }
}

fn header(frame_id: &str, seconds: f64) -> Header {
    Header {
        seq: 0,
        stamp: stamp(seconds),
        frame_id: frame_id.to_string(),
    }
}

fn imu_message(frame_id: &str, sample: &ImuSample) -> Imu {
    let vector = |v: [f32; 3]| Vector3 {
        x: v[0] as f64,
        y: v[1] as f64,
        z: v[2] as f64,
    };
    let mut orientation_covariance = [0.0; 9];
    // orientation unknown: identity with -1 covariance to flag it
    orientation_covariance[0] = -1.0;
    Imu {
        header: header(frame_id, sample.timestamp),
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance,
        angular_velocity: vector(sample.gyro_rad_s),
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: vector(sample.accel_m_s2()),
        linear_acceleration_covariance: [0.0; 9],
    }
}

// Byte offsets within one packed point, shared by the field table and the packing loop.
const OFFSET_X: usize = 0;
const OFFSET_Y: usize = 4;
const OFFSET_Z: usize = 8;
const OFFSET_FOURTH: usize = 12;
const OFFSET_FULL_TIME: usize = 16;
const OFFSET_FULL_RING: usize = 20;

impl PointFormat {
    fn point_step(self) -> usize {
        match self {
            PointFormat::Full => OFFSET_FULL_RING + 2,
            PointFormat::Minimal | PointFormat::Legacy => OFFSET_FOURTH + 4,
        }
    }
}

fn cloud_fields(format: PointFormat) -> Vec<PointField> {
    let field = |name: &str, offset: usize, datatype: u8| PointField {
        name: name.into(),
        offset: offset as i32,
        datatype,
        count: 1,
    };
    let (f32t, u32t, u16t) = (
        PointField::FLOAT32 as u8,
        PointField::UINT32 as u8,
        PointField::UINT16 as u8,
    );
    let mut fields = vec![
        field("x", OFFSET_X, f32t),
        field("y", OFFSET_Y, f32t),
        field("z", OFFSET_Z, f32t),
    ];
    match format {
        PointFormat::Full => fields.extend([
            field("intensity", OFFSET_FOURTH, f32t),
            field("offset_time", OFFSET_FULL_TIME, u32t),
            field("ring", OFFSET_FULL_RING, u16t),
        ]),
        PointFormat::Minimal => fields.push(field("offset_time", OFFSET_FOURTH, u32t)),
        PointFormat::Legacy => fields.push(field("intensity", OFFSET_FOURTH, f32t)),
    }
    fields
}

fn cloud_message(format: PointFormat, frame_id: &str, scan: &Scan) -> PointCloud2 {
    let step = format.point_step();
    let mut data = vec![0u8; step * scan.points.len()];
    for (p, out) in scan.points.iter().zip(data.chunks_exact_mut(step)) {
        out[OFFSET_X..OFFSET_X + 4].copy_from_slice(&p.x.to_le_bytes());
        out[OFFSET_Y..OFFSET_Y + 4].copy_from_slice(&p.y.to_le_bytes());
        out[OFFSET_Z..OFFSET_Z + 4].copy_from_slice(&p.z.to_le_bytes());
        // nanoseconds after the scan's stamp (its first point)
        let offset_ns = ((p.time - scan.timestamp) * 1e9).round().max(0.0) as u32;
        let intensity = p.intensity as f32;
        match format {
            PointFormat::Full => {
                out[OFFSET_FOURTH..OFFSET_FOURTH + 4].copy_from_slice(&intensity.to_le_bytes());
                out[OFFSET_FULL_TIME..OFFSET_FULL_TIME + 4]
                    .copy_from_slice(&offset_ns.to_le_bytes());
                out[OFFSET_FULL_RING..OFFSET_FULL_RING + 2].copy_from_slice(&p.ring.to_le_bytes());
            }
            PointFormat::Minimal => {
                out[OFFSET_FOURTH..OFFSET_FOURTH + 4].copy_from_slice(&offset_ns.to_le_bytes())
            }
            PointFormat::Legacy => {
                out[OFFSET_FOURTH..OFFSET_FOURTH + 4].copy_from_slice(&intensity.to_le_bytes())
            }
        }
    }
    let width = scan.points.len() as i32;
    PointCloud2 {
        header: header(frame_id, scan.timestamp),
        height: 1,
        width,
        fields: cloud_fields(format),
        is_bigendian: false,
        point_step: step as i32,
        row_step: step as i32 * width,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robosense_airy::Point;

    fn scan() -> Scan {
        let point = |x: f32, time: f64, ring: u16| Point {
            x,
            y: 2.0,
            z: 3.0,
            intensity: 7,
            ring,
            time,
            feature: 0,
        };
        Scan {
            seq: 0,
            timestamp: 1_700_000_000.5,
            rings: 96,
            points: vec![
                point(1.0, 1_700_000_000.5, 4),
                point(-1.0, 1_700_000_000.55, 95),
            ],
            complete: true,
        }
    }

    #[test]
    fn full_cloud_layout() {
        let cloud = cloud_message(PointFormat::Full, "front_lidar", &scan());
        assert_eq!(cloud.point_step, 22);
        assert_eq!(cloud.width, 2);
        assert_eq!(cloud.header.frame_id, "front_lidar");
        assert_eq!(
            (cloud.header.stamp.sec, cloud.header.stamp.nsec),
            (1_700_000_000, 500_000_000)
        );
        let names: Vec<&str> = cloud.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["x", "y", "z", "intensity", "offset_time", "ring"]);
        let second = &cloud.data[22..44];
        assert_eq!(f32::from_le_bytes(second[0..4].try_into().unwrap()), -1.0);
        assert_eq!(f32::from_le_bytes(second[12..16].try_into().unwrap()), 7.0);
        let offset = u32::from_le_bytes(second[16..20].try_into().unwrap());
        assert!((offset as i64 - 50_000_000).abs() < 1_000, "{offset}");
        assert_eq!(u16::from_le_bytes(second[20..22].try_into().unwrap()), 95);
    }

    #[test]
    fn minimal_and_legacy_layouts() {
        let minimal = cloud_message(PointFormat::Minimal, "f", &scan());
        assert_eq!(minimal.point_step, 16);
        assert_eq!(minimal.fields.last().unwrap().name, "offset_time");
        let legacy = cloud_message(PointFormat::Legacy, "f", &scan());
        assert_eq!(
            f32::from_le_bytes(legacy.data[12..16].try_into().unwrap()),
            7.0
        );
    }

    #[test]
    fn imu_message_is_si_with_unknown_orientation() {
        let sample = ImuSample {
            timestamp: 5.5,
            accel_g: [1.0, 0.0, 0.0],
            gyro_rad_s: [0.1, 0.2, 0.3],
            cnt: 0,
        };
        let msg = imu_message("front_lidar", &sample);
        assert_eq!(
            (msg.header.stamp.sec, msg.header.stamp.nsec),
            (5, 500_000_000)
        );
        assert_eq!(msg.orientation_covariance[0], -1.0);
        assert!((msg.linear_acceleration.x - 9.80665).abs() < 1e-5);
        assert!((msg.angular_velocity.z - 0.3).abs() < 1e-6);
    }

    #[test]
    fn config_needs_every_key() {
        let json = serde_json::json!({
            "group": "224.10.10.201", "host_ip": null, "lidar_ip": null, "msop_port": 6691, "difop_port": 7781,
            "imu_port": 6681, "enable_imu": true, "point_format": "full", "frame_id": "front_lidar",
            "imu_frame_id": "front_lidar", "imu_in_lidar_frame": true, "min_distance": 0.2, "max_distance": 60.0,
            "use_lidar_clock": true, "start_angle": 0.0, "end_angle": 360.0, "split_angle": 0.0, "voxel": 0.0,
            "decimate_points": 1, "decimate_scans": 1, "scan_queue": 1, "imu_queue": 1, "recv_buffer_bytes": null
        });
        let config: Config = serde_json::from_value(json.clone()).unwrap();
        let sensor = config.sensor_config();
        assert_eq!(sensor.group, Some("224.10.10.201".parse().unwrap()));
        assert_eq!(sensor.interface, Ipv4Addr::UNSPECIFIED);
        assert_eq!(sensor.imu_port, Some(6681));
        let mut missing = json;
        missing.as_object_mut().unwrap().remove("host_ip");
        assert!(serde_json::from_value::<Config>(missing).is_err());
    }
}
