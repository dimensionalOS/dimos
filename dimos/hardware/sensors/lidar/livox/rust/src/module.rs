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

//! Mid-360 native module: packet source -> pipeline -> PointCloud2/Imu.

use crate::live::{LiveConfig, LiveSource, Ports};
use crate::pcap::PcapSource;
use crate::pipeline::{imu_records, Frame, ImuRecord, PacketSource};
use crate::wire::{DataPacket, DataType};
use dimos_module::{native_config, Module, Output};
use lcm_msgs::geometry_msgs::{Quaternion, Vector3};
use lcm_msgs::sensor_msgs::{Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::runtime::Handle;

/// Python's `None`, sent as a JSON null under a key that is always present.
/// native_config forbids `Option` so an absent key can't pass as None; a null
/// under a present key is what the wrapper guarantees.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(transparent)]
pub struct Nullable<T>(Option<T>);

#[native_config]
#[derive(Clone)]
pub struct Config {
    host_ip: Nullable<String>,
    lidar_ip: String,
    #[validate(range(exclusive_min = 0.0))]
    frequency: f64,
    enable_imu: bool,
    point_format: String,
    frame_id: String,
    imu_frame_id: String,
    /// Replay this capture instead of a live sensor.
    pcap: Nullable<String>,
    /// Replay speed relative to capture time. Null runs flat-out.
    #[validate(custom(function = positive_replay_rate))]
    replay_rate: Nullable<f64>,
    /// Multicast group the device streams data to. Null receives unicast
    /// only, the loopback/virtual arrangement.
    multicast_ip: Nullable<String>,
    cmd_data_port: u16,
    push_msg_port: u16,
    point_data_port: u16,
    imu_data_port: u16,
    log_data_port: u16,
    host_cmd_data_port: u16,
    host_push_msg_port: u16,
    host_point_data_port: u16,
    host_imu_data_port: u16,
    host_log_data_port: u16,
}

fn positive_replay_rate(rate: &Nullable<f64>) -> Result<(), validator::ValidationError> {
    match rate.0 {
        Some(r) if r <= 0.0 => Err(validator::ValidationError::new(
            "replay_rate must be > 0 (or null for flat-out replay)",
        )),
        _ => Ok(()),
    }
}

// Wire layout per point, matching the Python Mid360Config docstring:
//   minimal x,y,z,offset_time                    - 16 B (default)
//   full    x,y,z,intensity,offset_time,tag,line - 22 B
//   legacy  x,y,z,intensity                      - 16 B
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PointFormat {
    Full,
    Minimal,
    Legacy,
}

impl PointFormat {
    fn parse(name: &str) -> PointFormat {
        match name {
            "full" => PointFormat::Full,
            "minimal" => PointFormat::Minimal,
            "legacy" => PointFormat::Legacy,
            other => panic!("point_format must be full, minimal or legacy, got '{other}'"),
        }
    }

    fn point_step(self) -> i32 {
        match self {
            PointFormat::Full => 22,
            PointFormat::Minimal | PointFormat::Legacy => 16,
        }
    }
}

impl Config {
    fn ports(&self) -> Ports {
        Ports {
            cmd_data: self.cmd_data_port,
            push_msg: self.push_msg_port,
            point_data: self.point_data_port,
            imu_data: self.imu_data_port,
            log_data: self.log_data_port,
            host_cmd_data: self.host_cmd_data_port,
            host_push_msg: self.host_push_msg_port,
            host_point_data: self.host_point_data_port,
            host_imu_data: self.host_imu_data_port,
            host_log_data: self.host_log_data_port,
        }
    }
}

#[derive(Module)]
#[module(name = "mid360", setup = start, teardown = stop)]
pub struct Mid360 {
    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = Imu::encode)]
    imu: Output<Imu>,

    #[config]
    config: Config,

    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Mid360 {
    async fn start(&mut self) {
        let format = PointFormat::parse(&self.config.point_format);
        let source = self.build_source();
        let handle = Handle::current();
        let lidar = self.lidar.clone();
        let imu = self.imu.clone();
        let config = self.config.clone();
        let stop = self.stop.clone();
        self.threads.push(std::thread::spawn(move || {
            run_pipeline(source, &config, format, &handle, &lidar, &imu, &stop);
        }));
    }

    async fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }

    fn build_source(&self) -> Box<dyn PacketSource + Send> {
        let config = &self.config;
        if let Nullable(Some(path)) = &config.pcap {
            assert!(
                !path.is_empty(),
                "pcap replay selected but the path is empty; set DIMOS_MID360_PCAP or config.pcap"
            );
            let source = PcapSource::from_file(
                path,
                config.point_data_port,
                config.imu_data_port,
                config.replay_rate.0,
            )
            .unwrap_or_else(|err| panic!("failed to open pcap '{path}': {err}"));
            tracing::info!(pcap = %path, rate = ?config.replay_rate.0, "replaying capture");
            return Box::new(source);
        }

        let host_ip: Ipv4Addr = config
            .host_ip
            .0
            .as_deref()
            .expect("host_ip is required for live capture")
            .parse()
            .expect("invalid host_ip");
        let lidar_ip: Ipv4Addr = config.lidar_ip.parse().expect("invalid lidar_ip");
        let multicast_ip = config
            .multicast_ip
            .0
            .as_deref()
            .map(|ip| ip.parse().expect("invalid multicast_ip"));
        let source = LiveSource::start(LiveConfig {
            host_ip,
            lidar_ip,
            multicast_ip,
            enable_imu: config.enable_imu,
            ports: config.ports(),
        })
        .unwrap_or_else(|err| panic!("failed to start live capture: {err}"));
        // Module shutdown must end the source's blocking recv.
        let live_stop = source.stop_flag();
        let module_stop = self.stop.clone();
        std::thread::spawn(move || {
            while !module_stop.load(Ordering::Relaxed) {
                std::thread::sleep(std::time::Duration::from_millis(100));
            }
            live_stop.store(true, Ordering::Relaxed);
        });
        Box::new(source)
    }
}

fn run_pipeline(
    mut source: Box<dyn PacketSource + Send>,
    config: &Config,
    format: PointFormat,
    handle: &Handle,
    lidar: &Output<PointCloud2>,
    imu: &Output<Imu>,
    stop: &AtomicBool,
) {
    let mut assembler = crate::pipeline::FrameAssembler::new(config.frequency);
    let mut buf = [0u8; 4096];
    while !stop.load(Ordering::Relaxed) {
        let Some(len) = source.recv(&mut buf) else {
            break;
        };
        let Ok(packet) = DataPacket::parse(&buf[..len]) else {
            continue;
        };
        match packet.data_type {
            DataType::Imu => {
                if config.enable_imu {
                    for record in imu_records(&packet) {
                        let msg = imu_message(&config.imu_frame_id, &record);
                        let _ = handle.block_on(imu.publish(&msg));
                    }
                }
            }
            _ => {
                if let Some(frame) = assembler.push(&packet) {
                    let msg = cloud_message(format, &config.frame_id, &frame);
                    let _ = handle.block_on(lidar.publish(&msg));
                }
            }
        }
    }
    if let Some(frame) = assembler.flush() {
        let msg = cloud_message(format, &config.frame_id, &frame);
        let _ = handle.block_on(lidar.publish(&msg));
    }
    tracing::info!("packet source ended");
}

fn header(frame_id: &str, ts_ns: u64) -> Header {
    Header {
        seq: 0,
        stamp: Time {
            sec: (ts_ns / 1_000_000_000) as i32,
            nsec: (ts_ns % 1_000_000_000) as i32,
        },
        frame_id: frame_id.to_string(),
    }
}

fn imu_message(frame_id: &str, record: &ImuRecord) -> Imu {
    let vector = |v: [f64; 3]| Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    };
    let mut orientation_covariance = [0.0; 9];
    // Orientation unknown: identity with -1 covariance to flag it.
    orientation_covariance[0] = -1.0;
    Imu {
        header: header(frame_id, record.ts_ns),
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance,
        angular_velocity: vector(record.gyro_rads),
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: vector(record.acc_ms2),
        linear_acceleration_covariance: [0.0; 9],
    }
}

fn cloud_message(format: PointFormat, frame_id: &str, frame: &Frame) -> PointCloud2 {
    let make_field = |name: &str, offset: i32, datatype: u8| PointField {
        name: name.into(),
        offset,
        datatype,
        count: 1,
    };
    let f32t = PointField::FLOAT32 as u8;
    let u32t = PointField::UINT32 as u8;
    let u8t = PointField::UINT8 as u8;
    let fields = match format {
        PointFormat::Full => vec![
            make_field("x", 0, f32t),
            make_field("y", 4, f32t),
            make_field("z", 8, f32t),
            make_field("intensity", 12, f32t),
            make_field("offset_time", 16, u32t),
            make_field("tag", 20, u8t),
            make_field("line", 21, u8t),
        ],
        PointFormat::Minimal => vec![
            make_field("x", 0, f32t),
            make_field("y", 4, f32t),
            make_field("z", 8, f32t),
            make_field("offset_time", 12, u32t),
        ],
        PointFormat::Legacy => vec![
            make_field("x", 0, f32t),
            make_field("y", 4, f32t),
            make_field("z", 8, f32t),
            make_field("intensity", 12, f32t),
        ],
    };

    let step = format.point_step() as usize;
    let mut data = vec![0u8; step * frame.points.len()];
    for (i, p) in frame.points.iter().enumerate() {
        let base = &mut data[i * step..(i + 1) * step];
        base[0..4].copy_from_slice(&p.xyz_m[0].to_le_bytes());
        base[4..8].copy_from_slice(&p.xyz_m[1].to_le_bytes());
        base[8..12].copy_from_slice(&p.xyz_m[2].to_le_bytes());
        match format {
            PointFormat::Full => {
                base[12..16].copy_from_slice(&p.intensity.to_le_bytes());
                base[16..20].copy_from_slice(&p.offset_ns.to_le_bytes());
                base[20] = p.tag;
                base[21] = 0; // Mid-360: single line
            }
            PointFormat::Minimal => {
                base[12..16].copy_from_slice(&p.offset_ns.to_le_bytes());
            }
            PointFormat::Legacy => {
                base[12..16].copy_from_slice(&p.intensity.to_le_bytes());
            }
        }
    }

    let n = frame.points.len() as i32;
    PointCloud2 {
        header: header(frame_id, frame.start_ns),
        height: 1,
        width: n,
        fields,
        is_bigendian: false,
        point_step: format.point_step(),
        row_step: format.point_step() * n,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::RawPoint;

    fn frame() -> Frame {
        Frame {
            start_ns: 1_700_000_000_123_000_000,
            points: vec![
                RawPoint {
                    xyz_m: [1.0, 2.0, 3.0],
                    intensity: 0.5,
                    offset_ns: 42,
                    tag: 7,
                },
                RawPoint {
                    xyz_m: [-1.0, 0.0, 0.5],
                    intensity: 1.0,
                    offset_ns: 50_000_000,
                    tag: 0,
                },
            ],
        }
    }

    #[test]
    fn full_cloud_layout_matches_cpp() {
        let cloud = cloud_message(PointFormat::Full, "lidar_link", &frame());
        assert_eq!(cloud.point_step, 22);
        assert_eq!(cloud.width, 2);
        assert_eq!(cloud.header.frame_id, "lidar_link");
        assert_eq!(cloud.header.stamp.sec, 1_700_000_000);
        assert_eq!(cloud.header.stamp.nsec, 123_000_000);
        let names: Vec<&str> = cloud.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(
            names,
            ["x", "y", "z", "intensity", "offset_time", "tag", "line"]
        );
        assert_eq!(
            f32::from_le_bytes(cloud.data[0..4].try_into().unwrap()),
            1.0
        );
        assert_eq!(
            u32::from_le_bytes(cloud.data[16..20].try_into().unwrap()),
            42
        );
        assert_eq!(cloud.data[20], 7);
        assert_eq!(cloud.data[21], 0);
    }

    #[test]
    fn minimal_cloud_packs_offset_at_12() {
        let cloud = cloud_message(PointFormat::Minimal, "lidar_link", &frame());
        assert_eq!(cloud.point_step, 16);
        let names: Vec<&str> = cloud.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["x", "y", "z", "offset_time"]);
        assert_eq!(
            u32::from_le_bytes(cloud.data[12..16].try_into().unwrap()),
            42
        );
    }

    #[test]
    fn legacy_cloud_is_xyzi() {
        let cloud = cloud_message(PointFormat::Legacy, "lidar_link", &frame());
        assert_eq!(cloud.point_step, 16);
        let names: Vec<&str> = cloud.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["x", "y", "z", "intensity"]);
        assert_eq!(
            f32::from_le_bytes(cloud.data[12..16].try_into().unwrap()),
            0.5
        );
    }

    #[test]
    fn imu_message_flags_unknown_orientation() {
        let record = ImuRecord {
            ts_ns: 5_500_000_000,
            gyro_rads: [0.1, 0.2, 0.3],
            acc_ms2: [0.0, 0.0, 9.8],
        };
        let msg = imu_message("imu_link", &record);
        assert_eq!(msg.header.stamp.sec, 5);
        assert_eq!(msg.header.stamp.nsec, 500_000_000);
        assert_eq!(msg.orientation_covariance[0], -1.0);
        assert_eq!(msg.orientation.w, 1.0);
        assert_eq!(msg.angular_velocity.z, 0.3);
        assert_eq!(msg.linear_acceleration.z, 9.8);
    }

    #[test]
    #[should_panic(expected = "point_format")]
    fn bad_point_format_panics() {
        PointFormat::parse("wat");
    }
}
