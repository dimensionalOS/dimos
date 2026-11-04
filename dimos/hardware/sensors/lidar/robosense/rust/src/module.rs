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

//! RoboSense native module: UDP (or pcap) packets -> robosense-lidar decoder -> PointCloud2/Imu.

use dimos_module::{native_config, Module, Output};
use lcm_msgs::geometry_msgs::{Quaternion, Vector3};
use lcm_msgs::sensor_msgs::{Imu, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use robosense_lidar::pcap::{self, UdpPacket};
use robosense_lidar::{
    Decoder, DecoderConfig, ImuSample, Model, Output as Decoded, PacketKind, Receiver,
    ReceiverConfig, Scan,
};
use serde::{Deserialize, Serialize};
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use tokio::runtime::Handle;

/// Python's `None`, sent as a JSON null under a key that is always present.
/// The hand-written deserializer keeps a missing key an error, where anything
/// `Option`-shaped would silently read it as None.
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
    /// airy, helios, bpearl, rs32 or e1
    #[validate(custom(function = known_model))]
    model: String,
    /// Keep only packets from this sensor address. Null accepts any sender.
    lidar_ip: Nullable<String>,
    /// Local interface (IPv4 address or name) to join the multicast group on, or to bind
    /// unicast sockets to. Null lets the kernel pick.
    interface: Nullable<String>,
    /// Multicast group the sensor streams to. Null receives unicast.
    multicast_ip: Nullable<String>,
    msop_port: u16,
    difop_port: u16,
    /// Null skips the IMU stream.
    imu_port: Nullable<u16>,
    min_distance: Nullable<f32>,
    max_distance: Nullable<f32>,
    /// Stamp with the sensor clock instead of the arrival time on this host.
    use_lidar_clock: bool,
    frame_id: String,
    imu_frame_id: String,
    /// Replay this capture instead of a live sensor.
    pcap: Nullable<String>,
    /// Replay speed relative to capture time. Null runs flat-out.
    #[validate(custom(function = positive_replay_rate))]
    replay_rate: Nullable<f64>,
    /// Seconds a replay holds its first packet, so consumers can subscribe first.
    #[validate(range(min = 0.0))]
    replay_delay: f64,
}

fn known_model(model: &str) -> Result<(), validator::ValidationError> {
    model.parse::<Model>().map(|_| ()).map_err(|_| {
        validator::ValidationError::new("model must be one of airy, helios, bpearl, rs32, e1")
    })
}

fn positive_replay_rate(rate: &Nullable<f64>) -> Result<(), validator::ValidationError> {
    match rate.0 {
        Some(r) if r <= 0.0 => Err(validator::ValidationError::new(
            "replay_rate must be > 0 (or null for flat-out replay)",
        )),
        _ => Ok(()),
    }
}

impl Config {
    fn model(&self) -> Model {
        self.model.parse().expect("model was validated")
    }

    fn decoder_config(&self) -> DecoderConfig {
        DecoderConfig {
            min_distance: self.min_distance.0,
            max_distance: self.max_distance.0,
            use_lidar_clock: self.use_lidar_clock,
            ..Default::default()
        }
    }

    fn receiver_config(&self) -> ReceiverConfig {
        let ip = |v: &Nullable<String>, what: &str| {
            v.0.as_deref().map(|s| {
                s.parse::<Ipv4Addr>()
                    .unwrap_or_else(|e| panic!("invalid {what} {s:?}: {e}"))
            })
        };
        ReceiverConfig {
            msop_port: self.msop_port,
            difop_port: self.difop_port,
            // only models with an IMU stream (Airy) listen for one
            imu_port: self.imu_port.0.filter(|_| self.model().has_imu_packets()),
            multicast_group: ip(&self.multicast_ip, "multicast_ip"),
            interface: self.interface.0.clone(),
            lidar_ip: ip(&self.lidar_ip, "lidar_ip"),
            ..ReceiverConfig::for_model(self.model())
        }
    }
}

#[derive(Module)]
#[module(name = "robosense_lidar", setup = start, handle = wait, teardown = stop)]
pub struct RoboSenseLidar {
    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = Imu::encode)]
    imu: Output<Imu>,

    #[config]
    config: Config,

    stop: Arc<AtomicBool>,
    failed: Arc<tokio::sync::Notify>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl RoboSenseLidar {
    async fn start(&mut self) {
        let source = Source::open(&self.config).unwrap_or_else(|err| panic!("{err}"));
        let handle = Handle::current();
        let lidar = self.lidar.clone();
        let imu = self.imu.clone();
        let config = self.config.clone();
        let stop = self.stop.clone();
        let failed = self.failed.clone();
        self.threads.push(std::thread::spawn(move || {
            run_pipeline(source, &config, &handle, &lidar, &imu, &stop, &failed);
        }));
    }

    /// Ends the module when the packet source dies, so the framework runs
    /// every teardown instead of the pipeline killing the process.
    async fn wait(&mut self) {
        self.failed.notified().await;
    }

    async fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        // Joined off the runtime: the pipeline thread may be parked in a
        // publish that needs this worker to drain.
        let threads = std::mem::take(&mut self.threads);
        let _ = tokio::task::spawn_blocking(move || {
            for t in threads {
                let _ = t.join();
            }
        })
        .await;
    }
}

fn now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

struct Packet {
    kind: PacketKind,
    payload: Vec<u8>,
    arrival: f64,
}

enum Source {
    Live(Receiver),
    Replay {
        packets: std::vec::IntoIter<(PacketKind, UdpPacket)>,
        rate: Option<f64>,
        delay: f64,
        /// wall clock and capture time of the first packet
        origin: Option<(Instant, f64)>,
    },
}

impl Source {
    fn open(config: &Config) -> Result<Source, String> {
        let rx = config.receiver_config();
        let Nullable(Some(path)) = &config.pcap else {
            tracing::info!(model = %config.model, ?rx, "listening");
            return Receiver::open(&rx)
                .map(Source::Live)
                .map_err(|e| format!("failed to start live capture: {e}"));
        };
        let all = pcap::read_udp(path).map_err(|e| format!("failed to open pcap '{path}': {e}"))?;
        let kind_of = |p: &UdpPacket| {
            let ours = rx.multicast_group.map(|g| g == p.dst).unwrap_or(true)
                && rx.lidar_ip.map(|ip| ip == p.src).unwrap_or(true);
            match p.dst_port {
                _ if !ours => None,
                port if port == rx.msop_port => Some(PacketKind::Msop),
                port if port == rx.difop_port => Some(PacketKind::Difop),
                port if Some(port) == rx.imu_port => Some(PacketKind::Imu),
                _ => None,
            }
        };
        let packets: Vec<_> = all
            .into_iter()
            .filter_map(|p| kind_of(&p).map(|k| (k, p)))
            .collect();
        tracing::info!(pcap = %path, packets = packets.len(), rate = ?config.replay_rate.0, "replaying capture");
        Ok(Source::Replay {
            packets: packets.into_iter(),
            rate: config.replay_rate.0,
            delay: config.replay_delay,
            origin: None,
        })
    }

    /// The next packet, `Ok(None)` when a replay ends or `stop` is set.
    fn next(&mut self, stop: &AtomicBool) -> Result<Option<Packet>, String> {
        match self {
            Source::Live(rx) => {
                while !stop.load(Ordering::Relaxed) {
                    match rx.recv_timeout(Duration::from_millis(200)) {
                        Ok(Some(d)) => {
                            return Ok(Some(Packet {
                                kind: d.kind,
                                payload: d.payload,
                                arrival: d.arrival,
                            }))
                        }
                        Ok(None) => continue,
                        Err(e) => return Err(format!("receive failed: {e}")),
                    }
                }
                Ok(None)
            }
            Source::Replay {
                packets,
                rate,
                delay,
                origin,
            } => {
                let Some((kind, p)) = packets.next() else {
                    return Ok(None);
                };
                let (wall0, capture0) = *origin.get_or_insert_with(|| {
                    sleep_unless_stopped(Duration::from_secs_f64(*delay), stop);
                    (Instant::now(), p.timestamp)
                });
                if let Some(rate) = rate {
                    let due = wall0
                        + Duration::from_secs_f64(((p.timestamp - capture0) / *rate).max(0.0));
                    sleep_unless_stopped(due.saturating_duration_since(Instant::now()), stop);
                }
                if stop.load(Ordering::Relaxed) {
                    return Ok(None);
                }
                // replays are stamped as if they arrived now, like a live sensor
                Ok(Some(Packet {
                    kind,
                    payload: p.payload,
                    arrival: now(),
                }))
            }
        }
    }
}

fn sleep_unless_stopped(mut left: Duration, stop: &AtomicBool) {
    while !left.is_zero() && !stop.load(Ordering::Relaxed) {
        let step = left.min(Duration::from_millis(100));
        std::thread::sleep(step);
        left -= step;
    }
}

/// Scan/IMU rates, logged every few seconds so a headless run shows it is healthy.
struct RateLog {
    since: Instant,
    scans: u64,
    points: u64,
    imu: u64,
}

impl RateLog {
    const PERIOD: Duration = Duration::from_secs(5);

    fn new() -> Self {
        RateLog {
            since: Instant::now(),
            scans: 0,
            points: 0,
            imu: 0,
        }
    }

    fn maybe_log(&mut self, decoder: &Decoder) {
        let elapsed = self.since.elapsed();
        if elapsed < Self::PERIOD {
            return;
        }
        let secs = elapsed.as_secs_f64();
        let stats = decoder.stats();
        tracing::info!(
            scans_per_s = format!("{:.2}", self.scans as f64 / secs),
            points_per_scan = self.points.checked_div(self.scans).unwrap_or(0),
            imu_hz = format!("{:.1}", self.imu as f64 / secs),
            lost = stats.lost,
            malformed = stats.malformed,
            waiting_for_calibration = stats.waiting_for_calibration,
            "robosense rates"
        );
        *self = RateLog::new();
    }
}

fn run_pipeline(
    mut source: Source,
    config: &Config,
    handle: &Handle,
    lidar: &Output<PointCloud2>,
    imu: &Output<Imu>,
    stop: &AtomicBool,
    failed: &tokio::sync::Notify,
) {
    let mut decoder = Decoder::new(config.model(), config.decoder_config());
    let mut rates = RateLog::new();
    loop {
        let packet = match source.next(stop) {
            Ok(Some(p)) => p,
            Ok(None) => break,
            Err(reason) => {
                tracing::error!("packet source failed: {reason}");
                failed.notify_one();
                return;
            }
        };
        let decoded = match packet.kind {
            PacketKind::Msop => decoder
                .push_msop(&packet.payload, packet.arrival)
                .map(Decoded::Scan),
            PacketKind::Difop => decoder.push_difop(&packet.payload, packet.arrival),
            PacketKind::Imu => decoder
                .push_imu(&packet.payload, packet.arrival)
                .map(Decoded::Imu),
        };
        match decoded {
            // the first scan after start began mid-rotation
            Some(Decoded::Scan(scan)) if scan.partial => {}
            Some(Decoded::Scan(scan)) => {
                rates.scans += 1;
                rates.points += scan.points.len() as u64;
                let _ = handle.block_on(lidar.publish(&cloud_message(&config.frame_id, &scan)));
            }
            Some(Decoded::Imu(sample)) => {
                rates.imu += 1;
                let _ = handle.block_on(imu.publish(&imu_message(&config.imu_frame_id, &sample)));
            }
            Some(Decoded::DeviceInfo(info)) => tracing::info!(%info, "lidar device info"),
            None => {}
        }
        rates.maybe_log(&decoder);
    }
    tracing::info!("packet source ended");
}

fn header(frame_id: &str, secs: f64) -> Header {
    let sec = secs.floor();
    Header {
        seq: 0,
        stamp: Time {
            sec: sec as i32,
            nsec: (((secs - sec) * 1e9) as i32).min(999_999_999),
        },
        frame_id: frame_id.to_string(),
    }
}

fn imu_message(frame_id: &str, sample: &ImuSample) -> Imu {
    let vector = |v: [f64; 3]| Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    };
    let mut orientation_covariance = [0.0; 9];
    // Orientation unknown: identity with -1 covariance to flag it.
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
        angular_velocity: vector(sample.angular_velocity),
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: vector(sample.linear_acceleration),
        linear_acceleration_covariance: [0.0; 9],
    }
}

// One packed point: x y z intensity (f32), time (f32 seconds after the header stamp),
// ring (u16), 2 bytes of padding.
const OFFSET_X: usize = 0;
const OFFSET_Y: usize = 4;
const OFFSET_Z: usize = 8;
const OFFSET_INTENSITY: usize = 12;
const OFFSET_TIME: usize = 16;
const OFFSET_RING: usize = 20;
const POINT_STEP: usize = 24;

fn cloud_fields() -> Vec<PointField> {
    let field = |name: &str, offset: usize, datatype: i32| PointField {
        name: name.into(),
        offset: offset as i32,
        datatype: datatype as u8,
        count: 1,
    };
    vec![
        field("x", OFFSET_X, PointField::FLOAT32 as i32),
        field("y", OFFSET_Y, PointField::FLOAT32 as i32),
        field("z", OFFSET_Z, PointField::FLOAT32 as i32),
        field("intensity", OFFSET_INTENSITY, PointField::FLOAT32 as i32),
        field("time", OFFSET_TIME, PointField::FLOAT32 as i32),
        field("ring", OFFSET_RING, PointField::UINT16 as i32),
    ]
}

fn cloud_message(frame_id: &str, scan: &Scan) -> PointCloud2 {
    let mut data = vec![0u8; POINT_STEP * scan.points.len()];
    for (i, p) in scan.points.iter().enumerate() {
        let out = &mut data[i * POINT_STEP..(i + 1) * POINT_STEP];
        out[OFFSET_X..OFFSET_X + 4].copy_from_slice(&p.x.to_le_bytes());
        out[OFFSET_Y..OFFSET_Y + 4].copy_from_slice(&p.y.to_le_bytes());
        out[OFFSET_Z..OFFSET_Z + 4].copy_from_slice(&p.z.to_le_bytes());
        out[OFFSET_INTENSITY..OFFSET_INTENSITY + 4]
            .copy_from_slice(&(p.intensity as f32).to_le_bytes());
        let time = (p.timestamp - scan.timestamp) as f32;
        out[OFFSET_TIME..OFFSET_TIME + 4].copy_from_slice(&time.to_le_bytes());
        out[OFFSET_RING..OFFSET_RING + 2].copy_from_slice(&p.ring.to_le_bytes());
    }
    let width = scan.points.len() as i32;
    PointCloud2 {
        header: header(frame_id, scan.timestamp),
        height: 1,
        width,
        fields: cloud_fields(),
        is_bigendian: false,
        point_step: POINT_STEP as i32,
        row_step: POINT_STEP as i32 * width,
        data,
        is_dense: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robosense_lidar::Point;

    fn scan() -> Scan {
        let point = |x: f32, intensity: u8, ring: u16, dt: f64| Point {
            x,
            y: 2.0,
            z: 3.0,
            intensity,
            ring,
            timestamp: 1_700_000_000.5 + dt,
        };
        Scan {
            seq: 4,
            timestamp: 1_700_000_000.5,
            points: vec![point(1.0, 16, 7, 0.0), point(-1.0, 255, 95, 0.0625)],
            packets: 1,
            partial: false,
        }
    }

    #[test]
    fn cloud_layout_is_xyz_intensity_time_ring() {
        let cloud = cloud_message("lidar_link", &scan());
        assert_eq!((cloud.point_step, cloud.width, cloud.row_step), (24, 2, 48));
        assert_eq!(cloud.header.frame_id, "lidar_link");
        assert_eq!(
            (cloud.header.stamp.sec, cloud.header.stamp.nsec),
            (1_700_000_000, 500_000_000)
        );
        let names: Vec<&str> = cloud.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["x", "y", "z", "intensity", "time", "ring"]);
        let f32_at = |o: usize| f32::from_le_bytes(cloud.data[o..o + 4].try_into().unwrap());
        assert_eq!(f32_at(24), -1.0);
        assert_eq!(f32_at(24 + OFFSET_INTENSITY), 255.0);
        assert_eq!(f32_at(24 + OFFSET_TIME), 0.0625);
        assert_eq!(
            u16::from_le_bytes([cloud.data[24 + 20], cloud.data[24 + 21]]),
            95
        );
    }

    #[test]
    fn imu_message_flags_unknown_orientation() {
        let sample = ImuSample {
            timestamp: 5.5,
            linear_acceleration: [0.0, 0.0, 9.8],
            angular_velocity: [0.1, 0.2, 0.3],
            counter: None,
        };
        let msg = imu_message("lidar_link", &sample);
        assert_eq!(
            (msg.header.stamp.sec, msg.header.stamp.nsec),
            (5, 500_000_000)
        );
        assert_eq!(msg.orientation_covariance[0], -1.0);
        assert_eq!(msg.angular_velocity.z, 0.3);
        assert_eq!(msg.linear_acceleration.z, 9.8);
    }

    fn config_json() -> serde_json::Value {
        serde_json::json!({
            "model": "airy",
            "lidar_ip": null,
            "interface": null,
            "multicast_ip": "224.10.10.201",
            "msop_port": 6691,
            "difop_port": 7781,
            "imu_port": 6681,
            "min_distance": 0.2,
            "max_distance": null,
            "use_lidar_clock": false,
            "frame_id": "lidar_link",
            "imu_frame_id": "lidar_link",
            "pcap": null,
            "replay_rate": 1.0,
            "replay_delay": 0.0
        })
    }

    #[test]
    fn config_accepts_null_but_not_missing_nullable_keys() {
        let full = config_json();
        let config: Config = serde_json::from_value(full.clone()).unwrap();
        let rx = config.receiver_config();
        assert_eq!(rx.multicast_group, Some(Ipv4Addr::new(224, 10, 10, 201)));
        assert_eq!(
            (rx.msop_port, rx.difop_port, rx.imu_port),
            (6691, 7781, Some(6681))
        );
        assert_eq!(config.decoder_config().min_distance, Some(0.2));
        let mut missing = full;
        missing.as_object_mut().unwrap().remove("interface");
        assert!(serde_json::from_value::<Config>(missing).is_err());
    }

    #[test]
    fn unknown_model_and_zero_replay_rate_are_rejected() {
        use validator::Validate;
        let ok: Config = serde_json::from_value(config_json()).unwrap();
        ok.validate().expect("valid config");
        for (key, value) in [
            ("model", serde_json::json!("rs128")),
            ("replay_rate", 0.0.into()),
        ] {
            let mut bad = config_json();
            bad[key] = value;
            let config: Config = serde_json::from_value(bad).unwrap();
            assert!(config.validate().is_err(), "{key}");
        }
    }
}
