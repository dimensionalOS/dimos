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

//! The Go2 on the graph: `cmd_vel` and `command` in, `odometry` (with its tf edge), the
//! head L1 `lidar` cloud and the front `video` out. One thread owns DDS, one the
//! videohub RTP socket; the handlers only forward onto the DDS thread.

use std::net::{Ipv4Addr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use cyclonedds_rs::{DdsWriter, SampleBuffer};
use dimos_module::nalgebra::{Isometry3, Quaternion as NQuaternion, Translation3, UnitQuaternion};
use dimos_module::{native_config, Input, Module, Output, Tf, Transform};
use lcm_msgs::builtin_interfaces::Time as VideoTime;
use lcm_msgs::foxglove_msgs::CompressedVideo;
use lcm_msgs::geometry_msgs::{
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{PointCloud2, PointField};
use lcm_msgs::std_msgs::{self, Header, Time};
use tokio::runtime::Handle;
use tracing::{info, warn};

use crate::dds::{runtime, topics, types};
use crate::h264::{Depacketizer, RtpClock};
use crate::rtp;

/// The optical frame, which the image shares with `camera_info`.
const CAMERA_FRAME: &str = "camera_optical";
/// StandUp alone is the stiff joint lock; RecoveryStand after it hands over to balance.
const STANDUP_RECOVERY_DELAY: Duration = Duration::from_millis(500);
/// The Go2's request readers must discover our writers before the first write lands.
const DISCOVERY_SETTLE: Duration = Duration::from_millis(800);
const POLL: Duration = Duration::from_millis(2);
/// No RTP for this long after traffic: the IGMP membership is gone, rejoin.
const SILENT_REJOIN: Duration = Duration::from_secs(30);

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// The interface CycloneDDS binds: the Go2's own eth0, or the Jetson's Go2 link.
    pub iface: String,
    pub domain_id: u32,
    pub odom_topic: String,
    pub lidar_topic: String,
    /// Spin the head L1 up at start (park it otherwise) and stream its cloud.
    pub lidar_on: bool,
    /// Join the videohub RTP multicast and stream the front camera.
    pub video_on: bool,
    pub video_group: String,
    pub video_port: u16,
    /// Hard clamp on `cmd_vel`, m/s and rad/s.
    #[validate(range(min = 0.0))]
    pub max_vx: f64,
    #[validate(range(min = 0.0))]
    pub max_vy: f64,
    #[validate(range(min = 0.0))]
    pub max_vyaw: f64,
    /// StopMove once `cmd_vel` has been silent this long.
    #[validate(range(min = 1))]
    pub deadman_ms: u64,
}

/// What the handlers hand the DDS thread.
#[derive(Debug, PartialEq)]
pub enum Cmd {
    Move { vx: f64, vy: f64, vyaw: f64 },
    Sport(i64),
    Lidar(bool),
}

/// A `command` verb: a sport name, a bare api id, or the L1 switch.
pub fn parse_verb(verb: &str) -> Option<Cmd> {
    let v = verb.trim().to_ascii_lowercase();
    Some(match v.as_str() {
        "lidar on" | "lidar-on" => Cmd::Lidar(true),
        "lidar off" | "lidar-off" => Cmd::Lidar(false),
        _ => Cmd::Sport(topics::sport_id(&v).or_else(|| v.parse().ok())?),
    })
}

#[derive(Module)]
#[module(name = "go2_dds", setup = start, teardown = stop)]
pub struct Go2Dds {
    #[input(decode = Twist::decode, handler = on_cmd_vel)]
    cmd_vel: Input<Twist>,

    #[input(decode = std_msgs::String::decode, handler = on_command)]
    command: Input<std_msgs::String>,

    #[output(encode = Odometry::encode)]
    odometry: Output<Odometry>,

    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    #[output(encode = CompressedVideo::encode)]
    video: Output<CompressedVideo>,

    #[config]
    config: Config,

    #[tf]
    tf: Tf,

    cmd_tx: Option<Sender<Cmd>>,
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl Go2Dds {
    async fn start(&mut self) {
        let (tx, rx) = mpsc::channel();
        self.cmd_tx = Some(tx);
        let handle = Handle::current();
        let dds = DdsLoop {
            config: self.config.clone(),
            odometry: self.odometry.clone(),
            lidar: self.lidar.clone(),
            tf: self.tf.clone(),
            stop: self.stop.clone(),
            handle: handle.clone(),
        };
        self.threads.push(std::thread::spawn(move || dds.run(rx)));
        if self.config.video_on {
            let video = VideoLoop {
                group: self
                    .config
                    .video_group
                    .parse()
                    .expect("video_group is an IPv4 address"),
                port: self.config.video_port,
                video: self.video.clone(),
                stop: self.stop.clone(),
                handle,
            };
            self.threads.push(std::thread::spawn(move || video.run()));
        }
    }

    async fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.cmd_tx = None;
        // Joined off the runtime: a thread may be parked in a publish this worker drains.
        let threads = std::mem::take(&mut self.threads);
        let _ = tokio::task::spawn_blocking(move || {
            for t in threads {
                let _ = t.join();
            }
        })
        .await;
    }

    async fn on_cmd_vel(&mut self, msg: Twist) {
        self.send(Cmd::Move {
            vx: msg.linear.x,
            vy: msg.linear.y,
            vyaw: msg.angular.z,
        });
    }

    async fn on_command(&mut self, msg: std_msgs::String) {
        match parse_verb(&msg.data) {
            Some(cmd) => {
                info!(verb = %msg.data, ?cmd, "command");
                self.send(cmd);
            }
            None => warn!(verb = %msg.data, "unknown command verb"),
        }
    }

    fn send(&self, cmd: Cmd) {
        if let Some(tx) = &self.cmd_tx {
            let _ = tx.send(cmd);
        }
    }
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

fn now_secs() -> f64 {
    now_ns() as f64 * 1e-9
}

fn time_of_secs(ts: f64) -> Time {
    let sec = ts.trunc();
    Time {
        sec: sec as i32,
        nsec: ((ts - sec) * 1e9) as i32,
    }
}

fn request(w: &mut DdsWriter<types::Request>, api_id: i64, parameter: impl Into<String>) {
    if let Err(e) = w.write(Arc::new(types::Request::new(api_id, parameter))) {
        warn!(api_id, ?e, "sport request write failed");
    }
}

fn set_lidar(w: &mut DdsWriter<types::StdString>, on: bool) {
    let msg = Arc::new(types::StdString {
        data: if on { "ON" } else { "OFF" }.into(),
    });
    for _ in 0..5 {
        let _ = w.write(msg.clone());
        std::thread::sleep(Duration::from_millis(100));
    }
    info!(on, "head lidar switch");
}

/// The dimos odometry and the tf edge it implies. Stamped at receipt: the hop is on-LAN
/// and the robot's clock is not this host's.
pub fn odometry(s: &types::Odometry, ts: f64) -> (Odometry, Transform) {
    let p = &s.pose.pose.position;
    let q = &s.pose.pose.orientation;
    let (l, a) = (&s.twist.twist.linear, &s.twist.twist.angular);
    let odom = Odometry {
        header: Header {
            seq: 0,
            stamp: time_of_secs(ts),
            frame_id: s.header.frame_id.clone(),
        },
        child_frame_id: s.child_frame_id.clone(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: p.x,
                    y: p.y,
                    z: p.z,
                },
                orientation: Quaternion {
                    x: q.x,
                    y: q.y,
                    z: q.z,
                    w: q.w,
                },
            },
            covariance: s.pose.covariance,
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: l.x,
                    y: l.y,
                    z: l.z,
                },
                angular: Vector3 {
                    x: a.x,
                    y: a.y,
                    z: a.z,
                },
            },
            covariance: s.twist.covariance,
        },
    };
    let iso = Isometry3::from_parts(
        Translation3::new(p.x, p.y, p.z),
        UnitQuaternion::from_quaternion(NQuaternion::new(q.w, q.x, q.y, q.z)),
    );
    let edge = Transform::new(s.header.frame_id.clone(), s.child_frame_id.clone(), ts, iso);
    (odom, edge)
}

pub fn pointcloud(s: &types::PointCloud2, ts: f64) -> PointCloud2 {
    PointCloud2 {
        header: Header {
            seq: 0,
            stamp: time_of_secs(ts),
            frame_id: s.header.frame_id.clone(),
        },
        height: s.height as i32,
        width: s.width as i32,
        fields: s
            .fields
            .iter()
            .map(|f| PointField {
                name: f.name.clone(),
                offset: f.offset as i32,
                datatype: f.datatype,
                count: f.count as i32,
            })
            .collect(),
        is_bigendian: s.is_bigendian,
        point_step: s.point_step as i32,
        row_step: s.row_step as i32,
        data: s.data.clone(),
        is_dense: s.is_dense,
    }
}

struct DdsLoop {
    config: Config,
    odometry: Output<Odometry>,
    lidar: Output<PointCloud2>,
    tf: Tf,
    stop: Arc<AtomicBool>,
    handle: Handle,
}

impl DdsLoop {
    fn run(self, cmd_rx: Receiver<Cmd>) {
        let c = &self.config;
        let p = runtime::participant(&c.iface, c.domain_id);
        let odom_reader = runtime::make_reader::<types::Odometry>(&p, &c.odom_topic);
        let cloud_reader = c
            .lidar_on
            .then(|| runtime::make_reader::<types::PointCloud2>(&p, &c.lidar_topic));
        let mut sport = runtime::make_writer::<types::Request>(&p, topics::SPORT_REQUEST);
        let mut switch = runtime::make_writer::<types::StdString>(&p, topics::LIDAR_SWITCH);
        std::thread::sleep(DISCOVERY_SETTLE);
        set_lidar(&mut switch, c.lidar_on);
        info!(iface = %c.iface, odom = %c.odom_topic, lidar = %c.lidar_topic, "dds up");

        let mut odom_buf = SampleBuffer::<types::Odometry>::new(64);
        let mut cloud_buf = SampleBuffer::<types::PointCloud2>::new(8);
        let deadman = Duration::from_millis(c.deadman_ms);
        let mut last_twist = Instant::now();
        let mut pending_recovery: Option<Instant> = None;
        while !self.stop.load(Ordering::Relaxed) {
            while let Ok(cmd) = cmd_rx.try_recv() {
                match cmd {
                    Cmd::Move { vx, vy, vyaw } => {
                        if !(vx.is_finite() && vy.is_finite() && vyaw.is_finite()) {
                            continue;
                        }
                        last_twist = Instant::now();
                        let vx = vx.clamp(-c.max_vx, c.max_vx);
                        let vy = vy.clamp(-c.max_vy, c.max_vy);
                        let vyaw = vyaw.clamp(-c.max_vyaw, c.max_vyaw);
                        let param = format!("{{\"x\":{vx:.3},\"y\":{vy:.3},\"z\":{vyaw:.3}}}");
                        request(&mut sport, topics::sport::MOVE, param);
                    }
                    Cmd::Sport(id) => {
                        request(&mut sport, id, "{}");
                        if id == topics::sport::STAND_UP {
                            pending_recovery = Some(Instant::now() + STANDUP_RECOVERY_DELAY);
                        }
                    }
                    Cmd::Lidar(on) => set_lidar(&mut switch, on),
                }
            }
            if pending_recovery.is_some_and(|at| Instant::now() >= at) {
                pending_recovery = None;
                request(&mut sport, topics::sport::RECOVERY_STAND, "{}");
            }
            if last_twist.elapsed() > deadman {
                request(&mut sport, topics::sport::STOP_MOVE, "");
                last_twist = Instant::now();
            }

            let n = odom_reader.take_now(&mut odom_buf).unwrap_or(0);
            for s in odom_buf.iter().take(n) {
                let (odom, edge) = odometry(s, now_secs());
                let _ = self.handle.block_on(self.odometry.publish(&odom));
                let _ = self.handle.block_on(self.tf.publish(&[edge]));
            }
            if let Some(reader) = &cloud_reader {
                let n = reader.take_now(&mut cloud_buf).unwrap_or(0);
                for s in cloud_buf.iter().take(n) {
                    let _ = self
                        .handle
                        .block_on(self.lidar.publish(&pointcloud(s, now_secs())));
                }
            }
            std::thread::sleep(POLL);
        }
    }
}

struct VideoLoop {
    group: Ipv4Addr,
    port: u16,
    video: Output<CompressedVideo>,
    stop: Arc<AtomicBool>,
    handle: Handle,
}

impl VideoLoop {
    fn open(&self) -> std::io::Result<UdpSocket> {
        let sock = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, self.port))?;
        sock.join_multicast_v4(&self.group, &Ipv4Addr::UNSPECIFIED)?;
        sock.set_read_timeout(Some(Duration::from_millis(500)))?;
        Ok(sock)
    }

    /// Socket setup retries with backoff; a recv error or a silent stream rebuilds it.
    fn run(self) {
        let mut delay = Duration::from_secs(1);
        while !self.stop.load(Ordering::Relaxed) {
            let sock = match self.open() {
                Ok(s) => s,
                Err(e) => {
                    warn!(%e, group = %self.group, port = self.port, "video socket, retrying");
                    std::thread::sleep(delay);
                    delay = (delay * 2).min(Duration::from_secs(5));
                    continue;
                }
            };
            delay = Duration::from_secs(1);
            info!(group = %self.group, port = self.port, "joined videohub multicast");
            let mut buf = vec![0u8; 2048];
            let mut depack = Depacketizer::default();
            let mut clock = RtpClock::default();
            let mut last_rx: Option<Instant> = None;
            while !self.stop.load(Ordering::Relaxed) {
                let n = match sock.recv(&mut buf) {
                    Ok(n) => n,
                    Err(e)
                        if matches!(
                            e.kind(),
                            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                        ) =>
                    {
                        if last_rx.is_some_and(|t| t.elapsed() > SILENT_REJOIN) {
                            warn!("no RTP for {}s, rejoining", SILENT_REJOIN.as_secs());
                            break;
                        }
                        continue;
                    }
                    Err(e) => {
                        warn!(%e, "video recv, rebuilding socket");
                        break;
                    }
                };
                last_rx = Some(Instant::now());
                let Some(pkt) = rtp::parse(&buf[..n]) else {
                    continue;
                };
                let Some(au) = depack.push(&pkt) else {
                    continue;
                };
                let ts_ns = clock.to_ns(au.rtp_ts, now_ns());
                let msg = CompressedVideo {
                    timestamp: VideoTime {
                        sec: (ts_ns / 1_000_000_000) as i32,
                        nanosec: (ts_ns % 1_000_000_000) as i32,
                    },
                    frame_id: CAMERA_FRAME.to_string(),
                    data: au.data,
                    format: "h264".to_string(),
                };
                let _ = self.handle.block_on(self.video.publish(&msg));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verbs_resolve() {
        assert_eq!(
            parse_verb("stand-up"),
            Some(Cmd::Sport(topics::sport::STAND_UP))
        );
        assert_eq!(
            parse_verb(" Hello "),
            Some(Cmd::Sport(topics::sport::HELLO))
        );
        assert_eq!(parse_verb("1016"), Some(Cmd::Sport(1016)));
        assert_eq!(parse_verb("lidar off"), Some(Cmd::Lidar(false)));
        assert_eq!(parse_verb("moonwalk"), None);
    }

    #[test]
    fn odometry_keeps_frames_and_pose() {
        let mut s = types::Odometry::default();
        s.header.frame_id = "odom".into();
        s.child_frame_id = "base_link".into();
        s.pose.pose.position = types::Vector3 {
            x: 1.0,
            y: 2.0,
            z: 0.3,
        };
        s.pose.pose.orientation = types::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 1.0,
            w: 0.0,
        };
        let (odom, edge) = odometry(&s, 1234.5);
        assert_eq!(odom.header.frame_id, "odom");
        assert_eq!(odom.child_frame_id, "base_link");
        assert_eq!(
            odom.header.stamp,
            Time {
                sec: 1234,
                nsec: 500_000_000
            }
        );
        assert_eq!(
            (edge.parent.as_str(), edge.child.as_str()),
            ("odom", "base_link")
        );
        assert_eq!(
            edge.translation(),
            dimos_module::nalgebra::Vector3::new(1.0, 2.0, 0.3)
        );
        assert!((edge.rotation().angle() - std::f64::consts::PI).abs() < 1e-9);
    }
}
