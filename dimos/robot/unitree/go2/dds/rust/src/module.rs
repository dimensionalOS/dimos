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

//! The Go2 on the graph: `cmd_vel` and `command` in; out go `odometry` (with its tf
//! edge), the head L1 clouds, the front camera (H.264 on `video` or JPEG on `image`),
//! and the body's `joint_state`, `imu`, `battery` and remote `joy`. One thread owns
//! DDS, one the videohub RTP socket; the handlers only forward onto the DDS thread.

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
use lcm_msgs::sensor_msgs::{
    BatteryState, CompressedImage, Imu, JointState, Joy, PointCloud2, PointField,
};
use lcm_msgs::std_msgs::{self, Header, Time};
use tokio::runtime::Handle;
use tracing::{info, warn};

use crate::dds::{runtime, topics, types};
use crate::h264::{Depacketizer, RtpClock};
use crate::rtp;

/// The optical frame, which the H.264 stream shares with `camera_info`.
const CAMERA_FRAME: &str = "camera_optical";
/// The videohub JPEG reply names no frame.
const JPEG_FRAME: &str = "front_camera";
/// The body IMU and the legs report in the body frame.
const BODY_FRAME: &str = "base_link";
/// The first 12 of LowState's 20 motors, in Unitree order.
const JOINTS: [&str; 12] = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
];
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
    /// Spin the head L1 up at start (park it otherwise) and stream its deskewed cloud.
    pub lidar_on: bool,
    /// Also the undeskewed sensor-frame cloud and the L1's own IMU.
    pub lidar_raw_on: bool,
    /// Body joint states, IMU and battery off `rt/lowstate`, decimated to `lowstate_hz`.
    pub lowstate_on: bool,
    #[validate(range(exclusive_min = 0.0))]
    pub lowstate_hz: f64,
    /// Stream the front camera: "h264" off the videohub RTP multicast onto `video`,
    /// "jpeg" polled from the videohub RPC at `video_fps` onto `image`.
    pub video_on: bool,
    #[validate(custom(function = "known_encoding"))]
    pub video_encoding: String,
    #[validate(range(exclusive_min = 0.0))]
    pub video_fps: f64,
    pub video_group: String,
    pub video_port: u16,
    /// StopMove once `cmd_vel` has been silent this long.
    #[validate(range(min = 1))]
    pub deadman_ms: u64,
}

fn known_encoding(v: &str) -> Result<(), validator::ValidationError> {
    match v {
        "h264" | "jpeg" => Ok(()),
        _ => Err(validator::ValidationError::new(
            "video_encoding must be h264 or jpeg",
        )),
    }
}

/// What the handlers hand the DDS thread.
#[derive(Debug, PartialEq)]
pub enum Cmd {
    Move {
        vx: f64,
        vy: f64,
        vyaw: f64,
    },
    Request {
        topic: &'static str,
        api_id: i64,
        parameter: String,
    },
    Lidar(bool),
}

fn sport(api_id: i64, parameter: &str) -> Cmd {
    Cmd::Request {
        topic: topics::SPORT_REQUEST,
        api_id,
        parameter: parameter.into(),
    }
}

fn vui(api_id: i64, parameter: String) -> Cmd {
    Cmd::Request {
        topic: topics::VUI_REQUEST,
        api_id,
        parameter,
    }
}

fn on_off(arg: &str) -> Option<bool> {
    match arg {
        "on" => Some(true),
        "off" => Some(false),
        _ => None,
    }
}

/// The `command` vocabulary: `<verb>` or `<verb> <arg>`, case-insensitive.
pub fn parse_verb(verb: &str) -> Option<Vec<Cmd>> {
    let v = verb.trim().to_ascii_lowercase();
    let (name, arg) = v
        .split_once(' ')
        .map_or((v.as_str(), ""), |(n, a)| (n, a.trim()));
    let level = || {
        arg.parse::<i64>()
            .ok()
            .map(|l| l.clamp(0, topics::vui::LEVEL_MAX))
    };
    Some(match name {
        "lidar" => vec![Cmd::Lidar(on_off(arg)?)],
        "obstacle-avoidance" => vec![Cmd::Request {
            topic: topics::OBSTACLES_AVOID_REQUEST,
            api_id: topics::OBSTACLE_AVOID_ENABLE,
            parameter: format!("{{\"enable\":{}}}", on_off(arg)? as i64),
        }],
        "joystick" => vec![sport(
            topics::sport::SWITCH_JOYSTICK,
            &format!("{{\"data\":{}}}", on_off(arg)?),
        )],
        // Rage needs the joystick listener; off returns to normal locomotion.
        "rage" if on_off(arg)? => vec![
            sport(topics::sport::RAGE_MODE, "{\"data\":true}"),
            sport(topics::sport::SWITCH_JOYSTICK, "{\"data\":true}"),
        ],
        "rage" => vec![sport(topics::sport::BALANCE_STAND, "")],
        "brightness" => vec![vui(
            topics::vui::SET_BRIGHTNESS,
            format!("{{\"brightness\":{}}}", level()?),
        )],
        "volume" => vec![vui(
            topics::vui::SET_VOLUME,
            format!("{{\"volume\":{}}}", level()?),
        )],
        // Brightness rides along so the colour is visible: 0 for off, else max.
        "led" if !arg.is_empty() && arg.chars().all(|c| c.is_ascii_alphanumeric()) => {
            let level = if arg == "off" {
                0
            } else {
                topics::vui::LEVEL_MAX
            };
            vec![
                vui(
                    topics::vui::SET_BRIGHTNESS,
                    format!("{{\"brightness\":{level}}}"),
                ),
                vui(
                    topics::vui::SET_LED,
                    format!("{{\"color\":\"{arg}\",\"flash_cycle\":0,\"time\":0}}"),
                ),
            ]
        }
        _ if arg.is_empty() => {
            vec![sport(
                topics::sport_id(name).or_else(|| name.parse().ok())?,
                "{}",
            )]
        }
        _ => return None,
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

    #[output(encode = CompressedImage::encode)]
    image: Output<CompressedImage>,

    #[output(encode = PointCloud2::encode)]
    lidar_raw: Output<PointCloud2>,

    #[output(encode = Imu::encode)]
    lidar_imu: Output<Imu>,

    #[output(encode = JointState::encode)]
    joint_state: Output<JointState>,

    #[output(encode = Imu::encode)]
    imu: Output<Imu>,

    #[output(encode = BatteryState::encode)]
    battery: Output<BatteryState>,

    #[output(encode = Joy::encode)]
    joy: Output<Joy>,

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
            image: self.image.clone(),
            lidar_raw: self.lidar_raw.clone(),
            lidar_imu: self.lidar_imu.clone(),
            joint_state: self.joint_state.clone(),
            imu: self.imu.clone(),
            battery: self.battery.clone(),
            joy: self.joy.clone(),
            tf: self.tf.clone(),
            stop: self.stop.clone(),
            handle: handle.clone(),
        };
        self.threads.push(std::thread::spawn(move || dds.run(rx)));
        if self.config.video_on && self.config.video_encoding == "h264" {
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
            Some(cmds) => {
                info!(verb = %msg.data, ?cmds, "command");
                cmds.into_iter().for_each(|c| self.send(c));
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

/// The newest reply that is a JPEG (SOI marker); the videohub also answers other calls.
pub fn newest_jpeg<'a>(replies: impl Iterator<Item = &'a [u8]>) -> Option<&'a [u8]> {
    replies.filter(|b| b.starts_with(&[0xFF, 0xD8])).last()
}

/// Fires at most every `1 / fps` seconds.
pub struct Cadence {
    period: Duration,
    next: Instant,
}

impl Cadence {
    pub fn new(fps: f64, now: Instant) -> Self {
        Self {
            period: Duration::from_secs_f64(1.0 / fps),
            next: now,
        }
    }

    pub fn due(&mut self, now: Instant) -> bool {
        if now < self.next {
            return false;
        }
        self.next = now + self.period;
        true
    }
}

fn header(frame_id: &str, ts: f64) -> Header {
    Header {
        seq: 0,
        stamp: time_of_secs(ts),
        frame_id: frame_id.into(),
    }
}

pub fn joint_state(s: &types::LowState, ts: f64) -> JointState {
    let m = &s.motor_state[..JOINTS.len()];
    JointState {
        header: header(BODY_FRAME, ts),
        name: JOINTS.iter().map(|n| n.to_string()).collect(),
        position: m.iter().map(|j| j.q as f64).collect(),
        velocity: m.iter().map(|j| j.dq as f64).collect(),
        effort: m.iter().map(|j| j.tau_est as f64).collect(),
    }
}

/// Unitree's quaternion is `[w, x, y, z]`; covariances unknown (-1 on the diagonal).
pub fn imu(s: &types::IMUState, ts: f64) -> Imu {
    let unknown = [-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0];
    let [w, x, y, z] = s.quaternion;
    let v3 = |a: [f32; 3]| Vector3 {
        x: a[0] as f64,
        y: a[1] as f64,
        z: a[2] as f64,
    };
    Imu {
        header: header(BODY_FRAME, ts),
        orientation: Quaternion {
            x: x as f64,
            y: y as f64,
            z: z as f64,
            w: w as f64,
        },
        orientation_covariance: unknown,
        angular_velocity: v3(s.gyroscope),
        angular_velocity_covariance: unknown,
        linear_acceleration: v3(s.accelerometer),
        linear_acceleration_covariance: unknown,
    }
}

/// Pass-through of a ROS2 `sensor_msgs/Imu` sample (the L1's own IMU).
pub fn lidar_imu(s: &types::Imu, ts: f64) -> Imu {
    let q = &s.orientation;
    let v3 = |v: &types::Vector3| Vector3 {
        x: v.x,
        y: v.y,
        z: v.z,
    };
    Imu {
        header: header(&s.header.frame_id, ts),
        orientation: Quaternion {
            x: q.x,
            y: q.y,
            z: q.z,
            w: q.w,
        },
        orientation_covariance: s.orientation_covariance,
        angular_velocity: v3(&s.angular_velocity),
        angular_velocity_covariance: s.angular_velocity_covariance,
        linear_acceleration: v3(&s.linear_acceleration),
        linear_acceleration_covariance: s.linear_acceleration_covariance,
    }
}

/// SI units as ROS wants them; charge and capacities are not reported (NaN).
pub fn battery(s: &types::LowState, ts: f64) -> BatteryState {
    let b = &s.bms_state;
    let status = match b.current.signum() {
        1 => BatteryState::POWER_SUPPLY_STATUS_CHARGING,
        -1 => BatteryState::POWER_SUPPLY_STATUS_DISCHARGING,
        _ => BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING,
    };
    BatteryState {
        header: header(BODY_FRAME, ts),
        voltage: s.power_v,
        temperature: b.bq_ntc[0] as f32,
        current: b.current as f32 / 1000.0,
        charge: f32::NAN,
        capacity: f32::NAN,
        design_capacity: f32::NAN,
        percentage: b.soc as f32 / 100.0,
        power_supply_status: status as u8,
        power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN as u8,
        power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LION as u8,
        present: true,
        cell_voltage: b
            .cell_vol
            .iter()
            .filter(|&&v| v > 0)
            .map(|&v| v as f32 / 1000.0)
            .collect(),
        cell_temperature: Vec::new(),
        location: String::new(),
        serial_number: String::new(),
    }
}

/// Sticks as axes, the 16-bit key mask as 16 buttons, bit 0 first.
pub fn joy(s: &types::WirelessController, ts: f64) -> Joy {
    Joy {
        header: header(BODY_FRAME, ts),
        axes: vec![s.lx, s.ly, s.rx, s.ry],
        buttons: (0..16).map(|i| ((s.keys >> i) & 1) as i32).collect(),
    }
}

/// The dimos odometry and its tf edge, stamped at receipt (the robot's clock is not ours).
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
    image: Output<CompressedImage>,
    lidar_raw: Output<PointCloud2>,
    lidar_imu: Output<Imu>,
    joint_state: Output<JointState>,
    imu: Output<Imu>,
    battery: Output<BatteryState>,
    joy: Output<Joy>,
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
        let raw_readers = (c.lidar_on && c.lidar_raw_on).then(|| {
            (
                runtime::make_reader::<types::PointCloud2>(&p, topics::CLOUD),
                runtime::make_reader::<types::Imu>(&p, topics::IMU),
            )
        });
        let low_reader = c
            .lowstate_on
            .then(|| runtime::make_reader::<types::LowState>(&p, topics::LOWSTATE));
        let joy_reader =
            runtime::make_reader::<types::WirelessController>(&p, topics::WIRELESS_CONTROLLER);
        let mut sport = runtime::make_writer::<types::Request>(&p, topics::SPORT_REQUEST);
        let mut vui = runtime::make_writer::<types::Request>(&p, topics::VUI_REQUEST);
        let mut obstacle =
            runtime::make_writer::<types::Request>(&p, topics::OBSTACLES_AVOID_REQUEST);
        let mut switch = runtime::make_writer::<types::StdString>(&p, topics::LIDAR_SWITCH);
        let jpeg = c.video_on && c.video_encoding == "jpeg";
        let mut videohub = jpeg.then(|| {
            (
                runtime::make_writer::<types::Request>(&p, topics::VIDEOHUB_REQUEST),
                runtime::make_reader::<types::Response>(&p, topics::VIDEOHUB_RESPONSE),
            )
        });
        std::thread::sleep(DISCOVERY_SETTLE);
        set_lidar(&mut switch, c.lidar_on);
        info!(iface = %c.iface, odom = %c.odom_topic, lidar = %c.lidar_topic, "dds up");

        let mut odom_buf = SampleBuffer::<types::Odometry>::new(64);
        let mut cloud_buf = SampleBuffer::<types::PointCloud2>::new(8);
        let mut reply_buf = SampleBuffer::<types::Response>::new(8);
        let mut raw_buf = SampleBuffer::<types::PointCloud2>::new(8);
        let mut imu_buf = SampleBuffer::<types::Imu>::new(64);
        let mut low_buf = SampleBuffer::<types::LowState>::new(64);
        let mut joy_buf = SampleBuffer::<types::WirelessController>::new(16);
        let mut jpeg_poll = Cadence::new(c.video_fps, Instant::now());
        let mut low_tick = Cadence::new(c.lowstate_hz, Instant::now());
        let mut battery_tick = Cadence::new(1.0, Instant::now());
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
                        let param = format!("{{\"x\":{vx:.3},\"y\":{vy:.3},\"z\":{vyaw:.3}}}");
                        request(&mut sport, topics::sport::MOVE, param);
                    }
                    Cmd::Request {
                        topic,
                        api_id,
                        parameter,
                    } => {
                        let writer = match topic {
                            topics::VUI_REQUEST => &mut vui,
                            topics::OBSTACLES_AVOID_REQUEST => &mut obstacle,
                            _ => &mut sport,
                        };
                        request(writer, api_id, parameter);
                        if topic == topics::SPORT_REQUEST && api_id == topics::sport::STAND_UP {
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
            if let Some((cloud, imu_r)) = &raw_readers {
                let n = cloud.take_now(&mut raw_buf).unwrap_or(0);
                for s in raw_buf.iter().take(n) {
                    let _ = self
                        .handle
                        .block_on(self.lidar_raw.publish(&pointcloud(s, now_secs())));
                }
                let n = imu_r.take_now(&mut imu_buf).unwrap_or(0);
                for s in imu_buf.iter().take(n) {
                    let _ = self
                        .handle
                        .block_on(self.lidar_imu.publish(&lidar_imu(s, now_secs())));
                }
            }
            if let Some(reader) = &low_reader {
                // Drained every tick, published on the decimated cadence: newest wins.
                let n = reader.take_now(&mut low_buf).unwrap_or(0);
                if let Some(s) = low_buf.iter().take(n).last() {
                    let now = Instant::now();
                    if low_tick.due(now) {
                        let ts = now_secs();
                        let _ = self
                            .handle
                            .block_on(self.joint_state.publish(&joint_state(s, ts)));
                        let _ = self
                            .handle
                            .block_on(self.imu.publish(&imu(&s.imu_state, ts)));
                    }
                    if battery_tick.due(now) {
                        let _ = self
                            .handle
                            .block_on(self.battery.publish(&battery(s, now_secs())));
                    }
                }
            }
            let n = joy_reader.take_now(&mut joy_buf).unwrap_or(0);
            if let Some(s) = joy_buf.iter().take(n).last() {
                let _ = self.handle.block_on(self.joy.publish(&joy(s, now_secs())));
            }
            if let Some((req, resp)) = &mut videohub {
                if jpeg_poll.due(Instant::now()) {
                    request(req, topics::video::GET_IMAGE_SAMPLE, "");
                }
                let n = resp.take_now(&mut reply_buf).unwrap_or(0);
                let replies = reply_buf.iter().take(n).map(|r| r.binary.as_slice());
                if let Some(data) = newest_jpeg(replies) {
                    let msg = CompressedImage {
                        header: Header {
                            seq: 0,
                            stamp: time_of_secs(now_secs()),
                            frame_id: JPEG_FRAME.to_string(),
                        },
                        format: "jpeg".to_string(),
                        data: data.to_vec(),
                    };
                    let _ = self.handle.block_on(self.image.publish(&msg));
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
        let req = |topic, api_id, parameter: &str| Cmd::Request {
            topic,
            api_id,
            parameter: parameter.into(),
        };
        let s = topics::SPORT_REQUEST;
        assert_eq!(
            parse_verb("stand-up"),
            Some(vec![req(s, topics::sport::STAND_UP, "{}")])
        );
        assert_eq!(
            parse_verb(" Hello "),
            Some(vec![req(s, topics::sport::HELLO, "{}")])
        );
        assert_eq!(parse_verb("1016"), Some(vec![req(s, 1016, "{}")]));
        assert_eq!(parse_verb("lidar off"), Some(vec![Cmd::Lidar(false)]));
        assert_eq!(
            parse_verb("obstacle-avoidance on"),
            Some(vec![req(
                topics::OBSTACLES_AVOID_REQUEST,
                1001,
                "{\"enable\":1}"
            )])
        );
        assert_eq!(
            parse_verb("joystick off"),
            Some(vec![req(
                s,
                topics::sport::SWITCH_JOYSTICK,
                "{\"data\":false}"
            )])
        );
        assert_eq!(parse_verb("rage on").map(|v| v.len()), Some(2));
        assert_eq!(
            parse_verb("rage off"),
            Some(vec![req(s, topics::sport::BALANCE_STAND, "")])
        );
        assert_eq!(
            parse_verb("brightness 42"),
            Some(vec![req(
                topics::VUI_REQUEST,
                topics::vui::SET_BRIGHTNESS,
                "{\"brightness\":10}"
            )])
        );
        assert_eq!(
            parse_verb("led red"),
            Some(vec![
                req(
                    topics::VUI_REQUEST,
                    topics::vui::SET_BRIGHTNESS,
                    "{\"brightness\":10}"
                ),
                req(
                    topics::VUI_REQUEST,
                    topics::vui::SET_LED,
                    "{\"color\":\"red\",\"flash_cycle\":0,\"time\":0}"
                ),
            ])
        );
        for bad in [
            "moonwalk",
            "lidar maybe",
            "brightness high",
            "led \"x",
            "sit now",
        ] {
            assert_eq!(parse_verb(bad), None, "{bad}");
        }
    }

    #[test]
    fn lowstate_converts() {
        let mut s = types::LowState::default();
        s.motor_state[2].q = 1.5;
        s.motor_state[2].dq = -0.5;
        s.motor_state[2].tau_est = 2.0;
        s.imu_state.quaternion = [0.0, 0.0, 0.0, 1.0];
        s.imu_state.gyroscope = [0.1, 0.2, 0.3];
        s.bms_state.soc = 80;
        s.bms_state.current = -1500;
        s.bms_state.cell_vol[0] = 4100;
        s.power_v = 28.5;
        let js = joint_state(&s, 1.0);
        assert_eq!(js.name.len(), 12);
        assert_eq!(js.name[2], "FR_calf_joint");
        assert_eq!(
            (js.position[2], js.velocity[2], js.effort[2]),
            (1.5, -0.5, 2.0)
        );
        let i = imu(&s.imu_state, 1.0);
        assert_eq!((i.orientation.w, i.orientation.z), (0.0, 1.0));
        assert!((i.angular_velocity.y - 0.2).abs() < 1e-6);
        let b = battery(&s, 1.0);
        assert_eq!((b.percentage, b.voltage, b.current), (0.8, 28.5, -1.5));
        assert_eq!(b.cell_voltage, vec![4.1]);
        assert_eq!(
            b.power_supply_status,
            BatteryState::POWER_SUPPLY_STATUS_DISCHARGING as u8
        );
        let remote = types::WirelessController {
            lx: 0.5,
            ly: 0.0,
            rx: 0.0,
            ry: -1.0,
            keys: 0b101,
        };
        let j = joy(&remote, 1.0);
        assert_eq!(j.axes, vec![0.5, 0.0, 0.0, -1.0]);
        assert_eq!(&j.buttons[..3], &[1, 0, 1]);
    }

    #[test]
    fn jpeg_replies_are_filtered_and_paced() {
        let frames: [&[u8]; 3] = [&[0xFF, 0xD8, 1], b"not jpeg", &[0xFF, 0xD8, 2]];
        assert_eq!(newest_jpeg(frames.into_iter()), Some(&[0xFF, 0xD8, 2][..]));
        assert_eq!(newest_jpeg([b"nope".as_slice()].into_iter()), None);
        let t0 = Instant::now();
        let mut c = Cadence::new(15.0, t0);
        assert!(c.due(t0));
        assert!(!c.due(t0 + Duration::from_millis(50)));
        assert!(c.due(t0 + Duration::from_millis(67)));
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
