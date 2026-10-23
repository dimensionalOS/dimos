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

//! GO2Connection's real-robot side: the Go2 over its WebRTC link. `cmd_vel` and `command` (one verb per message,
//! see `parse_verb`) in; `odom` (+ its tf chain), `lidar`, the decoded front camera on `color_image`, `camera_info`,
//! and `battery` out. Verbs run one at a time in order, fire-and-forget as on GO2DDS (a failure is logged); `stop`,
//! `damp` and teardown cancel the one running and the ones waiting. Poses are stamped with the Go2's own stamp moved onto the host clock
//! (clock.rs); frames with their first packet's arrival, de-jittered by their RTP tick spacing (CaptureClock), and
//! scans with their arrival, since the Go2 sends no usable capture time for either.
//! One thread decodes video, one the data channel; tokio runs the session and the requests.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::geometry_msgs::{
    Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Twist, Vector3,
};
use lcm_msgs::sensor_msgs::{BatteryState, CameraInfo, Image, PointCloud2, PointField};
use lcm_msgs::std_msgs::{self, Header, Time};
use lcm_msgs::tf2_msgs::TFMessage;
use serde_json::{json, Value};
use tokio::runtime::Handle;
use tokio::sync::{mpsc as tokio_mpsc, Notify};
use tracing::{error, info, warn};

use crate::clock::{monotonic_ns, now_ns, place, CaptureClock, HostClock};
use crate::conn::{Conn, Event};
use crate::h264::{Depacketizer, Packet, Reorder};
use crate::lidar::decode_voxels;
use crate::timing::{secs, Timing};

const SPORT: &str = "rt/api/sport/request";
const MOTION_SWITCHER: &str = "rt/api/motion_switcher/request";
const VUI: &str = "rt/api/vui/request";
const OBSTACLES_AVOID: &str = "rt/api/obstacles_avoid/request";
const LIDAR_SWITCH: &str = "rt/utlidar/switch";
const WIRELESS_CONTROLLER: &str = "rt/wirelesscontroller";
const LIDAR_TOPIC: &str = "rt/utlidar/voxel_map_compressed";
const POSE_TOPIC: &str = "rt/utlidar/robot_pose";
const LOWSTATE_TOPIC: &str = "rt/lf/lowstate";

const DAMP: i64 = 1001;
const BALANCE_STAND: i64 = 1002;
const STAND_UP: i64 = 1004;
const STAND_DOWN: i64 = 1005;
const MOVE: i64 = 1008;
const SWITCH_JOYSTICK: i64 = 1027;
const RAGE_MODE: i64 = 2059;
/// The motion switcher's CheckMode / SelectMode.
const CHECK_MODE: i64 = 1001;
const SELECT_MODE: i64 = 1002;
const VUI_SET_VOLUME: i64 = 1003;
const VUI_SET_BRIGHTNESS: i64 = 1005;
const VUI_SET_LED: i64 = 1007;
const VUI_LEVEL_MAX: i64 = 10;
const OBSTACLES_AVOID_SWITCH_SET: i64 = 1001;

#[native_config]
#[derive(Clone)]
pub struct Config {
    pub ip: String,
    /// Per-device AES-128 key (Go2 firmware >= 1.1.15), "" for older firmware.
    pub aes_128_key: Secret,
    /// "default" or "rage" (Rage Mode switched on at start).
    pub mode: String,
    pub lidar: bool,
    pub camera: bool,
    /// Drive with sport Move (1008) instead of the wireless-controller stick emulation.
    pub velocity_api: bool,
    /// "mcf" (stairs), "normal", or "" to leave the motion controller as it is.
    pub motion_mode: String,
    /// The robot's built-in obstacle avoidance, applied at start.
    pub obstacle_avoidance: bool,
    /// TF parent of the odometry (odom_frame_id -> base_link).
    pub odom_frame_id: String,
    pub publish_tf: bool,
    /// Namespace for this robot's frames ("" for none).
    pub frame_id_prefix: String,
    /// Reconnect when the Go2 drops the link (or refuses the first connect) instead of exiting.
    pub auto_reconnect: bool,
    #[serde(with = "camera_info_dict")]
    pub camera_info: CameraInfo,
    /// base_link -> camera_link translation (m)
    pub camera_offset: [f64; 3],
    pub timing: Timing,
}

/// A config string kept out of logs: the module prints its config at start.
#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(transparent)]
pub struct Secret(pub String);

impl std::fmt::Debug for Secret {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_empty() {
            "\"\""
        } else {
            "<redacted>"
        })
    }
}

/// python's `CameraInfo` config dict <-> the message; the stamp is set per publish.
mod camera_info_dict {
    use lcm_msgs::sensor_msgs::CameraInfo;
    use lcm_msgs::std_msgs::Header;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    #[derive(Serialize, Deserialize)]
    #[allow(non_snake_case)]
    struct Dict {
        frame_id: String,
        height: i32,
        width: i32,
        distortion_model: String,
        D: Vec<f64>,
        K: [f64; 9],
        R: [f64; 9],
        P: [f64; 12],
        #[serde(default)]
        binning_x: i32,
        #[serde(default)]
        binning_y: i32,
    }

    pub fn deserialize<'de, De: Deserializer<'de>>(d: De) -> Result<CameraInfo, De::Error> {
        let c = Dict::deserialize(d)?;
        Ok(CameraInfo {
            header: Header {
                frame_id: c.frame_id,
                ..Default::default()
            },
            height: c.height,
            width: c.width,
            distortion_model: c.distortion_model,
            D: c.D,
            K: c.K,
            R: c.R,
            P: c.P,
            binning_x: c.binning_x,
            binning_y: c.binning_y,
            ..Default::default()
        })
    }

    pub fn serialize<S: Serializer>(c: &CameraInfo, s: S) -> Result<S::Ok, S::Error> {
        Dict {
            frame_id: c.header.frame_id.clone(),
            height: c.height,
            width: c.width,
            distortion_model: c.distortion_model.clone(),
            D: c.D.clone(),
            K: c.K,
            R: c.R,
            P: c.P,
            binning_x: c.binning_x,
            binning_y: c.binning_y,
        }
        .serialize(s)
    }
}

fn prefixed(prefix: &str, name: &str) -> String {
    if prefix.is_empty() {
        name.to_string()
    } else {
        format!("{prefix}/{name}")
    }
}

fn time_of_ns(ns: u64) -> Time {
    Time {
        sec: (ns / 1_000_000_000) as i32,
        nsec: (ns % 1_000_000_000) as i32,
    }
}

fn header(ns: u64, frame_id: &str) -> Header {
    Header {
        seq: 0,
        stamp: time_of_ns(ns),
        frame_id: frame_id.to_string(),
    }
}

fn f(value: &Value) -> f64 {
    value.as_f64().unwrap_or(0.0)
}

/// The Go2's time for a pose (`header.stamp` sec/nanosec), ns on its clock.
fn pose_robot_ns(data: &Value) -> Option<i64> {
    let stamp = &data["header"]["stamp"];
    Some(stamp["sec"].as_i64()? * 1_000_000_000 + stamp["nanosec"].as_i64().unwrap_or(0))
}

/// What the session shares with the request handlers and the threads.
struct Shared {
    config: Config,
    handle: Handle,
    conn: Mutex<Option<Arc<Conn>>>,
    /// the last cmd_vel's arrival (monotonic_ns) while the base is being driven, 0 once stopped
    driving_since: AtomicU64,
    move_ids: AtomicU64,
    stopping: AtomicBool,
    lost: Notify,
    /// bumped by stop, damp and teardown: a verb started under an older value gives up at its next step
    cancels: AtomicU64,
    /// woken on every cancel, so the verb running drops out mid-request or mid-settle
    cancel_wake: Notify,
    /// held by the verb running and by the start sequence: one thing drives the robot at a time
    turn: tokio::sync::Mutex<()>,
    session: Mutex<Option<tokio::task::JoinHandle<()>>>,
}

impl Shared {
    fn conn(&self) -> Result<Arc<Conn>, String> {
        self.conn
            .lock()
            .unwrap()
            .clone()
            .filter(|c| !c.is_closed())
            .ok_or_else(|| "the Go2 isn't connected".into())
    }

    async fn request(
        &self,
        topic: &str,
        api_id: i64,
        parameter: Option<Value>,
    ) -> Result<Value, String> {
        self.conn()?
            .request(
                topic,
                api_id,
                parameter.as_ref(),
                secs(self.config.timing.answer_timeout),
            )
            .await
    }

    async fn sport(&self, api_id: i64, parameter: Option<Value>) -> Result<Value, String> {
        self.request(SPORT, api_id, parameter).await
    }

    /// One velocity command: the stick emulation, or sport Move with velocity_api.
    async fn drive(&self, x: f64, y: f64, yaw: f64) -> Result<(), String> {
        let conn = self.conn()?;
        if self.config.velocity_api {
            let id = self.move_ids.fetch_add(1, Ordering::Relaxed) + 1;
            let payload = json!({
                "header": { "identity": { "id": id, "api_id": MOVE } },
                "parameter": json!({ "x": x, "y": y, "z": yaw }).to_string(),
            });
            return conn.send_req(SPORT, payload).await;
        }
        conn.publish(
            WIRELESS_CONTROLLER,
            json!({ "lx": -y, "ly": x, "rx": -yaw, "ry": 0 }),
        )
        .await
    }

    /// Stops whatever runs, lies the robot down and closes the link: the robot keeps one WebRTC client at a time.
    async fn shut_down(&self) {
        self.stopping.store(true, Ordering::Relaxed);
        self.cancel();
        if let Some(session) = self.session.lock().unwrap().take() {
            session.abort();
        }
        // what was driving the robot drops out at once; wait for that before lying down
        let _turn = self.turn.lock().await;
        let _ = self.stop_movement().await;
        // lie down before the link closes, as GO2Connection.stop does on every backend
        if let Err(err) = self.sport(STAND_DOWN, None).await {
            warn!("lie down on stop: {err}");
        }
        let conn = self.conn.lock().unwrap().take();
        if let Some(conn) = conn {
            conn.close().await;
        }
    }

    /// Bumps the cancel count and wakes the verb running; returns the new count.
    fn cancel(&self) -> u64 {
        let generation = self.cancels.fetch_add(1, Ordering::Relaxed) + 1;
        self.cancel_wake.notify_waiters();
        generation
    }

    /// Runs `work` unless a cancel comes first, in which case it is dropped wherever it is waiting.
    async fn unless_cancelled<T>(
        &self,
        generation: u64,
        work: impl std::future::Future<Output = Result<T, String>>,
    ) -> Result<T, String> {
        let woken = self.cancel_wake.notified();
        tokio::pin!(woken);
        woken.as_mut().enable();
        self.cancelled(generation)?;
        tokio::select! {
            result = work => result,
            _ = woken => Err("cancelled by stop".into()),
        }
    }

    fn cancelled(&self, generation: u64) -> Result<(), String> {
        if self.cancels.load(Ordering::Relaxed) == generation {
            Ok(())
        } else {
            Err("cancelled by stop".into())
        }
    }

    /// Waits `seconds`, then says whether the verb may go on.
    async fn settle(&self, generation: u64, seconds: f64) -> Result<(), String> {
        self.unless_cancelled(generation, async {
            tokio::time::sleep(secs(seconds)).await;
            Ok(())
        })
        .await?;
        self.cancelled(generation)
    }

    async fn stop_movement(&self) -> Result<(), String> {
        self.driving_since.store(0, Ordering::Relaxed);
        self.drive(0.0, 0.0, 0.0).await
    }

    async fn set_rage_mode(&self, enable: bool, generation: u64) -> Result<Value, String> {
        // BalanceStand first; the FSM needs time to settle before SwitchJoystick
        if let Err(err) = self.sport(BALANCE_STAND, None).await {
            warn!("balance stand before the rage toggle failed: {err}");
        }
        self.settle(generation, self.config.timing.rage_balance_settle)
            .await?;
        self.sport(RAGE_MODE, Some(json!({ "data": enable })))
            .await?;
        self.settle(generation, self.config.timing.rage_switch_settle)
            .await?;
        self.sport(SWITCH_JOYSTICK, Some(json!({ "data": true })))
            .await
    }

    async fn set_motion_mode(&self, name: &str, generation: u64) -> Result<(), String> {
        let current = self
            .request(MOTION_SWITCHER, CHECK_MODE, None)
            .await
            .ok()
            .and_then(|answer| serde_json::from_str::<Value>(answer["data"]["data"].as_str()?).ok())
            .and_then(|data| data["name"].as_str().map(str::to_string));
        if current.as_deref() == Some(name) {
            return Ok(());
        }
        self.cancelled(generation)?;
        self.request(MOTION_SWITCHER, SELECT_MODE, Some(json!({ "name": name })))
            .await?;
        self.settle(generation, self.config.timing.motion_mode_settle)
            .await
    }

    /// Runs one verb; its result is the robot's answer where there is one.
    async fn run(&self, verb: Verb, generation: u64) -> Result<Value, String> {
        self.cancelled(generation)?;
        let done = |r: Result<(), String>| r.map(|_| Value::Null);
        match verb {
            Verb::Move {
                x,
                y,
                yaw,
                duration,
            } => {
                if duration > 0.0 {
                    let until = monotonic_ns() + (duration * 1e9) as u64;
                    let driven = async {
                        while monotonic_ns() < until {
                            // kept fresh so the deadman covers a timed move too
                            self.driving_since.store(monotonic_ns(), Ordering::Relaxed);
                            self.drive(x, y, yaw).await?;
                            self.settle(generation, self.config.timing.move_period)
                                .await?;
                        }
                        Ok(())
                    }
                    .await;
                    let stopped = self.stop_movement().await;
                    done(driven.and(stopped))
                } else {
                    self.driving_since.store(monotonic_ns(), Ordering::Relaxed);
                    done(self.drive(x, y, yaw).await)
                }
            }
            Verb::Stop => done(self.stop_movement().await),
            Verb::Sport(api_id) => self.sport(api_id, None).await,
            Verb::Rage(enable) => self.set_rage_mode(enable, generation).await,
            Verb::Joystick(enable) => {
                self.sport(SWITCH_JOYSTICK, Some(json!({ "data": enable })))
                    .await
            }
            Verb::Brightness(level) => {
                self.request(
                    VUI,
                    VUI_SET_BRIGHTNESS,
                    Some(json!({ "brightness": level })),
                )
                .await
            }
            Verb::ObstacleAvoidance(enable) => {
                self.request(
                    OBSTACLES_AVOID,
                    OBSTACLES_AVOID_SWITCH_SET,
                    Some(json!({ "enable": enable as i64 })),
                )
                .await
            }
            Verb::Lidar(on) => done(
                self.conn()?
                    .publish(LIDAR_SWITCH, json!(if on { "on" } else { "off" }))
                    .await,
            ),
            Verb::Volume(level) => {
                self.request(VUI, VUI_SET_VOLUME, Some(json!({ "volume": level })))
                    .await
            }
            Verb::Led(color) => {
                // brightness rides along so the colour is visible: 0 for off, else max
                let level = if color == "off" { 0 } else { VUI_LEVEL_MAX };
                self.request(
                    VUI,
                    VUI_SET_BRIGHTNESS,
                    Some(json!({ "brightness": level })),
                )
                .await?;
                self.request(
                    VUI,
                    VUI_SET_LED,
                    Some(json!({ "color": color, "flash_cycle": 0, "time": 0 })),
                )
                .await
            }
            Verb::MotionMode(name) => done(self.set_motion_mode(&name, generation).await),
            Verb::Request {
                topic,
                api_id,
                parameter,
            } => self.request(&topic, api_id, parameter).await,
        }
    }
}

/// A verb that cuts in: it cancels the one running and the ones waiting, and runs at once.
fn cuts_in(verb: &Verb) -> bool {
    matches!(verb, Verb::Stop | Verb::Sport(DAMP))
}

/// What a `command` message asks for.
#[derive(Debug, PartialEq)]
pub enum Verb {
    Move {
        x: f64,
        y: f64,
        yaw: f64,
        duration: f64,
    },
    Stop,
    Sport(i64),
    Rage(bool),
    Joystick(bool),
    Brightness(i64),
    Volume(i64),
    Led(String),
    Lidar(bool),
    ObstacleAvoidance(bool),
    MotionMode(String),
    Request {
        topic: String,
        api_id: i64,
        parameter: Option<Value>,
    },
}

fn on_off(arg: &str) -> Option<bool> {
    match arg {
        "on" => Some(true),
        "off" => Some(false),
        _ => None,
    }
}

/// Sport actions by name, as GO2DDS names them.
fn sport_id(name: &str) -> Option<i64> {
    Some(match name {
        "damp" => DAMP,
        "balance" => BALANCE_STAND,
        "stand-up" | "standup" => STAND_UP,
        "stand-down" | "standdown" => STAND_DOWN,
        "recovery" => 1006,
        "sit" => 1009,
        "rise" => 1010,
        "hello" => 1016,
        "stretch" => 1017,
        "wallow" => 1021,
        "dance1" => 1022,
        "dance2" => 1023,
        "scrape" => 1029,
        "flip" | "frontflip" => 1030,
        "jump" | "frontjump" => 1031,
        "pounce" | "frontpounce" => 1032,
        "wiggle" | "wigglehips" => 1033,
        "heart" => 1036,
        _ => return None,
    })
}

/// The `command` vocabulary, GO2DDS's: `<verb>` or `<verb> <arg>`. Sport actions by name or number (`stand-up`,
/// `sit`, `1016`), switches (`lidar on`, `joystick on`, `obstacle-avoidance off`), `brightness <0-10>`,
/// `volume <0-10>`, `led <color>` and `rage on|off` (GO2Connection's `set_rage_mode`: BalanceStand, the toggle,
/// then SwitchJoystick). Beyond GO2DDS's, for GO2Connection's rpcs: `move <x> <y> <yaw> <seconds>`, `stop`,
/// `motion-mode <name>` and `request <topic> <api_id> [<json parameter>]`.
pub fn parse_verb(verb: &str) -> Option<Verb> {
    let verb = verb.trim();
    let (name, arg) = verb
        .split_once(' ')
        .map_or((verb, ""), |(n, a)| (n, a.trim()));
    Some(match name.to_ascii_lowercase().as_str() {
        "move" => {
            let numbers: Vec<f64> = arg
                .split_whitespace()
                .map(str::parse)
                .collect::<Result<_, _>>()
                .ok()?;
            let [x, y, yaw, duration] = numbers[..] else {
                return None;
            };
            Verb::Move {
                x,
                y,
                yaw,
                duration,
            }
        }
        "stop" if arg.is_empty() => Verb::Stop,
        "rage" => Verb::Rage(on_off(arg)?),
        "joystick" => Verb::Joystick(on_off(arg)?),
        "obstacle-avoidance" => Verb::ObstacleAvoidance(on_off(arg)?),
        "brightness" => Verb::Brightness(arg.parse::<i64>().ok()?.clamp(0, VUI_LEVEL_MAX)),
        "volume" => Verb::Volume(arg.parse::<i64>().ok()?.clamp(0, VUI_LEVEL_MAX)),
        "led" if !arg.is_empty() && arg.chars().all(|c| c.is_ascii_alphanumeric()) => {
            Verb::Led(arg.to_string())
        }
        "lidar" => Verb::Lidar(on_off(arg)?),
        "motion-mode" if !arg.is_empty() => Verb::MotionMode(arg.to_string()),
        "request" => {
            let mut parts = arg.splitn(3, ' ');
            let topic = parts.next()?.to_string();
            let api_id = parts.next()?.parse().ok()?;
            let parameter = match parts.next() {
                Some(json) => Some(serde_json::from_str(json).ok()?),
                None => None,
            };
            Verb::Request {
                topic,
                api_id,
                parameter,
            }
        }
        name if arg.is_empty() => Verb::Sport(sport_id(name).or_else(|| name.parse().ok())?),
        _ => return None,
    })
}

#[derive(Module)]
#[module(name = "go2_webrtc", setup = start, teardown = stop)]
pub struct Go2WebRtc {
    #[input(decode = Twist::decode, handler = on_cmd_vel)]
    cmd_vel: Input<Twist>,

    #[input(decode = std_msgs::String::decode, handler = on_command)]
    command: Input<std_msgs::String>,

    #[output(encode = PoseStamped::encode)]
    odom: Output<PoseStamped>,

    #[output(encode = PointCloud2::encode)]
    lidar: Output<PointCloud2>,

    /// GO2Connection declares it (the Pointcloud spec); nothing publishes on it.
    #[allow(dead_code)]
    #[output(encode = PointCloud2::encode)]
    pointcloud: Output<PointCloud2>,

    #[output(encode = Image::encode)]
    color_image: Output<Image>,

    #[output(encode = CameraInfo::encode)]
    camera_info: Output<CameraInfo>,

    #[output(encode = TFMessage::encode)]
    tf: Output<TFMessage>,

    #[output(encode = BatteryState::encode)]
    battery: Output<BatteryState>,

    #[config]
    config: Config,

    shared: Option<Arc<Shared>>,
    /// verbs waiting their turn, with the cancel count they were sent under
    queue: Option<tokio_mpsc::UnboundedSender<Queued>>,
}

struct Queued {
    verb: Verb,
    generation: u64,
}

impl Go2WebRtc {
    async fn start(&mut self) {
        let shared = Arc::new(Shared {
            config: self.config.clone(),
            handle: Handle::current(),
            conn: Mutex::new(None),
            driving_since: AtomicU64::new(0),
            move_ids: AtomicU64::new(0),
            stopping: AtomicBool::new(false),
            lost: Notify::new(),
            cancels: AtomicU64::new(0),
            cancel_wake: Notify::new(),
            turn: tokio::sync::Mutex::new(()),
            session: Mutex::new(None),
        });
        self.shared = Some(shared.clone());

        // one verb at a time, in the order they came
        let (queue, mut queued) = tokio_mpsc::unbounded_channel::<Queued>();
        self.queue = Some(queue);
        {
            let shared = shared.clone();
            tokio::spawn(async move {
                while let Some(Queued { verb, generation }) = queued.recv().await {
                    let shown = format!("{verb:?}");
                    let _turn = shared.turn.lock().await;
                    if let Err(err) = shared
                        .unless_cancelled(generation, shared.run(verb, generation))
                        .await
                    {
                        warn!(verb = shown, "command failed: {err}");
                    }
                }
            });
        }

        let (video_tx, video_rx) = mpsc::sync_channel(self.config.timing.video_queue_packets);
        let video_dropped = Arc::new(AtomicBool::new(false));
        let (data_tx, data_rx) = mpsc::channel();
        if self.config.camera {
            let video = VideoLoop {
                shared: shared.clone(),
                color_image: self.color_image.clone(),
                camera_info: self.camera_info.clone(),
                dropped: video_dropped.clone(),
            };
            std::thread::Builder::new()
                .name("go2-video".into())
                .spawn(move || video.run(video_rx))
                .expect("spawn the video thread");
        }
        let data = DataLoop {
            shared: shared.clone(),
            odom: self.odom.clone(),
            lidar: self.lidar.clone(),
            tf: self.tf.clone(),
            battery: self.battery.clone(),
        };
        std::thread::Builder::new()
            .name("go2-data".into())
            .spawn(move || data.run(data_rx))
            .expect("spawn the data thread");

        let on_event: crate::conn::OnEvent = {
            let shared = shared.clone();
            let video_tx: SyncSender<Event> = video_tx;
            let data_tx = Mutex::new(data_tx);
            Arc::new(move |event| match event {
                Event::Rtp { .. } => {
                    if let Err(TrySendError::Full(_)) = video_tx.try_send(event) {
                        video_dropped.store(true, Ordering::Relaxed);
                    }
                }
                Event::Data { .. } => {
                    let _ = data_tx.lock().unwrap().send(event);
                }
                Event::Lost => shared.lost.notify_one(),
            })
        };
        *shared.session.lock().unwrap() = Some(tokio::spawn(session(shared.clone(), on_event)));
        // a parent that dies without stopping us (no parent-death signal on macOS) mustn't leave the robot held
        {
            let shared = shared.clone();
            let parent = std::os::unix::process::parent_id();
            tokio::spawn(async move {
                loop {
                    tokio::time::sleep(secs(shared.config.timing.heartbeat_period)).await;
                    if std::os::unix::process::parent_id() != parent {
                        error!("the dimos process is gone: letting go of the Go2");
                        shared.shut_down().await;
                        std::process::exit(1);
                    }
                }
            });
        }
        tokio::spawn(deadman(shared));
    }

    async fn stop(&mut self) {
        if let Some(shared) = &self.shared {
            shared.shut_down().await;
        }
    }

    async fn on_cmd_vel(&mut self, msg: Twist) {
        let Some(shared) = &self.shared else { return };
        shared
            .driving_since
            .store(monotonic_ns(), Ordering::Relaxed);
        if let Err(err) = shared
            .drive(msg.linear.x, msg.linear.y, msg.angular.z)
            .await
        {
            warn!("cmd_vel: {err}");
        }
    }

    async fn on_command(&mut self, msg: std_msgs::String) {
        let (Some(shared), Some(queue)) = (self.shared.clone(), self.queue.clone()) else {
            return;
        };
        let Some(verb) = parse_verb(&msg.data) else {
            warn!(verb = %msg.data, "unknown command verb");
            return;
        };
        info!(?verb, "command");
        if cuts_in(&verb) {
            let generation = shared.cancel();
            tokio::spawn(async move {
                if let Err(err) = shared.run(verb, generation).await {
                    warn!(verb = %msg.data, "command failed: {err}");
                }
            });
            return;
        }
        let generation = shared.cancels.load(Ordering::Relaxed);
        let _ = queue.send(Queued { verb, generation });
    }
}

/// Connects, sets the link up, runs GO2Connection's start sequence once, then waits for the link to die; with
/// auto_reconnect it connects again (no stand-up), otherwise the process exits so the run sees the Go2 is gone.
async fn session(shared: Arc<Shared>, on_event: crate::conn::OnEvent) {
    let config = &shared.config;
    let mut started = false;
    loop {
        info!(ip = %config.ip, "connecting to the Go2");
        match Conn::connect(
            &config.ip,
            &config.aes_128_key.0,
            &config.timing,
            on_event.clone(),
        )
        .await
        {
            Ok(conn) => {
                *shared.conn.lock().unwrap() = Some(conn.clone());
                if let Err(err) = link_up(&shared, &conn).await {
                    warn!("setting up the Go2 link: {err}");
                }
                if shared.stopping.load(Ordering::Relaxed) {
                    return;
                }
                if !started {
                    started = true;
                    start_sequence(&shared).await;
                } else {
                    // link_up put the robot back on "ai"
                    restore_motion_mode(&shared).await;
                }
                shared.lost.notified().await;
                conn.close().await;
                if shared.stopping.load(Ordering::Relaxed) {
                    return;
                }
                error!("the Go2 dropped the WebRTC link");
            }
            Err(err) => error!("connecting to the Go2 at {} failed: {err}", config.ip),
        }
        if shared.stopping.load(Ordering::Relaxed) {
            return;
        }
        if !config.auto_reconnect {
            error!("auto_reconnect is off: exiting");
            std::process::exit(1);
        }
        tokio::time::sleep(secs(config.timing.reconnect_pause)).await;
    }
}

/// What every connection needs: big messages unthrottled, the "ai" controller,
/// the subscriptions and the camera.
async fn link_up(shared: &Shared, conn: &Conn) -> Result<(), String> {
    conn.send_raw(
        "rtc_inner_req",
        json!({ "req_type": "disable_traffic_saving", "instruction": "on" }),
    )
    .await?;
    if let Err(err) = conn
        .request(
            MOTION_SWITCHER,
            SELECT_MODE,
            Some(&json!({ "name": "ai" })),
            secs(shared.config.timing.answer_timeout),
        )
        .await
    {
        warn!("selecting the ai motion controller: {err}");
    }
    if shared.config.lidar {
        conn.subscribe(LIDAR_TOPIC).await?;
    }
    conn.subscribe(POSE_TOPIC).await?;
    conn.subscribe(LOWSTATE_TOPIC).await?;
    if shared.config.camera {
        conn.send_raw("vid", json!("on")).await?;
        conn.request_keyframe().await;
    }
    Ok(())
}

/// After a reconnect: config.motion_mode again, without the rest of the start sequence.
async fn restore_motion_mode(shared: &Shared) {
    let mode = &shared.config.motion_mode;
    if mode.is_empty() {
        return;
    }
    let _turn = shared.turn.lock().await;
    let generation = shared.cancels.load(Ordering::Relaxed);
    if let Err(err) = shared
        .unless_cancelled(generation, shared.set_motion_mode(mode, generation))
        .await
    {
        warn!("motion mode after reconnect: {err}");
    }
}

/// The start sequence: motion mode, stand, balance, rage, obstacle avoidance.
async fn start_sequence(shared: &Shared) {
    let _turn = shared.turn.lock().await;
    let generation = shared.cancels.load(Ordering::Relaxed);
    if let Err(err) = shared
        .unless_cancelled(generation, start_steps(shared, generation))
        .await
    {
        warn!("start sequence: {err}");
    }
}

async fn start_steps(shared: &Shared, generation: u64) -> Result<(), String> {
    let config = &shared.config;
    let warned = |step: &str, result: Result<Value, String>| {
        if let Err(err) = result {
            warn!("{step}: {err}");
        }
    };
    if !config.motion_mode.is_empty() {
        warned(
            "motion mode",
            shared
                .set_motion_mode(&config.motion_mode, generation)
                .await
                .map(|_| Value::Null),
        );
    }
    shared.cancelled(generation)?;
    warned("stand up", shared.sport(STAND_UP, None).await);
    shared
        .settle(generation, config.timing.stand_up_settle)
        .await?;
    warned("balance stand", shared.sport(BALANCE_STAND, None).await);
    if config.mode == "rage" {
        shared.cancelled(generation)?;
        warned("rage mode", shared.set_rage_mode(true, generation).await);
    }
    shared.cancelled(generation)?;
    warned(
        "obstacle avoidance",
        shared
            .request(
                OBSTACLES_AVOID,
                OBSTACLES_AVOID_SWITCH_SET,
                Some(json!({ "enable": config.obstacle_avoidance as i64 })),
            )
            .await,
    );
    Ok(())
}

/// Stops the base once `cmd_vel` has been silent for cmd_vel_timeout (checked ten times per timeout).
async fn deadman(shared: Arc<Shared>) {
    let timeout_ns = secs(shared.config.timing.cmd_vel_timeout).as_nanos() as u64;
    loop {
        tokio::time::sleep(secs(shared.config.timing.cmd_vel_timeout / 10.0)).await;
        let since = shared.driving_since.load(Ordering::Relaxed);
        if since != 0 && monotonic_ns().saturating_sub(since) > timeout_ns {
            if let Err(err) = shared.stop_movement().await {
                warn!("deadman stop: {err}");
            }
        }
    }
}

struct DataLoop {
    shared: Arc<Shared>,
    odom: Output<PoseStamped>,
    lidar: Output<PointCloud2>,
    tf: Output<TFMessage>,
    battery: Output<BatteryState>,
}

impl DataLoop {
    fn run(self, rx: Receiver<Event>) {
        let config = &self.shared.config;
        let prefix = config.frame_id_prefix.as_str();
        let (base_link, camera_link, camera_optical) = (
            prefixed(prefix, "base_link"),
            prefixed(prefix, "camera_link"),
            prefixed(prefix, "camera_optical"),
        );
        let mut pose_clock = HostClock::new(config.timing.clock_window, config.timing.clock_reset);
        let mut stats = Stats::new(config.timing.stats_period);
        while let Ok(event) = rx.recv() {
            let Event::Data {
                topic,
                json,
                binary,
                arrived_ns,
            } = event
            else {
                continue;
            };
            let data = &json["data"];
            match topic.as_str() {
                POSE_TOPIC => {
                    let robot_ns = pose_robot_ns(data);
                    let ns = place(&mut pose_clock, robot_ns, arrived_ns);
                    stats.pose(robot_ns, arrived_ns, &pose_clock);
                    let p = &data["pose"]["position"];
                    let o = &data["pose"]["orientation"];
                    let position = Point {
                        x: f(&p["x"]),
                        y: f(&p["y"]),
                        z: f(&p["z"]),
                    };
                    let orientation = Quaternion {
                        x: f(&o["x"]),
                        y: f(&o["y"]),
                        z: f(&o["z"]),
                        w: o["w"].as_f64().unwrap_or(1.0),
                    };
                    let odom = PoseStamped {
                        header: header(ns, &config.odom_frame_id),
                        pose: Pose {
                            position: position.clone(),
                            orientation: orientation.clone(),
                        },
                    };
                    if config.publish_tf {
                        let edge =
                            |parent: &str,
                             child: &str,
                             translation: Vector3,
                             rotation: Quaternion| TransformStamped {
                                header: header(ns, parent),
                                child_frame_id: child.to_string(),
                                transform: Transform {
                                    translation,
                                    rotation,
                                },
                            };
                        let transforms = vec![
                            edge(
                                &config.odom_frame_id,
                                &base_link,
                                Vector3 {
                                    x: position.x,
                                    y: position.y,
                                    z: position.z,
                                },
                                orientation,
                            ),
                            edge(
                                &base_link,
                                &camera_link,
                                Vector3 {
                                    x: config.camera_offset[0],
                                    y: config.camera_offset[1],
                                    z: config.camera_offset[2],
                                },
                                Quaternion {
                                    x: 0.0,
                                    y: 0.0,
                                    z: 0.0,
                                    w: 1.0,
                                },
                            ),
                            edge(
                                &camera_link,
                                &camera_optical,
                                Vector3::default(),
                                Quaternion {
                                    x: -0.5,
                                    y: 0.5,
                                    z: -0.5,
                                    w: 0.5,
                                },
                            ),
                        ];
                        let _ = self
                            .shared
                            .handle
                            .block_on(self.tf.publish(&TFMessage { transforms }));
                    }
                    let _ = self.shared.handle.block_on(self.odom.publish(&odom));
                }
                LIDAR_TOPIC => {
                    let Some(binary) = binary else { continue };
                    let points = match decode_voxels(data, &binary) {
                        Ok(points) => points,
                        Err(err) => {
                            warn!("lidar: {err}");
                            continue;
                        }
                    };
                    // a scan's `stamp` comes as a float32 (128 s steps at today's epoch): arrival is the better time
                    let ns = arrived_ns;
                    let _ = self
                        .shared
                        .handle
                        .block_on(self.lidar.publish(&cloud(ns, points)));
                }
                LOWSTATE_TOPIC => {
                    if let Some(state) = battery(data, arrived_ns, &base_link) {
                        let _ = self.shared.handle.block_on(self.battery.publish(&state));
                    }
                }
                _ => {}
            }
        }
    }
}

/// The lowstate's battery in SI units, if it carries one; what the pack doesn't report is NaN.
fn battery(data: &Value, ns: u64, frame_id: &str) -> Option<BatteryState> {
    let bms = &data["bms_state"];
    let soc = bms["soc"].as_f64()?;
    let current = f(&bms["current"]);
    let status = if current > 0.0 {
        BatteryState::POWER_SUPPLY_STATUS_CHARGING
    } else if current < 0.0 {
        BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
    } else {
        BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
    };
    Some(BatteryState {
        header: header(ns, frame_id),
        voltage: f(&data["power_v"]) as f32,
        temperature: f(&bms["bq_ntc"][0]) as f32,
        current: current as f32 / 1000.0,
        charge: f32::NAN,
        capacity: f32::NAN,
        design_capacity: f32::NAN,
        percentage: soc as f32 / 100.0,
        power_supply_status: status as u8,
        power_supply_health: BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN as u8,
        power_supply_technology: BatteryState::POWER_SUPPLY_TECHNOLOGY_LION as u8,
        present: true,
        cell_voltage: bms["cell_vol"]
            .as_array()
            .into_iter()
            .flatten()
            .map(f)
            .filter(|&v| v > 0.0)
            .map(|v| v as f32 / 1000.0)
            .collect(),
        cell_temperature: Vec::new(),
        location: String::new(),
        serial_number: String::new(),
    })
}

/// xyz float32 points as a PointCloud2 in "world".
fn cloud(ns: u64, points: Vec<[f32; 3]>) -> PointCloud2 {
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let mut data = Vec::with_capacity(points.len() * 12);
    for point in &points {
        for value in point {
            data.extend_from_slice(&value.to_le_bytes());
        }
    }
    PointCloud2 {
        header: header(ns, "world"),
        height: 1,
        width: points.len() as i32,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points.len() as i32,
        data,
        is_dense: true,
    }
}

struct VideoLoop {
    shared: Arc<Shared>,
    color_image: Output<Image>,
    camera_info: Output<CameraInfo>,
    dropped: Arc<AtomicBool>,
}

impl VideoLoop {
    fn want_keyframe(&self) {
        if let Ok(conn) = self.shared.conn() {
            self.shared
                .handle
                .spawn(async move { conn.request_keyframe().await });
        }
    }

    fn run(self, rx: Receiver<Event>) {
        let frame_id = prefixed(&self.shared.config.frame_id_prefix, "camera_optical");
        // NoFlush: flushing after every decode errors on a stream with reordered frames
        let decoder_config = openh264::decoder::DecoderConfig::new()
            .flush_after_decode(openh264::decoder::Flush::NoFlush);
        let mut decoder = match openh264::decoder::Decoder::with_api_config(
            openh264::OpenH264API::from_source(),
            decoder_config,
        ) {
            Ok(decoder) => decoder,
            Err(err) => {
                error!("openh264 didn't start, no color_image: {err}");
                return;
            }
        };
        let timing = &self.shared.config.timing;
        let mut depacketizer = Depacketizer::new(timing.max_frame_bytes);
        let reorder_wait_ns = secs(timing.reorder_wait).as_nanos() as u64;
        let mut reorder = Reorder::new(reorder_wait_ns);
        let mut capture = CaptureClock::new(timing.capture_max_waiting, timing.clock_window);
        let mut ssrc = None;
        // when each RTP time's first packet arrived (a frame's packets share one)
        let mut first_arrival: VecDeque<(u32, u64)> = VecDeque::new();
        let mut rgb = Vec::new();
        let mut last_info_ns = 0u64;
        let mut last_keyframe_ask_ns = 0u64;
        let keyframe_retry_ns =
            secs(self.shared.config.timing.keyframe_retry_period).as_nanos() as u64;
        let mut stats = Stats::new(timing.stats_period);
        while let Ok(event) = rx.recv() {
            let Event::Rtp { packet, arrived_ns } = event else {
                continue;
            };
            // a new session is a new stream: unrelated sequence numbers and RTP times
            if ssrc
                .replace(packet.header.ssrc)
                .is_some_and(|old| old != packet.header.ssrc)
            {
                depacketizer = Depacketizer::new(timing.max_frame_bytes);
                reorder = Reorder::new(reorder_wait_ns);
                capture = CaptureClock::new(timing.capture_max_waiting, timing.clock_window);
                first_arrival.clear();
                self.want_keyframe();
            }
            for (packet, arrived_ns) in reorder.push(
                packet.header.sequence_number,
                (packet, arrived_ns),
                arrived_ns,
            ) {
                // a lost keyframe request (or one the Go2 ignored) would leave the camera dark until its next keyframe
                if !depacketizer.synced()
                    && arrived_ns.saturating_sub(last_keyframe_ask_ns) > keyframe_retry_ns
                {
                    last_keyframe_ask_ns = arrived_ns;
                    self.want_keyframe();
                }
                if self.dropped.swap(false, Ordering::Relaxed) {
                    // packets were dropped before this one: the frame in progress is broken
                    depacketizer = Depacketizer::new(timing.max_frame_bytes);
                    reorder = Reorder::new(reorder_wait_ns);
                    capture = CaptureClock::new(timing.capture_max_waiting, timing.clock_window);
                    self.want_keyframe();
                }
                if first_arrival
                    .back()
                    .is_none_or(|&(rtp, _)| rtp != packet.header.timestamp)
                {
                    first_arrival.push_back((packet.header.timestamp, arrived_ns));
                    // the frames in the decoder, and the one the depacketizer is building
                    if first_arrival.len() > timing.capture_max_waiting + 1 {
                        first_arrival.pop_front();
                    }
                }
                let was_synced = depacketizer.synced();
                let view = Packet {
                    sequence_number: packet.header.sequence_number,
                    timestamp: packet.header.timestamp,
                    marker: packet.header.marker,
                    payload: &packet.payload,
                };
                let unit = depacketizer.push(&view);
                if was_synced && !depacketizer.synced() {
                    self.want_keyframe();
                }
                let Some(unit) = unit else { continue };
                let unit_first_arrival = first_arrival
                    .iter()
                    .find(|&&(rtp, _)| rtp == unit.rtp_ts)
                    .map_or(arrived_ns, |&(_, at)| at);
                capture.submitted(unit.rtp_ts, unit_first_arrival);
                let decoded = match decoder.decode(&unit.data) {
                    Ok(Some(yuv)) => yuv,
                    Ok(None) => continue,
                    Err(err) => {
                        warn!("H.264 decode failed ({err}); waiting for a keyframe");
                        depacketizer = Depacketizer::new(timing.max_frame_bytes);
                        capture =
                            CaptureClock::new(timing.capture_max_waiting, timing.clock_window);
                        self.want_keyframe();
                        continue;
                    }
                };
                // the Go2 sends no capture time for a frame: its arrival, de-jittered by the RTP tick spacing
                let ns = capture.decoded().unwrap_or(arrived_ns);
                stats.frame(ns, now_ns());
                use openh264::formats::YUVSource;
                let (width, height) = decoded.dimensions();
                rgb.resize(width * height * 3, 0);
                decoded.write_rgb8(&mut rgb);
                let image = Image {
                    header: header(ns, &frame_id),
                    height: height as i32,
                    width: width as i32,
                    encoding: "rgb8".into(),
                    is_bigendian: 0,
                    step: (width * 3) as i32,
                    data: std::mem::take(&mut rgb),
                };
                let _ = self
                    .shared
                    .handle
                    .block_on(self.color_image.publish(&image));
                if ns.saturating_sub(last_info_ns)
                    >= secs(timing.camera_info_period).as_nanos() as u64
                {
                    last_info_ns = ns;
                    let mut info = self.shared.config.camera_info.clone();
                    info.header = header(ns, &frame_id);
                    let _ = self.shared.handle.block_on(self.camera_info.publish(&info));
                }
            }
        }
    }
}

/// Every stats_period, how each stream's stamps sit against the host clock: the offset in use and the stamp -> arrival
/// (and -> published, for video) spread. The numbers that say whether the Go2's clocks agree with each other.
struct Stats {
    period_ns: u64,
    since_ns: u64,
    pose: Vec<i64>,
    frame: Vec<i64>,
}

impl Stats {
    fn new(period: f64) -> Self {
        Self {
            period_ns: secs(period).as_nanos() as u64,
            since_ns: 0,
            pose: Vec::new(),
            frame: Vec::new(),
        }
    }

    fn gap(robot_ns: Option<i64>, arrived_ns: u64, clock: &HostClock) -> Option<i64> {
        Some(arrived_ns as i64 - clock.host_ns(robot_ns?)? as i64)
    }

    fn pose(&mut self, robot_ns: Option<i64>, arrived_ns: u64, clock: &HostClock) {
        self.pose.extend(Self::gap(robot_ns, arrived_ns, clock));
        self.report(arrived_ns, clock.offset_ns(), "pose");
    }

    fn frame(&mut self, stamp_ns: u64, published_ns: u64) {
        self.frame.push(published_ns as i64 - stamp_ns as i64);
        self.report(stamp_ns, None, "video");
    }

    fn report(&mut self, now: u64, offset_ns: Option<i64>, stream: &str) {
        if self.since_ns == 0 {
            self.since_ns = now;
        }
        if now.saturating_sub(self.since_ns) < self.period_ns {
            return;
        }
        let ms = |v: &mut Vec<i64>| -> String {
            if v.is_empty() {
                return "-".into();
            }
            v.sort_unstable();
            let pick = |q: f64| v[((v.len() - 1) as f64 * q) as usize] as f64 / 1e6;
            format!(
                "p50 {:.1} p95 {:.1} max {:.1} ms (n {})",
                pick(0.5),
                pick(0.95),
                pick(1.0),
                v.len()
            )
        };
        info!(
            stream,
            offset_ms = offset_ns.map(|o| o as f64 / 1e6),
            pose_capture_to_arrival = ms(&mut self.pose),
            video_stamp_to_published = ms(&mut self.frame),
            "[go2 clock]"
        );
        self.pose.clear();
        self.frame.clear();
        self.since_ns = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The two files GO2Connection's tests check it against: the config it sends and every verb its rpcs send.
    #[test]
    fn python_side_contract() {
        let config: Config =
            serde_json::from_str(include_str!("../tests/fixtures/config.json")).unwrap();
        assert_eq!(config.ip, "192.168.123.161");
        for line in include_str!("../tests/fixtures/verbs.txt").lines() {
            assert!(parse_verb(line).is_some(), "{line}");
        }
    }

    #[test]
    fn verbs_resolve() {
        assert_eq!(parse_verb("stand-up"), Some(Verb::Sport(STAND_UP)));
        assert_eq!(parse_verb(" 1016 "), Some(Verb::Sport(1016)));
        assert_eq!(parse_verb("sit"), Some(Verb::Sport(1009)));
        assert_eq!(parse_verb("damp"), Some(Verb::Sport(DAMP)));
        assert_eq!(parse_verb("lidar off"), Some(Verb::Lidar(false)));
        assert_eq!(parse_verb("volume 42"), Some(Verb::Volume(10)));
        assert_eq!(parse_verb("led red"), Some(Verb::Led("red".into())));
        assert!(cuts_in(&parse_verb("damp").unwrap()) && cuts_in(&Verb::Stop));
        assert_eq!(parse_verb("rage on"), Some(Verb::Rage(true)));
        assert_eq!(
            parse_verb("obstacle-avoidance off"),
            Some(Verb::ObstacleAvoidance(false))
        );
        assert_eq!(parse_verb("brightness 42"), Some(Verb::Brightness(10)));
        assert_eq!(
            parse_verb("move 0.5 0 -0.2 1.5"),
            Some(Verb::Move {
                x: 0.5,
                y: 0.0,
                yaw: -0.2,
                duration: 1.5
            })
        );
        assert_eq!(
            parse_verb(r#"request rt/api/vui/request 1005 {"brightness": 3}"#),
            Some(Verb::Request {
                topic: VUI.into(),
                api_id: 1005,
                parameter: Some(json!({ "brightness": 3 })),
            })
        );
        for bad in [
            "",
            "rage maybe",
            "move 1 2",
            "brightness high",
            "jump around",
            "request t",
        ] {
            assert_eq!(parse_verb(bad), None, "{bad}");
        }
    }

    #[test]
    fn the_key_stays_out_of_debug_output() {
        assert_eq!(
            format!("{:?}", Secret("00112233445566778899aabbccddeeff".into())),
            "<redacted>"
        );
        assert_eq!(format!("{:?}", Secret::default()), "\"\"");
    }

    #[test]
    fn stamps_come_from_the_robots_fields() {
        let pose = json!({ "header": { "stamp": { "sec": 1746565669, "nanosec": 448350564 } } });
        assert_eq!(pose_robot_ns(&pose), Some(1_746_565_669_448_350_564));
    }

    #[test]
    fn a_cloud_packs_xyz_float32() {
        let cloud = cloud(1_500_000_000, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        assert_eq!(
            (
                cloud.width,
                cloud.point_step,
                cloud.row_step,
                cloud.data.len()
            ),
            (2, 12, 24, 24)
        );
        assert_eq!(&cloud.data[12..16], &4.0f32.to_le_bytes());
        assert_eq!(
            (cloud.header.stamp.sec, cloud.header.stamp.nsec),
            (1, 500_000_000)
        );
    }

    #[test]
    fn frames_are_namespaced_only_with_a_prefix() {
        assert_eq!(prefixed("", "base_link"), "base_link");
        assert_eq!(prefixed("go2_a", "base_link"), "go2_a/base_link");
    }
}
