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
//! The M20 controller as one dimos module: the motor bus, the batteries and, built in, the walking policy that
//! replaces rl_deploy. Same outputs in every mode:
//!
//! - `tap` (default): read the frames rl_deploy exchanges, from the `ecan_tap.so` shim's stream. Receive-only.
//! - `owner` (rl_deploy stopped): own the motor bus with m20-ecan's bus owner (rl_deploy's startup, then 1 kHz),
//!   with the charge-enable request from m20-battery's rule while docked and lying.
//!   - `policy` off (default): rl_deploy's zero-gain sitting command every 1 ms; nothing holds or moves a joint.
//!   - `policy` on: rl_deploy's state machine and ONNX runners at 1 kHz (m20_controller.toml), driven by `cmd_vel`
//!     (metric, through each runner's REAL_STEER divisor), `motion_cmd` (passive, stand, rl_control, lie_down,
//!     joint_damping) and `gait_cmd` (vendor gait code), on the body IMU from `imu`. Gains reach the motors only
//!     with `allow_gains`; off (the default) every command that holds a joint goes out as the sitting command.
//!   - docking (policy on): `dock_cmd` ("dock", "undock", "cancel") runs charge_manager's sequence (crate::dock)
//!     on the dock pose from `dock_pose` (m20_dock_detector_module) and the packs' charge MOS; `dock_state` is its
//!     status as JSON.
//!
//! Every frame is published as it arrives; a publish still in flight when newer data lands makes the older data be
//! skipped (latest wins).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::geometry_msgs::{PoseStamped, Twist};
use lcm_msgs::sensor_msgs::{BatteryState, Imu, JointState};
use lcm_msgs::std_msgs::{self, Bool, Float32, UInt32};
use m20_ecan::tap::{TapRecord, TapSocket};
use m20_ecan::Direction;
use tokio::sync::watch;
use tracing::{info, warn};

use crate::dock::docking::{DockFeed, Request};
use crate::dock::vendor::ChargeManagerToml;
use crate::dock::{ChargeReading, DockPose};
use crate::hardware::ecan::{ImuSample, ImuSource};
use crate::publish::frames::{Batteries, FrameHandler, Published};
use crate::types::{Target, UserCommand};

/// The controller's own m20_controller.toml, used when `controller_config` is empty.
pub const BUILT_IN_CONFIG: &str = include_str!("../m20_controller.toml");

fn valid_mode(mode: &str) -> Result<(), validator::ValidationError> {
    match mode {
        "tap" | "owner" => Ok(()),
        _ => Err(validator::ValidationError::new("mode must be tap or owner")),
    }
}

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// `tap` (read rl_deploy's frames through the shim) or `owner` (own the bus; rl_deploy stopped).
    #[validate(custom(function = "valid_mode"))]
    mode: String,
    /// tap: the unix datagram socket the shim streams to; this module binds it.
    socket_path: String,
    /// owner: the motor-bus interface.
    interface: String,
    /// owner: send the BMS charge-enable request when m20-battery's rule asks (docked, lying).
    charge_enable: bool,
    /// frame_id of every header.
    frame_id: String,
    /// owner: stop owning the bus on a reply with any joint torque (N m) or speed (rad/s) above these, or, once the
    /// startup is through, any status word other than 1. With `allow_gains` these must cover what the policy does.
    #[validate(range(exclusive_min = 0.0))]
    guard_max_torque: f64,
    #[validate(range(exclusive_min = 0.0))]
    guard_max_velocity: f64,
    /// Seconds without a frame before warning.
    #[validate(range(exclusive_min = 0.0, max = 3600.0))]
    silence_warning_s: f64,
    /// owner: board replies between progress log lines.
    #[validate(range(min = 1))]
    progress_log_replies: u64,
    /// owner: run the walking policy (state machine + ONNX runners) on the bus; off sends only the sitting command.
    policy: bool,
    /// owner + policy: let commands carrying kp / kd onto the motors. Never on without an operator's go.
    allow_gains: bool,
    /// policy: an m20_controller.toml path; empty uses the one built in.
    controller_config: String,
    /// policy: the directory of the extracted rl_deploy ONNX policies; empty is ~/.cache/dimos/m20_policies.
    policy_dir: String,
    /// policy: seconds an IMU sample stays usable; older, the state is stale and the state machine faults.
    #[validate(range(exclusive_min = 0.0))]
    imu_timeout_s: f64,
    /// policy: seconds without a new command before the velocity is zeroed, over the TOML's
    /// safety.command_timeout_s; 0 (unset) keeps the TOML's.
    #[validate(range(min = 0.0))]
    command_timeout_s: f64,
    /// policy: the vendor gait code walking starts on until a gait_cmd changes it (Python default 0x1001 basic flat).
    gait: u32,
    /// policy: charge_manager.toml (NOS's, docking reads it); when this board has none, the copy the dock detector
    /// module publishes on charge_manager_config is taken
    charge_manager_toml: String,
}

#[derive(Module)]
#[module(name = "m20_controller", setup = start, teardown = stop)]
pub struct M20Controller {
    #[input(decode = Twist::decode, handler = on_cmd_vel)]
    cmd_vel: Input<Twist>,
    #[input(decode = Imu::decode, handler = on_imu)]
    imu: Input<Imu>,
    #[input(decode = std_msgs::String::decode, handler = on_motion_cmd)]
    motion_cmd: Input<std_msgs::String>,
    #[input(decode = UInt32::decode, handler = on_gait_cmd)]
    gait_cmd: Input<UInt32>,
    /// the dock in base_link, from m20_dock_detector_module (one per sweep that sees it)
    #[input(decode = PoseStamped::decode, handler = on_dock_pose)]
    dock_pose: Input<PoseStamped>,
    /// "dock", "undock" or "cancel"
    #[input(decode = std_msgs::String::decode, handler = on_dock_cmd)]
    dock_cmd: Input<std_msgs::String>,
    /// charge_manager's obstacle check from m20_dock_detector_module: the nearest occupied cell (m), -1 for none
    #[input(decode = Float32::decode, handler = on_dock_obstacle)]
    dock_obstacle: Input<Float32>,
    /// charge_manager.toml's text from m20_dock_detector_module (read on NOS)
    #[input(decode = std_msgs::String::decode, handler = on_charge_manager_config)]
    charge_manager_config: Input<std_msgs::String>,

    #[output(encode = JointState::encode)]
    joint_state: Output<JointState>,
    #[output(encode = JointState::encode)]
    joint_command: Output<JointState>,
    #[output(encode = BatteryState::encode)]
    battery: Output<BatteryState>,
    #[output(encode = BatteryState::encode)]
    battery_left: Output<BatteryState>,
    #[output(encode = BatteryState::encode)]
    battery_right: Output<BatteryState>,
    /// policy: in RL control without a fault (velocity is being followed).
    #[output(encode = Bool::encode)]
    command_ready: Output<Bool>,
    /// policy: the state machine's state, and its fault if any.
    #[output(encode = std_msgs::String::encode)]
    controller_state: Output<std_msgs::String>,
    /// policy: the docking sequence's status (JSON: phase, active, result, error, charging, packs, fine_point)
    #[output(encode = std_msgs::String::encode)]
    dock_state: Output<std_msgs::String>,
    /// policy: base_link's height above the wheels' contact (m, from the joints), for the dock's obstacle check
    #[output(encode = Float32::encode)]
    body_height: Output<Float32>,

    #[config]
    config: Config,

    /// What the operator asks for now; each change goes to the 1 kHz loop.
    current: UserCommand,
    shared: Arc<Shared>,
    thread: Option<std::thread::JoinHandle<()>>,
}

/// Between the module's handlers and the 1 kHz loop.
#[derive(Default)]
pub struct Shared {
    command: Mutex<Option<UserCommand>>,
    imu: Mutex<Option<(ImuSample, std::time::Instant)>>,
    stop: AtomicBool,
    /// docking's inputs (dock pose, battery reading, requests)
    dock: Arc<DockFeed>,
    /// dock poses received
    dock_poses: std::sync::atomic::AtomicU64,
    /// obstacle readings received
    dock_obstacles: std::sync::atomic::AtomicU64,
    /// charge_manager.toml was taken (this board's or the detector module's copy)
    vendor_read: AtomicBool,
}

/// The body IMU as the loop reads it: the newest sample, if it is fresh.
pub struct SharedImu {
    shared: Arc<Shared>,
    timeout: Duration,
}

impl ImuSource for SharedImu {
    fn latest(&mut self) -> Option<ImuSample> {
        let guard = self.shared.imu.lock().unwrap_or_else(|p| p.into_inner());
        guard
            .as_ref()
            .filter(|(_, at)| at.elapsed() <= self.timeout)
            .map(|(sample, _)| *sample)
    }
}

/// The operator's commands as the loop polls them: the newest change since the last tick.
pub struct SharedCommands {
    shared: Arc<Shared>,
}

impl crate::commands::CommandSource for SharedCommands {
    fn poll(&mut self, _now: f64) -> Option<UserCommand> {
        self.shared
            .command
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
    }
}

/// A motion command by name (passive, stand, rl_control, lie_down, joint_damping).
pub fn parse_target(name: &str) -> Option<Target> {
    serde_json::from_value(serde_json::Value::String(name.trim().to_string())).ok()
}

/// The ImuSample of a sensor_msgs/Imu in base_link's axes (orientation w, x, y, z; rad/s; m/s^2).
pub fn imu_sample(msg: &Imu) -> ImuSample {
    let q = &msg.orientation;
    let (g, a) = (&msg.angular_velocity, &msg.linear_acceleration);
    ImuSample {
        quat: [q.w, q.x, q.y, q.z],
        gyro: [g.x, g.y, g.z],
        accel: [a.x, a.y, a.z],
        stamp: msg.header.stamp.sec as f64 + msg.header.stamp.nsec as f64 * 1e-9,
    }
}

/// The owner's guard: a reply showing a torque or speed above the limits (or not a number), or, once the startup is
/// through, a status word other than 1 (or no joints). Returns the largest |torque| and |speed| seen, and whether to stop.
pub fn guard(
    joints: &[m20_ecan::JointState],
    max_torque: f64,
    max_velocity: f64,
    in_startup: bool,
) -> (f64, f64, bool) {
    // f32::max drops NaN, so a non-finite reading counts as infinite
    let largest = |values: &mut dyn Iterator<Item = f32>| {
        values
            .map(|v| {
                if v.is_finite() {
                    v.abs()
                } else {
                    f32::INFINITY
                }
            })
            .fold(0.0f32, f32::max) as f64
    };
    let torque = largest(&mut joints.iter().map(|j| j.torque));
    let velocity = largest(&mut joints.iter().map(|j| j.velocity));
    // a reply with no joint records says nothing about the joints; a missing joint state is the stale-state fault's job
    let status_bad = !in_startup
        && joints
            .iter()
            .any(|j| j.status_word != m20_ecan::joint::STATUS_OK);
    (
        torque,
        velocity,
        torque > max_torque || velocity > max_velocity || status_bad,
    )
}

/// The one-time setup that grants the controller CAP_NET_RAW (motor bus) and SCHED_FIFO (CAP_SYS_NICE, LimitRTPRIO).
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
const REALTIME_FIX: &str =
    "on AOS run `m20tool setup-step aos/dimos-host-realtime`, then restart dimos-host";

/// What to run when the motor bus is refused for want of a permission; "" otherwise.
#[cfg_attr(not(target_os = "linux"), allow(dead_code))]
fn setup_fix(error: &str) -> &'static str {
    if error.contains("Operation not permitted") || error.contains("Permission denied") {
        REALTIME_FIX
    } else {
        ""
    }
}

/// Publish whatever the channel holds, as often as the output keeps up.
fn forward<T: Clone + Send + Sync + 'static>(
    mut latest: watch::Receiver<Option<T>>,
    output: Output<T>,
    name: &'static str,
) {
    tokio::spawn(async move {
        while latest.changed().await.is_ok() {
            let message = latest.borrow_and_update().clone();
            if let Some(message) = message {
                if let Err(error) = output.publish(&message).await {
                    warn!(%error, stream = name, "cannot publish");
                }
            }
        }
    });
}

/// The latest message per output, handed to the publishing tasks.
struct Latest {
    joint_state: watch::Sender<Option<JointState>>,
    joint_command: watch::Sender<Option<JointState>>,
    battery: watch::Sender<Option<BatteryState>>,
    left: watch::Sender<Option<BatteryState>>,
    right: watch::Sender<Option<BatteryState>>,
    handler: FrameHandler,
}

impl Latest {
    fn see(&self, timestamp_ns: u64, direction: Direction, frame: &[u8]) {
        match self.handler.handle(timestamp_ns, direction, frame) {
            Ok(Some(Published::JointState(m))) => drop(self.joint_state.send_replace(Some(m))),
            Ok(Some(Published::JointCommand(m))) => drop(self.joint_command.send_replace(Some(m))),
            Ok(Some(Published::Battery(b))) => {
                let Batteries {
                    left,
                    right,
                    combined,
                } = *b;
                self.left.send_replace(Some(left));
                self.right.send_replace(Some(right));
                self.battery.send_replace(Some(combined));
            }
            Ok(None) => {}
            Err(error) => warn!(%error, "bad motor-bus frame"),
        }
    }
}

/// What the policy loop publishes on a state change and once per second of ticks.
#[derive(Clone, PartialEq)]
struct Status {
    ready: bool,
    state: String,
}

impl M20Controller {
    async fn start(&mut self) {
        let config = self.config.clone();
        self.current.gait = config.gait;
        let (joint_state, joint_state_rx) = watch::channel(None);
        let (joint_command, joint_command_rx) = watch::channel(None);
        let (battery, battery_rx) = watch::channel(None);
        let (left, left_rx) = watch::channel(None);
        let (right, right_rx) = watch::channel(None);
        let (status, status_rx) = watch::channel(None::<Status>);
        forward(joint_state_rx, self.joint_state.clone(), "joint_state");
        forward(
            joint_command_rx,
            self.joint_command.clone(),
            "joint_command",
        );
        forward(battery_rx, self.battery.clone(), "battery");
        forward(left_rx, self.battery_left.clone(), "battery_left");
        forward(right_rx, self.battery_right.clone(), "battery_right");
        forward_status(
            status_rx,
            self.command_ready.clone(),
            self.controller_state.clone(),
        );
        let (dock_status, dock_status_rx) = watch::channel(None::<std_msgs::String>);
        forward(dock_status_rx, self.dock_state.clone(), "dock_state");
        let (body_height, body_height_rx) = watch::channel(None::<Float32>);
        forward(body_height_rx, self.body_height.clone(), "body_height");
        if config.policy && !config.charge_manager_toml.is_empty() {
            match ChargeManagerToml::load(&config.charge_manager_toml) {
                Ok((t, _)) => {
                    info!(path = %config.charge_manager_toml, "charge_manager.toml read");
                    self.shared.vendor_read.store(true, Ordering::Relaxed);
                    *self
                        .shared
                        .dock
                        .vendor
                        .lock()
                        .unwrap_or_else(|p| p.into_inner()) = Some(t);
                }
                Err(error) => info!(
                    error = format!("{error:#}"),
                    "no charge_manager.toml on this board; docking waits for the dock detector module's copy"
                ),
            }
        }
        let latest = Latest {
            joint_state,
            joint_command,
            battery,
            left,
            right,
            handler: FrameHandler::new(&config.frame_id),
        };
        if config.policy && config.mode != "owner" {
            warn!("policy needs mode = owner (the controller owns the bus); running the tap without it");
        }
        let shared = self.shared.clone();
        let spawned = if config.mode == "owner" && config.policy {
            std::thread::Builder::new()
                .name("m20-controller".into())
                .spawn(move || {
                    run_policy(&config, latest, shared, status, dock_status, body_height)
                })
        } else if config.mode == "owner" {
            std::thread::Builder::new()
                .name("m20-ecan-owner".into())
                .spawn(move || run_owner(&config, &latest))
        } else {
            let mut tap = match TapSocket::bind(&config.socket_path) {
                Ok(tap) => tap,
                Err(error) => {
                    warn!(%error, path = %config.socket_path, "cannot bind the tap socket; no joint or battery data");
                    return;
                }
            };
            info!(path = %config.socket_path, "waiting for the frames rl_deploy's shim streams");
            let _ = tap.set_read_timeout(Some(Duration::from_secs_f64(config.silence_warning_s)));
            std::thread::Builder::new()
                .name("m20-ecan-tap".into())
                .spawn(move || run_tap(&config, &mut tap, &latest))
        };
        match spawned {
            Ok(thread) => self.thread = Some(thread),
            Err(error) => warn!(%error, "cannot start the motor-bus thread"),
        }
    }

    /// Policy mode: the loop sends its shutdown damping and returns; the other modes run until the process ends.
    async fn stop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if self.config.policy && self.config.mode == "owner" {
            if let Some(thread) = self.thread.take() {
                let _ = tokio::task::spawn_blocking(move || thread.join()).await;
            }
        }
    }

    fn send_command(&self) {
        *self
            .shared
            .command
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(self.current.clone());
    }

    async fn on_cmd_vel(&mut self, msg: Twist) {
        self.current.velocity = Some([msg.linear.x, msg.linear.y, msg.angular.z]);
        self.send_command();
    }

    async fn on_motion_cmd(&mut self, msg: std_msgs::String) {
        match parse_target(&msg.data) {
            Some(target) => {
                info!(motion = %msg.data, "motion command");
                self.current.target = target;
                self.current.motion_seq += 1;
                // a state change starts from standing still
                self.current.velocity = None;
                self.send_command();
            }
            None => {
                warn!(motion = %msg.data, "unknown motion command (passive, stand, rl_control, lie_down, joint_damping)")
            }
        }
    }

    async fn on_gait_cmd(&mut self, msg: UInt32) {
        info!(gait = format!("{:#x}", msg.data as u32), "gait command");
        self.current.gait = msg.data as u32;
        self.current.motion_seq += 1;
        self.current.velocity = None;
        self.send_command();
    }

    async fn on_dock_pose(&mut self, msg: PoseStamped) {
        let q = &msg.pose.orientation;
        let pose = DockPose {
            x: msg.pose.position.x,
            y: msg.pose.position.y,
            yaw: (2.0 * (q.w * q.z + q.x * q.y)).atan2(1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
        };
        if ![pose.x, pose.y, pose.yaw].iter().all(|v| v.is_finite()) {
            warn!(?pose, "non-finite dock pose; ignored");
            return;
        }
        let seq = self.shared.dock_poses.fetch_add(1, Ordering::Relaxed) + 1;
        *self
            .shared
            .dock
            .pose
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some((pose, seq));
    }

    async fn on_dock_obstacle(&mut self, msg: Float32) {
        let seq = self.shared.dock_obstacles.fetch_add(1, Ordering::Relaxed) + 1;
        let reading = (msg.data >= 0.0).then_some(msg.data);
        *self
            .shared
            .dock
            .obstacle
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some((reading, seq));
    }

    async fn on_charge_manager_config(&mut self, msg: std_msgs::String) {
        if self.shared.vendor_read.load(Ordering::Relaxed) {
            return;
        }
        match ChargeManagerToml::parse(&msg.data) {
            Ok(t) => {
                info!("charge_manager.toml from the dock detector module");
                self.shared.vendor_read.store(true, Ordering::Relaxed);
                *self
                    .shared
                    .dock
                    .vendor
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = Some(t);
            }
            Err(error) => warn!(
                error = format!("{error:#}"),
                "bad charge_manager.toml from the dock detector module"
            ),
        }
    }

    async fn on_dock_cmd(&mut self, msg: std_msgs::String) {
        match Request::parse(&msg.data) {
            Some(request) => {
                info!(?request, "dock command");
                if !(self.config.mode == "owner" && self.config.policy) {
                    warn!("docking needs mode = owner with the policy on; ignored");
                    return;
                }
                *self
                    .shared
                    .dock
                    .request
                    .lock()
                    .unwrap_or_else(|p| p.into_inner()) = Some(request);
            }
            None => warn!(command = %msg.data, "unknown dock command (dock, undock, cancel)"),
        }
    }

    async fn on_imu(&mut self, msg: Imu) {
        *self.shared.imu.lock().unwrap_or_else(|p| p.into_inner()) =
            Some((imu_sample(&msg), std::time::Instant::now()));
    }
}

fn forward_status(
    mut latest: watch::Receiver<Option<Status>>,
    ready: Output<Bool>,
    state: Output<std_msgs::String>,
) {
    tokio::spawn(async move {
        while latest.changed().await.is_ok() {
            let Some(status) = latest.borrow_and_update().clone() else {
                continue;
            };
            let results = [
                ready.publish(&Bool { data: status.ready }).await,
                state
                    .publish(&std_msgs::String { data: status.state })
                    .await,
            ];
            for error in results.into_iter().filter_map(Result::err) {
                warn!(%error, "cannot publish the controller state");
            }
        }
    });
}

#[cfg(target_os = "linux")]
fn run_policy(
    config: &Config,
    latest: Latest,
    shared: Arc<Shared>,
    status: watch::Sender<Option<Status>>,
    dock_status: watch::Sender<Option<std_msgs::String>>,
    body_height: watch::Sender<Option<Float32>>,
) {
    use crate::dock::docking::{Docking, DockingCommands};
    use crate::dock::obstacle;
    use crate::hardware::bus::{BusHardware, Sink};
    use crate::policy::{RunnerSet, ThreadedPolicy};
    use crate::rt::{self, Pacing};
    use crate::state_machine::{Controller, State};
    let cfg = match controller_config(config) {
        Ok(cfg) => cfg,
        Err(error) => {
            warn!(
                error = format!("{error:#}"),
                "bad controller config; not owning the bus"
            );
            return;
        }
    };
    let policy_dir = if config.policy_dir.is_empty() {
        std::path::PathBuf::from(std::env::var("HOME").unwrap_or_default())
            .join(".cache/dimos/m20_policies")
    } else {
        std::path::PathBuf::from(&config.policy_dir)
    };
    let runners = match RunnerSet::load(&cfg, &policy_dir) {
        Ok(runners) => runners,
        Err(error) => {
            warn!(error = format!("{error:#}"), dir = %policy_dir.display(), "cannot load the policies; not owning the bus");
            return;
        }
    };
    let rt_cfg = cfg.realtime.clone();
    let mut policy = match ThreadedPolicy::spawn(runners, move || {
        if let Err(error) =
            rt::apply_realtime(rt_cfg.policy_cpu, rt_cfg.policy_fifo_priority, false)
        {
            tracing::error!(
                error = format!("{error:#}"),
                fix = REALTIME_FIX,
                "the policy thread is not real-time; refusing to run"
            );
            std::process::exit(1);
        }
    }) {
        Ok(policy) => policy,
        Err(error) => {
            warn!(
                error = format!("{error:#}"),
                "cannot start the policy thread"
            );
            return;
        }
    };
    // the guard on every reply; status words count once the startup is through (set from the loop)
    let startup_done = Arc::new(AtomicBool::new(false));
    let tripped = Arc::new(AtomicBool::new(false));
    let (guard_shared, guard_startup, guard_tripped) =
        (shared.clone(), startup_done.clone(), tripped.clone());
    let charge_feed = shared.dock.clone();
    let mut battery_reports = 0u64;
    let (max_torque, max_velocity) = (config.guard_max_torque, config.guard_max_velocity);
    let sink: Sink = Box::new(move |t, direction, frame| {
        if let Ok((_, m20_ecan::Message::JointStates { joints, .. })) =
            m20_ecan::decode_ethernet(frame, direction)
        {
            let (torque, velocity, stop) = guard(
                &joints,
                max_torque,
                max_velocity,
                !guard_startup.load(Ordering::Relaxed),
            );
            if stop && !guard_tripped.swap(true, Ordering::Relaxed) {
                let statuses: Vec<(String, u16)> = joints
                    .iter()
                    .map(|j| (format!("{:?}", j.joint), j.status_word))
                    .collect();
                tracing::error!(
                    torque,
                    velocity,
                    ?statuses,
                    "guard: stopping the controller"
                );
                guard_shared.stop.store(true, Ordering::Relaxed);
            }
        }
        if let Some(reading) = charge_reading(direction, frame, battery_reports + 1) {
            battery_reports += 1;
            if let Ok(mut slot) = charge_feed.charge.try_lock() {
                *slot = Some(reading);
            }
        }
        latest.see(t, direction, frame);
    });
    let imu = SharedImu {
        shared: shared.clone(),
        timeout: Duration::from_secs_f64(config.imu_timeout_s),
    };
    let owner = m20_ecan::owner::OwnerConfig {
        allow_gains: cfg.ecan.allow_gains,
    };
    let mut hardware = match BusHardware::open(
        &cfg.ecan.interface,
        owner,
        cfg.ecan.charge_enable,
        cfg.ecan.reply_timeout_s,
        Box::new(imu),
        sink,
    ) {
        Ok(hardware) => hardware,
        Err(error) => {
            warn!(
                error = format!("{error:#}"),
                fix = setup_fix(&format!("{error:#}")),
                "cannot own the motor bus"
            );
            return;
        }
    };
    if let Err(error) = rt::control_realtime(&cfg.realtime) {
        tracing::error!(
            error = format!("{error:#}"),
            fix = REALTIME_FIX,
            "the 1 kHz loop is not real-time; refusing to run"
        );
        std::process::exit(1);
    }
    info!(
        interface = %cfg.ecan.interface,
        allow_gains = cfg.ecan.allow_gains,
        "owning the motor bus with the walking policy: rl_deploy's startup, then 1 kHz"
    );
    let docking = Docking::new(&cfg, cfg.ecan.charge_enable, None);
    let mut commands = DockingCommands::new(
        SharedCommands {
            shared: shared.clone(),
        },
        docking,
        shared.dock.clone(),
        Box::new(move |s| {
            info!(phase = ?s.phase, result = %s.result, error = s.error, charging = s.charging, "docking");
            let data = serde_json::to_string(s).unwrap_or_default();
            dock_status.send_replace(Some(std_msgs::String { data }));
        }),
    );
    let mut controller = Controller::new(cfg.clone());
    let status_every = (cfg.timing.control_hz.round() as u64).max(1);
    let mut last: Option<Status> = None;
    let mut last_height = f64::NAN;
    let result = rt::run(
        &cfg,
        &mut hardware,
        &mut controller,
        &mut policy,
        &mut commands,
        Pacing::Realtime,
        &shared.stop,
        |hw, tick| {
            startup_done.store(!hw.in_startup(), Ordering::Relaxed);
            let height =
                obstacle::body_height(&cfg.robot, &tick.state.q, cfg.dock.obstacle.wheel_radius_m);
            if !((height - last_height).abs() < cfg.dock.obstacle.body_height_step_m) {
                last_height = height;
                body_height.send_replace(Some(Float32 {
                    data: height as f32,
                }));
            }
            let c = tick.controller;
            let now = Status {
                ready: c.state == State::RlControl && c.fault.is_none(),
                state: match &c.fault {
                    Some(fault) => format!("{:?} (fault: {fault})", c.state),
                    None => format!("{:?}", c.state),
                },
            };
            if last.as_ref() != Some(&now) || tick.stats.ticks % status_every == 0 {
                if last.as_ref() != Some(&now) {
                    info!(state = %now.state, runner = ?c.current_runner(), "controller state");
                }
                status.send_replace(Some(now.clone()));
                last = Some(now);
            }
        },
    );
    match result {
        Ok(stats) => info!(?stats, "policy loop stopped"),
        Err(error) => warn!(error = format!("{error:#}"), "policy loop failed"),
    }
    if tripped.load(Ordering::Relaxed) {
        std::process::exit(2);
    }
}

/// The packs' charge state from a battery frame off the board, as docking reads it.
pub fn charge_reading(direction: Direction, frame: &[u8], seq: u64) -> Option<ChargeReading> {
    let (_, message) = m20_ecan::decode_ethernet(frame, direction).ok()?;
    if !matches!(message, m20_ecan::Message::Battery(_)) {
        return None;
    }
    let report = m20_battery::BatteryReport::from_message(&message).ok()?;
    let packs = report.packs();
    let present: Vec<_> = packs.iter().filter(|p| p.voltage_v > 0.0).collect();
    Some(ChargeReading {
        seq,
        packs_present: present.len() as u8,
        packs_charge_mos_on: present.iter().filter(|p| p.charge_mos_on()).count() as u8,
        charger_connected: present
            .iter()
            .any(|p| m20_battery::charge::charger_connected(p)),
    })
}

#[cfg(not(target_os = "linux"))]
fn run_policy(
    _: &Config,
    _: Latest,
    _: Arc<Shared>,
    _: watch::Sender<Option<Status>>,
    _: watch::Sender<Option<std_msgs::String>>,
    _: watch::Sender<Option<Float32>>,
) {
    warn!("the policy on the motor bus needs Linux (the vendor motor-bus socket)");
}

/// The controller config in force: the TOML with the module's own fields over it.
pub fn controller_config(config: &Config) -> anyhow::Result<crate::config::Config> {
    let mut cfg = load_controller_config(&config.controller_config)?;
    cfg.ecan.interface = config.interface.clone();
    cfg.ecan.charge_enable = config.charge_enable;
    cfg.ecan.allow_gains = config.allow_gains;
    if config.command_timeout_s > 0.0 {
        cfg.safety.command_timeout_s = config.command_timeout_s;
    }
    Ok(cfg)
}

/// The controller's m20_controller.toml: the built-in one, or a path.
pub fn load_controller_config(path: &str) -> anyhow::Result<crate::config::Config> {
    if path.is_empty() {
        let cfg: crate::config::Config = toml::from_str(BUILT_IN_CONFIG)?;
        cfg.validate()?;
        Ok(cfg)
    } else {
        crate::config::Config::load(std::path::Path::new(path))
    }
}

fn run_tap(config: &Config, tap: &mut TapSocket, latest: &Latest) {
    let (mut ring_dropped, mut send_dropped, mut receiving) = (0u32, 0u32, false);
    loop {
        let records = match tap.recv() {
            Ok(records) => records,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                warn!(
                    seconds = config.silence_warning_s,
                    "no motor-bus frames; is rl_deploy running with the ecan_tap.so shim?"
                );
                receiving = false;
                continue;
            }
            Err(error) => {
                warn!(%error, "bad tap datagram");
                continue;
            }
        };
        if !receiving {
            info!("motor-bus frames arriving");
            receiving = true;
        }
        for TapRecord {
            timestamp_ns,
            direction,
            ring_dropped: ring,
            send_dropped: send,
            frame,
        } in records
        {
            if ring > ring_dropped || send > send_dropped {
                warn!(
                    ring_dropped = ring,
                    send_dropped = send,
                    "the stream dropped frames (its ring or this socket was full)"
                );
                (ring_dropped, send_dropped) = (ring, send);
            }
            latest.see(timestamp_ns, direction, &frame);
        }
    }
}

#[cfg(target_os = "linux")]
fn run_owner(config: &Config, latest: &Latest) {
    use m20_battery::charge::BusCharge;
    use m20_ecan::bus::Bus;
    use m20_ecan::owner::OwnerConfig;
    // zero gains only: this module has no command source and never lets a gain through
    let mut bus = match Bus::open(&config.interface, OwnerConfig { allow_gains: false }) {
        Ok(bus) => bus,
        Err(error) => {
            warn!(%error, fix = setup_fix(&error.to_string()), "cannot own the motor bus");
            return;
        }
    };
    info!(interface = %config.interface, "owning the motor bus: rl_deploy's startup, then the zero-gain sitting command at 1 kHz");
    let mut charge = config.charge_enable.then(BusCharge::new);
    let silence = Duration::from_secs_f64(config.silence_warning_s);
    let mut last_reply = std::time::Instant::now();
    let mut started = false;
    let mut last_statuses: Option<std::collections::BTreeSet<u16>> = None;
    let mut replies = 0u64;
    let mut send_failing = false;
    loop {
        match bus.tick(None) {
            Ok(sent) => {
                if std::mem::take(&mut send_failing) {
                    info!("motor-bus sends work again");
                }
                if let Some(sent) = sent {
                    latest.see(sent.timestamp_ns, sent.direction, &sent.frame);
                    if let Some(charge) = charge.as_mut() {
                        charge.on_frame(sent.direction, &sent.frame);
                    }
                }
            }
            // once per failing stretch, not 1000 times a second
            Err(error) if !send_failing => {
                warn!(%error, "motor-bus send failed");
                send_failing = true;
            }
            Err(_) => {}
        }
        if !started && !bus.owner.in_startup() {
            info!("startup sent; polling");
            started = true;
        }
        for frame in bus.drain() {
            last_reply = std::time::Instant::now();
            if let Ok((_, m20_ecan::Message::JointStates { joints, .. })) =
                m20_ecan::decode_ethernet(&frame.frame, frame.direction)
            {
                let statuses: std::collections::BTreeSet<u16> =
                    joints.iter().map(|j| j.status_word).collect();
                let (torque, velocity, stop) = guard(
                    &joints,
                    config.guard_max_torque,
                    config.guard_max_velocity,
                    bus.owner.in_startup(),
                );
                if last_statuses.as_ref() != Some(&statuses) {
                    info!(
                        ?statuses,
                        startup = bus.owner.in_startup(),
                        "joint status words"
                    );
                    last_statuses = Some(statuses.clone());
                }
                replies += 1;
                if stop {
                    tracing::error!(
                        torque,
                        velocity,
                        ?statuses,
                        replies,
                        "guard: stopping the bus owner"
                    );
                    std::process::exit(2);
                }
                if replies.is_multiple_of(config.progress_log_replies) {
                    let (receive_errors, last_receive_error) = bus.receive_errors();
                    info!(
                        replies,
                        refused = bus.owner.refused,
                        receive_errors,
                        last_receive_error = ?last_receive_error,
                        max_torque = torque,
                        max_velocity = velocity,
                        "polling"
                    );
                }
            }
            latest.see(frame.timestamp_ns, frame.direction, &frame.frame);
            if let Some(charge) = charge.as_mut() {
                if charge.on_frame(frame.direction, &frame.frame) {
                    info!(decision = ?charge.last, "charge-enable request");
                    bus.owner.queue_charge_enable();
                }
            }
        }
        if last_reply.elapsed() > silence {
            warn!(
                seconds = config.silence_warning_s,
                "no replies from the motor board"
            );
            last_reply = std::time::Instant::now();
        }
    }
}

#[cfg(not(target_os = "linux"))]
fn run_owner(_: &Config, _: &Latest) {
    warn!("owner mode needs Linux (the vendor motor-bus socket)");
}

#[cfg(test)]
mod tests {
    use super::*;
    use m20_ecan::{JointId, JointState};

    fn joints(torque: f32, velocity: f32, status_word: u16) -> Vec<JointState> {
        JointId::all()
            .map(|joint| JointState {
                board_id: 0x79,
                joint,
                position: 0.0,
                velocity,
                torque,
                motor_temperature_c: 30.0,
                driver_temperature_c: 30.0,
                status_word,
                data_id: 0,
            })
            .collect()
    }

    #[test]
    fn the_built_in_config_loads_with_gains_off() {
        let cfg = load_controller_config("").unwrap();
        assert!(!cfg.ecan.allow_gains);
    }

    fn module_config(command_timeout_s: f64) -> Config {
        let fields = serde_json::json!({
            "mode": "owner", "socket_path": "", "interface": "eth1", "charge_enable": false,
            "frame_id": "base_link", "guard_max_torque": 1.0, "guard_max_velocity": 0.5,
            "silence_warning_s": 5.0, "progress_log_replies": 5000, "policy": true, "allow_gains": false,
            "controller_config": "", "policy_dir": "", "imu_timeout_s": 0.05,
            "command_timeout_s": command_timeout_s, "gait": 0x3002, "charge_manager_toml": "",
        });
        serde_json::from_value(fields).unwrap()
    }

    #[test]
    fn command_timeout_overrides_the_toml_only_when_set() {
        let toml = load_controller_config("").unwrap().safety.command_timeout_s;
        let unset = controller_config(&module_config(0.0)).unwrap();
        assert_eq!(unset.safety.command_timeout_s, toml);
        let set = controller_config(&module_config(0.3)).unwrap();
        assert_eq!(set.safety.command_timeout_s, 0.3);
        assert!(!set.ecan.allow_gains);
    }

    /// A battery frame off the board with both packs at `mos` (bit 0 = charge MOS), charge-detect `detect`.
    fn battery_frame(mos: u8, detect: u8) -> Vec<u8> {
        let mut payload = Vec::new();
        for _ in 0..2 {
            let mut record = [0u8; m20_ecan::message::BATTERY_PACK_LEN];
            record[0..2].copy_from_slice(&8300u16.to_le_bytes());
            record[20] = mos;
            record[112..114].copy_from_slice(&8400u16.to_le_bytes());
            record[114] = detect;
            payload.extend_from_slice(&record);
        }
        let body =
            m20_ecan::frame::encode(0, 6, 0x21, m20_ecan::FrameType::Battery, &payload).unwrap();
        m20_ecan::ethernet::wrap([2, 0, 0, 0, 0, 1], &body)
    }

    #[test]
    fn docking_reads_each_packs_charge_mos_from_the_battery_frames() {
        let charging = charge_reading(Direction::FromBoard, &battery_frame(3, 0), 7).unwrap();
        assert_eq!(
            (
                charging.seq,
                charging.packs_present,
                charging.packs_charge_mos_on,
                charging.charger_connected
            ),
            (7, 2, 2, true)
        );
        let off = charge_reading(Direction::FromBoard, &battery_frame(2, 1), 8).unwrap();
        assert_eq!(off.packs_charge_mos_on, 0);
        assert!(charge_reading(Direction::FromBoard, &[0u8; 20], 9).is_none());
    }

    #[test]
    fn motion_commands_by_name() {
        assert_eq!(parse_target("stand"), Some(Target::Stand));
        assert_eq!(parse_target(" lie_down "), Some(Target::LieDown));
        assert_eq!(parse_target("rl_control"), Some(Target::RlControl));
        assert_eq!(parse_target("walk"), None);
    }

    #[test]
    fn guard_trips_on_limits_status_and_nan() {
        assert!(!guard(&joints(0.1, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(1.5, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, -0.6, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, 0.1, 0x21), 1.0, 0.5, false).2);
        assert!(!guard(&joints(0.1, 0.1, 0x21), 1.0, 0.5, true).2);
        // a NaN reading used to fold away to 0 and pass
        assert!(guard(&joints(f32::NAN, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, f32::INFINITY, 1), 1.0, 0.5, false).2);
        assert!(!guard(&[], 1.0, 0.5, false).2);
    }
}
