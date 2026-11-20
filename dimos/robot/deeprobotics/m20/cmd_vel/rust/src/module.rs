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
//! dimos `cmd_vel` to rl_deploy, the M20's vendor walking controller, on its own REAL_STEER topic: the path the
//! vendor's NAV_CMD -> basic_server -> REAL_STEER chain takes, without basic_server or the drdds-zenoh-bridge (rustdds
//! talks RTPS to rl_deploy's Fast-DDS participant directly). Limits and gating are the vendor bridge's m20 mode, all
//! config: each axis clamped, only while rl_deploy reports RL control on a fresh MOTION_INFO and HES_STATUS is clear,
//! zero otherwise, a final zero when commands stop. Sends no gait or motion-state command: standing up and choosing a
//! gait stay with the vendor remote or app. Also publishes command_ready, motion_state and gait_state, as the bridge did.

use std::time::Duration;

use dimos_module::{native_config, Input, Module, Output};
use lcm_msgs::geometry_msgs::Twist;
use lcm_msgs::std_msgs::{Bool, Int32, UInt32};
use tokio::sync::watch;
use tracing::warn;

use crate::dds::{self, Status, Topics};
use crate::gate::{GateConfig, Limits};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// DDS domain rl_deploy is on.
    #[validate(range(max = 232))]
    domain_id: u16,
    /// rl_deploy's metric velocity topic and its DDS type (basic_server writes it from NAV_CMD).
    steer_topic: String,
    steer_type: String,
    /// rl_deploy's state (~19 Hz), and the topic and type of its e-stop chain status.
    motion_info_topic: String,
    motion_info_type: String,
    hes_status_topic: String,
    hes_status_type: String,
    /// Clamp per axis: m/s forward, m/s left, rad/s counterclockwise (vendor bridge: 2.0, 1.0, 2.0).
    #[validate(range(min = 0.0))]
    max_linear_x: f64,
    #[validate(range(min = 0.0))]
    max_linear_y: f64,
    #[validate(range(min = 0.0))]
    max_angular_z: f64,
    /// Seconds without a cmd_vel before the final zero (vendor bridge: 0.4).
    #[validate(range(exclusive_min = 0.0))]
    command_timeout_s: f64,
    /// Seconds without MOTION_INFO before rl_deploy counts as not ready (vendor bridge: 2.5).
    #[validate(range(exclusive_min = 0.0))]
    state_timeout_s: f64,
    /// The MOTION_INFO state that takes velocity (17, RL control).
    rl_control_state: i32,
    /// Write nothing while another REAL_STEER writer (basic_server) is up, so two never interleave.
    exclusive: bool,
    /// Seconds teardown waits for rl_deploy to acknowledge the final zero.
    #[validate(range(min = 0.0))]
    release_ack_wait_s: f64,
}

#[derive(Module)]
#[module(name = "m20_cmd_vel", setup = start, teardown = stop)]
pub struct M20CmdVel {
    #[input(decode = Twist::decode, handler = on_cmd_vel)]
    cmd_vel: Input<Twist>,

    #[output(encode = Bool::encode)]
    command_ready: Output<Bool>,
    #[output(encode = Int32::encode)]
    motion_state: Output<Int32>,
    #[output(encode = UInt32::encode)]
    gait_state: Output<UInt32>,

    #[config]
    config: Config,

    commands: Option<watch::Sender<Option<(f64, f64, f64)>>>,
    stop_tx: Option<watch::Sender<bool>>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl M20CmdVel {
    async fn start(&mut self) {
        let config = self.config.clone();
        let topics = Topics {
            domain_id: config.domain_id,
            steer_topic: config.steer_topic.clone(),
            steer_type: config.steer_type.clone(),
            motion_info_topic: config.motion_info_topic.clone(),
            motion_info_type: config.motion_info_type.clone(),
            hes_status_topic: config.hes_status_topic.clone(),
            hes_status_type: config.hes_status_type.clone(),
        };
        let gate = GateConfig {
            limits: Limits {
                max_linear_x: config.max_linear_x,
                max_linear_y: config.max_linear_y,
                max_angular_z: config.max_angular_z,
            },
            command_timeout: Duration::from_secs_f64(config.command_timeout_s),
            state_timeout: Duration::from_secs_f64(config.state_timeout_s),
            rl_control_state: config.rl_control_state,
        };
        let (commands, commands_rx) = watch::channel(None);
        let (status, status_rx) = watch::channel(None);
        let (stop_tx, stop_rx) = watch::channel(false);
        self.forward_status(status_rx);
        let release_ack_wait = Duration::from_secs_f64(config.release_ack_wait_s);
        // DDS on its own thread and runtime, off the module's
        let spawned = std::thread::Builder::new()
            .name("m20-cmd-vel".into())
            .spawn(move || {
                match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(runtime) => runtime.block_on(dds::run(
                        topics,
                        gate,
                        config.exclusive,
                        release_ack_wait,
                        commands_rx,
                        status,
                        stop_rx,
                    )),
                    Err(error) => warn!(%error, "cannot start the DDS runtime"),
                }
            });
        match spawned {
            Ok(thread) => self.thread = Some(thread),
            Err(error) => warn!(%error, "cannot start the command thread"),
        }
        self.commands = Some(commands);
        self.stop_tx = Some(stop_tx);
    }

    /// command_ready, motion_state and gait_state after each MOTION_INFO; one still publishing makes newer ones wait (latest wins).
    fn forward_status(&self, mut latest: watch::Receiver<Option<Status>>) {
        let (ready, motion, gait) = (
            self.command_ready.clone(),
            self.motion_state.clone(),
            self.gait_state.clone(),
        );
        tokio::spawn(async move {
            while latest.changed().await.is_ok() {
                let Some(status) = *latest.borrow_and_update() else {
                    continue;
                };
                let results = [
                    ready.publish(&Bool { data: status.ready }).await,
                    motion
                        .publish(&Int32 {
                            data: status.motion_state,
                        })
                        .await,
                    gait.publish(&UInt32 {
                        data: status.gait as i32,
                    })
                    .await,
                ];
                for error in results.into_iter().filter_map(Result::err) {
                    warn!(%error, "cannot publish rl_deploy's state");
                }
            }
        });
    }

    async fn stop(&mut self) {
        if let Some(stop) = self.stop_tx.take() {
            stop.send_replace(true);
        }
        self.commands = None;
        if let Some(thread) = self.thread.take() {
            let _ = tokio::task::spawn_blocking(move || thread.join()).await;
        }
    }

    async fn on_cmd_vel(&mut self, msg: Twist) {
        if let Some(commands) = &self.commands {
            commands.send_replace(Some((msg.linear.x, msg.linear.y, msg.angular.z)));
        }
    }
}
