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
//! The M20's joint state, rl_deploy's joint commands and both battery packs, from the motor-bus frames the
//! `ecan_tap.so` shim streams out of rl_deploy. Receive-only: nothing here can send to the motor bus.
//!
//! The board answers only rl_deploy's 1 kHz commands (with rl_deploy stopped, eth1 receives nothing), so this module
//! needs rl_deploy running with the shim installed (`record.sh install` on AOS). Every reply is published as it
//! arrives; a publish still in flight when newer data lands makes the older data be skipped (latest wins).

use std::time::Duration;

use dimos_module::{native_config, Module, Output};
use lcm_msgs::sensor_msgs::{BatteryState, JointState};
use m20_battery::BatteryReport;
use m20_ecan::tap::{TapRecord, TapSocket};
use m20_ecan::{decode_ethernet, Calibration, Message};
use tokio::sync::watch;
use tracing::{info, warn};

use crate::convert;

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Unix datagram socket the shim inside rl_deploy streams to; this module binds it.
    socket_path: String,
    /// frame_id of every header.
    frame_id: String,
    /// Seconds without a frame before warning that rl_deploy or its shim is not running.
    #[validate(range(exclusive_min = 0.0, max = 3600.0))]
    silence_warning_s: f64,
}

#[derive(Module)]
#[module(name = "m20_ecan", setup = start)]
pub struct M20Ecan {
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

    #[config]
    config: Config,
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

impl M20Ecan {
    async fn start(&mut self) {
        let config = self.config.clone();
        let (joint_state, joint_state_rx) = watch::channel(None);
        let (joint_command, joint_command_rx) = watch::channel(None);
        let (battery, battery_rx) = watch::channel(None);
        let (left, left_rx) = watch::channel(None);
        let (right, right_rx) = watch::channel(None);
        forward(joint_state_rx, self.joint_state.clone(), "joint_state");
        forward(
            joint_command_rx,
            self.joint_command.clone(),
            "joint_command",
        );
        forward(battery_rx, self.battery.clone(), "battery");
        forward(left_rx, self.battery_left.clone(), "battery_left");
        forward(right_rx, self.battery_right.clone(), "battery_right");
        let mut tap = match TapSocket::bind(&config.socket_path) {
            Ok(tap) => tap,
            Err(error) => {
                warn!(%error, path = %config.socket_path, "cannot bind the tap socket; no joint or battery data");
                return;
            }
        };
        info!(path = %config.socket_path, "waiting for rl_deploy's ecan_tap.so shim");
        let silence = Duration::from_secs_f64(config.silence_warning_s);
        let _ = tap.set_read_timeout(Some(silence));
        std::thread::Builder::new()
            .name("m20-ecan-tap".into())
            .spawn(move || {
                let calibration = Calibration::m20();
                let frame_id = config.frame_id.as_str();
                let (mut ring_dropped, mut send_dropped, mut receiving) = (0u32, 0u32, false);
                loop {
                    let records = match tap.recv() {
                        Ok(records) => records,
                        Err(error) if matches!(error.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                            warn!(seconds = config.silence_warning_s, "no motor-bus frames; is rl_deploy running with the ecan_tap.so shim?");
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
                    for TapRecord { timestamp_ns, direction, ring_dropped: ring, send_dropped: send, frame } in records {
                        if ring > ring_dropped || send > send_dropped {
                            warn!(ring_dropped = ring, send_dropped = send, "the shim dropped frames (its ring or this socket was full)");
                            (ring_dropped, send_dropped) = (ring, send);
                        }
                        match decode_ethernet(&frame, direction) {
                            Ok((_, Message::JointStates { joints, .. })) => {
                                joint_state.send_replace(Some(convert::measured(&joints, &calibration, timestamp_ns, frame_id)));
                            }
                            Ok((_, Message::JointCommands(commands))) => {
                                joint_command.send_replace(Some(convert::commanded(&commands, &calibration, timestamp_ns, frame_id)));
                            }
                            Ok((_, message @ Message::Battery(_))) => match BatteryReport::from_message(&message) {
                                Ok(report) => {
                                    left.send_replace(Some(convert::pack_state(&report.left, "left", timestamp_ns, frame_id)));
                                    right.send_replace(Some(convert::pack_state(&report.right, "right", timestamp_ns, frame_id)));
                                    battery.send_replace(Some(convert::combined_state(&[&report.left, &report.right], timestamp_ns, frame_id)));
                                }
                                Err(error) => warn!(%error, "bad battery frame"),
                            },
                            Ok(_) => {}
                            Err(error) => warn!(%error, "bad motor-bus frame"),
                        }
                    }
                }
            })
            .expect("spawn the tap reader");
    }
}
