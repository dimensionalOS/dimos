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
//! The M20's joint state, the joint commands on the motor bus and both battery packs. Two modes, same outputs:
//!
//! - `tap` (default): read the frames rl_deploy exchanges, from the `ecan_tap.so` shim's stream (or from the Rust
//!   controller, which streams the frames it exchanges the same way). Receive-only: nothing here sends.
//! - `owner` (rust mode, rl_deploy stopped): own the motor bus with m20-ecan's bus owner: rl_deploy's startup, then
//!   its zero-gain sitting command every 1 ms (nothing that holds or moves a joint), and the charge-enable request
//!   from m20-battery's rule while docked and lying. Refuses to start while rl_deploy runs.
//!
//! Every frame is published as it arrives; a publish still in flight when newer data lands makes the older data be
//! skipped (latest wins).

use std::time::Duration;

use dimos_module::{native_config, Module, Output};
use lcm_msgs::sensor_msgs::{BatteryState, JointState};
use m20_ecan::tap::{TapRecord, TapSocket};
use m20_ecan::Direction;
use tokio::sync::watch;
use tracing::{info, warn};

use crate::frames::{Batteries, FrameHandler, Published};

fn valid_mode(mode: &str) -> Result<(), validator::ValidationError> {
    match mode {
        "tap" | "owner" => Ok(()),
        _ => Err(validator::ValidationError::new("mode must be tap or owner")),
    }
}

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// `tap` (read rl_deploy's frames through the shim) or `owner` (own the bus, zero gains only).
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
    /// owner: stop owning the bus (the process exits) on a reply with any joint torque (N m) or speed (rad/s) above
    /// these, or, once the startup is through, any status word other than 1; a passive owner never moves anything.
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
    let status_bad = !in_startup
        && (joints.is_empty()
            || joints
                .iter()
                .any(|j| j.status_word != m20_ecan::joint::STATUS_OK));
    (
        torque,
        velocity,
        torque > max_torque || velocity > max_velocity || status_bad,
    )
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
        let latest = Latest {
            joint_state,
            joint_command,
            battery,
            left,
            right,
            handler: FrameHandler::new(&config.frame_id),
        };
        let spawned = if config.mode == "owner" {
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
            info!(path = %config.socket_path, "waiting for the frames of whoever owns the motor bus (rl_deploy's shim or the Rust controller)");
            let _ = tap.set_read_timeout(Some(Duration::from_secs_f64(config.silence_warning_s)));
            std::thread::Builder::new()
                .name("m20-ecan-tap".into())
                .spawn(move || run_tap(&config, &mut tap, &latest))
        };
        if let Err(error) = spawned {
            warn!(%error, "cannot start the motor-bus thread");
        }
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
            warn!(%error, "cannot own the motor bus");
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
                if replies % config.progress_log_replies == 0 {
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
    fn guard_trips_on_limits_status_and_nan() {
        assert!(!guard(&joints(0.1, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(1.5, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, -0.6, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, 0.1, 0x21), 1.0, 0.5, false).2);
        assert!(!guard(&joints(0.1, 0.1, 0x21), 1.0, 0.5, true).2);
        // a NaN reading used to fold away to 0 and pass
        assert!(guard(&joints(f32::NAN, 0.1, 1), 1.0, 0.5, false).2);
        assert!(guard(&joints(0.1, f32::INFINITY, 1), 1.0, 0.5, false).2);
        assert!(guard(&[], 1.0, 0.5, false).2);
    }
}
