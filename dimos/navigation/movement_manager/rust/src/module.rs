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

//! The robot-side half of the old python `MovementManager`: teleop preempts
//! nav on `cmd_vel`, and a watchdog zeros `cmd_vel` when nav goes quiet.
//!
//! The click-to-goal half stays in python next to rerun, including the NaN goal
//! that cancels a plan. Both halves subscribe `tele_cmd_vel`, so one keystroke
//! lands on both; nothing routes the cancel back from here.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{error_throttled, Input, Module, Output};
use lcm_msgs::geometry_msgs::Twist;
use lcm_msgs::std_msgs::Bool;
use tracing::{info, warn};

use crate::mux::{scale_twist, Config, MuxState};

/// The watchdog's tick rate. Fixed rather than configurable: `nav_stale_s` says
/// when the deadman trips, this only says how finely it is sampled.
const WATCHDOG_HZ: f64 = 10.0;

#[derive(Module)]
#[module(name = "cmd_vel_mux", setup = spawn_watchdog, teardown = stop_watchdog)]
pub struct CmdVelMux {
    #[input(decode = Twist::decode, handler = on_nav)]
    nav_cmd_vel: Input<Twist>,

    #[input(decode = Twist::decode, handler = on_teleop)]
    tele_cmd_vel: Input<Twist>,

    #[output(encode = Twist::encode)]
    cmd_vel: Output<Twist>,

    #[output(encode = Bool::encode)]
    stop_movement: Output<Bool>,

    #[config]
    config: Config,

    // Written by the handlers, read by the watchdog.
    state: Arc<Mutex<MuxState>>,

    watchdog: Option<tokio::task::JoinHandle<()>>,
}

impl CmdVelMux {
    async fn spawn_watchdog(&mut self) {
        let watchdog = Watchdog {
            state: Arc::clone(&self.state),
            cooldown: self.config.cooldown(),
            stale: self.config.nav_stale(),
            cmd_vel: self.cmd_vel.clone(),
        };
        self.watchdog = Some(tokio::spawn(watchdog.run()));
    }

    /// Stop the watchdog and leave `cmd_vel` at zero. A dead mux must no more
    /// leave a twist standing on the robot than a dead nav does — waiting for
    /// the bridge's own timeout means walking away from a stopped stack.
    ///
    /// Abort first so the watchdog cannot land a twist after ours; the handlers
    /// are already done by teardown, so nothing else can publish here.
    async fn stop_watchdog(&mut self) {
        if let Some(handle) = self.watchdog.take() {
            handle.abort();
        }
        publish_twist(&self.cmd_vel, &Twist::default()).await;
    }

    /// Nav is forwarded unmodified, or dropped while teleop holds the cooldown.
    async fn on_nav(&mut self, msg: Twist) {
        let forward = self
            .state
            .lock()
            .expect("mux state mutex")
            .on_nav(Instant::now(), self.config.cooldown());
        if forward {
            publish_twist(&self.cmd_vel, &msg).await;
        }
    }

    /// A teleop keystroke preempts nav, stops the follower, and drives.
    async fn on_teleop(&mut self, msg: Twist) {
        self.state
            .lock()
            .expect("mux state mutex")
            .on_teleop(Instant::now());

        if let Err(e) = self.stop_movement.publish(&Bool { data: true }).await {
            error_throttled!(
                Duration::from_secs(1),
                error = %e,
                topic = %self.stop_movement.topic,
                "stop_movement failed to publish",
            );
        }
        let scaled = scale_twist(
            &msg,
            &self.config.tele_scale_linear,
            &self.config.tele_scale_angular,
        );
        publish_twist(&self.cmd_vel, &scaled).await;
    }
}

/// The deadman the network link used to provide for free. With the follower
/// co-located there is no dropped link to stop `cmd_vel`, so a dead or wedged
/// follower would otherwise leave the last twist standing on the robot.
struct Watchdog {
    state: Arc<Mutex<MuxState>>,
    cooldown: Duration,
    stale: Duration,
    cmd_vel: Output<Twist>,
}

impl Watchdog {
    async fn run(self) {
        let mut ticker = tokio::time::interval(Duration::from_secs_f64(1.0 / WATCHDOG_HZ));
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut zeroing = false;
        loop {
            ticker.tick().await;
            let stale = self.state.lock().expect("mux state mutex").nav_is_stale(
                Instant::now(),
                self.cooldown,
                self.stale,
            );
            if stale {
                if !zeroing {
                    warn!(
                        nav_stale_s = self.stale.as_secs_f64(),
                        "nav_cmd_vel went stale, holding cmd_vel at zero"
                    );
                    zeroing = true;
                }
                // Every tick, not just the edge: a single zero can be lost.
                publish_twist(&self.cmd_vel, &Twist::default()).await;
            } else if zeroing {
                info!("nav_cmd_vel recovered, releasing cmd_vel");
                zeroing = false;
            }
        }
    }
}

async fn publish_twist(out: &Output<Twist>, msg: &Twist) {
    if let Err(e) = out.publish(msg).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "cmd_vel failed to publish",
        );
    }
}
