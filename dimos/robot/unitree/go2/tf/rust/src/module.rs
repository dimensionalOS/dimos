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

//! The Go2's tf tree, published on the robot.
//!
//! The mount tree and the `odom -> mid360_link` edge are the tf half of
//! `GO2Zenoh`, which runs on the laptop — so today the robot's planner and
//! follower wait on a wifi round trip for a mount offset that is entirely local
//! knowledge. Baked into the robot host beside them, that leg never leaves the
//! robot.
//!
//! What stays on the laptop is the rest of `GO2Zenoh`: the bridge's streams, the
//! action verbs, and the camera intrinsics.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use dimos_module::{error_throttled, Input, Module, Output};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::tf2_msgs::TFMessage;
use tracing::info;

use crate::tree::{mount_tree, odom_edge, stamp_secs, Config, Edge};

#[derive(Module)]
#[module(name = "go2_tf", setup = spawn_static, teardown = stop_static)]
pub struct Go2Tf {
    #[input(decode = Odometry::decode, handler = on_odometry)]
    odometry: Input<Odometry>,

    #[output(encode = TFMessage::encode)]
    tf: Output<TFMessage>,

    #[config]
    config: Config,

    statics: Option<tokio::task::JoinHandle<()>>,
}

impl Go2Tf {
    async fn spawn_static(&mut self) {
        let tree = mount_tree(&self.config);
        info!(
            edges = tree.len(),
            publish_hz = self.config.publish_hz,
            "publishing the go2 mount tree"
        );
        self.statics = Some(tokio::spawn(
            StaticTree {
                tree,
                period: Duration::from_secs_f64(1.0 / self.config.publish_hz),
                tf: self.tf.clone(),
            }
            .run(),
        ));
    }

    async fn stop_static(&mut self) {
        if let Some(handle) = self.statics.take() {
            handle.abort();
        }
    }

    /// The one moving edge; the go2web bridge publishes no tf of its own.
    async fn on_odometry(&mut self, msg: Odometry) {
        // python's `Transform` reads a zero stamp as "now"; without the same
        // fallback an unstamped odometry would land at the epoch, outside every
        // consumer's tf window, and the body would simply stop resolving.
        let stamp = stamp_secs(&msg.header);
        let ts = if stamp == 0.0 { now_secs() } else { stamp };
        publish(&self.tf, &[odom_edge(&msg)], ts).await;
    }
}

/// The mount tree on a timer. tf has no latched path, so a one-shot publish
/// would be missed by everything that subscribes later — including a recorder
/// that wants the mount geometry in its own tf stream.
struct StaticTree {
    tree: Vec<Edge>,
    period: Duration,
    tf: Output<TFMessage>,
}

impl StaticTree {
    async fn run(self) {
        let mut ticker = tokio::time::interval(self.period);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        loop {
            ticker.tick().await;
            publish(&self.tf, &self.tree, now_secs()).await;
        }
    }
}

/// One TFMessage carrying every edge, all stamped alike — the shape python
/// publishes, so a consumer sees the tree arrive atomically.
async fn publish(out: &Output<TFMessage>, edges: &[Edge], ts: f64) {
    let msg = TFMessage {
        transforms: edges.iter().map(|e| e.to_stamped(ts)).collect(),
    };
    if let Err(e) = out.publish(&msg).await {
        error_throttled!(
            Duration::from_secs(1),
            error = %e,
            topic = %out.topic,
            "tf failed to publish",
        );
    }
}

fn now_secs() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}
