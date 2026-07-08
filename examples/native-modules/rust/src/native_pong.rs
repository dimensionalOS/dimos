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

use dimos_module::{native_config, run, Input, LcmTransport, Module, Output};
use lcm_msgs::geometry_msgs::{Twist, Vector3};

#[native_config]
struct PongConfig {
    #[validate(range(min = 0, max = 1000))]
    sample_config: i64,
}

#[derive(Module)]
struct Pong {
    #[input(decode = Twist::decode)]
    data: Input<Twist>,

    #[output(encode = Twist::encode)]
    confirm: Output<Twist>,

    #[config]
    config: PongConfig,
}

impl Pong {
    async fn handle_data(&mut self, msg: Twist) {
        let reply = Twist {
            linear: msg.linear,
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: self.config.sample_config as f64,
            },
        };
        self.confirm.publish(&reply).await.ok();
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    run::<Pong, _>(transport).await;
}
