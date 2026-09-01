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

use dimos_module::{run_with_transport, Input, Module, Output};
use lcm_msgs::geometry_msgs::{Twist, Vector3};
use tokio::time::{interval, Duration};

#[derive(Module)]
#[module(setup = start_publisher)]
struct Ping {
    #[input(decode = Twist::decode)]
    confirm: Input<Twist>,

    #[output(encode = Twist::encode)]
    data: Output<Twist>,
}

impl Ping {
    async fn start_publisher(&mut self) {
        let data = self.data.clone();
        tokio::spawn(async move {
            let mut ticker = interval(Duration::from_millis(200));
            let mut seq = 0u64;
            loop {
                ticker.tick().await;
                let msg = Twist {
                    linear: Vector3 {
                        x: seq as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                };
                data.publish(&msg).await.ok();
                seq += 1;
            }
        });
    }

    async fn handle_confirm(&mut self, echo: Twist) {
        tracing::info!(
            seq = echo.linear.x as u64,
            sample_config = echo.angular.z as i64,
            "echo received (rust)",
        );
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<Ping>().await;
}
