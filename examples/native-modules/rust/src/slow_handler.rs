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

use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::sensor_msgs::Image;
use tokio::time::{sleep, Duration};

#[native_config]
struct SlowHandlerConfig {
    #[validate(range(min = 0, max = 10_000))]
    handler_delay_ms: i64,
}

#[derive(Module)]
struct SlowHandler {
    #[input(decode = Image::decode)]
    frames: Input<Image>,

    #[output(encode = Image::encode)]
    handled: Output<Image>,

    #[config]
    config: SlowHandlerConfig,
}

impl SlowHandler {
    async fn handle_frames(&mut self, msg: Image) {
        sleep(Duration::from_millis(self.config.handler_delay_ms as u64)).await;
        self.handled.publish(&msg).await.ok();
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<SlowHandler>().await;
}
