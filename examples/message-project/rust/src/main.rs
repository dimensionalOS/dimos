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

use dimos_module::{Input, Module, Output, cdr, run_with_transport};
use story_messages_messages::story_msgs::msg::DeviceReading;

#[derive(Module)]
struct Processor {
    #[input(decode = cdr::decode)]
    processed: Input<DeviceReading>,
    #[output(encode = cdr::encode)]
    checked: Output<DeviceReading>,
}

impl Processor {
    async fn handle_processed(&mut self, mut message: DeviceReading) {
        message.value += 1.0;
        if let Err(error) = self.checked.publish(&message).await {
            tracing::error!(%error, "publish failed");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<Processor>().await;
}
