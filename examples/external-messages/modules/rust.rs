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

use acme_telemetry_messages::telemetry_msgs::msg::DeviceReading;
use dimos_module::{Input, Module, Output, cdr, run_with_transport};

fn encode(value: &DeviceReading) -> Vec<u8> {
    cdr::encode(value).expect("serialize a valid DeviceReading")
}

#[derive(Module)]
struct Relay {
    #[input(decode = cdr::decode)]
    cpp: Input<DeviceReading>,
    #[output(encode = encode)]
    done: Output<DeviceReading>,
}
impl Relay {
    async fn handle_cpp(&mut self, mut value: DeviceReading) {
        value.value += 1.0;
        value.label.push_str("/rust");
        self.done
            .publish(&value)
            .await
            .expect("publish DeviceReading");
    }
}
#[tokio::main]
async fn main() {
    run_with_transport::<Relay>().await;
}
