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

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use dimos_module::{Input, Module, Output, run_with_transport};
use external_telemetry_messages::{codec::Message, demo_msgs::msg::Telemetry};

#[derive(Module)]
struct Relay {
    #[input(decode = Telemetry::decode)]
    telemetry_in: Input<Telemetry>,
    #[output(encode = Telemetry::encode)]
    telemetry_out: Output<Telemetry>,
}

impl Relay {
    async fn handle_telemetry_in(&mut self, mut message: Telemetry) {
        message.application_note.push_str("/rust-native");
        if let Err(error) = self.telemetry_out.publish(&message).await {
            tracing::error!(%error, "external message publish failed");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<Relay>().await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io;

    #[test]
    fn generated_external_codecs_match_sdk_ports_and_report_invalid_data() {
        let encode: fn(&Telemetry) -> io::Result<Vec<u8>> = Telemetry::encode;
        let decode: fn(&[u8]) -> io::Result<Telemetry> = Telemetry::decode;
        let message = Telemetry {
            label: "x".repeat(33),
            ..Default::default()
        };
        assert_eq!(
            encode(&message).unwrap_err().kind(),
            io::ErrorKind::InvalidInput
        );
        assert_eq!(
            decode(&[0, 1, 0, 0]).unwrap_err().kind(),
            io::ErrorKind::InvalidData
        );
    }
}
