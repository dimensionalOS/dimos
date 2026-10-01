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

use dimos_generated_messages::{dimos_msgs::msg::LineSegments3D, sensor_msgs::msg::Image};
use dimos_module::{cdr, run_with_transport, Input, Module, Output};

#[derive(Module)]
struct CdrRelay {
    #[input(decode = cdr::decode)]
    lines_in: Input<LineSegments3D>,
    #[output(encode = cdr::encode)]
    lines_out: Output<LineSegments3D>,
    #[input(decode = cdr::decode)]
    image_in: Input<Image>,
    #[output(encode = cdr::encode)]
    image_out: Output<Image>,
}

impl CdrRelay {
    async fn handle_lines_in(&mut self, mut message: LineSegments3D) {
        for segment in &mut message.segments {
            segment.weight += 1.0;
        }
        if let Err(error) = self.lines_out.publish(&message).await {
            tracing::error!(%error, "failed to publish custom message");
        }
    }
    async fn handle_image_in(&mut self, message: Image) {
        if let Err(error) = self.image_out.publish(&message).await {
            tracing::error!(%error, "failed to publish image");
        }
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<CdrRelay>().await;
}
