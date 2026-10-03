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

use dimos_generated_messages::{codec::Message, demo_msgs::msg::Telemetry};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let arguments: Vec<String> = std::env::args().collect();
    if arguments.len() != 4 {
        return Err("usage: relay <edit|echo|echo-be|defaults> <input.cdr> <output.cdr>".into());
    }
    let mut value = if arguments[1] == "defaults" { Telemetry::default() } else { Telemetry::decode(&std::fs::read(&arguments[2])?)? };
    println!(
        "Rust received: frame={} sequence={} label={} temperature={} x={} axes={:?}",
        value.header.frame_id,
        value.sequence,
        value.label,
        value.reading.temperature,
        value.position.x,
        value.axes,
    );
    match arguments[1].as_str() {
        "edit" => {
            value.sequence += 1;
            value.hops.push(3);
            value.label.push_str("/rust");
        }
        "echo" | "echo-be" | "defaults" => {}
        _ => return Err("Expected edit, echo, echo-be, or defaults mode".into()),
    }
    std::fs::write(&arguments[3], value.encode_endian(arguments[1] != "echo-be")?)?;
    Ok(())
}
