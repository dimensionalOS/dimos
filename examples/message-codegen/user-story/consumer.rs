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

use story_messages_messages::story_msgs::msg::device_reading::DeviceReading;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: story-consumer INPUT OUTPUT".into());
    }
    let wire = std::fs::read(&args[1])?;
    if wire.get(..4) != Some(&[0, 1, 0, 0]) {
        return Err("expected little-endian CDR".into());
    }
    let (mut message, consumed) =
        re_cdr::from_bytes::<DeviceReading, re_cdr::LittleEndian>(&wire[4..])?;
    if consumed != wire.len() - 4 {
        return Err("trailing CDR bytes".into());
    }
    message.sequence += 1;
    message.value += 1.0;
    message.label.push_str("/rust");
    println!(
        "Rust consumer: value={} label={}",
        message.value, message.label
    );
    let mut output = vec![0, 1, 0, 0];
    output.extend(re_cdr::to_vec::<_, re_cdr::LittleEndian>(&message)?);
    std::fs::write(&args[2], output)?;
    Ok(())
}
