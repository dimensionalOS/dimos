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

use story_messages_messages::{codec::Message, story_msgs::msg::DeviceReading};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        return Err("usage: story-consumer INPUT OUTPUT".into());
    }
    let mut message = DeviceReading::decode(&std::fs::read(&args[1])?)?;
    message.sequence += 1;
    message.value += 1.0;
    message.label.push_str("/rust");
    println!("Rust consumer: value={} label={}", message.value, message.label);
    std::fs::write(&args[2], message.encode()?)?;
    Ok(())
}
