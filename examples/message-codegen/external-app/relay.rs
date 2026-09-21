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

use external_telemetry_messages::{codec::Message, demo_msgs::msg::Telemetry};
use std::{env, fs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().collect();
    let mut value = Telemetry::decode(&fs::read(&args[1])?)?;
    println!("Packaged Rust crate received: {}", value.application_note);
    value.application_note.push_str("/rust");
    fs::write(&args[2], value.encode()?)?;
    Ok(())
}
