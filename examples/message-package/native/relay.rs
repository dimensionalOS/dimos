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

use dimos_generated_messages::std_msgs::msg::header::Header;
use std::{env, fs};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args().collect();
    let bytes = fs::read(&args[1])?;
    let mut value: Header = cdr::deserialize(&bytes)?;
    value.frame_id.push_str("/rust");
    let output = cdr::serialize::<_, _, cdr::CdrLe>(&value, cdr::Infinite)?;
    fs::write(&args[2], output)?;
    Ok(())
}
