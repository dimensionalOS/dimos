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
// A standalone POSIX lifecycle probe: no SDK, crates, transport, or hardware.
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

static STOPPING: AtomicBool = AtomicBool::new(false);

extern "C" fn stop(_: i32) {
    STOPPING.store(true, Ordering::Relaxed);
}

extern "C" {
    fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 2 || args[0] != "--message_file" {
        return Err("expected --message_file PATH".into());
    }
    let resource = fs::read_to_string(&args[1])?;
    let message = resource.lines().next().ok_or("empty message resource")?;
    let report = std::env::var("DIMOS_PACKAGE_REPORT")?;
    // POSIX SIGINT/SIGTERM. The handler only sets a lock-free atomic flag.
    unsafe {
        signal(2, stop);
        signal(15, stop);
    }
    fs::write(&report, format!("ready {} {message}\n", std::process::id()))?;
    println!("{message}");
    io::stdout().flush()?;
    while !STOPPING.load(Ordering::Relaxed) {
        std::thread::sleep(Duration::from_millis(10));
    }
    writeln!(OpenOptions::new().append(true).open(report)?, "stopped")?;
    Ok(())
}
