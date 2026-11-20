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

//! Stores the window a crashed event-mode recording left on disk into its recording.

use std::path::PathBuf;
use std::process::ExitCode;

use dimos_memory_recorder::recovery::recover;

fn main() -> ExitCode {
    let windows: Vec<PathBuf> = std::env::args_os().skip(1).map(PathBuf::from).collect();
    if windows.is_empty() {
        eprintln!("usage: dimos-memory-recover <window_dir>...");
        return ExitCode::from(2);
    }
    let mut failed = false;
    for window in &windows {
        match recover(window) {
            Ok(written) => println!("{}: recovered {written} observations", window.display()),
            Err(error) => {
                eprintln!("{}: recovery failed: {error:#}", window.display());
                failed = true;
            }
        }
    }
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
