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

use anyhow::{bail, Context, Result};
use std::path::Path;
use std::process::Command;
use std::time::{Duration, Instant};

pub fn checked(command: &mut Command) -> Result<()> {
    let status = command
        .status()
        .with_context(|| format!("starting {command:?}"))?;
    if !status.success() {
        bail!("{command:?} failed with {status}");
    }
    Ok(())
}

pub fn checked_timeout(command: &mut Command, timeout: Duration) -> Result<()> {
    let mut child = command
        .spawn()
        .with_context(|| format!("starting {command:?}"))?;
    let start = Instant::now();
    loop {
        if let Some(status) = child.try_wait()? {
            if !status.success() {
                bail!("{command:?} failed with {status}");
            }
            return Ok(());
        }
        if start.elapsed() >= timeout {
            child.kill()?;
            child.wait()?;
            bail!(
                "Dependency check timed out after {} seconds: {command:?}",
                timeout.as_secs()
            );
        }
        std::thread::sleep(Duration::from_millis(50));
    }
}

pub fn esc(value: &str) -> String {
    format!("'{}'", value.replace('\'', "'\\''"))
}

pub fn command(project: &Path, args: &[String]) -> Command {
    let mut command = Command::new("bash");
    command
        .args([
            "--noprofile",
            "--norc",
            "-c",
            "source \"$1\" || exit; shift; exec \"$@\"",
            "create-dimos",
        ])
        .arg(project.join(".dimos/activate.sh"))
        .args(args)
        .current_dir(project);
    command
}
