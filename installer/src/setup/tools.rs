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
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use super::environment::checked;

pub struct Tools {
    pub pixi: PathBuf,
    pub uv: PathBuf,
    pub nix: PathBuf,
}

fn locate(name: &str) -> Option<PathBuf> {
    which::which(name).ok().or_else(|| {
        let home = dirs::home_dir()?;
        [
            home.join(".local/bin"),
            home.join(".pixi/bin"),
            home.join(".nix-profile/bin"),
            PathBuf::from("/nix/var/nix/profiles/default/bin"),
        ]
        .into_iter()
        .map(|dir| dir.join(name))
        .find(|path| which::which(path).is_ok())
    })
}

fn ensure(name: &str, url: &str, args: &[&str], dir: &Path) -> Result<PathBuf> {
    if let Some(path) = locate(name) {
        return Ok(path);
    }
    let script = dir.join(format!("bootstrap-{name}.sh"));
    checked(
        Command::new("curl")
            .args([
                "--fail",
                "--show-error",
                "--location",
                "--proto",
                "=https",
                "--tlsv1.2",
                "--connect-timeout",
                "30",
                "--max-time",
                "120",
                url,
                "--output",
            ])
            .arg(&script),
    )?;
    let mut command = Command::new("bash");
    command
        .arg(&script)
        .args(args)
        .env("PIXI_NO_PATH_UPDATE", "1")
        .env("UV_NO_MODIFY_PATH", "1");
    checked(&mut command).with_context(|| format!("Installing {name}"))?;
    fs::remove_file(script)?;
    locate(name)
        .with_context(|| format!("{name} installer succeeded but its executable was not found"))
}

pub fn prepare(dir: &Path, unattended: bool) -> Result<Tools> {
    let pixi = ensure("pixi", "https://pixi.sh/install.sh", &[], dir)?;
    let uv = ensure("uv", "https://astral.sh/uv/install.sh", &[], dir)?;
    if locate("nix").is_none() && unattended {
        // The multi-user installer needs privilege. Avoid hanging unattended
        // jobs at a password prompt, and never change to a different backend.
        checked(Command::new("sudo").args(["-n", "true"]))
            .context("Nix bootstrap requires passwordless sudo in unattended setup")?;
    }
    let nix = ensure(
        "nix",
        "https://nixos.org/nix/install",
        &["--daemon", "--yes"],
        dir,
    )?;
    checked(Command::new(&nix).args([
        "--extra-experimental-features",
        "nix-command flakes",
        "store",
        "info",
    ]))
    .context("Nix is required for custom native modules; its store is unavailable")?;
    for tool in [&pixi, &uv] {
        checked(Command::new(tool).arg("--version"))?;
    }
    if !pixi.is_absolute() || !uv.is_absolute() || !nix.is_absolute() {
        bail!("Tool paths must be absolute");
    }
    Ok(Tools { pixi, uv, nix })
}
