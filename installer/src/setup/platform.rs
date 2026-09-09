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

use anyhow::{bail, Result};
use std::fs;

pub fn classify(arch: &str, os_release: &str, model: &str, tegra_release: &str) -> Result<String> {
    let ubuntu = os_release
        .lines()
        .any(|l| l == "ID=ubuntu" || l == "ID=\"ubuntu\"");
    let version = os_release
        .lines()
        .find_map(|l| l.strip_prefix("VERSION_ID="))
        .unwrap_or("")
        .trim_matches('"');
    match arch {
        "x86_64" if ubuntu && matches!(version, "22.04" | "24.04") => Ok(format!("ubuntu-{version}-x86_64")),
        "x86_64" => Ok("unverified-linux-x86_64".into()),
        "aarch64" if ubuntu && version == "22.04" && model.to_lowercase().contains("jetson agx orin") && tegra_release.contains("R36 (release)") && tegra_release.contains("REVISION: 4.3,") => Ok("jetson-agx-orin-jetpack-6.2".into()),
        _ => bail!("Target platforms: Ubuntu 22.04/24.04 x86_64, or Jetson AGX Orin with JetPack 6.2 (L4T 36.4.3). Detected architecture: {arch}, OS: {version}, model: {model}"),
    }
}

pub fn detect() -> Result<String> {
    if !cfg!(target_os = "linux") {
        bail!("This installer currently targets Linux");
    }
    classify(
        std::env::consts::ARCH,
        &fs::read_to_string("/etc/os-release")?,
        &fs::read_to_string("/proc/device-tree/model").unwrap_or_default(),
        &fs::read_to_string("/etc/nv_tegra_release").unwrap_or_default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orin_uses_board_and_l4t_instead_of_nvidia_smi() {
        let os = "ID=ubuntu\nVERSION_ID=\"22.04\"\n";
        let board = "NVIDIA Jetson AGX Orin Developer Kit\0";
        assert!(classify(
            "aarch64",
            os,
            board,
            "# R36 (release), REVISION: 4.3, GCID: 1"
        )
        .is_ok());
        assert!(classify("aarch64", os, board, "# R35 (release), REVISION: 4.1,").is_err());
        assert!(classify(
            "aarch64",
            os,
            "Raspberry Pi",
            "# R36 (release), REVISION: 4.3,"
        )
        .is_err());
        assert!(classify("x86_64", os, "", "")
            .unwrap()
            .starts_with("ubuntu"));
    }
}
