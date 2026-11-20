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

//! Shared test setup: the committed config and the extracted vendor policies.

#![allow(dead_code)]

use std::path::PathBuf;

use m20_controller::config::Config;

pub fn config_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("m20_controller.toml")
}

pub fn config() -> Config {
    Config::load(&config_path()).expect("config")
}

/// The extracted policies (vendor IP, not committed), or None to skip.
pub fn policy_dir() -> Option<PathBuf> {
    let dir = std::env::var("M20_POLICY_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| {
            PathBuf::from(std::env::var("HOME").unwrap_or_default())
                .join(".cache/dimos/m20_policies")
        });
    if dir.join("policy_m20_50.onnx").exists() {
        Some(dir)
    } else {
        eprintln!(
            "skipping: no policies in {} (see extract_policies.py)",
            dir.display()
        );
        None
    }
}
