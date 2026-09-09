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

use clap::{Parser, ValueEnum};
use std::path::PathBuf;

fn default_branch() -> String {
    "main".into()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Profile {
    Navigation,
    Manipulation,
}

impl Profile {
    pub fn name(self) -> &'static str {
        match self {
            Self::Navigation => "navigation",
            Self::Manipulation => "manipulation",
        }
    }
    pub fn extras(self) -> &'static [&'static str] {
        match self {
            Self::Navigation => &["unitree", "cpu"],
            Self::Manipulation => &["manipulation", "cpu"],
        }
    }
    pub fn blueprint(self) -> &'static str {
        match self {
            Self::Navigation => "unitree-go2",
            Self::Manipulation => "xarm7-planner-coordinator",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum InstallMode {
    Library,
    Dev,
}

#[derive(Parser, Clone)]
pub struct SetupArgs {
    /// Required for unattended setup; interactive setup asks
    #[arg(long, value_enum)]
    pub profile: Option<Profile>,
    /// Dedicated library project or contributor checkout
    #[arg(long, value_enum)]
    pub mode: Option<InstallMode>,
    /// Project directory (must be empty for a new library project)
    #[arg(long)]
    pub project_dir: PathBuf,
    /// Branch used only when cloning a new contributor checkout
    #[arg(long, default_value_t = default_branch())]
    pub branch: String,
    /// Install a local wheel instead of the bundled PyPI version (library mode)
    #[arg(long)]
    pub wheel: Option<PathBuf>,
}
