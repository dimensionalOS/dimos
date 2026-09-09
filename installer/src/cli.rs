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

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum InstallMode {
    Sdk,
    Contributor,
}

#[derive(Parser, Debug)]
#[command(
    name = "create-dimos",
    about = "Create or restore a DimOS development workspace"
)]
pub struct SetupArgs {
    /// Workspace directory (prompted when omitted interactively)
    pub project_dir: Option<PathBuf>,
    #[arg(long, value_enum)]
    pub profile: Option<Profile>,
    /// Prepare a DimOS source checkout instead of an SDK project
    #[arg(long, conflicts_with = "restore")]
    pub contributor: bool,
    /// Restore an existing workspace from its configuration and lockfiles
    #[arg(long)]
    pub restore: bool,
    /// Branch for a new contributor clone
    #[arg(long, default_value = "main")]
    pub branch: String,
    /// Test an SDK wheel instead of the matching published version
    #[arg(long, conflicts_with = "contributor")]
    pub wheel: Option<PathBuf>,
    #[arg(long)]
    pub non_interactive: bool,
    #[arg(long)]
    pub dry_run: bool,
}
