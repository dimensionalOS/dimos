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

//! Schema metadata for opaque CDR recording.

use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CdrSchema {
    pub name: String,
    pub data: String,
}

impl CdrSchema {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.name.split('/').count() == 3 && self.name.contains("/msg/"),
            "CDR schema requires package/msg/Type identity"
        );
        ensure!(
            !self.data.trim().is_empty(),
            "CDR recording requires complete ros2msg schema data"
        );
        Ok(())
    }
}
