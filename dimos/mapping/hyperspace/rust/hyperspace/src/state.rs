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

//! Persistence of everything a query needs (keyframes, tf, intrinsics).

use crate::keyframe_store::Keyframe;
use crate::tf::{TfTree, Transform};
use crate::CameraIntrinsics;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

#[derive(Serialize, Deserialize)]
pub struct SavedState {
    pub voxel_size: f64,
    pub intrinsics: HashMap<String, CameraIntrinsics>,
    pub keyframes: Vec<Keyframe>,
    pub transforms: Vec<Transform>,
}

impl SavedState {
    pub fn save(&self, path: &Path) -> Result<(), String> {
        let bytes = bincode::serialize(self).map_err(|e| e.to_string())?;
        std::fs::write(path, bytes).map_err(|e| e.to_string())
    }

    pub fn load(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
        bincode::deserialize(&bytes).map_err(|e| e.to_string())
    }

    pub fn tf_tree(&self) -> TfTree {
        let mut tree = TfTree::default();
        for transform in &self.transforms {
            tree.insert(transform);
        }
        tree
    }
}
