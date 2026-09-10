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

//! What a kept keyframe stores: no pose, only where/when it was taken.

use crate::patch::PatchGrid;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Keyframe {
    /// tf frame of the color camera.
    pub camera_frame: String,
    pub timestamp: f64,
    pub grid: PatchGrid,
    /// Fused depth (m) per patch, row-major like `grid`; NaN = unknown.
    pub patch_depth: Vec<f32>,
    /// Fused depth (mm) subsampled by `depth_stride` for scene rendering; empty if not kept.
    pub depth_thumbnail_mm: Vec<u16>,
    pub depth_stride: u32,
    /// Full image size the patch grid and thumbnail refer to.
    pub width: u32,
    pub height: u32,
}
