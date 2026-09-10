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

//! Jeff's depth2depth (Depth-Anything-V2 affine-anchored to sensor depth) as a DepthFuser.

use crate::depth::DepthFuser;
use crate::ImageFrame;
use candle_core::{DType, Device};
use std::path::Path;

pub struct Depth2DepthFuser {
    inner: depth2depth::Depth2Depth,
}

impl Depth2DepthFuser {
    /// `weights_dir` holds `dinov2_vits14.safetensors` and `da2_head_vits.safetensors`
    /// (see depth2depth's tools/convert_weights.py).
    pub fn load(
        weights_dir: &Path,
        device: Device,
        dtype: DType,
        config: depth2depth::Config,
    ) -> Result<Self, String> {
        let inner = depth2depth::Depth2Depth::new(
            weights_dir
                .join("dinov2_vits14.safetensors")
                .to_str()
                .ok_or("bad path")?,
            weights_dir
                .join("da2_head_vits.safetensors")
                .to_str()
                .ok_or("bad path")?,
            device,
            dtype,
            config,
        )
        .map_err(|e| format!("loading depth2depth: {e}"))?;
        Ok(Depth2DepthFuser { inner })
    }

    pub fn reset(&mut self) {
        self.inner.reset()
    }
}

impl DepthFuser for Depth2DepthFuser {
    fn fuse(&mut self, rgb: &ImageFrame, raw_depth_m: &[f32]) -> Result<Vec<f32>, String> {
        let fusion = self
            .inner
            .fuse(
                &rgb.data,
                raw_depth_m,
                rgb.height as usize,
                rgb.width as usize,
            )
            .map_err(|e| e.to_string())?;
        Ok(fusion.fused)
    }
}
