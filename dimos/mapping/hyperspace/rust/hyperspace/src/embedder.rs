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

//! Model backends behind traits so the core builds and tests without a GPU.

use crate::patch::{normalize, PatchGrid};
use crate::ImageFrame;
use std::collections::HashMap;

pub trait Embedder: Send {
    /// Per-patch, text-aligned, L2-normalized embeddings for one RGB frame.
    fn embed(&mut self, frame: &ImageFrame) -> Result<PatchGrid, String>;
    fn grid_shape(&self) -> (usize, usize);
    fn dim(&self) -> usize;
}

pub trait TextEmbedder: Send {
    /// L2-normalized text embedding in the same space as the patch embeddings.
    fn embed_text(&mut self, text: &str) -> Result<Vec<f32>, String>;
}

/// Deterministic pseudo-embeddings from image content (no model). Two frames
/// with identical pixels get identical grids; different pixels, different grids.
/// Good enough to exercise the pipeline, meaningless for real queries.
pub struct HashEmbedder {
    pub rows: usize,
    pub cols: usize,
    pub dim: usize,
}

impl Default for HashEmbedder {
    fn default() -> Self {
        HashEmbedder {
            rows: 24,
            cols: 24,
            dim: 32,
        }
    }
}

impl Embedder for HashEmbedder {
    fn embed(&mut self, frame: &ImageFrame) -> Result<PatchGrid, String> {
        let mut data = Vec::with_capacity(self.rows * self.cols * self.dim);
        let patch_h = (frame.height as usize / self.rows).max(1);
        let patch_w = (frame.width as usize / self.cols).max(1);
        for row in 0..self.rows {
            for col in 0..self.cols {
                let mut seed: u64 = 0xcbf29ce484222325;
                for y in
                    (row * patch_h..((row + 1) * patch_h).min(frame.height as usize)).step_by(4)
                {
                    for x in
                        (col * patch_w..((col + 1) * patch_w).min(frame.width as usize)).step_by(4)
                    {
                        let offset = (y * frame.width as usize + x) * 3;
                        for byte in &frame.data[offset..offset + 3] {
                            seed = (seed ^ (*byte as u64)).wrapping_mul(0x100000001b3);
                        }
                    }
                }
                let mut vector: Vec<f32> = (0..self.dim)
                    .map(|k| {
                        seed = seed
                            .wrapping_mul(6364136223846793005)
                            .wrapping_add(1442695040888963407 + k as u64);
                        ((seed >> 33) as f32 / (1u64 << 31) as f32) - 0.5
                    })
                    .collect();
                normalize(&mut vector);
                data.extend(vector);
            }
        }
        Ok(PatchGrid::from_f32(self.rows, self.cols, self.dim, &data))
    }

    fn grid_shape(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }

    fn dim(&self) -> usize {
        self.dim
    }
}

/// Text embeddings looked up from a fixed table (tests, offline tools).
#[derive(Default)]
pub struct TableTextEmbedder {
    pub table: HashMap<String, Vec<f32>>,
}

impl TextEmbedder for TableTextEmbedder {
    fn embed_text(&mut self, text: &str) -> Result<Vec<f32>, String> {
        let mut vector = self
            .table
            .get(text)
            .cloned()
            .ok_or_else(|| format!("no table embedding for {text:?}"))?;
        normalize(&mut vector);
        Ok(vector)
    }
}

/// Scripted per-frame grids keyed by (camera_frame, timestamp) for tests; falls
/// back to a hash grid for unscripted frames.
#[derive(Default)]
pub struct ScriptedEmbedder {
    pub grids: HashMap<(String, u64), PatchGrid>,
    pub fallback: Option<HashEmbedder>,
    pub rows: usize,
    pub cols: usize,
    pub dim: usize,
}

impl ScriptedEmbedder {
    pub fn key(camera_frame: &str, timestamp: f64) -> (String, u64) {
        (camera_frame.to_string(), (timestamp * 1e6).round() as u64)
    }

    pub fn script(&mut self, camera_frame: &str, timestamp: f64, grid: PatchGrid) {
        self.grids.insert(Self::key(camera_frame, timestamp), grid);
    }
}

impl Embedder for ScriptedEmbedder {
    fn embed(&mut self, frame: &ImageFrame) -> Result<PatchGrid, String> {
        if let Some(grid) = self
            .grids
            .get(&Self::key(&frame.camera_frame, frame.timestamp))
        {
            return Ok(grid.clone());
        }
        match &mut self.fallback {
            Some(fallback) => fallback.embed(frame),
            None => Err(format!(
                "no scripted grid for {} @ {}",
                frame.camera_frame, frame.timestamp
            )),
        }
    }

    fn grid_shape(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }

    fn dim(&self) -> usize {
        self.dim
    }
}
