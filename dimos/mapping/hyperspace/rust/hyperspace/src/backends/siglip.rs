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

//! SigLIP2 (fixed-resolution, e.g. google/siglip2-so400m-patch16-384) on candle:
//! per-patch text-aligned image embeddings and text embeddings.

use super::siglip_model::{Config as ModelConfig, Model};
use crate::embedder::{Embedder, TextEmbedder};
use crate::patch::{normalize, PatchGrid};
use crate::ImageFrame;
use candle_core::{DType, Device, Module, Tensor};
use candle_nn::VarBuilder;
use std::path::Path;
use tokenizers::Tokenizer;

pub struct SigLip2 {
    model: Model,
    tokenizer: Tokenizer,
    device: Device,
    dtype: DType,
    image_size: usize,
    patches_per_side: usize,
    text_length: usize,
    hidden_size: usize,
}

impl SigLip2 {
    /// `model_dir` holds config.json, tokenizer.json and model.safetensors
    /// (an HF snapshot directory works as-is).
    pub fn load(model_dir: &Path, device: Device, dtype: DType) -> Result<Self, String> {
        let config: ModelConfig = serde_json::from_str(
            &std::fs::read_to_string(model_dir.join("config.json")).map_err(|e| e.to_string())?,
        )
        .map_err(|e| format!("config.json: {e}"))?;
        let tokenizer =
            Tokenizer::from_file(model_dir.join("tokenizer.json")).map_err(|e| e.to_string())?;
        let weights: Vec<std::path::PathBuf> = std::fs::read_dir(model_dir)
            .map_err(|e| e.to_string())?
            .filter_map(|entry| entry.ok().map(|e| e.path()))
            .filter(|path| path.extension().is_some_and(|ext| ext == "safetensors"))
            .collect();
        if weights.is_empty() {
            return Err(format!("no .safetensors in {}", model_dir.display()));
        }
        let var_builder = unsafe { VarBuilder::from_mmaped_safetensors(&weights, dtype, &device) }
            .map_err(|e| e.to_string())?;
        let model =
            Model::new(&config, var_builder).map_err(|e| format!("loading SigLIP2: {e}"))?;
        let image_size = config.vision_config.image_size;
        let patches_per_side = image_size / config.vision_config.patch_size;
        Ok(SigLip2 {
            model,
            tokenizer,
            device,
            dtype,
            image_size,
            patches_per_side,
            text_length: config.text_config.max_position_embeddings,
            hidden_size: config.vision_config.hidden_size,
        })
    }

    /// Resize (bilinear, no crop — the HF SiglipImageProcessor default) to the
    /// model size and normalize with mean 0.5 / std 0.5; returns `[1, 3, S, S]`.
    pub fn preprocess(&self, frame: &ImageFrame) -> Result<Tensor, String> {
        let image = image::RgbImage::from_raw(frame.width, frame.height, frame.data.clone())
            .ok_or("bad rgb8 buffer")?;
        let resized = image::imageops::resize(
            &image,
            self.image_size as u32,
            self.image_size as u32,
            image::imageops::FilterType::Triangle,
        );
        self.tensor_from_rgb(resized.as_raw(), self.image_size, self.image_size)
    }

    /// Already-resized rgb8 pixels (S×S) → normalized tensor `[1, 3, S, S]`.
    pub fn tensor_from_rgb(
        &self,
        rgb: &[u8],
        width: usize,
        height: usize,
    ) -> Result<Tensor, String> {
        let floats: Vec<f32> = rgb
            .iter()
            .map(|v| (*v as f32 / 255.0 - 0.5) / 0.5)
            .collect();
        Tensor::from_vec(floats, (height, width, 3), &self.device)
            .and_then(|t| t.permute((2, 0, 1)))
            .and_then(|t| t.unsqueeze(0))
            .and_then(|t| t.to_dtype(self.dtype))
            .and_then(|t| t.contiguous())
            .map_err(|e| e.to_string())
    }

    /// Per-patch pooled embeddings `[patches, hidden]` as f32, L2-normalized.
    pub fn embed_tensor(&self, pixel_values: &Tensor) -> Result<Vec<f32>, String> {
        let (_pooled, per_patch) = self
            .model
            .vision_model
            .vision_model
            .pooled_and_per_patch(pixel_values)
            .map_err(|e| e.to_string())?;
        let per_patch = super::siglip_model::div_l2_norm(
            &per_patch.to_dtype(DType::F32).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        per_patch
            .flatten_all()
            .and_then(|t| t.to_vec1::<f32>())
            .map_err(|e| e.to_string())
    }

    /// Whole-image pooled embedding, L2-normalized (for parity checks).
    pub fn embed_pooled(&self, pixel_values: &Tensor) -> Result<Vec<f32>, String> {
        let (pooled, _) = self
            .model
            .vision_model
            .vision_model
            .pooled_and_per_patch(pixel_values)
            .map_err(|e| e.to_string())?;
        let pooled = super::siglip_model::div_l2_norm(
            &pooled.to_dtype(DType::F32).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?;
        pooled
            .flatten_all()
            .and_then(|t| t.to_vec1::<f32>())
            .map_err(|e| e.to_string())
    }

    pub fn hidden_size(&self) -> usize {
        self.hidden_size
    }

    /// Tokenize like the HF processor: pad/truncate to `max_position_embeddings`
    /// with the pad token, eos appended by the tokenizer itself.
    pub fn tokenize(&self, text: &str) -> Result<Tensor, String> {
        let encoding = self
            .tokenizer
            .encode(text, true)
            .map_err(|e| e.to_string())?;
        let mut ids: Vec<u32> = encoding.get_ids().to_vec();
        let pad_id = self.tokenizer.token_to_id("<pad>").unwrap_or(0);
        ids.truncate(self.text_length);
        while ids.len() < self.text_length {
            ids.push(pad_id);
        }
        Tensor::new(ids.as_slice(), &self.device)
            .and_then(|t| t.unsqueeze(0))
            .map_err(|e| e.to_string())
    }
}

impl Embedder for SigLip2 {
    fn embed(&mut self, frame: &ImageFrame) -> Result<PatchGrid, String> {
        let pixel_values = self.preprocess(frame)?;
        let data = self.embed_tensor(&pixel_values)?;
        let dim = data.len() / (self.patches_per_side * self.patches_per_side);
        Ok(PatchGrid::from_f32(
            self.patches_per_side,
            self.patches_per_side,
            dim,
            &data,
        ))
    }

    fn grid_shape(&self) -> (usize, usize) {
        (self.patches_per_side, self.patches_per_side)
    }

    fn dim(&self) -> usize {
        self.hidden_size()
    }
}

impl TextEmbedder for SigLip2 {
    fn embed_text(&mut self, text: &str) -> Result<Vec<f32>, String> {
        let ids = self.tokenize(text)?;
        let features = self
            .model
            .text_model
            .forward(&ids)
            .map_err(|e| e.to_string())?;
        let mut vector = features
            .to_dtype(DType::F32)
            .and_then(|t| t.flatten_all())
            .and_then(|t| t.to_vec1::<f32>())
            .map_err(|e| e.to_string())?;
        normalize(&mut vector);
        Ok(vector)
    }
}
