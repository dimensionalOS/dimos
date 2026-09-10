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

//! The module's launch config, and the backends it selects.
//!
//! Native configs cannot hold `Option`, so every switchable gate uses a
//! sentinel: a negative limit is "off", an empty path is "no model".

use dimos_module::native_config;
use hyperspace::embedder::{HashEmbedder, TableTextEmbedder};
use hyperspace::keyframe::KeyframeConfig;
use hyperspace::query::QueryConfig;
use hyperspace::{DepthFuser, Embedder, PassthroughDepthFuser, TextEmbedder};

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Voxel edge length of the answer raster, in metres.
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f64,
    /// Frame queries are answered in unless a request names another.
    pub world_frame: String,
    /// Frame the quality gate measures camera motion against.
    pub motion_reference_frame: String,

    /// SigLIP2 snapshot directory (config.json, tokenizer.json, *.safetensors).
    /// Empty runs the stub embedder: the pipeline works, the answers do not mean
    /// anything. Requires the `siglip` cargo feature.
    pub model_dir: String,
    /// depth2depth weights (dinov2_vits14.safetensors, da2_head_vits.safetensors).
    /// Empty uses the raw sensor depth as it arrives. Requires `depth2depth`.
    pub depth_weights_dir: String,
    /// Run the models on CUDA. Requires the `cuda` cargo feature.
    pub cuda: bool,

    /// Frames held before the middle one is judged; odd, 11 at 5 Hz is ~2 s.
    #[validate(range(min = 1))]
    pub buffer_len: u32,
    /// Mean per-patch (1 - cosine) against the last kept keyframe needed to keep one.
    #[validate(range(min = 0.0))]
    pub novelty_threshold: f32,
    /// Single-patch change that keeps a frame even when the view barely moved.
    /// Negative disables it.
    pub patch_novelty_threshold: f32,
    /// Drop frames whose camera turns faster than this (rad/s). Negative disables.
    pub max_angular_velocity: f64,
    /// Drop frames whose camera moves faster than this (m/s). Negative disables.
    pub max_linear_velocity: f64,
    /// Drop frames darker than this fraction of near-black pixels. Negative disables.
    pub max_dark_fraction: f32,
    /// Drop frames brighter than this fraction of near-white pixels. Negative disables.
    pub max_bright_fraction: f32,
    /// Never keep two keyframes closer together than this (s). Negative disables.
    pub min_keyframe_interval: f64,

    /// A colour frame pairs with the depth frame within this many seconds of it.
    #[validate(range(min = 0.0))]
    pub depth_max_dt: f64,
    /// Depth frames buffered per sensor while waiting for their colour frame.
    #[validate(range(min = 1))]
    pub depth_history: u32,
    /// Stride of the depth thumbnail kept per keyframe for `scene_map`.
    /// Zero keeps none, and then `scene_map` stays empty.
    pub depth_thumbnail_stride: u32,

    /// Patch score (query minus best background prompt) needed to be "hot".
    pub hot_threshold: f32,
    /// Ceiling on hot patches per query; the highest scoring ones win.
    #[validate(range(min = 1))]
    pub max_hot_patches: u32,
    /// Pyramid caps as fractions of the patch depth.
    #[validate(range(exclusive_min = 0.0))]
    pub cap_near: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub cap_far: f32,
    /// Prompts contrasted against the query, comma separated.
    pub background_prompts: String,

    /// Publish `scene_map` every Nth kept keyframe. Zero never publishes it.
    pub scene_emit_every: u32,
    /// Depth samples a voxel needs before it appears in `scene_map`.
    #[validate(range(min = 1))]
    pub scene_min_samples: u32,
}

/// Negative means "gate off".
fn optional_f32(value: f32) -> Option<f32> {
    (value >= 0.0).then_some(value)
}

fn optional_f64(value: f64) -> Option<f64> {
    (value >= 0.0).then_some(value)
}

impl Config {
    pub fn keyframe_config(&self) -> KeyframeConfig {
        KeyframeConfig {
            buffer_len: self.buffer_len as usize,
            max_angular_velocity: optional_f64(self.max_angular_velocity),
            max_linear_velocity: optional_f64(self.max_linear_velocity),
            max_dark_fraction: optional_f32(self.max_dark_fraction),
            max_bright_fraction: optional_f32(self.max_bright_fraction),
            min_interval: optional_f64(self.min_keyframe_interval),
            novelty_threshold: self.novelty_threshold,
            patch_novelty_threshold: optional_f32(self.patch_novelty_threshold),
            ..KeyframeConfig::default()
        }
    }

    pub fn query_config(&self) -> QueryConfig {
        let prompts: Vec<String> = self
            .background_prompts
            .split(',')
            .map(|prompt| prompt.trim().to_string())
            .filter(|prompt| !prompt.is_empty())
            .collect();
        let defaults = QueryConfig::default();
        QueryConfig {
            hot_threshold: self.hot_threshold,
            max_hot_patches: self.max_hot_patches as usize,
            cap_near: self.cap_near,
            cap_far: self.cap_far,
            background_prompts: if prompts.is_empty() {
                defaults.background_prompts.clone()
            } else {
                prompts
            },
            ..defaults
        }
    }

    pub fn hyperspace_config(&self) -> hyperspace::Config {
        hyperspace::Config {
            voxel_size: self.voxel_size,
            world_frame: self.world_frame.clone(),
            motion_reference_frame: self.motion_reference_frame.clone(),
            default_keyframe: self.keyframe_config(),
            depth_max_dt: self.depth_max_dt,
            depth_history: self.depth_history as usize,
            depth_thumbnail_stride: self.depth_thumbnail_stride,
            query: self.query_config(),
            ..hyperspace::Config::default()
        }
    }
}

type Backends = (
    Box<dyn Embedder>,
    Box<dyn TextEmbedder>,
    Box<dyn DepthFuser>,
);

/// Pick the embedder and depth fuser the config asks for. Returns Err when a
/// model is configured but the binary was built without the feature that loads
/// it, so a robot cannot quietly answer nonsense.
pub fn build_backends(config: &Config) -> Result<Backends, String> {
    let (embedder, text_embedder): (Box<dyn Embedder>, Box<dyn TextEmbedder>) =
        if config.model_dir.is_empty() {
            (
                Box::new(HashEmbedder::default()),
                Box::new(TableTextEmbedder::default()),
            )
        } else {
            #[cfg(feature = "siglip")]
            {
                let model = load_siglip(&config.model_dir, config.cuda)?;
                let text = load_siglip(&config.model_dir, config.cuda)?;
                (Box::new(model), Box::new(text))
            }
            #[cfg(not(feature = "siglip"))]
            {
                return Err(format!(
                "model_dir is set to {:?} but this binary was built without the `siglip` feature",
                config.model_dir
            ));
            }
        };

    let depth_fuser: Box<dyn DepthFuser> = if config.depth_weights_dir.is_empty() {
        Box::new(PassthroughDepthFuser)
    } else {
        #[cfg(feature = "depth2depth")]
        {
            Box::new(load_depth2depth(&config.depth_weights_dir, config.cuda)?)
        }
        #[cfg(not(feature = "depth2depth"))]
        {
            return Err(format!(
                "depth_weights_dir is set to {:?} but this binary was built without the `depth2depth` feature",
                config.depth_weights_dir
            ));
        }
    };

    Ok((embedder, text_embedder, depth_fuser))
}

#[cfg(feature = "siglip")]
fn load_siglip(
    model_dir: &str,
    cuda: bool,
) -> Result<hyperspace::backends::siglip::SigLip2, String> {
    use hyperspace::backends::siglip::candle::{DType, Device};
    let device = if cuda {
        Device::new_cuda(0).map_err(|e| format!("cuda device: {e}"))?
    } else {
        Device::Cpu
    };
    let dtype = if cuda { DType::F16 } else { DType::F32 };
    hyperspace::backends::siglip::SigLip2::load(std::path::Path::new(model_dir), device, dtype)
}

#[cfg(feature = "depth2depth")]
fn load_depth2depth(
    weights_dir: &str,
    cuda: bool,
) -> Result<hyperspace::backends::depth2depth::Depth2DepthFuser, String> {
    use hyperspace::backends::siglip::candle::{DType, Device};
    let device = if cuda {
        Device::new_cuda(0).map_err(|e| format!("cuda device: {e}"))?
    } else {
        Device::Cpu
    };
    let dtype = if cuda { DType::F16 } else { DType::F32 };
    hyperspace::backends::depth2depth::Depth2DepthFuser::load(
        std::path::Path::new(weights_dir),
        device,
        dtype,
        Default::default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            voxel_size: 0.1,
            world_frame: "odom".into(),
            motion_reference_frame: "odom".into(),
            model_dir: String::new(),
            depth_weights_dir: String::new(),
            cuda: false,
            buffer_len: 11,
            novelty_threshold: 0.05,
            patch_novelty_threshold: 0.5,
            max_angular_velocity: 1.5,
            max_linear_velocity: -1.0,
            max_dark_fraction: 0.6,
            max_bright_fraction: -1.0,
            min_keyframe_interval: 0.1,
            depth_max_dt: 0.05,
            depth_history: 64,
            depth_thumbnail_stride: 4,
            hot_threshold: 0.02,
            max_hot_patches: 6000,
            cap_near: 0.9,
            cap_far: 1.1,
            background_prompts: "a photo, a wall".into(),
            scene_emit_every: 10,
            scene_min_samples: 3,
        }
    }

    #[test]
    fn negative_limits_turn_gates_off() {
        let keyframe = config().keyframe_config();
        assert_eq!(keyframe.max_angular_velocity, Some(1.5));
        assert_eq!(keyframe.max_linear_velocity, None);
        assert_eq!(keyframe.max_dark_fraction, Some(0.6));
        assert_eq!(keyframe.max_bright_fraction, None);
        assert_eq!(keyframe.buffer_len, 11);
    }

    #[test]
    fn background_prompts_are_split_and_fall_back() {
        assert_eq!(
            config().query_config().background_prompts,
            ["a photo", "a wall"]
        );
        let mut empty = config();
        empty.background_prompts = "  ,  ".into();
        assert_eq!(
            empty.query_config().background_prompts,
            QueryConfig::default().background_prompts
        );
    }

    #[test]
    fn no_model_dir_builds_stub_backends() {
        assert!(build_backends(&config()).is_ok());
    }

    #[cfg(not(feature = "siglip"))]
    #[test]
    fn a_model_dir_without_the_feature_is_an_error() {
        let mut with_model = config();
        with_model.model_dir = "/models/siglip2".into();
        assert!(build_backends(&with_model).is_err());
    }
}
