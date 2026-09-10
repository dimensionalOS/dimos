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

//! Hyperspace: open-vocabulary 3D semantic querying over camera recordings.
//! One struct ([`Hyperspace`]) holds all state; feed it transforms, camera
//! intrinsics, RGB frames and depth frames, then ask it text questions and get
//! a voxel heat map back. Design: `plan.md`, background: `docs/principles.md`.

#[cfg(any(feature = "siglip", feature = "depth2depth"))]
pub mod backends;
pub mod depth;
pub mod embedder;
pub mod keyframe;
pub mod keyframe_store;
pub mod patch;
pub mod query;
pub mod state;
pub mod tf;

pub use depth::{DepthFuser, DepthImage, PassthroughDepthFuser};
pub use embedder::{Embedder, TextEmbedder};
pub use keyframe::KeyframeConfig;
pub use keyframe_store::Keyframe;
pub use patch::PatchGrid;
pub use query::{QueryConfig, VoxelHeatmap};
pub use state::SavedState;
pub use tf::{TfTree, Transform};

use keyframe::{BufferedFrame, GateVerdict, RollingBuffer};
use nalgebra::Point3;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Config {
    /// Voxel edge length in meters (output raster and lidar grid).
    pub voxel_size: f64,
    /// Hits required before a lidar voxel counts as occupied.
    pub min_hits: i64,
    /// Frame the optional lidar voxel grid lives in.
    pub world_frame: String,
    /// Frame used to measure camera speed for the quality gate.
    pub motion_reference_frame: String,
    /// Keyframing knobs per camera frame; `default_keyframe` for unlisted cameras.
    pub keyframe_per_camera: HashMap<String, KeyframeConfig>,
    pub default_keyframe: KeyframeConfig,
    /// A kept frame pairs with the depth image closest in time within this (s).
    pub depth_max_dt: f64,
    /// Depth images kept per depth sensor for pairing.
    pub depth_history: usize,
    /// Stride of the per-keyframe depth thumbnail (0 = don't keep one).
    pub depth_thumbnail_stride: u32,
    pub query: QueryConfig,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            voxel_size: 0.10,
            min_hits: 16,
            world_frame: "world".into(),
            motion_reference_frame: "odom".into(),
            keyframe_per_camera: HashMap::new(),
            default_keyframe: KeyframeConfig::default(),
            depth_max_dt: 0.05,
            depth_history: 64,
            depth_thumbnail_stride: 4,
            query: QueryConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    /// tf frame of the camera (also keys the camera; one entry per camera).
    pub camera_frame: String,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion_model: String,
    pub distortion: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ImageFrame {
    /// tf frame of the camera that took it (matches a CameraIntrinsics entry).
    pub camera_frame: String,
    pub timestamp: f64,
    pub width: u32,
    pub height: u32,
    /// Only "rgb8" is accepted.
    pub encoding: String,
    pub data: Vec<u8>,
}

/// A point cloud in some tf frame at some time. Points are transformed into the
/// world frame on ingestion.
#[derive(Debug, Clone)]
pub struct PointCloud {
    pub frame: String,
    pub timestamp: f64,
    pub points: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelSign {
    /// Point hits: increment voxel hit counts.
    Additive,
    /// Carving (rays passed through): decrement hit counts, floor at zero.
    Subtractive,
}

/// Why a frame was dropped, or that it was embedded / kept.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IngestStats {
    pub images: usize,
    pub gated_fast: usize,
    pub gated_exposure: usize,
    pub embedded: usize,
    pub kept: usize,
    pub kept_without_depth: usize,
    pub embed_seconds: f64,
    pub depth_seconds: f64,
}

/// Pixels + paired depth for a frame sitting in the rolling buffer.
struct Pending {
    image: ImageFrame,
    depth: Option<DepthImage>,
}

pub struct Hyperspace {
    pub config: Config,
    pub tf: TfTree,
    pub stats: IngestStats,
    intrinsics: HashMap<String, CameraIntrinsics>,
    embedder: Box<dyn Embedder>,
    text_embedder: Box<dyn TextEmbedder>,
    depth_fuser: Box<dyn DepthFuser>,
    buffers: HashMap<String, RollingBuffer<Pending>>,
    /// Recent depth images per depth sensor frame, newest last.
    depth_history: HashMap<String, VecDeque<DepthImage>>,
    keyframes: Vec<Keyframe>,
    /// Hit count per voxel index (world frame / voxel_size, floored). Optional lidar evidence.
    voxel_hits: HashMap<[i32; 3], i64>,
    background_cache: Option<Vec<Vec<f32>>>,
}

impl Hyperspace {
    pub fn new(
        config: Config,
        embedder: Box<dyn Embedder>,
        text_embedder: Box<dyn TextEmbedder>,
        depth_fuser: Box<dyn DepthFuser>,
    ) -> Self {
        Hyperspace {
            config,
            tf: TfTree::default(),
            stats: IngestStats::default(),
            intrinsics: HashMap::new(),
            embedder,
            text_embedder,
            depth_fuser,
            buffers: HashMap::new(),
            depth_history: HashMap::new(),
            keyframes: Vec::new(),
            voxel_hits: HashMap::new(),
            background_cache: None,
        }
    }

    /// Rebuild from a saved state (keyframes + tf + intrinsics); the model
    /// backends can be stubs when only queries are needed.
    pub fn from_saved(
        config: Config,
        saved: SavedState,
        embedder: Box<dyn Embedder>,
        text_embedder: Box<dyn TextEmbedder>,
        depth_fuser: Box<dyn DepthFuser>,
    ) -> Self {
        let mut hyperspace = Self::new(
            Config {
                voxel_size: saved.voxel_size,
                ..config
            },
            embedder,
            text_embedder,
            depth_fuser,
        );
        hyperspace.tf = saved.tf_tree();
        hyperspace.intrinsics = saved.intrinsics;
        hyperspace.keyframes = saved.keyframes;
        hyperspace
    }

    pub fn saved_state(&self) -> SavedState {
        SavedState {
            voxel_size: self.config.voxel_size,
            intrinsics: self.intrinsics.clone(),
            keyframes: self.keyframes.clone(),
            transforms: self
                .tf
                .samples()
                .map(|(parent, child, t, pose)| Transform::from_isometry(parent, child, t, pose))
                .collect(),
        }
    }

    /// Feed one transform into the internal TF tree (query it via `self.tf.get`).
    /// A transform with the same parent/child/timestamp as an earlier one replaces it.
    pub fn update(&mut self, transform: &Transform) {
        self.tf.insert(transform)
    }

    /// Register or refresh the intrinsics of one camera (keyed by camera_frame).
    pub fn set_camera_intrinsics(&mut self, intrinsics: CameraIntrinsics) {
        self.intrinsics
            .insert(intrinsics.camera_frame.clone(), intrinsics);
    }

    pub fn camera_intrinsics(&self, camera_frame: &str) -> Option<&CameraIntrinsics> {
        self.intrinsics.get(camera_frame)
    }

    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// Ingest one raw depth frame; kept for pairing with color frames.
    pub fn add_depth(&mut self, depth: DepthImage) {
        let history = self
            .depth_history
            .entry(depth.camera_frame.clone())
            .or_default();
        history.push_back(depth);
        while history.len() > self.config.depth_history.max(1) {
            history.pop_front();
        }
    }

    fn keyframe_config(&self, camera_frame: &str) -> KeyframeConfig {
        self.config
            .keyframe_per_camera
            .get(camera_frame)
            .cloned()
            .unwrap_or_else(|| self.config.default_keyframe.clone())
    }

    /// Ingest one RGB frame: quality gate -> embed -> rolling buffer -> maybe a keyframe.
    /// Returns Ok(true) when this call produced a keyframe.
    pub fn add_image(&mut self, frame: ImageFrame) -> Result<bool, String> {
        if frame.encoding != "rgb8" {
            return Err(format!(
                "unsupported encoding {:?}, want rgb8",
                frame.encoding
            ));
        }
        if frame.data.len() != (frame.width * frame.height * 3) as usize {
            return Err("image data length does not match width*height*3".into());
        }
        self.stats.images += 1;
        let keyframe_config = self.keyframe_config(&frame.camera_frame);
        let speeds = self.tf.speeds(
            &self.config.motion_reference_frame,
            &frame.camera_frame,
            frame.timestamp,
            keyframe_config.velocity_half_window,
        );
        match keyframe::quality_gate(&keyframe_config, &frame, speeds) {
            GateVerdict::Pass => {}
            GateVerdict::TooFast { .. } => {
                self.stats.gated_fast += 1;
                return Ok(false);
            }
            _ => {
                self.stats.gated_exposure += 1;
                return Ok(false);
            }
        }
        let started = std::time::Instant::now();
        let grid = self.embedder.embed(&frame)?;
        self.stats.embed_seconds += started.elapsed().as_secs_f64();
        self.stats.embedded += 1;
        // Quality: slower camera = better; sharpness metrics can be plugged in later.
        let quality = match speeds {
            Some((angular, linear)) => 1.0 / (1.0 + angular as f32 + 0.25 * linear as f32),
            None => 1.0,
        };
        let depth = self.nearest_depth(&frame.camera_frame, frame.timestamp);
        let buffer = self
            .buffers
            .entry(frame.camera_frame.clone())
            .or_insert_with(|| RollingBuffer::new(keyframe_config));
        let kept = buffer.push(BufferedFrame {
            timestamp: frame.timestamp,
            grid,
            quality,
            payload: Pending {
                image: frame,
                depth,
            },
        });
        match kept {
            Some(kept) => {
                self.finish_keyframe(kept)?;
                Ok(true)
            }
            None => Ok(false),
        }
    }

    /// End of stream: judge what is still in the buffers. Returns keyframes added.
    pub fn flush(&mut self) -> Result<usize, String> {
        let cameras: Vec<String> = self.buffers.keys().cloned().collect();
        let mut added = 0;
        for camera in cameras {
            let kept = self
                .buffers
                .get_mut(&camera)
                .expect("buffer exists")
                .flush();
            for frame in kept {
                self.finish_keyframe(frame)?;
                added += 1;
            }
        }
        Ok(added)
    }

    /// Depth image closest in time (any depth sensor) within `depth_max_dt`,
    /// preferring one whose frame is tf-connected to the color camera.
    fn nearest_depth(&self, camera_frame: &str, timestamp: f64) -> Option<DepthImage> {
        let mut best: Option<(f64, &DepthImage)> = None;
        for history in self.depth_history.values() {
            for depth in history {
                let dt = (depth.timestamp - timestamp).abs();
                if dt > self.config.depth_max_dt {
                    continue;
                }
                let connected = depth.camera_frame == camera_frame
                    || self
                        .tf
                        .get(camera_frame, &depth.camera_frame, timestamp)
                        .is_some();
                if !connected {
                    continue;
                }
                if best.is_none_or(|(best_dt, _)| dt < best_dt) {
                    best = Some((dt, depth));
                }
            }
        }
        best.map(|(_, depth)| depth.clone())
    }

    fn finish_keyframe(&mut self, kept: BufferedFrame<Pending>) -> Result<(), String> {
        let image = kept.payload.image;
        let (rows, cols) = (kept.grid.rows, kept.grid.cols);
        let color = self
            .intrinsics
            .get(&image.camera_frame)
            .ok_or_else(|| format!("no intrinsics for {}", image.camera_frame))?
            .clone();
        let (patch_depth, thumbnail) = match kept.payload.depth {
            Some(depth) => {
                let started = std::time::Instant::now();
                let depth_intrinsics = self
                    .intrinsics
                    .get(&depth.camera_frame)
                    .ok_or_else(|| format!("no intrinsics for {}", depth.camera_frame))?;
                let color_from_depth = if depth.camera_frame == image.camera_frame {
                    nalgebra::Isometry3::identity()
                } else {
                    self.tf
                        .get(&image.camera_frame, &depth.camera_frame, image.timestamp)
                        .ok_or("depth frame not connected")?
                };
                let raw =
                    depth::reproject_depth(&depth, depth_intrinsics, &color, &color_from_depth);
                let fused = self.depth_fuser.fuse(&image, &raw)?;
                self.stats.depth_seconds += started.elapsed().as_secs_f64();
                let patch_depth = depth::per_patch_depth(
                    &fused,
                    color.width as usize,
                    color.height as usize,
                    rows,
                    cols,
                );
                let stride = self.config.depth_thumbnail_stride as usize;
                let thumbnail = if stride == 0 {
                    Vec::new()
                } else {
                    let mut out = Vec::new();
                    for y in (0..color.height as usize).step_by(stride) {
                        for x in (0..color.width as usize).step_by(stride) {
                            let z = fused[y * color.width as usize + x];
                            out.push(if z > 0.0 && z.is_finite() {
                                (z * 1000.0).min(65535.0) as u16
                            } else {
                                0
                            });
                        }
                    }
                    out
                };
                (patch_depth, thumbnail)
            }
            None => {
                self.stats.kept_without_depth += 1;
                (vec![f32::NAN; rows * cols], Vec::new())
            }
        };
        self.keyframes.push(Keyframe {
            camera_frame: image.camera_frame,
            timestamp: image.timestamp,
            grid: kept.grid,
            patch_depth,
            depth_thumbnail_mm: thumbnail,
            depth_stride: self.config.depth_thumbnail_stride,
            width: image.width,
            height: image.height,
        });
        self.stats.kept += 1;
        Ok(())
    }

    fn background_embeddings(&mut self) -> Result<Vec<Vec<f32>>, String> {
        if let Some(cached) = &self.background_cache {
            return Ok(cached.clone());
        }
        let mut out = Vec::new();
        for prompt in self.config.query.background_prompts.clone() {
            out.push(self.text_embedder.embed_text(&prompt)?);
        }
        self.background_cache = Some(out.clone());
        Ok(out)
    }

    /// Text query -> voxel heat map in `target_frame`, placing every keyframe
    /// through the *current* tf tree.
    pub fn query(&mut self, text: &str, target_frame: &str) -> Result<VoxelHeatmap, String> {
        let query = self.text_embedder.embed_text(text)?;
        self.query_embedding(&query, target_frame)
    }

    pub fn query_embedding(
        &mut self,
        query: &[f32],
        target_frame: &str,
    ) -> Result<VoxelHeatmap, String> {
        let cutoff = self.config.query.background_synonym_cutoff;
        let backgrounds: Vec<Vec<f32>> = self
            .background_embeddings()?
            .into_iter()
            .filter(|background| {
                background
                    .iter()
                    .zip(query)
                    .map(|(a, b)| a * b)
                    .sum::<f32>()
                    < cutoff
            })
            .collect();
        let hot = query::hot_patches(&self.keyframes, query, &backgrounds, &self.config.query);
        let mut heatmap = query::heatmap(
            &self.keyframes,
            &hot,
            &self.intrinsics,
            &self.tf,
            target_frame,
            self.config.voxel_size,
            &self.config.query,
        );
        heatmap.stats.background_prompts_used = backgrounds.len();
        Ok(heatmap)
    }

    /// Occupied voxels in `target_frame` from the keyframes' depth thumbnails
    /// (for rendering the scene); each voxel carries how many depth samples hit it.
    pub fn scene_voxels(&self, target_frame: &str, min_samples: u32) -> Vec<([i32; 3], u32)> {
        let mut counts: HashMap<[i32; 3], u32> = HashMap::new();
        for keyframe in &self.keyframes {
            if keyframe.depth_thumbnail_mm.is_empty() || keyframe.depth_stride == 0 {
                continue;
            }
            let Some(camera) = self.intrinsics.get(&keyframe.camera_frame) else {
                continue;
            };
            let pose = if keyframe.camera_frame == target_frame {
                Some(nalgebra::Isometry3::identity())
            } else {
                self.tf
                    .get(target_frame, &keyframe.camera_frame, keyframe.timestamp)
            };
            let Some(pose) = pose else { continue };
            let stride = keyframe.depth_stride as usize;
            let cols = (keyframe.width as usize).div_ceil(stride);
            for (index, mm) in keyframe.depth_thumbnail_mm.iter().enumerate() {
                if *mm == 0 {
                    continue;
                }
                let z = *mm as f64 * 0.001;
                let u = (index % cols * stride) as f64;
                let v = (index / cols * stride) as f64;
                let point = pose
                    * Point3::new(
                        (u - camera.cx) / camera.fx * z,
                        (v - camera.cy) / camera.fy * z,
                        z,
                    );
                *counts
                    .entry(self.voxel_index(&[point.x, point.y, point.z]))
                    .or_insert(0) += 1;
            }
        }
        counts
            .into_iter()
            .filter(|(_, n)| *n >= min_samples)
            .collect()
    }

    /// Ingest a point cloud as additive (hits) or subtractive (carving) voxel
    /// evidence. Points are moved into the world frame via the TF tree at the
    /// cloud's timestamp; returns Err if the cloud's frame is not connected.
    pub fn add_pointcloud(&mut self, cloud: &PointCloud, sign: VoxelSign) -> Result<(), String> {
        let world_from_cloud = if cloud.frame == self.config.world_frame {
            nalgebra::Isometry3::identity()
        } else {
            self.tf
                .get(&self.config.world_frame, &cloud.frame, cloud.timestamp)
                .ok_or_else(|| {
                    format!(
                        "no tf path from {} to {} at t={}",
                        cloud.frame, self.config.world_frame, cloud.timestamp
                    )
                })?
        };
        for point in &cloud.points {
            let world_point = world_from_cloud * Point3::new(point[0], point[1], point[2]);
            let index = self.voxel_index(&[world_point.x, world_point.y, world_point.z]);
            match sign {
                VoxelSign::Additive => {
                    *self.voxel_hits.entry(index).or_insert(0) += 1;
                }
                VoxelSign::Subtractive => {
                    if let Some(hits) = self.voxel_hits.get_mut(&index) {
                        *hits -= 1;
                        if *hits <= 0 {
                            self.voxel_hits.remove(&index);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    pub fn voxel_index(&self, point: &[f64; 3]) -> [i32; 3] {
        [
            (point[0] / self.config.voxel_size).floor() as i32,
            (point[1] / self.config.voxel_size).floor() as i32,
            (point[2] / self.config.voxel_size).floor() as i32,
        ]
    }

    /// Voxel indices whose lidar hit count has reached config.min_hits.
    pub fn occupied_voxels(&self) -> impl Iterator<Item = [i32; 3]> + '_ {
        self.voxel_hits
            .iter()
            .filter(|(_, hits)| **hits >= self.config.min_hits)
            .map(|(index, _)| *index)
    }

    pub fn pending_image_count(&self) -> usize {
        self.buffers.values().map(|b| b.len()).sum()
    }
}

/// Convenience: a Hyperspace with stub backends (hash embedder, table text
/// embedder, passthrough depth). Fine for tests and for loading saved states.
pub fn with_stub_backends(config: Config) -> Hyperspace {
    Hyperspace::new(
        config,
        Box::new(embedder::HashEmbedder::default()),
        Box::new(embedder::TableTextEmbedder::default()),
        Box::new(PassthroughDepthFuser),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world_cloud(points: Vec<[f64; 3]>) -> PointCloud {
        PointCloud {
            frame: "world".into(),
            timestamp: 0.0,
            points,
        }
    }

    #[test]
    fn additive_then_subtractive_voxels() {
        let mut hyperspace = with_stub_backends(Config {
            min_hits: 2,
            ..Config::default()
        });
        let point = [0.05, 0.05, 0.05];
        hyperspace
            .add_pointcloud(&world_cloud(vec![point]), VoxelSign::Additive)
            .unwrap();
        assert_eq!(hyperspace.occupied_voxels().count(), 0);
        hyperspace
            .add_pointcloud(&world_cloud(vec![point]), VoxelSign::Additive)
            .unwrap();
        assert_eq!(hyperspace.occupied_voxels().count(), 1);
        hyperspace
            .add_pointcloud(&world_cloud(vec![point]), VoxelSign::Subtractive)
            .unwrap();
        assert_eq!(hyperspace.occupied_voxels().count(), 0);
    }

    #[test]
    fn pointcloud_is_transformed_through_tf() {
        let mut hyperspace = with_stub_backends(Config {
            min_hits: 1,
            ..Config::default()
        });
        hyperspace.update(&Transform {
            parent_frame: "world".into(),
            child_frame: "lidar".into(),
            timestamp: 0.0,
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        });
        let cloud = PointCloud {
            frame: "lidar".into(),
            timestamp: 0.0,
            points: vec![[0.05, 0.05, 0.05]],
        };
        hyperspace
            .add_pointcloud(&cloud, VoxelSign::Additive)
            .unwrap();
        assert_eq!(hyperspace.occupied_voxels().next().unwrap(), [10, 0, 0]);
    }

    #[test]
    fn unknown_frame_errors() {
        let mut hyperspace = with_stub_backends(Config::default());
        let cloud = PointCloud {
            frame: "ghost".into(),
            timestamp: 0.0,
            points: vec![[0.0; 3]],
        };
        assert!(hyperspace
            .add_pointcloud(&cloud, VoxelSign::Additive)
            .is_err());
    }

    #[test]
    fn images_flow_through_the_buffer() {
        let config = Config {
            default_keyframe: KeyframeConfig {
                buffer_len: 3,
                ..KeyframeConfig::permissive()
            },
            ..Config::default()
        };
        let mut hyperspace = with_stub_backends(config);
        hyperspace.set_camera_intrinsics(CameraIntrinsics {
            camera_frame: "camera".into(),
            width: 64,
            height: 48,
            fx: 40.0,
            fy: 40.0,
            cx: 32.0,
            cy: 24.0,
            distortion_model: "plumb_bob".into(),
            distortion: vec![],
        });
        for i in 0..6 {
            let frame = ImageFrame {
                camera_frame: "camera".into(),
                timestamp: i as f64,
                width: 64,
                height: 48,
                encoding: "rgb8".into(),
                data: vec![(i * 40) as u8; 64 * 48 * 3],
            };
            hyperspace.add_image(frame).unwrap();
        }
        hyperspace.flush().unwrap();
        assert_eq!(hyperspace.stats.images, 6);
        assert!(hyperspace.stats.kept >= 1);
        assert_eq!(hyperspace.pending_image_count(), 0);
        assert!(hyperspace.saved_state().keyframes.len() == hyperspace.stats.kept);
    }
}
