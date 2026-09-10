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

//! Query side: score stored patches, turn hot patches into depth-capped
//! pyramids, rasterize them into a sparse voxel hash, pool per voxel.

use crate::keyframe_store::Keyframe;
use crate::tf::TfTree;
use crate::CameraIntrinsics;
use nalgebra::{Isometry3, Point3};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QueryConfig {
    /// Patch contrast score (query cosine minus best background cosine) needed to be hot.
    pub hot_threshold: f32,
    /// Hard cap on hot patches per query (highest scores win), bounds rasterization cost.
    pub max_hot_patches: usize,
    /// Background prompts contrasted against the query.
    pub background_prompts: Vec<String>,
    /// Background prompts closer than this (text-text cosine) to the query are dropped.
    pub background_synonym_cutoff: f32,
    /// Pyramid caps as fractions of the patch depth.
    pub cap_near: f32,
    pub cap_far: f32,
    /// Log-sum-exp temperature across frames.
    pub lse_temperature: f32,
    /// Yaw bins around a voxel; a bin is hot when its best frame score passes `yaw_hot_threshold`.
    pub yaw_bins: usize,
    pub yaw_hot_threshold: f32,
    /// Percentile (0..1) that maps to score 1.0.
    pub normalize_percentile: f32,
}

impl Default for QueryConfig {
    fn default() -> Self {
        QueryConfig {
            hot_threshold: 0.02,
            max_hot_patches: 6000,
            background_prompts: [
                "a photo",
                "an office",
                "a room",
                "an indoor scene",
                "a wall",
                "a floor",
                "a ceiling",
                "furniture",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect(),
            background_synonym_cutoff: 0.85,
            cap_near: 0.9,
            cap_far: 1.1,
            lse_temperature: 0.02,
            yaw_bins: 8,
            yaw_hot_threshold: 0.04,
            normalize_percentile: 0.999,
        }
    }
}

/// Sparse voxel heat map in `frame`: voxel index -> score in [0, 1].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoxelHeatmap {
    pub frame: String,
    pub voxel_size: f64,
    pub voxels: Vec<([i32; 3], f32)>,
    pub stats: QueryStats,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct QueryStats {
    pub keyframes: usize,
    pub keyframes_placed: usize,
    pub hot_patches: usize,
    pub hot_patches_without_depth: usize,
    pub voxels_touched: usize,
    pub background_prompts_used: usize,
}

/// One hot patch, ready to be placed.
struct HotPatch {
    keyframe: usize,
    patch: usize,
    score: f32,
}

/// Per-voxel evidence: (keyframe index, score, yaw bin).
type Evidence = Vec<(u32, f32, u8)>;
/// One rasterized voxel hit: (voxel index, keyframe index, score, yaw bin).
type VoxelHit = ([i32; 3], u32, f32, u8);

/// Score every patch of every keyframe against `query` minus the best of `backgrounds`.
pub fn hot_patches(
    keyframes: &[Keyframe],
    query: &[f32],
    backgrounds: &[Vec<f32>],
    config: &QueryConfig,
) -> Vec<(usize, usize, f32)> {
    let mut hot: Vec<(usize, usize, f32)> = keyframes
        .par_iter()
        .enumerate()
        .flat_map_iter(|(keyframe_index, keyframe)| {
            let query_scores = keyframe.grid.scores(query);
            let background_scores: Vec<Vec<f32>> = backgrounds
                .iter()
                .map(|b| keyframe.grid.scores(b))
                .collect();
            (0..query_scores.len())
                .filter_map(|patch| {
                    let background = background_scores
                        .iter()
                        .map(|s| s[patch])
                        .fold(f32::NEG_INFINITY, f32::max);
                    let contrast = if background.is_finite() {
                        query_scores[patch] - background
                    } else {
                        query_scores[patch]
                    };
                    (contrast > config.hot_threshold).then_some((keyframe_index, patch, contrast))
                })
                .collect::<Vec<_>>()
        })
        .collect();
    hot.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    hot.truncate(config.max_hot_patches);
    hot
}

/// Place hot patches as depth-capped pyramids in `target_frame`, rasterize into
/// voxels, pool. Intrinsics are looked up by camera frame.
pub fn heatmap(
    keyframes: &[Keyframe],
    hot: &[(usize, usize, f32)],
    intrinsics: &HashMap<String, CameraIntrinsics>,
    tf: &TfTree,
    target_frame: &str,
    voxel_size: f64,
    config: &QueryConfig,
) -> VoxelHeatmap {
    let mut stats = QueryStats {
        keyframes: keyframes.len(),
        hot_patches: hot.len(),
        ..Default::default()
    };
    // Resolve each keyframe's pose once.
    let mut poses: HashMap<usize, Isometry3<f64>> = HashMap::new();
    for (keyframe_index, _, _) in hot {
        if poses.contains_key(keyframe_index) {
            continue;
        }
        let keyframe = &keyframes[*keyframe_index];
        let pose = if keyframe.camera_frame == target_frame {
            Some(Isometry3::identity())
        } else {
            tf.get(target_frame, &keyframe.camera_frame, keyframe.timestamp)
        };
        if let Some(pose) = pose {
            poses.insert(*keyframe_index, pose);
        }
    }
    stats.keyframes_placed = poses.len();

    let grouped: Vec<HotPatch> = hot
        .iter()
        .map(|(k, p, s)| HotPatch {
            keyframe: *k,
            patch: *p,
            score: *s,
        })
        .collect();
    let evidence_per_patch: Vec<Option<Vec<VoxelHit>>> = grouped
        .par_iter()
        .map(|hot_patch| {
            let keyframe = &keyframes[hot_patch.keyframe];
            let pose = poses.get(&hot_patch.keyframe)?;
            let camera = intrinsics.get(&keyframe.camera_frame)?;
            let depth = keyframe.patch_depth[hot_patch.patch];
            if depth <= 0.0 || !depth.is_finite() {
                return Some(Vec::new()); // counted as "without depth" below
            }
            Some(rasterize_pyramid(
                keyframe, camera, pose, hot_patch, depth, voxel_size, config,
            ))
        })
        .collect();

    let mut voxels: HashMap<[i32; 3], Evidence> = HashMap::new();
    for (hot_patch, evidence) in grouped.iter().zip(evidence_per_patch) {
        let Some(evidence) = evidence else { continue };
        let keyframe = &keyframes[hot_patch.keyframe];
        let depth = keyframe.patch_depth[hot_patch.patch];
        if evidence.is_empty() && !(depth > 0.0 && depth.is_finite()) {
            stats.hot_patches_without_depth += 1;
        }
        for (index, keyframe_index, score, yaw_bin) in evidence {
            voxels
                .entry(index)
                .or_default()
                .push((keyframe_index, score, yaw_bin));
        }
    }
    stats.voxels_touched = voxels.len();

    let mut scored: Vec<([i32; 3], f32)> = voxels
        .into_par_iter()
        .map(|(index, evidence)| (index, pool(&evidence, config)))
        .collect();
    normalize_scores(&mut scored, config.normalize_percentile);
    // Deterministic order: score desc, then voxel index (ties are common after clamping).
    scored.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap().then_with(|| a.0.cmp(&b.0)));
    VoxelHeatmap {
        frame: target_frame.to_string(),
        voxel_size,
        voxels: scored,
        stats,
    }
}

/// Voxels whose centers fall inside the patch's pixel rectangle and within
/// [cap_near·d, cap_far·d] along the camera z axis.
fn rasterize_pyramid(
    keyframe: &Keyframe,
    camera: &CameraIntrinsics,
    target_from_camera: &Isometry3<f64>,
    hot_patch: &HotPatch,
    depth: f32,
    voxel_size: f64,
    config: &QueryConfig,
) -> Vec<VoxelHit> {
    let (rows, cols) = (keyframe.grid.rows, keyframe.grid.cols);
    let row = hot_patch.patch / cols;
    let col = hot_patch.patch % cols;
    let (w, h) = (camera.width as f64, camera.height as f64);
    let u0 = col as f64 * w / cols as f64;
    let u1 = (col + 1) as f64 * w / cols as f64;
    let v0 = row as f64 * h / rows as f64;
    let v1 = (row + 1) as f64 * h / rows as f64;
    let near = (depth * config.cap_near) as f64;
    let far = (depth * config.cap_far) as f64;

    // Bounding box in the target frame from the 8 corners of the truncated pyramid.
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for z in [near, far] {
        for (u, v) in [(u0, v0), (u1, v0), (u0, v1), (u1, v1)] {
            let corner = target_from_camera
                * Point3::new(
                    (u - camera.cx) / camera.fx * z,
                    (v - camera.cy) / camera.fy * z,
                    z,
                );
            for axis in 0..3 {
                min[axis] = min[axis].min(corner[axis]);
                max[axis] = max[axis].max(corner[axis]);
            }
        }
    }
    let camera_from_target = target_from_camera.inverse();
    let camera_position = target_from_camera * Point3::origin();
    let lo = min.map(|m| (m / voxel_size).floor() as i32);
    let hi = max.map(|m| (m / voxel_size).floor() as i32);
    let mut out = Vec::new();
    for x in lo[0]..=hi[0] {
        for y in lo[1]..=hi[1] {
            for z in lo[2]..=hi[2] {
                let center = Point3::new(
                    (x as f64 + 0.5) * voxel_size,
                    (y as f64 + 0.5) * voxel_size,
                    (z as f64 + 0.5) * voxel_size,
                );
                let in_camera = camera_from_target * center;
                if in_camera.z < near || in_camera.z > far {
                    continue;
                }
                let u = in_camera.x / in_camera.z * camera.fx + camera.cx;
                let v = in_camera.y / in_camera.z * camera.fy + camera.cy;
                if u < u0 || u >= u1 || v < v0 || v >= v1 {
                    continue;
                }
                let bearing = (camera_position.y - center.y).atan2(camera_position.x - center.x);
                let bin = (((bearing + std::f64::consts::PI) / (2.0 * std::f64::consts::PI))
                    * config.yaw_bins as f64)
                    .floor() as usize
                    % config.yaw_bins.max(1);
                out.push((
                    [x, y, z],
                    hot_patch.keyframe as u32,
                    hot_patch.score,
                    bin as u8,
                ));
            }
        }
    }
    out
}

/// max per frame -> log-sum-exp across frames -> × sqrt(hot yaw bins).
pub fn pool(evidence: &[(u32, f32, u8)], config: &QueryConfig) -> f32 {
    let mut best_per_frame: HashMap<u32, (f32, u8)> = HashMap::new();
    for (keyframe, score, bin) in evidence {
        let entry = best_per_frame.entry(*keyframe).or_insert((*score, *bin));
        if *score > entry.0 {
            *entry = (*score, *bin);
        }
    }
    let t = config.lse_temperature.max(1e-6);
    let max_score = best_per_frame
        .values()
        .map(|(s, _)| *s)
        .fold(f32::NEG_INFINITY, f32::max);
    let lse = max_score
        + t * best_per_frame
            .values()
            .map(|(s, _)| ((s - max_score) / t).exp())
            .sum::<f32>()
            .ln();
    let bin_count = config.yaw_bins.max(1);
    let mut hot_bins = vec![false; bin_count];
    for (score, bin) in best_per_frame.values() {
        if *score > config.yaw_hot_threshold {
            hot_bins[*bin as usize % bin_count] = true;
        }
    }
    let bins = hot_bins.iter().filter(|b| **b).count().max(1) as f32;
    lse * bins.sqrt()
}

fn normalize_scores(scored: &mut [([i32; 3], f32)], percentile: f32) {
    if scored.is_empty() {
        return;
    }
    let mut values: Vec<f32> = scored.iter().map(|(_, s)| *s).collect();
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let rank = ((values.len() as f32 - 1.0) * percentile.clamp(0.0, 1.0)).round() as usize;
    let top = values[rank].max(1e-9);
    for (_, score) in scored.iter_mut() {
        *score = (*score / top).clamp(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pooling_rewards_multiple_frames_and_directions() {
        let config = QueryConfig {
            lse_temperature: 0.02,
            yaw_hot_threshold: 0.04,
            ..Default::default()
        };
        let one = pool(&[(0, 0.1, 0)], &config);
        let two_same_dir = pool(&[(0, 0.1, 0), (1, 0.1, 0)], &config);
        let two_dirs = pool(&[(0, 0.1, 0), (1, 0.1, 4)], &config);
        let cold_extra = pool(&[(0, 0.1, 0), (1, 0.001, 4)], &config);
        assert!(two_same_dir > one);
        assert!(two_dirs > two_same_dir);
        assert!(cold_extra < two_same_dir);
        assert!((cold_extra - one).abs() < 0.01);
    }
}
