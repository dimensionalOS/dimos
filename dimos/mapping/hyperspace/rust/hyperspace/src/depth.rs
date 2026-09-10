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

//! Depth frames: reprojection into the color camera, fusion backend trait, and
//! per-patch depth for pyramid caps.

use crate::{CameraIntrinsics, ImageFrame};
use nalgebra::{Isometry3, Point3};

/// One metric depth image in meters (0 or non-finite = no reading).
#[derive(Debug, Clone)]
pub struct DepthImage {
    /// tf frame of the depth sensor (e.g. camera_depth_optical_frame).
    pub camera_frame: String,
    pub timestamp: f64,
    pub width: u32,
    pub height: u32,
    pub depth_m: Vec<f32>,
}

impl DepthImage {
    pub fn from_millimeters(
        camera_frame: &str,
        timestamp: f64,
        width: u32,
        height: u32,
        mm: &[u16],
    ) -> Self {
        DepthImage {
            camera_frame: camera_frame.into(),
            timestamp,
            width,
            height,
            depth_m: mm.iter().map(|v| *v as f32 * 0.001).collect(),
        }
    }
}

/// Densify / denoise a raw depth image using the matching RGB frame. Output is
/// the same size as `raw_depth_m` (already in the color camera's pixel grid).
pub trait DepthFuser: Send {
    fn fuse(&mut self, rgb: &ImageFrame, raw_depth_m: &[f32]) -> Result<Vec<f32>, String>;
}

/// Uses the raw sensor depth as-is (holes stay holes).
pub struct PassthroughDepthFuser;

impl DepthFuser for PassthroughDepthFuser {
    fn fuse(&mut self, _rgb: &ImageFrame, raw_depth_m: &[f32]) -> Result<Vec<f32>, String> {
        Ok(raw_depth_m.to_vec())
    }
}

/// Re-render a depth image taken by `depth` into the pixel grid of `color`,
/// given `color_from_depth` (pose of the depth optical frame in the color
/// optical frame). Nearest-surface wins where several depth pixels land on one
/// color pixel. Pixels nothing landed on are 0.
pub fn reproject_depth(
    image: &DepthImage,
    depth: &CameraIntrinsics,
    color: &CameraIntrinsics,
    color_from_depth: &Isometry3<f64>,
) -> Vec<f32> {
    let (cw, ch) = (color.width as usize, color.height as usize);
    let mut out = vec![0f32; cw * ch];
    let identity = color_from_depth.translation.vector.norm() < 1e-9
        && color_from_depth.rotation.angle() < 1e-9
        && depth.width == color.width
        && depth.height == color.height
        && (depth.fx - color.fx).abs() < 1e-6
        && (depth.cx - color.cx).abs() < 1e-6;
    if identity {
        return image.depth_m.clone();
    }
    for v in 0..image.height as usize {
        for u in 0..image.width as usize {
            let z = image.depth_m[v * image.width as usize + u] as f64;
            if z <= 0.0 || !z.is_finite() {
                continue;
            }
            let point_depth = Point3::new(
                (u as f64 + 0.5 - depth.cx) / depth.fx * z,
                (v as f64 + 0.5 - depth.cy) / depth.fy * z,
                z,
            );
            let point_color = color_from_depth * point_depth;
            if point_color.z <= 0.0 {
                continue;
            }
            let uc = (point_color.x / point_color.z * color.fx + color.cx).floor();
            let vc = (point_color.y / point_color.z * color.fy + color.cy).floor();
            if uc < 0.0 || vc < 0.0 || uc >= cw as f64 || vc >= ch as f64 {
                continue;
            }
            let slot = &mut out[vc as usize * cw + uc as usize];
            if *slot == 0.0 || (point_color.z as f32) < *slot {
                *slot = point_color.z as f32;
            }
        }
    }
    out
}

/// Median valid depth inside each patch of a `rows × cols` grid laid over a
/// `width × height` depth image. NaN where a patch has no valid depth.
pub fn per_patch_depth(
    depth_m: &[f32],
    width: usize,
    height: usize,
    rows: usize,
    cols: usize,
) -> Vec<f32> {
    let mut out = Vec::with_capacity(rows * cols);
    let mut samples = Vec::new();
    for row in 0..rows {
        let y0 = row * height / rows;
        let y1 = (row + 1) * height / rows;
        for col in 0..cols {
            let x0 = col * width / cols;
            let x1 = (col + 1) * width / cols;
            samples.clear();
            for y in y0..y1 {
                for x in x0..x1 {
                    let z = depth_m[y * width + x];
                    if z > 0.0 && z.is_finite() {
                        samples.push(z);
                    }
                }
            }
            if samples.is_empty() {
                out.push(f32::NAN);
            } else {
                samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
                out.push(samples[samples.len() / 2]);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intrinsics(frame: &str) -> CameraIntrinsics {
        CameraIntrinsics {
            camera_frame: frame.into(),
            width: 8,
            height: 4,
            fx: 4.0,
            fy: 4.0,
            cx: 4.0,
            cy: 2.0,
            distortion_model: "plumb_bob".into(),
            distortion: vec![],
        }
    }

    #[test]
    fn reproject_identity_is_a_copy() {
        let image = DepthImage {
            camera_frame: "d".into(),
            timestamp: 0.0,
            width: 8,
            height: 4,
            depth_m: vec![1.5; 32],
        };
        let out = reproject_depth(
            &image,
            &intrinsics("d"),
            &intrinsics("c"),
            &Isometry3::identity(),
        );
        assert_eq!(out, vec![1.5; 32]);
    }

    #[test]
    fn reproject_shifts_with_baseline() {
        // A 0.25 m sideways baseline at 1 m depth with fx=4 moves pixels by one column.
        let mut depth_m = vec![0.0; 32];
        depth_m[2 * 8 + 4] = 1.0;
        let image = DepthImage {
            camera_frame: "d".into(),
            timestamp: 0.0,
            width: 8,
            height: 4,
            depth_m,
        };
        let color_from_depth = Isometry3::translation(0.25, 0.0, 0.0);
        let out = reproject_depth(
            &image,
            &intrinsics("d"),
            &intrinsics("c"),
            &color_from_depth,
        );
        assert_eq!(out[2 * 8 + 5], 1.0);
        assert_eq!(out.iter().filter(|z| **z > 0.0).count(), 1);
    }

    #[test]
    fn per_patch_median_ignores_holes() {
        let depth_m = vec![0.0, 2.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0];
        let patches = per_patch_depth(&depth_m, 4, 2, 1, 2);
        assert_eq!(patches[0], 4.0); // median of [2, 4] -> upper middle
        assert!(patches[1].is_nan());
    }
}
