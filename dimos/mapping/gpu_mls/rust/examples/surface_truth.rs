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

//! Surface heights and step verdicts against a dense real-world reference.
//!
//! The input is an accumulated, world-registered lidar cloud as raw
//! little-endian f32 xyz triples (see accumulate_lidar.py next to this file). Its reference
//! surface is the median height of each layer of points in each 2 cm column; the GPU's plane heights and today's whole-voxel top faces are
//! scored against it, as are step verdicts between neighboring cells.
//!
//! `cargo run --release --example surface_truth -- cloud.xyz.f32`

use std::collections::HashMap;

use dimos_gpu_mls::brick_map::{BrickMap, MapConfig, VoxelKey};
use dimos_gpu_mls::gpu::Gpu;
use dimos_gpu_mls::surfaces::{SurfaceConfig, SurfaceExtractor, Window};

/// Edge of a reference column: 2 cm, where a few seconds of a still Mid-360
/// put enough points in each to take a median.
const REFERENCE_CELL: f32 = 0.02;
/// Points a reference layer needs before its median counts.
const REFERENCE_MIN_POINTS: usize = 3;
/// A vertical gap this large between sorted points starts a new layer.
const LAYER_GAP: f32 = 0.05;
/// Width of the strip either side of a shared face that a face height is read from.
const FACE_STRIP: f32 = 0.02;
/// Step thresholds the verdicts are scored at: the planner default, and two
/// near the 13 to 14 cm rise of the recording's stairs.
const STEP_THRESHOLDS: [f32; 3] = [0.10, 0.12, 0.16];

/// Robust heights of each layer of points, per 1 cm column.
struct Reference {
    layers: HashMap<(i32, i32), Vec<f32>>,
}

impl Reference {
    fn build(cloud: &[(f32, f32, f32)]) -> Self {
        let mut columns: HashMap<(i32, i32), Vec<f32>> = HashMap::new();
        for &(x, y, z) in cloud {
            let key = (
                (x / REFERENCE_CELL).floor() as i32,
                (y / REFERENCE_CELL).floor() as i32,
            );
            columns.entry(key).or_default().push(z);
        }
        let layers = columns
            .into_iter()
            .map(|(key, mut zs)| {
                zs.sort_by(f32::total_cmp);
                let mut medians = Vec::new();
                let mut start = 0;
                for i in 1..=zs.len() {
                    if i == zs.len() || zs[i] - zs[i - 1] > LAYER_GAP {
                        if i - start >= REFERENCE_MIN_POINTS {
                            medians.push(zs[(start + i) / 2]);
                        }
                        start = i;
                    }
                }
                (key, medians)
            })
            .collect();
        Self { layers }
    }

    /// The median of the reference heights nearest `near` over the 1 cm
    /// columns inside a box, None when fewer than a quarter hold one.
    fn height(
        &self,
        (x0, x1): (f32, f32),
        (y0, y1): (f32, f32),
        near: f32,
        reach: f32,
    ) -> Option<f32> {
        let mut found = Vec::new();
        let mut total = 0;
        let cell = |v: f32| (v / REFERENCE_CELL).floor() as i32;
        for cx in cell(x0)..cell(x1 - 1e-6) + 1 {
            for cy in cell(y0)..cell(y1 - 1e-6) + 1 {
                total += 1;
                let best = self.layers.get(&(cx, cy)).and_then(|layers| {
                    layers
                        .iter()
                        .copied()
                        .filter(|h| (h - near).abs() <= reach)
                        .min_by(|a, b| (a - near).abs().total_cmp(&(b - near).abs()))
                });
                found.extend(best);
            }
        }
        if found.len() * 4 < total {
            return None;
        }
        found.sort_by(f32::total_cmp);
        Some(found[found.len() / 2])
    }
}

fn percentile(sorted: &[f32], p: f32) -> f32 {
    if sorted.is_empty() {
        return f32::NAN;
    }
    sorted[((sorted.len() - 1) as f32 * p).round() as usize]
}

fn summarize(errors: &mut Vec<f32>) -> String {
    errors.sort_by(f32::total_cmp);
    format!(
        "median {:.2} cm, p95 {:.2} cm",
        100.0 * percentile(errors, 0.5),
        100.0 * percentile(errors, 0.95)
    )
}

struct Cell {
    height: f32,
    normal: Option<[f32; 3]>,
}

impl Cell {
    /// The GPU's link-time height: the plane evaluated at an offset from the column center.
    fn at(&self, offset: (f32, f32)) -> f32 {
        let Some(n) = self.normal else {
            return self.height;
        };
        let reach = 2.0 * offset.0.abs().max(offset.1.abs());
        self.height + (-(n[0] * offset.0 + n[1] * offset.1) / n[2].max(1e-3)).clamp(-reach, reach)
    }
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("an accumulated .xyz.f32 cloud");
    let bytes = std::fs::read(&path).expect("readable cloud");
    let cloud: Vec<(f32, f32, f32)> = bytemuck::cast_slice::<u8, f32>(&bytes)
        .chunks_exact(3)
        .map(|p| (p[0], p[1], p[2]))
        .collect();
    let reference = Reference::build(&cloud);
    println!(
        "{path}: {} points, {} reference columns",
        cloud.len(),
        reference.layers.len()
    );
    let gpu = Gpu::new().expect("a GPU");
    println!(
        "adapter {} ({:?})",
        gpu.adapter_info.name, gpu.adapter_info.backend
    );
    let config = SurfaceConfig::mls_defaults();

    for voxel in [0.05f32, 0.08] {
        let mut map = BrickMap::new(
            gpu.clone(),
            MapConfig {
                voxel_size: voxel,
                fine_divisor: 3,
                max_range: 30.0,
                ray_subsample: 1,
                shadow_depth: 0.1,
                grace_depth: 0.2,
                min_health: -1,
                max_health: 5,
                graze_cos: 0.7,
                brick_capacity: 1 << 15,
                voxel_capacity: 1 << 24,
                max_points: cloud.len() as u32,
                list_capacity: 1 << 22,
            },
        )
        .expect("brick map fits the device");
        map.seed(&cloud);
        let keys = map
            .snapshot()
            .into_iter()
            .filter(|v| v.health > 0)
            .map(|v| v.key);
        let window =
            Window::covering(keys, config.closing_cells(voxel) + 1).expect("an occupied map");
        let mut extractor =
            SurfaceExtractor::new(&map, window, config).expect("window fits the device");
        extractor.extract(&mut map);
        map.finish();
        let result = extractor.read();
        let cells: HashMap<VoxelKey, Cell> = result
            .surface
            .iter()
            .zip(result.heights.iter().zip(&result.normals))
            .map(|(&k, (&height, &normal))| (k, Cell { height, normal }))
            .collect();

        // Heights at each column's center 4 cm square, against the reference layer near each estimate.
        let (mut plane_errors, mut voxel_errors) = (Vec::new(), Vec::new());
        for (&(x, y, z), cell) in &cells {
            let (cx, cy) = ((x as f32 + 0.5) * voxel, (y as f32 + 0.5) * voxel);
            let top_face = (z + 1) as f32 * voxel;
            let square = ((cx - 0.02, cx + 0.02), (cy - 0.02, cy + 0.02));
            let Some(truth) = reference.height(square.0, square.1, cell.height, voxel) else {
                continue;
            };
            plane_errors.push((cell.height - truth).abs());
            voxel_errors.push((top_face - truth).abs());
        }
        println!(
            "voxel {voxel}: {} surface cells ({:.0}% with a plane), {} with a reference; height error: plane {}; whole-voxel top face {}",
            cells.len(),
            100.0 * cells.values().filter(|c| c.normal.is_some()).count() as f32 / cells.len() as f32,
            plane_errors.len(),
            summarize(&mut plane_errors),
            summarize(&mut voxel_errors)
        );

        // Step verdicts between 4-neighbors the GPU considers, against the reference step at the shared face.
        // Scored at thresholds around a stair's rise too, where verdicts are close.
        for threshold in STEP_THRESHOLDS {
            let step_cells = SurfaceConfig {
                step_threshold_m: threshold,
                ..config
            }
            .step_cells(voxel);
            let (mut pairs, mut plane_right, mut voxel_right, mut fixed, mut broken) =
                (0, 0, 0, 0, 0);
            let mut near_threshold = 0;
            for (&(x, y, z), a) in &cells {
                for (dx, dy) in [(1, 0), (0, 1)] {
                    for dz in -(step_cells + 1)..=step_cells + 1 {
                        let Some(b) = cells.get(&(x + dx, y + dy, z + dz)) else {
                            continue;
                        };
                        let face = (
                            (x as f32 + 0.5 + 0.5 * dx as f32) * voxel,
                            (y as f32 + 0.5 + 0.5 * dy as f32) * voxel,
                        );
                        let half = (0.5 * voxel * dx as f32, 0.5 * voxel * dy as f32);
                        let strip = |side: f32| {
                            let along = 0.5 * voxel - 0.01;
                            if dx != 0 {
                                let x0 = face.0 + side * FACE_STRIP;
                                (
                                    (x0.min(face.0), x0.max(face.0)),
                                    (face.1 - along, face.1 + along),
                                )
                            } else {
                                let y0 = face.1 + side * FACE_STRIP;
                                (
                                    (face.0 - along, face.0 + along),
                                    (y0.min(face.1), y0.max(face.1)),
                                )
                            }
                        };
                        let (sa, sb) = (strip(-1.0), strip(1.0));
                        let (ha, hb) = (a.at(half), b.at((-half.0, -half.1)));
                        let (Some(ra), Some(rb)) = (
                            reference.height(sa.0, sa.1, ha, voxel),
                            reference.height(sb.0, sb.1, hb, voxel),
                        ) else {
                            continue;
                        };
                        let truth = (ra - rb).abs() <= threshold;
                        let plane = (ha - hb).abs() <= threshold;
                        let whole = dz.abs() <= step_cells;
                        pairs += 1;
                        near_threshold += ((ra - rb).abs() > 0.5 * threshold) as usize;
                        plane_right += (plane == truth) as usize;
                        voxel_right += (whole == truth) as usize;
                        fixed += (plane == truth && whole != truth) as usize;
                        broken += (plane != truth && whole == truth) as usize;
                    }
                }
            }
            println!(
            "voxel {voxel}: {pairs} neighbor pairs scored ({near_threshold} with a reference step over half the \
             {threshold} m threshold); verdicts matching the reference: plane {plane_right}, whole-voxel {voxel_right}; \
             flipped to match {fixed}, flipped away {broken}"
        );
        }
    }
}
