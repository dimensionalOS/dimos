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

//
// Port of `height_cost_occupancy` from dimos/mapping/pointclouds/occupancy.py:
// terrain-slope costmap from a world-frame point cloud.
//
// The scipy.ndimage building blocks are reimplemented with scipy's exact
// conventions: gaussian_filter (truncate=4.0, mode="reflect"), sobel
// (derivative [-1,0,1] x smoothing [1,2,1], mode="reflect"), binary_erosion
// (4-connectivity cross, border_value=0). Filters accumulate in f64 and store
// f32, mirroring numpy's float32 array pipeline.

/// Mirrors `HeightCostConfig` (dimos/mapping/pointclouds/occupancy.py).
#[derive(Debug, Clone)]
pub struct HeightCostConfig {
    pub resolution: f64,
    pub can_pass_under: f64,
    pub can_climb: f64,
    pub ignore_noise: f64,
    pub smoothing: f64,
}

/// Occupancy result: row-major (height x width) costs, 0..100 or -1 unknown.
pub struct CostGrid {
    pub width: usize,
    pub height: usize,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<i8>,
}

const PADDING: f64 = 1.0;

pub fn height_cost_occupancy(points: &[(f64, f64, f64)], cfg: &HeightCostConfig) -> CostGrid {
    if points.is_empty() {
        // Python: OccupancyGrid(width=1, height=1) -> a single unknown cell.
        return CostGrid {
            width: 1,
            height: 1,
            origin_x: 0.0,
            origin_y: 0.0,
            data: vec![-1],
        };
    }

    let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y, _) in points {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    min_x -= PADDING;
    max_x += PADDING;
    min_y -= PADDING;
    max_y += PADDING;

    let width = ((max_x - min_x) / cfg.resolution).ceil() as usize;
    let height = ((max_y - min_y) / cfg.resolution).ceil() as usize;
    let n = width * height;

    // Step 1: min/max height per cell (NaN = no observation). Port of
    // `_height_map_kernel`: gx = int((x - min_x) * inv_res + 0.5).
    let inv_res = 1.0 / cfg.resolution;
    let mut min_map = vec![f32::NAN; n];
    let mut max_map = vec![f32::NAN; n];
    for &(x, y, z) in points {
        let gx = ((x - min_x) * inv_res + 0.5) as i64;
        let gy = ((y - min_y) * inv_res + 0.5) as i64;
        if gx >= 0 && (gx as usize) < width && gy >= 0 && (gy as usize) < height {
            let i = gy as usize * width + gx as usize;
            let z = z as f32;
            if z < min_map[i] || min_map[i].is_nan() {
                min_map[i] = z;
            }
            if z > max_map[i] || max_map[i].is_nan() {
                max_map[i] = z;
            }
        }
    }

    // Step 2: pass-under rule. Gap > can_pass_under => overhead structure, use
    // the floor (min); otherwise solid obstacle, use max. NaN gap compares
    // false, so unobserved cells take max (= NaN), like the numpy `where`.
    let can_pass_under = cfg.can_pass_under as f32;
    let mut height_map: Vec<f32> = (0..n)
        .map(|i| {
            if max_map[i] - min_map[i] > can_pass_under {
                min_map[i]
            } else {
                max_map[i]
            }
        })
        .collect();

    let mut observed: Vec<bool> = height_map.iter().map(|v| !v.is_nan()).collect();

    // Step 3: weighted gaussian smoothing, interpolating only from observed
    // cells and leaving fully-unknown regions NaN.
    if cfg.smoothing > 0.0 && observed.iter().any(|&o| o) {
        let weights: Vec<f32> = observed
            .iter()
            .map(|&o| if o { 1.0 } else { 0.0 })
            .collect();
        let filled: Vec<f32> = height_map
            .iter()
            .zip(&observed)
            .map(|(&v, &o)| if o { v } else { 0.0 })
            .collect();

        let sm_heights = gaussian_filter(&filled, height, width, cfg.smoothing);
        let sm_weights = gaussian_filter(&weights, height, width, cfg.smoothing);

        for i in 0..n {
            if !observed[i] {
                height_map[i] = if sm_weights[i] > 0.01 {
                    sm_heights[i] / sm_weights[i]
                } else {
                    f32::NAN
                };
            }
        }
        for i in 0..n {
            observed[i] = !height_map[i].is_nan();
        }
    }

    // Step 4: slope cost from sobel gradient magnitude.
    let mut data = vec![-1i8; n];
    if observed.iter().any(|&o| o) {
        let for_grad: Vec<f32> = height_map
            .iter()
            .zip(&observed)
            .map(|(&v, &o)| if o { v } else { 0.0 })
            .collect();

        let scale = (8.0 * cfg.resolution) as f32;
        let grad_x = sobel(&for_grad, height, width, Axis::X);
        let grad_y = sobel(&for_grad, height, width, Axis::Y);

        // Only trust gradients whose full 4-neighborhood is observed.
        let valid = binary_erosion_cross(&observed, height, width);

        let ignore_noise = cfg.ignore_noise as f32;
        let can_climb = cfg.can_climb as f32;
        let resolution = cfg.resolution as f32;
        for i in 0..n {
            if !valid[i] {
                continue;
            }
            let gx = grad_x[i] / scale;
            let gy = grad_y[i] / scale;
            let magnitude = (gx * gx + gy * gy).sqrt();
            let mut per_cell = magnitude * resolution;
            if per_cell < ignore_noise {
                per_cell = 0.0;
            }
            let cost = (per_cell / can_climb * 100.0).clamp(0.0, 100.0);
            data[i] = cost as i8; // numpy .astype(int8): truncation toward zero
        }
    }

    CostGrid {
        width,
        height,
        origin_x: min_x,
        origin_y: min_y,
        data,
    }
}

enum Axis {
    X, // along a row (columns), scipy axis=1
    Y, // along a column (rows), scipy axis=0
}

/// scipy 'reflect' boundary: (d c b a | a b c d), edge sample duplicated.
#[inline]
fn reflect(mut idx: i64, len: i64) -> usize {
    loop {
        if idx < 0 {
            idx = -idx - 1;
        } else if idx >= len {
            idx = 2 * len - idx - 1;
        } else {
            return idx as usize;
        }
    }
}

/// scipy.ndimage.correlate1d: out[i] = sum_j w[j] * in[i + j - radius].
///
/// Interior cells take a branch-free path; only the `radius`-wide edges pay
/// for boundary reflection. The Y pass accumulates whole rows (unit-stride
/// inner loops the compiler can vectorize) in the same j-order as the naive
/// loop, so results are bit-identical.
fn correlate1d(src: &[f32], h: usize, w: usize, weights: &[f64], axis: Axis) -> Vec<f32> {
    let radius = weights.len() / 2;
    let mut out = vec![0.0f32; src.len()];
    match axis {
        Axis::X => {
            for r in 0..h {
                let row = &src[r * w..(r + 1) * w];
                let out_row = &mut out[r * w..(r + 1) * w];
                let interior_end = w.saturating_sub(radius);
                for (c, o) in out_row.iter_mut().enumerate() {
                    let mut acc = 0.0f64;
                    if c >= radius && c < interior_end {
                        let base = c - radius;
                        for (j, &wt) in weights.iter().enumerate() {
                            acc += wt * row[base + j] as f64;
                        }
                    } else {
                        for (j, &wt) in weights.iter().enumerate() {
                            let idx = reflect(c as i64 + j as i64 - radius as i64, w as i64);
                            acc += wt * row[idx] as f64;
                        }
                    }
                    *o = acc as f32;
                }
            }
        }
        Axis::Y => {
            let mut acc = vec![0.0f64; w];
            for r in 0..h {
                acc.fill(0.0);
                for (j, &wt) in weights.iter().enumerate() {
                    let src_r = reflect(r as i64 + j as i64 - radius as i64, h as i64);
                    let src_row = &src[src_r * w..(src_r + 1) * w];
                    for (a, &s) in acc.iter_mut().zip(src_row) {
                        *a += wt * s as f64;
                    }
                }
                let out_row = &mut out[r * w..(r + 1) * w];
                for (o, &a) in out_row.iter_mut().zip(&acc) {
                    *o = a as f32;
                }
            }
        }
    }
    out
}

/// scipy.ndimage.gaussian_filter kernel: radius = int(truncate*sigma + 0.5),
/// weights exp(-0.5 (x/sigma)^2) normalized; applied separably per axis.
fn gaussian_weights(sigma: f64) -> Vec<f64> {
    let radius = (4.0 * sigma + 0.5) as i64;
    let mut w: Vec<f64> = (-radius..=radius)
        .map(|x| (-0.5 * (x as f64 / sigma).powi(2)).exp())
        .collect();
    let sum: f64 = w.iter().sum();
    for v in &mut w {
        *v /= sum;
    }
    w
}

fn gaussian_filter(src: &[f32], h: usize, w: usize, sigma: f64) -> Vec<f32> {
    let weights = gaussian_weights(sigma);
    let pass1 = correlate1d(src, h, w, &weights, Axis::Y);
    correlate1d(&pass1, h, w, &weights, Axis::X)
}

/// scipy.ndimage.sobel: derivative [-1, 0, 1] along `axis`, smoothing
/// [1, 2, 1] along the other.
fn sobel(src: &[f32], h: usize, w: usize, axis: Axis) -> Vec<f32> {
    const DERIV: [f64; 3] = [-1.0, 0.0, 1.0];
    const SMOOTH: [f64; 3] = [1.0, 2.0, 1.0];
    match axis {
        Axis::X => {
            let d = correlate1d(src, h, w, &DERIV, Axis::X);
            correlate1d(&d, h, w, &SMOOTH, Axis::Y)
        }
        Axis::Y => {
            let d = correlate1d(src, h, w, &DERIV, Axis::Y);
            correlate1d(&d, h, w, &SMOOTH, Axis::X)
        }
    }
}

/// scipy.ndimage.binary_erosion with generate_binary_structure(2, 1) (the
/// 4-connectivity cross) and border_value=0: a cell survives only if it and
/// all 4 neighbors are set; outside the array counts as unset.
fn binary_erosion_cross(mask: &[bool], h: usize, w: usize) -> Vec<bool> {
    let mut out = vec![false; mask.len()];
    for r in 0..h {
        for c in 0..w {
            if r == 0 || r == h - 1 || c == 0 || c == w - 1 {
                continue; // border neighbor is outside => eroded
            }
            let i = r * w + c;
            out[i] = mask[i] && mask[i - w] && mask[i + w] && mask[i - 1] && mask[i + 1];
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> HeightCostConfig {
        HeightCostConfig {
            resolution: 0.05,
            can_pass_under: 0.6,
            can_climb: 0.15,
            ignore_noise: 0.05,
            smoothing: 1.0,
        }
    }

    #[test]
    fn empty_cloud_gives_single_unknown_cell() {
        let g = height_cost_occupancy(&[], &cfg());
        assert_eq!((g.width, g.height), (1, 1));
        assert_eq!(g.data, vec![-1]);
    }

    #[test]
    fn flat_floor_is_free() {
        // A dense flat 2x2m floor at z=0.
        let mut pts = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                pts.push((i as f64 * 0.05, j as f64 * 0.05, 0.0));
            }
        }
        let g = height_cost_occupancy(&pts, &cfg());
        // Interior observed cells must be cost 0; nothing should be >0.
        assert!(g.data.contains(&0));
        assert!(g.data.iter().all(|&v| v <= 0));
    }

    #[test]
    fn wall_costs_100() {
        // Flat floor with a tall wall crossing it.
        let mut pts = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                pts.push((i as f64 * 0.05, j as f64 * 0.05, 0.0));
            }
        }
        for j in 0..40 {
            for k in 1..10 {
                pts.push((1.0, j as f64 * 0.05, k as f64 * 0.05));
            }
        }
        let g = height_cost_occupancy(&pts, &cfg());
        assert!(g.data.contains(&100));
    }

    #[test]
    fn gaussian_weights_match_scipy_sigma1() {
        // scipy.ndimage.gaussian_filter1d(sigma=1, truncate=4) uses radius 4.
        let w = gaussian_weights(1.0);
        assert_eq!(w.len(), 9);
        let sum: f64 = w.iter().sum();
        assert!((sum - 1.0).abs() < 1e-12);
        // Center weight for sigma=1: 1/sum(exp(-k^2/2), k=-4..4) ~= 0.39894
        assert!((w[4] - 0.3989434).abs() < 1e-4);
    }

    #[test]
    fn erosion_kills_borders_and_holes() {
        // 3x3 all-true: only the center survives.
        let mask = vec![true; 9];
        let out = binary_erosion_cross(&mask, 3, 3);
        assert_eq!(out.iter().filter(|&&v| v).count(), 1);
        assert!(out[4]);
    }
}
