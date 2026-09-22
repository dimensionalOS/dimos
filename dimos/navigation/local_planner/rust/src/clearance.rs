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

//! Per-waypoint room from the obstacle model's hard set; the planner stamps
//! it into the path (`stamps::encode_precision`). A hint, not a safety
//! contract. Spec: `obstacles.py::path_clearance`.

use std::collections::HashMap;

/// Grid cell size (m); a performance knob only, the query is exact at any size.
const CELL: f64 = 0.5;

type Cell = (i32, i32);

/// Band points bucketed by cell, for nearest-neighbour queries.
struct Grid {
    cells: HashMap<Cell, Vec<[f64; 2]>>,
    /// Occupied cell bounds, where the ring walk stops.
    min: Cell,
    max: Cell,
}

impl Grid {
    fn of(band: &[[f64; 2]]) -> Self {
        let mut cells: HashMap<Cell, Vec<[f64; 2]>> = HashMap::new();
        let (mut min, mut max) = ((i32::MAX, i32::MAX), (i32::MIN, i32::MIN));
        for &p in band {
            let c = cell_of(p);
            min = (min.0.min(c.0), min.1.min(c.1));
            max = (max.0.max(c.0), max.1.max(c.1));
            cells.entry(c).or_default().push(p);
        }
        Self { cells, min, max }
    }

    /// Exact distance to the nearest band point, or infinity if there are none.
    /// Nothing unseen at ring `k` is closer than `(k-1) * CELL`, so the early stop is exact.
    fn nearest(&self, q: [f64; 2]) -> f64 {
        if self.cells.is_empty() {
            return f64::INFINITY;
        }
        let (cx, cy) = cell_of(q);
        // past this no ring meets an occupied cell, so a far query still terminates
        let last = [
            (cx - self.min.0).abs(),
            (cx - self.max.0).abs(),
            (cy - self.min.1).abs(),
            (cy - self.max.1).abs(),
        ]
        .into_iter()
        .max()
        .unwrap_or(0);
        let mut best = f64::INFINITY;
        for k in 0..=last {
            if k > 0 && best <= ((k - 1) as f64) * CELL {
                break;
            }
            for (gx, gy) in ring(cx, cy, k) {
                let Some(points) = self.cells.get(&(gx, gy)) else {
                    continue;
                };
                for p in points {
                    let (dx, dy) = (p[0] - q[0], p[1] - q[1]);
                    let d = (dx * dx + dy * dy).sqrt();
                    if d < best {
                        best = d;
                    }
                }
            }
        }
        best
    }
}

fn cell_of(p: [f64; 2]) -> Cell {
    ((p[0] / CELL).floor() as i32, (p[1] / CELL).floor() as i32)
}

/// The cells at Chebyshev distance exactly `k` from `(cx, cy)`.
fn ring(cx: i32, cy: i32, k: i32) -> Vec<Cell> {
    if k == 0 {
        return vec![(cx, cy)];
    }
    let mut out = Vec::with_capacity((8 * k) as usize);
    for d in -k..=k {
        out.push((cx + d, cy - k));
        out.push((cx + d, cy + k));
    }
    for d in (-k + 1)..k {
        out.push((cx - k, cy + d));
        out.push((cx + k, cy + d));
    }
    out
}

/// Per-waypoint room (m): nearest obstacle minus the body half-width; infinite
/// with no obstacles. Every row of `points` (xyz, f32) counts.
pub fn path_clearance(xy: &[[f64; 2]], points: &[[f32; 3]], half_width: f64) -> Vec<f64> {
    if xy.is_empty() {
        return Vec::new();
    }
    let band: Vec<[f64; 2]> = points.iter().map(|p| [p[0] as f64, p[1] as f64]).collect();
    if band.is_empty() {
        return vec![f64::INFINITY; xy.len()];
    }
    let grid = Grid::of(&band);
    xy.iter().map(|&q| grid.nearest(q) - half_width).collect()
}
