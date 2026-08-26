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

//! The room hint: how much space each waypoint has, from a set of obstacles.
//!
//! Shared facility, not a law. Both deployed modules need it and need it to
//! agree -- the planner stamps the profile it computes here into the path
//! (`stamps::encode_precision`), and the follower on the hinted track
//! recomputes the same quantity from its own copy of the map. If the two
//! disagreed, the follower's governor would be reading a different world than
//! the one the plan was priced for.
//!
//! The python side is `control/world.py:path_clearance` (planner) and
//! `adapter/follower.py:path_clearance` (follower), which are the same
//! function written twice; both are the specification.
//!
//! The input is an obstacle model's hard set (`obstacles.rs`): every point
//! handed in is something the body can hit, and z rides along unread. There is
//! no band here, deliberately -- the planner searches the SAME set, so a rule
//! of our own would price a world nobody planned, and would cut off any body
//! taller than whatever band this file happened to carry.
//!
//! A HINT, not a safety contract. It is the nearest thing that could touch the
//! body, minus the body -- and nothing here is allowed to be the reason a
//! robot does or does not collide.

use std::collections::HashMap;

/// Grid cell size (m). Only a performance knob: the query below is exact at
/// any cell size, because it keeps expanding rings until the ring itself is
/// farther than the best point found. Sized so a voxel map at typical
/// resolution resolves most queries in the first ring or two.
const CELL: f64 = 0.5;

type Cell = (i32, i32);

/// Band points bucketed by cell, for nearest-neighbour queries.
struct Grid {
    cells: HashMap<Cell, Vec<[f64; 2]>>,
    /// Occupied cell bounds, so the ring walk has somewhere to stop.
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
    ///
    /// Rings are searched outwards from the query's own cell. Entering ring
    /// `k`, rings `0 ..= k-1` are done, so any point still unseen sits in a
    /// cell at ring `>= k`. The query lies somewhere inside its own cell, and
    /// the near edge of a ring-`k` cell is more than `(k-1) * CELL` from
    /// anywhere in it, so nothing unseen can be closer than that. Once `best`
    /// is under that bound, no later ring can beat it -- which is what makes
    /// stopping early exact rather than approximate.
    ///
    /// The bound is `(k-1)`, not `k`: a ring-`k` cell can reach back to within
    /// one cell of the query, so stopping on `best <= k * CELL` would discard
    /// a point that is genuinely nearer.
    fn nearest(&self, q: [f64; 2]) -> f64 {
        if self.cells.is_empty() {
            return f64::INFINITY;
        }
        let (cx, cy) = cell_of(q);
        // Past this the ring cannot intersect an occupied cell at all, so the
        // walk has somewhere to stop even when the early exit below has not
        // fired -- a query far outside a sparse grid would otherwise ring
        // outwards forever.
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

/// Per-waypoint room (m): nearest obstacle minus the body half-width.
///
/// A port of the two python `path_clearance` twins. `points` is the obstacle
/// model's hard set (xyz, f32) -- every row counts, z included or not; the
/// widening to f64 happens here so both callers cannot do it differently.
///
/// No obstacles or an empty path is infinite room, matching the python: a map
/// with nothing the body can hit is not a tight map, it is an empty one, and
/// the governor saturates to cruise on it.
///
/// Only the DISTANCE is returned, never which point produced it, which is why
/// this is free to use a different search structure from the python's
/// `cKDTree` and still agree with it bit for bit -- an exact nearest-neighbour
/// distance is unique even when the nearest point is not.
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
