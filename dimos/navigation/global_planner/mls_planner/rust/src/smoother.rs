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

//! Elastic-band smoothing of the planner's string-pulled path.

use crate::adjacency::rise;
use crate::edges::PlannerGraph;
use crate::nodes::penalty_of;
use crate::planner::WallCost;

/// Densify target between smoothed waypoints, in voxels.
const SMOOTH_SPACING_CELLS: f32 = 2.0;
/// Elastic-band passes over the path.
const SMOOTH_ITERS: u32 = 30;
/// Pull toward the neighbor midpoint per pass.
const SMOOTH_ALPHA: f32 = 0.25;
/// Push up the wall-clearance gradient per pass, in voxels at the clearance edge.
const SMOOTH_WALL_GAIN: f32 = 0.35;
/// Cap on a point's total move per pass, in voxels.
const SMOOTH_MAX_STEP_CELLS: f32 = 0.5;
/// Proposed moves below this are treated as converged (m).
const CONVERGED_MOVE_M: f32 = 1e-3;

/// Elastic-band smoothing: pull interior waypoints toward their neighbor
/// midpoint and up the wall-clearance gradient. A move is kept only when its
/// two chords stay feasible and their penalized cost does not increase.
pub(crate) fn smooth_path(
    plg: &PlannerGraph,
    waypoints: Vec<(f32, f32, f32)>,
    step_cells: i32,
    wc: &WallCost,
    step_weight: f32,
) -> Vec<(f32, f32, f32)> {
    if waypoints.len() < 3 {
        return waypoints;
    }
    let vs = wc.voxel_size;
    let mut pts = densify(&waypoints, SMOOTH_SPACING_CELLS * vs);

    // Cached cost of each chord pts[i] -> pts[i+1], updated on accepted moves.
    let mut costs: Vec<f32> = pts
        .windows(2)
        .map(|w| chord_cost(plg, w[0], w[1], step_cells, wc, step_weight))
        .collect();

    for _ in 0..SMOOTH_ITERS {
        let mut moved = false;
        for i in 1..pts.len() - 1 {
            let (prev, p, next) = (pts[i - 1], pts[i], pts[i + 1]);
            let mid = ((prev.0 + next.0) * 0.5, (prev.1 + next.1) * 0.5);
            let mut dx = SMOOTH_ALPHA * (mid.0 - p.0);
            let mut dy = SMOOTH_ALPHA * (mid.1 - p.1);

            if let Some((gx, gy)) = wall_push(plg, p, wc) {
                dx += gx;
                dy += gy;
            }

            let cap = SMOOTH_MAX_STEP_CELLS * vs;
            let norm = (dx * dx + dy * dy).sqrt();
            if norm < CONVERGED_MOVE_M {
                continue;
            }
            if norm > cap {
                dx *= cap / norm;
                dy *= cap / norm;
            }

            let (nx, ny) = (p.0 + dx, p.1 + dy);
            let (ix, iy) = ((nx / vs).floor() as i32, (ny / vs).floor() as i32);
            let Some(iz) = nearest_surface_iz(plg, ix, iy, p.2, vs) else {
                continue;
            };
            let nz = (iz as f32 + 1.0) * vs;
            if (nz - p.2).abs() > (step_cells as f32 + 0.5) * vs {
                continue;
            }
            let cand = (nx, ny, nz);
            let new_in = chord_cost(plg, prev, cand, step_cells, wc, step_weight);
            if !new_in.is_finite() {
                continue;
            }
            let new_out = chord_cost(plg, cand, next, step_cells, wc, step_weight);
            if new_in + new_out <= costs[i - 1] + costs[i] + 1e-4 {
                pts[i] = cand;
                costs[i - 1] = new_in;
                costs[i] = new_out;
                moved = true;
            }
        }
        if !moved {
            break;
        }
    }
    pts
}

/// Subdivide each segment so no gap exceeds `spacing` in the ground plane.
fn densify(waypoints: &[(f32, f32, f32)], spacing: f32) -> Vec<(f32, f32, f32)> {
    let mut out = Vec::with_capacity(waypoints.len() * 2);
    out.push(waypoints[0]);
    for w in waypoints.windows(2) {
        let (a, b) = (w[0], w[1]);
        let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let n = (len / spacing).ceil().max(1.0) as usize;
        for k in 1..=n {
            let t = k as f32 / n as f32;
            out.push((
                a.0 + t * (b.0 - a.0),
                a.1 + t * (b.1 - a.1),
                a.2 + t * (b.2 - a.2),
            ));
        }
    }
    out
}

/// Surface cell in the column nearest the given world z.
fn nearest_surface_iz(
    plg: &PlannerGraph,
    ix: i32,
    iy: i32,
    z: f32,
    voxel_size: f32,
) -> Option<i32> {
    let zs = plg.surface_lookup.get(&(ix, iy))?;
    zs.iter().copied().min_by(|&a, &b| {
        let za = ((a as f32 + 1.0) * voxel_size - z).abs();
        let zb = ((b as f32 + 1.0) * voxel_size - z).abs();
        za.total_cmp(&zb)
    })
}

/// Wall distance of the surface cell under a world point.
fn wall_dist_at(plg: &PlannerGraph, ix: i32, iy: i32, z: f32, voxel_size: f32) -> Option<f32> {
    let iz = nearest_surface_iz(plg, ix, iy, z, voxel_size)?;
    let id = plg.cells.id((ix, iy, iz))?;
    Some(
        plg.wall_state
            .dist
            .get(id as usize)
            .copied()
            .unwrap_or(f32::INFINITY),
    )
}

/// Step up the wall-distance gradient, scaled by depth into the soft buffer.
fn wall_push(plg: &PlannerGraph, p: (f32, f32, f32), wc: &WallCost) -> Option<(f32, f32)> {
    let vs = wc.voxel_size;
    let (ix, iy) = ((p.0 / vs).floor() as i32, (p.1 / vs).floor() as i32);
    let d0 = wall_dist_at(plg, ix, iy, p.2, vs)?;
    let outer = wc.clearance_m + wc.buffer_m;
    if d0 >= outer {
        return None;
    }
    // A missing neighbor column counts as a wall.
    let sample = |jx: i32, jy: i32| wall_dist_at(plg, jx, jy, p.2, vs).unwrap_or(0.0);
    let gx = sample(ix + 1, iy) - sample(ix - 1, iy);
    let gy = sample(ix, iy + 1) - sample(ix, iy - 1);
    let norm = (gx * gx + gy * gy).sqrt();
    if norm < 1e-6 {
        return None;
    }
    let depth = ((outer - d0) / wc.buffer_m.max(1e-3)).clamp(0.0, 1.0);
    let step = SMOOTH_WALL_GAIN * depth * vs;
    Some((gx / norm * step, gy / norm * step))
}

/// Wall-penalized length plus step penalty of the straight chord between two
/// world points. INF when the chord is infeasible.
fn chord_cost(
    plg: &PlannerGraph,
    a: (f32, f32, f32),
    b: (f32, f32, f32),
    step_cells: i32,
    wc: &WallCost,
    step_weight: f32,
) -> f32 {
    let vs = wc.voxel_size;
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let len = (dx * dx + dy * dy).sqrt();
    let samples = ((len / vs) * 2.0).ceil() as i32;
    if samples == 0 {
        return if dz.abs() < vs { 0.0 } else { f32::INFINITY };
    }
    let dlen = len / samples as f32;
    let mut last_col = (i32::MIN, i32::MIN);
    let mut prev_iz: Option<i32> = None;
    let mut cost = 0.0_f32;
    let mut rise_cells = 0i32;
    let mut pen = 1.0_f32;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let (x, y, z) = (a.0 + t * dx, a.1 + t * dy, a.2 + t * dz);
        let (ix, iy) = ((x / vs).floor() as i32, (y / vs).floor() as i32);
        if (ix, iy) != last_col {
            last_col = (ix, iy);
            let Some(iz) = nearest_surface_iz(plg, ix, iy, z, vs) else {
                return f32::INFINITY;
            };
            if ((iz as f32 + 1.0) * vs - z).abs() > (step_cells as f32 + 0.5) * vs {
                return f32::INFINITY;
            }
            if let Some(p) = prev_iz {
                let step = (iz - p).abs();
                if step > step_cells {
                    return f32::INFINITY;
                }
                rise_cells += step;
            }
            prev_iz = Some(iz);
            // Off-graph surface columns carry no wall penalty, matching the
            // cell-path validator.
            pen = match plg.cells.id((ix, iy, iz)) {
                Some(id) => {
                    let dist = plg
                        .wall_state
                        .dist
                        .get(id as usize)
                        .copied()
                        .unwrap_or(f32::INFINITY);
                    penalty_of(dist, wc.clearance_m, wc.buffer_m, wc.buffer_weight)
                }
                None => 1.0,
            };
            if !pen.is_finite() {
                return f32::INFINITY;
            }
        }
        if k > 0 {
            cost += pen * dlen;
        }
    }
    cost + step_weight * rise(rise_cells, vs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::voxel::VoxelKey;

    const VOXEL: f32 = 0.1;

    fn surface_graph(cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg
    }

    fn wc() -> WallCost {
        WallCost {
            clearance_m: 0.2,
            buffer_m: 0.3,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        }
    }

    fn xy_len(wp: &[(f32, f32, f32)]) -> f32 {
        wp.windows(2)
            .map(|w| ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn smooth_path_straightens_a_zigzag_on_open_floor() {
        let mut cells: Vec<VoxelKey> = Vec::new();
        for x in 0..20 {
            for y in 0..8 {
                cells.push((x, y, 0));
            }
        }
        let plg = surface_graph(&cells);
        let wc = wc();
        let zig = vec![
            (0.15, 0.35, 0.1),
            (0.55, 0.75, 0.1),
            (0.95, 0.15, 0.1),
            (1.35, 0.75, 0.1),
            (1.85, 0.35, 0.1),
        ];
        let before = xy_len(&zig);
        let out = smooth_path(&plg, zig.clone(), 2, &wc, 0.0);
        assert_eq!(out.first(), zig.first(), "start endpoint must stay fixed");
        assert_eq!(out.last(), zig.last(), "goal endpoint must stay fixed");
        assert!(
            xy_len(&out) < before * 0.8,
            "zigzag not straightened: {} vs {}",
            xy_len(&out),
            before
        );
        for w in out.windows(2) {
            assert!(chord_cost(&plg, w[0], w[1], 2, &wc, 0.0).is_finite());
        }
    }

    #[test]
    fn smooth_path_does_not_cut_a_missing_corner() {
        // L corridor with the inside of the bend missing: smoothing must round
        // the corner along the surface, never shortcut across the hole.
        let mut cells: Vec<VoxelKey> = Vec::new();
        for x in 0..3 {
            for y in 0..12 {
                cells.push((x, y, 0));
            }
        }
        for x in 3..12 {
            for y in 9..12 {
                cells.push((x, y, 0));
            }
        }
        let plg = surface_graph(&cells);
        let wc = wc();
        let path = vec![(0.15, 0.15, 0.1), (0.15, 1.05, 0.1), (1.15, 1.05, 0.1)];
        let out = smooth_path(&plg, path, 2, &wc, 0.0);
        for p in &out {
            let ix = (p.0 / VOXEL).floor() as i32;
            let iy = (p.1 / VOXEL).floor() as i32;
            assert!(
                plg.surface_lookup.contains_key(&(ix, iy)),
                "smoothed point left the surface at {p:?}"
            );
        }
        for w in out.windows(2) {
            assert!(chord_cost(&plg, w[0], w[1], 2, &wc, 0.0).is_finite());
        }
    }
}
