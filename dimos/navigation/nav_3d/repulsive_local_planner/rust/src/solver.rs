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

// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Navigation-function local planner — a faithful port of the measured Python
//! `plan_path` in `repulsive_field/local_planner.py`: one Dijkstra wavefront
//! rooted at the robot over a repulsive cost field (travel + clearance +
//! global-path adherence + temporal commitment), targeted at a route "carrot"
//! (furthest reachable point along the global path within an arc budget, with
//! a bounded gap hop for reachability flicker), backtracked, horizon-truncated,
//! smoothed, and given headings. Each tunable's measured story lives in the
//! Python config comments; values here are the course-tuned defaults.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::costmap::{Costmap, LETHAL_THRESHOLD};

#[derive(Clone, Debug)]
pub struct SolverConfig {
    pub vehicle_width: f32,
    pub safety_margin: f32,
    /// Robot bounding-box footprint (m), oriented to the travel heading. `length`
    /// is fore/aft, `width` is lateral. The narrow (`width`) dimension is what the
    /// Dijkstra traversal uses so the robot can thread any gap at least as wide as
    /// itself; the full oriented box then validates the produced path (so the long
    /// axis can't clip an obstacle it swept over). `offset` shifts the box fore(+)/
    /// aft(-) along the heading (e.g. the geometric center vs the odom origin).
    pub robot_length: f32,
    pub robot_width: f32,
    pub footprint_offset: f32,
    pub influence_radius: f32,
    pub clearance_weight: f32,
    pub path_weight: f32,
    pub commitment_weight: f32,
    pub carrot_lookahead: f32,
    pub carrot_lookahead_time_s: f32,
    pub carrot_lookahead_max: f32,
    pub carrot_gap_max: f32,
    pub dijkstra_radius: f32,
    pub horizon: f32,
    pub goal_tolerance: f32,
    pub smoothing_iterations: usize,
    pub face_forward_weight: f32,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            vehicle_width: 0.5,
            safety_margin: 0.1,
            robot_length: 0.7,
            robot_width: 0.33,
            footprint_offset: 0.0,
            influence_radius: 0.8,
            clearance_weight: 4.0,
            path_weight: 0.35,
            commitment_weight: 2.0,
            carrot_lookahead: 4.0,
            carrot_lookahead_time_s: 4.0,
            carrot_lookahead_max: 8.0,
            carrot_gap_max: 1.0,
            dijkstra_radius: 6.0,
            horizon: 3.0,
            goal_tolerance: 0.15,
            smoothing_iterations: 12,
            face_forward_weight: 0.8,
        }
    }
}

impl SolverConfig {
    /// Speed-adaptive preview (port of to_params): carrot grows with speed at
    /// constant TIME, the search window follows it, the horizon is 3/4 carrot.
    pub fn scaled(&self, speed: f32) -> (f32, f32, f32) {
        let mut carrot = self.carrot_lookahead;
        if self.carrot_lookahead_time_s > 0.0 && speed > 0.0 && carrot > 0.0 {
            carrot = (speed * self.carrot_lookahead_time_s)
                .max(carrot)
                .min(self.carrot_lookahead_max);
        }
        let radius = if self.dijkstra_radius > 0.0 {
            self.dijkstra_radius.max(carrot + 2.0)
        } else {
            0.0
        };
        let horizon = if self.horizon > 0.0 {
            self.horizon.max(0.75 * carrot)
        } else {
            0.0
        };
        (carrot, radius, horizon)
    }
}

#[derive(Copy, Clone, PartialEq)]
struct HeapEntry {
    cost: f32,
    index: usize,
}
impl Eq for HeapEntry {}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on cost.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

const NEIGHBORS: [(isize, isize, f32); 8] = [
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, std::f32::consts::SQRT_2),
    (-1, 1, std::f32::consts::SQRT_2),
    (1, -1, std::f32::consts::SQRT_2),
    (1, 1, std::f32::consts::SQRT_2),
];

/// Chamfer distance (m) from every cell to the polyline's rasterised cells —
/// the adherence/commitment fields. Uses the costmap's chamfer machinery.
fn polyline_distance_field(map: &Costmap, polyline: &[(f32, f32)]) -> Vec<f32> {
    let n = map.width * map.height;
    let mut seed = vec![0i8; n];
    let mut any = false;
    for pair in polyline.windows(2) {
        let (x0, y0) = pair[0];
        let (x1, y1) = pair[1];
        let steps = ((x1 - x0).hypot(y1 - y0) / map.resolution).ceil().max(1.0) as usize;
        for k in 0..=steps {
            let f = k as f32 / steps as f32;
            if let Some((r, c)) = map.cell(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f) {
                seed[r * map.width + c] = crate::costmap::LETHAL;
                any = true;
            }
        }
    }
    if !any {
        if let Some(&(x, y)) = polyline.first().map(|p| p) {
            if let Some((r, c)) = map.cell(x, y) {
                seed[r * map.width + c] = crate::costmap::LETHAL;
            }
        }
    }
    crate::costmap::chamfer_distance(&seed, map.width, map.height, map.resolution)
}

pub struct Plan {
    /// World-frame (x, y, yaw).
    pub poses: Vec<(f32, f32, f32)>,
}

/// One full solve (port of plan_path). The global path's last point is the
/// goal; the planner steers toward a carrot chosen along the densified path.
pub fn plan(
    map: &Costmap,
    global_path: &[(f32, f32)],
    robot: (f32, f32, f32),
    speed: f32,
    previous_path: Option<&[(f32, f32)]>,
    cfg: &SolverConfig,
) -> Plan {
    if global_path.is_empty() {
        return Plan { poses: Vec::new() };
    }
    let (carrot_budget, radius, horizon) = cfg.scaled(speed);

    // Densify the path (0.25 m): the carrot scan and the gap-hop walk path
    // POINTS, so sparse vertices (a 2-point straight route) would blow the arc
    // budget in one stride and collapse the carrot to the robot cell.
    let mut scan_path: Vec<(f32, f32)> = vec![global_path[0]];
    for pair in global_path.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let d = (b.0 - a.0).hypot(b.1 - a.1);
        let steps = (d / 0.25).ceil().max(1.0) as usize;
        for k in 1..=steps {
            let f = k as f32 / steps as f32;
            scan_path.push((a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f));
        }
    }
    let goal = *scan_path.last().unwrap();

    if (robot.0 - goal.0).hypot(robot.1 - goal.1) <= cfg.goal_tolerance {
        return Plan {
            poses: vec![(robot.0, robot.1, robot.2)],
        };
    }

    let width = map.width;
    let height = map.height;
    let n = width * height;
    // Traversal keep-out uses the robot's NARROW (width) half-extent, not an
    // isotropic circle: in a corridor the robot orients ALONG the gap, so lateral
    // width is the binding constraint. The long (length) axis is enforced after
    // the search by the oriented-box path validation. This lets the robot thread
    // any gap at least as wide as itself, where the old circle (which used the
    // width in every direction PLUS margin) walled the robot out of its own cell.
    let inflate = cfg.robot_width * 0.5 + cfg.safety_margin;

    let free_strict: Vec<bool> = (0..n)
        .map(|i| map.cost[i] < LETHAL_THRESHOLD && map.distance[i] >= inflate)
        .collect();

    // Robot cell. NEVER GIVE UP: prefer a keep-out-clear start, but if the robot
    // is pinned in a sub-width spot (no reachable cell is a full keep-out clear),
    // fall back to traversing any non-lethal cell so it can still crawl toward
    // open space instead of returning a dead stub. The clearance term in `entry`
    // biases that crawl toward the widest opening, and the oriented-box validation
    // keeps it from physically clipping a wall on the way.
    let (robot_cell, free) = match nearest_free_cell(map, &free_strict, robot.0, robot.1) {
        Some(c) => (c, free_strict),
        None => {
            let free_relaxed: Vec<bool> = (0..n).map(|i| map.cost[i] < LETHAL_THRESHOLD).collect();
            match nearest_free_cell(map, &free_relaxed, robot.0, robot.1) {
                Some(c) => (c, free_relaxed),
                None => return Plan { poses: Vec::new() },
            }
        }
    };

    // Entry cost: travel + clearance ramp within influence_radius past the
    // body + adherence to the REAL global path + temporal commitment.
    let path_dist = polyline_distance_field(map, global_path);
    let prev_dist = previous_path
        .filter(|p| p.len() >= 2 && cfg.commitment_weight > 0.0)
        .map(|p| polyline_distance_field(map, p));
    let mut entry = vec![0f32; n];
    for i in 0..n {
        let clearance = {
            let d = map.distance[i];
            if d <= inflate {
                1.0
            } else if d >= inflate + cfg.influence_radius {
                0.0
            } else {
                1.0 - (d - inflate) / cfg.influence_radius
            }
        };
        entry[i] = 1.0 + cfg.clearance_weight * clearance + cfg.path_weight * path_dist[i];
        if let Some(pd) = &prev_dist {
            entry[i] += cfg.commitment_weight * pd[i];
        }
    }
    // Dijkstra from the robot, window-capped.
    let max_cells = if radius > 0.0 {
        (radius / map.resolution).ceil() as isize
    } else {
        isize::MAX
    };
    let mut dist = vec![f32::MAX; n];
    let mut parent = vec![usize::MAX; n];
    let start = robot_cell.0 * width + robot_cell.1;
    dist[start] = 0.0;
    parent[start] = start;
    let mut heap = BinaryHeap::new();
    heap.push(HeapEntry {
        cost: 0.0,
        index: start,
    });
    while let Some(HeapEntry { cost, index }) = heap.pop() {
        if cost > dist[index] {
            continue;
        }
        let row = (index / width) as isize;
        let col = (index % width) as isize;
        for (dr, dc, step) in NEIGHBORS {
            let (r, c) = (row + dr, col + dc);
            if r < 0 || c < 0 || r as usize >= height || c as usize >= width {
                continue;
            }
            if (r - robot_cell.0 as isize).abs() > max_cells
                || (c - robot_cell.1 as isize).abs() > max_cells
            {
                continue;
            }
            let j = r as usize * width + c as usize;
            if !free[j] {
                continue;
            }
            let next = cost + step * entry[j];
            if next < dist[j] {
                dist[j] = next;
                parent[j] = index;
                heap.push(HeapEntry {
                    cost: next,
                    index: j,
                });
            }
        }
    }
    let reachable = |i: usize| parent[i] != usize::MAX;

    // Carrot: furthest reachable scan-path point within the arc budget,
    // hopping unreachable runs up to carrot_gap_max (single-cell flicker must
    // not collapse the plan; real walls span far more arc and still stop it).
    // The nearest-point anchor pins the scan to the point on the path closest
    // to the robot, so progress is measured forward from there.
    let start_idx = scan_path
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
            let da = (a.0 - robot.0).hypot(a.1 - robot.1);
            let db = (b.0 - robot.0).hypot(b.1 - robot.1);
            da.partial_cmp(&db).unwrap_or(Ordering::Equal)
        })
        .map(|(i, _)| i)
        .unwrap_or(0);
    let mut best: Option<(usize, usize)> = None; // (cell index, path index)
    let mut arc = 0.0;
    let mut gap = 0.0;
    let mut started = false;
    for i in start_idx..scan_path.len() {
        let step = if i > start_idx {
            let a = scan_path[i - 1];
            let b = scan_path[i];
            (b.0 - a.0).hypot(b.1 - a.1)
        } else {
            0.0
        };
        arc += step;
        if carrot_budget > 0.0 && arc > carrot_budget {
            break;
        }
        let Some((r, c)) = map.cell(scan_path[i].0, scan_path[i].1) else {
            if started {
                gap += step;
                if gap > cfg.carrot_gap_max {
                    break;
                }
            }
            continue;
        };
        let idx = r * width + c;
        if reachable(idx) {
            best = Some((idx, i));
            started = true;
            gap = 0.0;
        } else if started {
            gap += step;
            if gap > cfg.carrot_gap_max {
                break;
            }
        }
    }
    let (target, goal_reachable) = match best {
        Some((cell, i)) => (cell, i == scan_path.len() - 1),
        None => {
            // Fallback: reachable cell nearest the route's projection point.
            let p = scan_path[start_idx];
            let Some((pr, pc)) = map.cell(p.0, p.1) else {
                return Plan { poses: Vec::new() };
            };
            let mut best_cell = start;
            let mut best_d = f32::MAX;
            for i in 0..n {
                if !reachable(i) {
                    continue;
                }
                let dr = (i / width) as f32 - pr as f32;
                let dc = (i % width) as f32 - pc as f32;
                let d = dr * dr + dc * dc;
                if d < best_d {
                    best_d = d;
                    best_cell = i;
                }
            }
            (best_cell, false)
        }
    };

    // Backtrack.
    let mut cells = Vec::new();
    let mut cur = target;
    while cur != start {
        cells.push(cur);
        cur = parent[cur];
        if cells.len() > n {
            break; // corrupt parents guard
        }
    }
    cells.push(start);
    cells.reverse();
    let mut pts: Vec<(f32, f32)> = cells
        .iter()
        .map(|&i| map.cell_center(i / width, i % width))
        .collect();
    pts[0] = (robot.0, robot.1);
    if goal_reachable {
        let last = *pts.last().unwrap();
        if (last.0 - goal.0).hypot(last.1 - goal.1) > map.resolution {
            pts.push(goal);
        } else {
            *pts.last_mut().unwrap() = goal;
        }
    }

    // Horizon truncation.
    if horizon > 0.0 {
        let mut travelled = 0.0;
        let mut cut = pts.len();
        for i in 1..pts.len() {
            travelled += (pts[i].0 - pts[i - 1].0).hypot(pts[i].1 - pts[i - 1].1);
            if travelled >= horizon {
                cut = i + 1;
                break;
            }
        }
        pts.truncate(cut);
    }

    // Obstacle-aware smoothing (port of _smooth_positions: midpoint low-pass,
    // endpoints pinned, rejected when the smoothed point lands blocked).
    for _ in 0..cfg.smoothing_iterations {
        if pts.len() < 3 {
            break;
        }
        for i in 1..pts.len() - 1 {
            let sx = 0.25 * pts[i - 1].0 + 0.5 * pts[i].0 + 0.25 * pts[i + 1].0;
            let sy = 0.25 * pts[i - 1].1 + 0.5 * pts[i].1 + 0.25 * pts[i + 1].1;
            if let Some((r, c)) = map.cell(sx, sy) {
                if free[r * width + c] {
                    pts[i] = (sx, sy);
                }
            }
        }
    }

    // Headings: travel direction blended toward the goal direction near the
    // end (port of _assign_headings' face_forward blend).
    let mut poses = Vec::with_capacity(pts.len());
    for i in 0..pts.len() {
        let travel = if i + 1 < pts.len() {
            let (dx, dy) = (pts[i + 1].0 - pts[i].0, pts[i + 1].1 - pts[i].1);
            dy.atan2(dx)
        } else if i > 0 {
            let (dx, dy) = (pts[i].0 - pts[i - 1].0, pts[i].1 - pts[i - 1].1);
            dy.atan2(dx)
        } else {
            robot.2
        };
        let goal_dir = (goal.1 - pts[i].1).atan2(goal.0 - pts[i].0);
        let w = cfg.face_forward_weight;
        // Blend on the unit circle to avoid wrap artefacts.
        let (sy, cy) = (
            w * travel.sin() + (1.0 - w) * goal_dir.sin(),
            w * travel.cos() + (1.0 - w) * goal_dir.cos(),
        );
        poses.push((pts[i].0, pts[i].1, sy.atan2(cy)));
    }
    // Oriented-box validation: the search used only the narrow (width) keep-out,
    // so the long axis could still sweep a pose's footprint over an obstacle. Walk
    // the poses and cut the plan at the first one whose oriented box overlaps a
    // lethal cell — commit only as far as the WHOLE robot actually fits.
    let half_len = cfg.robot_length * 0.5;
    let half_w = cfg.robot_width * 0.5;
    let mut safe = poses.len();
    for (k, &(px, py, pyaw)) in poses.iter().enumerate() {
        if box_hits_lethal(map, px, py, pyaw, half_len, half_w, cfg.footprint_offset) {
            safe = k.max(1); // always keep the robot's own (footprint-cleared) pose
            break;
        }
    }
    poses.truncate(safe);
    Plan { poses }
}

/// True if the robot's oriented footprint box overlaps any lethal costmap cell.
/// The box is centered at the pose shifted by `offset` fore/aft along `yaw`, with
/// half-extents `half_len` along the heading and `half_w` lateral. Rasterises the
/// box's axis-aligned bound and tests each lethal cell center against the rotated
/// rectangle. Used to keep a plan from committing the robot's long axis into a
/// wall the narrow-keep-out search stepped over.
fn box_hits_lethal(
    map: &Costmap,
    x: f32,
    y: f32,
    yaw: f32,
    half_len: f32,
    half_w: f32,
    offset: f32,
) -> bool {
    let (cyaw, syaw) = (yaw.cos(), yaw.sin());
    let cx = x + offset * cyaw;
    let cy = y + offset * syaw;
    // Axis-aligned bound of the rotated box, in cells.
    let ext_x = half_len * cyaw.abs() + half_w * syaw.abs();
    let ext_y = half_len * syaw.abs() + half_w * cyaw.abs();
    let Some((r0, c0)) = map.cell(cx, cy) else {
        return false;
    };
    let r_cells = (ext_y / map.resolution).ceil() as isize;
    let c_cells = (ext_x / map.resolution).ceil() as isize;
    for dr in -r_cells..=r_cells {
        for dc in -c_cells..=c_cells {
            let r = r0 as isize + dr;
            let c = c0 as isize + dc;
            if r < 0 || c < 0 || r as usize >= map.height || c as usize >= map.width {
                continue;
            }
            let idx = r as usize * map.width + c as usize;
            if map.cost[idx] < LETHAL_THRESHOLD {
                continue;
            }
            let (wx, wy) = map.cell_center(r as usize, c as usize);
            let (lx, ly) = (wx - cx, wy - cy);
            let along = lx * cyaw + ly * syaw;
            let lat = -lx * syaw + ly * cyaw;
            if along.abs() <= half_len && lat.abs() <= half_w {
                return true;
            }
        }
    }
    false
}

fn nearest_free_cell(map: &Costmap, free: &[bool], x: f32, y: f32) -> Option<(usize, usize)> {
    let (r0, c0) = map.cell(x, y)?;
    if free[r0 * map.width + c0] {
        return Some((r0, c0));
    }
    // Expanding ring search (bounded to ~1 m — a robot boxed deeper than that
    // has no meaningful free start anyway).
    let max_ring = (1.0 / map.resolution).ceil() as isize;
    for ring in 1..=max_ring {
        for dr in -ring..=ring {
            for dc in -ring..=ring {
                if dr.abs() != ring && dc.abs() != ring {
                    continue;
                }
                let (r, c) = (r0 as isize + dr, c0 as isize + dc);
                if r < 0 || c < 0 || r as usize >= map.height || c as usize >= map.width {
                    continue;
                }
                if free[r as usize * map.width + c as usize] {
                    return Some((r as usize, c as usize));
                }
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::{chamfer_distance, Costmap, LETHAL};

    /// Horizontal corridor spanning the full x-width: cells within `free_half_rows`
    /// of the center row are free, the rest are lethal walls. res 0.1 m, 70x40. The
    /// clearance at the center row is `(free_half_rows + 1) * res` and the physical
    /// wall edge is `(free_half_rows + 0.5) * res` from center.
    fn corridor(free_half_rows: usize) -> Costmap {
        let (res, width, height) = (0.1f32, 70usize, 40usize);
        let origin = (0.0f32, -2.0f32);
        let mid = height / 2;
        let mut cost = vec![0i8; width * height];
        for r in 0..height {
            if (r as isize - mid as isize).unsigned_abs() > free_half_rows {
                for c in 0..width {
                    cost[r * width + c] = LETHAL;
                }
            }
        }
        let distance = chamfer_distance(&cost, width, height, res);
        Costmap {
            width,
            height,
            resolution: res,
            origin,
            cost,
            distance,
        }
    }

    fn cfg(robot_width: f32, safety_margin: f32) -> SolverConfig {
        SolverConfig {
            robot_width,
            robot_length: 0.7,
            safety_margin,
            ..SolverConfig::default()
        }
    }

    fn strict_free(map: &Costmap, inflate: f32) -> Vec<bool> {
        (0..map.width * map.height)
            .map(|i| map.cost[i] < LETHAL_THRESHOLD && map.distance[i] >= inflate)
            .collect()
    }

    // Criterion B (free-check level): the skinny box opens a start the fat circle walls out.
    #[test]
    fn box_width_frees_a_gap_the_circle_walls_out() {
        let map = corridor(3); // center clearance 0.4 m
                               // 0.3-wide box -> keep-out 0.25 m -> the gap HAS a clear start.
        assert!(nearest_free_cell(&map, &strict_free(&map, 0.3 * 0.5 + 0.1), 3.5, 0.0).is_some());
        // 0.7-wide "circle" -> keep-out 0.45 m -> NO clear start in the same gap.
        assert!(nearest_free_cell(&map, &strict_free(&map, 0.7 * 0.5 + 0.1), 3.5, 0.0).is_none());
    }

    // Criterion B (plan level): the oriented box threads a gap it fits, refuses one it doesn't.
    #[test]
    fn box_plan_threads_a_gap_it_fits_and_refuses_one_it_doesnt() {
        let map = corridor(3); // gap edge ~0.35 m from center
        let path = &[(0.5, 0.0), (6.5, 0.0)];
        let robot = (0.5, 0.0, 0.0);
        let fits = plan(&map, path, robot, 0.0, None, &cfg(0.3, 0.1));
        let too_wide = plan(&map, path, robot, 0.0, None, &cfg(0.9, 0.1));
        assert!(
            fits.poses.len() >= 2,
            "0.3-wide box must thread the gap, got {}",
            fits.poses.len()
        );
        assert!(
            too_wide.poses.len() < 2,
            "0.9-wide box cannot fit, got {}",
            too_wide.poses.len()
        );
    }

    // Criterion C: pinned (no keep-out-clear start) but physically passable -> escape path, not a stub.
    #[test]
    fn never_give_up_escapes_a_pinned_but_passable_gap() {
        let map = corridor(3); // clearance 0.4 m, edge 0.35 m
        let c = cfg(0.5, 0.2); // keep-out 0.45 m > 0.4 -> pinned; box half 0.25 < 0.35 -> fits
        let inflate = c.robot_width * 0.5 + c.safety_margin;
        assert!(
            nearest_free_cell(&map, &strict_free(&map, inflate), 3.5, 0.0).is_none(),
            "precondition: pinned (no keep-out-clear start)"
        );
        let out = plan(
            &map,
            &[(0.5, 0.0), (6.5, 0.0)],
            (0.5, 0.0, 0.0),
            0.0,
            None,
            &c,
        );
        assert!(
            out.poses.len() >= 2,
            "never-give-up must escape when pinned but passable, got {}",
            out.poses.len()
        );
    }
}
