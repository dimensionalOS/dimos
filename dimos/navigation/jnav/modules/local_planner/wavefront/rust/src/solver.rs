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

//! Navigation-function local planner — a faithful port of the measured Python
//! `plan_path` in `wavefront/local_planner.py`: one Dijkstra wavefront
//! rooted at the robot over a clearance-shaped cost field (travel + clearance +
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
        if let Some(&(x, y)) = polyline.first() {
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
///
/// Solves on the strict (keep-out-clear) mask first; if that makes no
/// meaningful progress ALONG THE ROUTE while the goal is still far, re-solves
/// on the relaxed (any non-lethal) mask and keeps whichever advances further.
/// The start-selection fallback inside the solve only relaxes when the ROBOT
/// has no strict cell nearby — it cannot help when a strict pocket exists
/// behind the robot but the way FORWARD is sub-clearance (the stairs, where
/// the terrain slice reads the treads at ~0.1 m clearance).
///
/// Progress is measured along the route, not as raw reach: when only the
/// pocket BEHIND is strict-reachable, the carrot fallback emits a plan that
/// walks meters AWAY from the goal — a healthy-looking reach pointing the
/// wrong way. Alternating those with forward relaxed solves is exactly the
/// flip-flopping local path seen on the stairs climb (2026-07-19): headings
/// swinging 90-180 deg tick to tick.
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
    let scan_path = densify_path(global_path);
    let first = plan_masked(map, &scan_path, robot, speed, previous_path, cfg, false);
    let goal_far = global_path
        .last()
        .map(|g| (g.0 - robot.0).hypot(g.1 - robot.1) > 1.0)
        .unwrap_or(false);
    if goal_far {
        // Degenerate two ways: no progress ALONG the route (a healthy-reach plan
        // pointing the wrong way), or no reach at all (a stub, whose snapped
        // route projection can overshoot its true progress by a scan vertex).
        let p_first = route_progress(&scan_path, robot, &first);
        let reach_first = first
            .poses
            .last()
            .map(|e| (e.0 - robot.0).hypot(e.1 - robot.1))
            .unwrap_or(0.0);
        if p_first < DEGENERATE_REACH || reach_first < DEGENERATE_REACH {
            let relaxed = plan_masked(map, &scan_path, robot, speed, previous_path, cfg, true);
            if route_progress(&scan_path, robot, &relaxed) > p_first {
                return relaxed;
            }
        }
    }
    first
}

/// A plan that advances less than this along the route is a stub — the same
/// threshold the module's degenerate-plan recovery uses for reach.
const DEGENERATE_REACH: f32 = 0.3;

/// Densify a route to ~0.25 m spacing: the carrot scan and the gap-hop walk
/// path POINTS, so sparse vertices (a 2-point straight route) would blow the
/// arc budget in one stride and collapse the carrot to the robot cell.
fn densify_path(global_path: &[(f32, f32)]) -> Vec<(f32, f32)> {
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
    scan_path
}

/// Signed arc-length progress of `plan`'s endpoint along the densified route,
/// measured from the route point nearest the robot. Negative = the plan ends
/// BEHIND the robot's route anchor (walking away from the goal). The endpoint
/// is matched only within the plan's own arc length of the anchor: a route
/// that folds back near itself in 2D (a stairs switchback) would otherwise
/// snap a short stub onto a far fold and report meters of phantom progress.
fn route_progress(scan_path: &[(f32, f32)], robot: (f32, f32, f32), plan: &Plan) -> f32 {
    let Some(&(ex, ey, _)) = plan.poses.last() else {
        return 0.0;
    };
    let plan_len: f32 = plan
        .poses
        .windows(2)
        .map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1))
        .sum();
    // Cumulative arc length along the route.
    let mut arc = Vec::with_capacity(scan_path.len());
    let mut acc = 0.0f32;
    arc.push(0.0);
    for pair in scan_path.windows(2) {
        acc += (pair[1].0 - pair[0].0).hypot(pair[1].1 - pair[0].1);
        arc.push(acc);
    }
    let mut anchor = 0usize;
    let mut best_d = f32::MAX;
    for (i, &(px, py)) in scan_path.iter().enumerate() {
        let d = (px - robot.0).hypot(py - robot.1);
        if d < best_d {
            best_d = d;
            anchor = i;
        }
    }
    let window = plan_len + 0.5;
    let mut end = anchor;
    best_d = f32::MAX;
    for (i, &(px, py)) in scan_path.iter().enumerate() {
        if (arc[i] - arc[anchor]).abs() > window {
            continue;
        }
        let d = (px - ex).hypot(py - ey);
        if d < best_d {
            best_d = d;
            end = i;
        }
    }
    arc[end] - arc[anchor]
}

#[allow(clippy::too_many_arguments)]
fn plan_masked(
    map: &Costmap,
    scan_path: &[(f32, f32)],
    robot: (f32, f32, f32),
    speed: f32,
    previous_path: Option<&[(f32, f32)]>,
    cfg: &SolverConfig,
    relax: bool,
) -> Plan {
    if scan_path.is_empty() {
        return Plan { poses: Vec::new() };
    }
    let (carrot_budget, radius, horizon) = cfg.scaled(speed);
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
    // keeps it from physically clipping a wall on the way. With `relax` the whole
    // solve runs on the non-lethal mask (see `plan`: the strict solve stubbed).
    let relaxed_mask = || (0..n).map(|i| map.cost[i] < LETHAL_THRESHOLD).collect::<Vec<bool>>();
    let (robot_cell, free) = if relax {
        let free_relaxed = relaxed_mask();
        match nearest_free_cell(map, &free_relaxed, robot.0, robot.1) {
            Some(c) => (c, free_relaxed),
            None => return Plan { poses: Vec::new() },
        }
    } else {
        match nearest_free_cell(map, &free_strict, robot.0, robot.1) {
            Some(c) => (c, free_strict),
            None => {
                let free_relaxed = relaxed_mask();
                match nearest_free_cell(map, &free_relaxed, robot.0, robot.1) {
                    Some(c) => (c, free_relaxed),
                    None => return Plan { poses: Vec::new() },
                }
            }
        }
    };

    // Entry cost: travel + clearance ramp within influence_radius past the
    // body + adherence to the REAL global path + temporal commitment.
    let path_dist = polyline_distance_field(map, scan_path);
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
    //
    // ESCAPE EXCEPTION: a robot whose CURRENT stance is collision-free may
    // always execute the first body-length of the plan. The box gate tests
    // each pose at its final PLANNED heading, but a departing robot's
    // orientation is transient — vetoing the first poses on that heading
    // deadlocks it: parked snug against a wall, every solve wants to turn
    // away, the turned box's tail sweeps the wall, the plan truncates to a
    // stub, and the robot never moves again (the wp3 cross-wall and stairs
    // freezes, 2026-07-15; on stairs the terrain slice additionally renders
    // the not-yet-visible upper steps as walls, boxing the stance in on every
    // side but the climb direction). The search's centerline is non-lethal by
    // construction and the stance proves the robot fits here, so one body
    // length of it is safe to commit; solves re-run continuously, so the full
    // gate re-engages as the robot advances. A robot that genuinely cannot
    // fit (its stance box itself overlaps lethal, e.g. wider than the gap)
    // gets no exception and still refuses to commit. Beyond the escape
    // segment the planned heading is what the robot will actually hold, so
    // the gate is unchanged.
    let half_len = cfg.robot_length * 0.5;
    let half_w = cfg.robot_width * 0.5;
    let stance_clear = !box_hits_lethal(
        map,
        robot.0,
        robot.1,
        robot.2,
        half_len,
        half_w,
        cfg.footprint_offset,
    );
    let escape = cfg.robot_length;
    let mut safe = poses.len();
    for (k, &(px, py, pyaw)) in poses.iter().enumerate() {
        if stance_clear && (px - robot.0).hypot(py - robot.1) <= escape {
            continue;
        }
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

    // Regression (wp3 cross-wall freeze, 2026-07-15): a robot parked snug beside a
    // wall — CURRENT-heading box clear by millimetres, but any turn-away pose's box
    // sweeps its tail through the wall. The box veto on the planned heading used to
    // truncate every plan to a 1-pose stub, freezing the robot in place forever.
    // The escape exception must let it drive out along the route instead.
    #[test]
    fn escape_turn_beside_a_wall_is_not_vetoed() {
        // Open 7 x 4 m field, one wall row at y = 0.2 spanning x >= 3.4
        // (cell centers = origin + idx * res, row 22 -> y = 0.2).
        let (res, width, height) = (0.1f32, 70usize, 40usize);
        let mut cost = vec![0i8; width * height];
        for c in 34..width {
            cost[22 * width + c] = LETHAL;
        }
        let distance = chamfer_distance(&cost, width, height, res);
        let map = Costmap {
            width,
            height,
            resolution: res,
            origin: (0.0, -2.0),
            cost,
            distance,
        };
        // Robot 0.19 m south of the wall, heading along it; route turns away NW.
        let robot = (3.5f32, 0.01f32, 0.0f32);
        let path = &[(3.5, 0.01), (2.6, 0.85), (1.5, 1.5)];
        let c = cfg(0.33, 0.1);
        // Preconditions that make this THE freeze geometry: the current-heading
        // box clears the wall, the turn-away (path-heading) box does not.
        let (half_len, half_w) = (c.robot_length * 0.5, c.robot_width * 0.5);
        assert!(
            !box_hits_lethal(&map, robot.0, robot.1, robot.2, half_len, half_w, 0.0),
            "precondition: the robot's current stance is clear"
        );
        let turn = f32::atan2(path[1].1 - path[0].1, path[1].0 - path[0].0);
        assert!(
            box_hits_lethal(&map, robot.0, robot.1, turn, half_len, half_w, 0.0),
            "precondition: the turned box sweeps the wall"
        );
        let out = plan(&map, path, robot, 0.0, None, &c);
        let reach = out
            .poses
            .last()
            .map(|p| (p.0 - robot.0).hypot(p.1 - robot.1))
            .unwrap_or(0.0);
        assert!(
            out.poses.len() >= 2 && reach >= 1.0,
            "escape from a snug wall must not be vetoed: {} poses, reach {:.2} m",
            out.poses.len(),
            reach
        );
    }

    // Regression (stairs wedge 2026-07-15 + flip-flop 2026-07-19): a strict-free
    // pocket BEHIND the robot with a passable-but-sub-clearance corridor AHEAD.
    // Start selection finds the strict pocket and the strict solve emits a plan
    // TOWARD it — a backward stub whose raw reach can look healthy (the robot
    // sits deep enough in the corridor that walking back to the pocket covers
    // >0.3 m). Selecting by reach alternated backward-strict with forward-
    // relaxed solves tick to tick: the local path swung 90-180 deg on the
    // stairs climb. Route-PROGRESS selection must pick the forward relaxed plan.
    #[test]
    fn strict_pocket_behind_does_not_stub_a_passable_corridor() {
        let (res, width, height) = (0.1f32, 70usize, 40usize);
        let mid = height / 2;
        let mut cost = vec![0i8; width * height];
        // Open (strict-clear) field for x < 2.0; beyond it a 4-row corridor:
        // 0.2 m centerline clearance — under the 0.265 m strict keep-out, over
        // the 0.165 m box half-width, so it is passable but never strict.
        for r in 0..height {
            let in_band = (mid - 1..=mid + 2).contains(&r);
            if !in_band {
                for c in 20..width {
                    cost[r * width + c] = LETHAL;
                }
            }
        }
        let distance = chamfer_distance(&cost, width, height, res);
        let map = Costmap {
            width,
            height,
            resolution: res,
            origin: (0.0, -2.0),
            cost,
            distance,
        };
        let c = cfg(0.33, 0.1); // inflate 0.265 > corridor edge 0.25 -> corridor is not strict
        let robot = (2.6f32, 0.0f32, 0.0f32); // deep enough that the backward stub has reach
        let out = plan(&map, &[(2.6, 0.0), (6.0, 0.0)], robot, 0.0, None, &c);
        let last = out.poses.last().copied().unwrap_or((robot.0, robot.1, 0.0));
        assert!(
            last.0 - robot.0 >= 1.0,
            "must advance down the passable corridor, not stub toward the strict pocket \
             behind: {} poses, forward {:.2} m",
            out.poses.len(),
            last.0 - robot.0
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
