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
    /// Carrot-tail trim at sharp route reversals (deg): a shortest-path solver
    /// cuts any hairpin below the waypoint's advance radius, so the path pins
    /// at true reversals (measured: forced hairpins scored 52-57 deg/m churn
    /// and missed the top-of-stairs marker).
    pub tail_reversal_trim_deg: f32,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            vehicle_width: 0.5,
            safety_margin: 0.1,
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
            tail_reversal_trim_deg: 100.0,
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
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
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

/// Trim the route tail at the first sharp reversal from the approach heading
/// and densify it (port of _carrot_extension).
pub fn carrot_extension(
    global_path: &[(f32, f32)],
    tail: &[(f32, f32)],
    cfg: &SolverConfig,
) -> Vec<(f32, f32)> {
    if global_path.is_empty() || tail.is_empty() {
        return Vec::new();
    }
    let trim_cos = (cfg.tail_reversal_trim_deg.to_radians()).cos();
    // Route-end heading over the last >=0.5 m of ARC, not the final raw
    // segment: MLS routes can end with a centimeter-scale doubled-back vertex
    // (the goal snap point sitting slightly behind the previous vertex), and
    // hl62/63 that flipped the heading south so a genuinely-reversing tail
    // (wp3 -> wp4) read as "straight ahead" (dot 0.99), survived the trim,
    // doubled back past the robot, and captured the carrot into a 0.22 m stub
    // — parking the course 1.84 m short of wp3.
    let mut heading: Option<(f32, f32)> = None;
    {
        let end = *global_path.last().unwrap();
        let mut arc = 0.0;
        let mut anchor = None;
        for w in global_path.windows(2).rev() {
            arc += (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1);
            anchor = Some(w[0]);
            if arc >= 0.5 {
                break;
            }
        }
        if let Some(a) = anchor {
            let norm = (end.0 - a.0).hypot(end.1 - a.1);
            if norm > 1e-6 {
                heading = Some(((end.0 - a.0) / norm, (end.1 - a.1) / norm));
            }
        }
    }
    let mut budget = cfg.carrot_lookahead_max + 2.0;
    let mut pts: Vec<(f32, f32)> = vec![*global_path.last().unwrap()];
    for &p in tail {
        if budget <= 0.0 {
            break;
        }
        let last = *pts.last().unwrap();
        let (dx, dy) = (p.0 - last.0, p.1 - last.1);
        let norm = dx.hypot(dy);
        if norm <= 1e-6 {
            continue;
        }
        if let Some(h) = heading {
            if (dx * h.0 + dy * h.1) / norm < trim_cos {
                break; // sharp reversal: pin the path at this waypoint
            }
        }
        heading = Some((dx / norm, dy / norm));
        budget -= norm;
        pts.push(p);
    }
    // Densify at 0.25 m (sparse marker segments would leap the carrot's arc
    // budget in one stride), dropping the duplicated global end point.
    let mut out = Vec::new();
    for pair in pts.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let steps = ((b.0 - a.0).hypot(b.1 - a.1) / 0.25).ceil().max(1.0) as usize;
        for k in 1..=steps {
            let f = k as f32 / steps as f32;
            out.push((a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f));
        }
    }
    out
}

pub struct Plan {
    /// World-frame (x, y, yaw).
    pub poses: Vec<(f32, f32, f32)>,
}

/// One full solve (port of plan_path). `scan_extension` extends the carrot and
/// the goal past the global path's end but is EXCLUDED from the adherence
/// field (its straight segments only approximate the future route).
pub fn plan(
    map: &Costmap,
    global_path: &[(f32, f32)],
    scan_extension: &[(f32, f32)],
    robot: (f32, f32, f32),
    speed: f32,
    previous_path: Option<&[(f32, f32)]>,
    cfg: &SolverConfig,
) -> Plan {
    if global_path.is_empty() {
        return Plan { poses: Vec::new() };
    }
    let (carrot_budget, radius, horizon) = cfg.scaled(speed);

    // Densify the scan path (0.25 m): the carrot scan and the gap-hop walk
    // path POINTS, so sparse vertices (a 2-point straight route) would blow the
    // arc budget in one stride and collapse the carrot to the robot cell.
    let mut sparse: Vec<(f32, f32)> = global_path.to_vec();
    sparse.extend_from_slice(scan_extension);
    let mut scan_path: Vec<(f32, f32)> = vec![sparse[0]];
    let mut route_scan_len = 1; // scan_path prefix that belongs to the ROUTE
    for (si, pair) in sparse.windows(2).enumerate() {
        let (a, b) = (pair[0], pair[1]);
        let d = (b.0 - a.0).hypot(b.1 - a.1);
        let steps = (d / 0.25).ceil().max(1.0) as usize;
        for k in 1..=steps {
            let f = k as f32 / steps as f32;
            scan_path.push((a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f));
        }
        if si + 2 <= global_path.len() {
            route_scan_len = scan_path.len();
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
    let inflate = cfg.vehicle_width * 0.5 + cfg.safety_margin;

    let free: Vec<bool> = (0..n)
        .map(|i| map.cost[i] < LETHAL_THRESHOLD && map.distance[i] >= inflate)
        .collect();

    // Robot cell (nearest free within a small ring if the exact cell is not).
    let Some(robot_cell) = nearest_free_cell(map, &free, robot.0, robot.1) else {
        return Plan { poses: Vec::new() };
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
    heap.push(HeapEntry { cost: 0.0, index: start });
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
            if (r - robot_cell.0 as isize).abs() > max_cells || (c - robot_cell.1 as isize).abs() > max_cells {
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
                heap.push(HeapEntry { cost: next, index: j });
            }
        }
    }
    let reachable = |i: usize| parent[i] != usize::MAX;

    // Carrot: furthest reachable scan-path point within the arc budget,
    // hopping unreachable runs up to carrot_gap_max (single-cell flicker must
    // not collapse the plan; real walls span far more arc and still stop it).
    // The nearest-point anchor searches the ROUTE prefix only: an extension
    // that doubles back past the robot (wp3 -> wp4 hairpin, hl62/63) would
    // otherwise capture the scan onto the tail branch and skip the entire
    // unfinished leg.
    let start_idx = scan_path[..route_scan_len.min(scan_path.len())]
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
    Plan { poses }
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
