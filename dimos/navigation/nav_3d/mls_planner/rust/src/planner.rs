// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{rise, CellId, SurfaceCells, SurfaceLookup};
use crate::dijkstra::walk_preds;
use crate::edges::{NodeEdgeIdx, NodeId, PlannerGraph, NO_NODE};
use crate::mls_planner::Config;
use crate::nodes::penalty_of;
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Robot-rooted candidate search radius, in multiples of node spacing.
const CANDIDATE_RADIUS_FACTOR: f32 = 3.0;

/// Horizontal search radius when snapping a pose to the surface.
const SNAP_SEARCH_RADIUS_M: f32 = 1.5;

/// Max snap candidates tried when connecting the start.
const MAX_SNAP_ATTEMPTS: usize = 64;

/// On a blocked path, stop this far short of the last traversable point.
const BEST_EFFORT_DISTANCE_M: f32 = 1.0;

/// World-frame waypoints paired with the string-pulled cell path that produced
/// them. The cell path is cached for later safe truncation.
type PlannedPath = (Vec<(f32, f32, f32)>, Vec<VoxelKey>);

/// Surface cells near the pose, nearest first in xy.
pub fn snap_candidates(
    surface_lookup: &SurfaceLookup,
    pose: (f32, f32, f32),
    voxel_size: f32,
    tolerance_m: f32,
) -> Vec<VoxelKey> {
    let ix = (pose.0 / voxel_size).floor() as i32;
    let iy = (pose.1 / voxel_size).floor() as i32;
    let target_iz = (pose.2 / voxel_size).floor() as i32 - 1;
    let tol_cells = (tolerance_m / voxel_size).ceil() as i32;
    let search_radius = (SNAP_SEARCH_RADIUS_M / voxel_size).ceil() as i32;

    let mut found: Vec<(i32, VoxelKey)> = Vec::new();
    for dix in -search_radius..=search_radius {
        for diy in -search_radius..=search_radius {
            if let Some(cell) =
                best_iz_in_column(surface_lookup, ix + dix, iy + diy, target_iz, tol_cells)
            {
                found.push((dix * dix + diy * diy, cell));
            }
        }
    }
    found.sort_by_key(|&(d2, _)| d2);
    found.into_iter().map(|(_, c)| c).collect()
}

/// Snap a pose to the nearest surface cell.
pub fn snap_pose_to_cell(
    surface_lookup: &SurfaceLookup,
    pose: (f32, f32, f32),
    voxel_size: f32,
    tolerance_m: f32,
) -> Option<VoxelKey> {
    snap_candidates(surface_lookup, pose, voxel_size, tolerance_m)
        .into_iter()
        .next()
}

fn best_iz_in_column(
    surface_lookup: &SurfaceLookup,
    ix: i32,
    iy: i32,
    target_iz: i32,
    tol_cells: i32,
) -> Option<VoxelKey> {
    let zs = surface_lookup.get(&(ix, iy))?;
    let mut best: Option<(i32, i32)> = None;
    for &iz in zs {
        let d = (iz - target_iz).abs();
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, iz));
        }
    }
    let (bd, iz) = best?;
    if bd > tol_cells {
        return None;
    }
    Some((ix, iy, iz))
}

/// Plan path from start pose to goal pose using the node graph.
/// Returns the waypoints and the string-pulled cell path, or none if either
/// pose can't be snapped to surface or there is no valid path.
pub fn plan(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    config: &Config,
) -> Option<PlannedPath> {
    let voxel_size = config.voxel_size;
    let z_tolerance_m = config.robot_height;
    let start_candidates =
        snap_candidates(&plg.surface_lookup, start_pose, voxel_size, z_tolerance_m);
    if start_candidates.is_empty() {
        tracing::warn!(
            ?start_pose,
            "plan failed: start does not snap to any surface cell"
        );
        return None;
    }
    let Some(goal_coord) =
        snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)
    else {
        tracing::warn!(
            ?goal_pose,
            "plan failed: goal does not snap to any surface cell"
        );
        return None;
    };
    let Some(goal_cell) = plg.cells.id(goal_coord) else {
        tracing::warn!(?goal_coord, "plan failed: goal cell is not in the graph");
        return None;
    };

    let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();

    // The penalized Voronoi cannot own sub-clearance goals, so fall back to the
    // nearest node by hops. A real failure then reports as disconnected below.
    let mut goal_segment = walk_preds(&plg.cell_state, goal_cell);
    let mut goal_node = *goal_segment
        .last()
        .expect("walk_preds returns at least the start cell");
    if !node_cells.contains(&goal_node) {
        let Some((node, path)) = nearest_node(&plg.cells, goal_cell, &node_cells) else {
            tracing::warn!(
                ?goal_coord,
                "plan failed: goal's connected component has no graph node"
            );
            return None;
        };
        goal_node = node;
        goal_segment = path;
    }

    // Rooted at the goal so one pass covers every node's cost-to-go.
    let (cost_to_go, pred_to_goal) = node_dijkstra(plg, goal_node);

    let radius = (config.node_spacing_m * CANDIDATE_RADIUS_FACTOR).max(voxel_size);
    let mut entry: Option<(Vec<CellId>, Vec<NodeId>)> = None;
    for &candidate in start_candidates.iter().take(MAX_SNAP_ATTEMPTS) {
        let Some(start_cell) = plg.cells.id(candidate) else {
            continue;
        };
        entry = select_entry(
            plg,
            start_cell,
            goal_node,
            &cost_to_go,
            &pred_to_goal,
            &node_cells,
            radius,
        );
        if entry.is_some() {
            break;
        }
    }
    let Some((lead_in, node_seq)) = entry else {
        tracing::warn!(
            candidates = start_candidates.len().min(MAX_SNAP_ATTEMPTS),
            reachable_nodes = cost_to_go.len(),
            total_nodes = plg.nodes.len(),
            "plan failed: start and goal lie on separate connected surface components",
        );
        return None;
    };

    // Max traversable step in cells, floored to match the graph adjacency.
    let step_cells = (config.step_threshold_m / voxel_size).floor() as i32;

    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };
    let cells = assemble_cells(plg, &node_seq, &lead_in, &goal_segment);
    let cells = string_pull(plg, &cells, step_cells, &wall_cost);
    let waypoints = cells_to_waypoints(plg, &cells, start_pose, goal_pose, voxel_size);
    let path_cells: Vec<VoxelKey> = cells.iter().map(|&id| plg.cells.coord(id)).collect();
    Some((waypoints, path_cells))
}

/// Re-validate a cached path against the current surface. Returns the route
/// ahead up to a standoff short of the first blockage, or empty when nothing
/// ahead is safe, which the follower reads as a stop.
pub fn truncate_to_safe(
    plg: &PlannerGraph,
    cached: &[VoxelKey],
    start_pose: (f32, f32, f32),
    config: &Config,
) -> Vec<(f32, f32, f32)> {
    if cached.len() < 2 {
        return Vec::new();
    }
    let voxel_size = config.voxel_size;
    let step_cells = (config.step_threshold_m / voxel_size).floor() as i32;
    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };

    // The cached path's head is the stale original start. Resume from where the
    // robot sits on it so the follower is pulled forward, never back to it.
    let resume = resume_segment(cached, start_pose, voxel_size);

    // Walk each chord ahead at surface resolution. On a blockage, keep up to the
    // last traversable cell so the path ends at the obstacle, not the prior anchor.
    let mut waypoints = vec![start_pose];
    let mut blocked = false;
    for j in resume..cached.len() - 1 {
        let (last_safe, cut) = last_safe_on_chord(
            plg,
            cached[j],
            cached[j + 1],
            step_cells,
            &wall_cost,
            voxel_size,
        );
        if cut {
            if let Some(p) = last_safe {
                if waypoints.last() != Some(&p) {
                    waypoints.push(p);
                }
            }
            blocked = true;
            break;
        }
        let (ix, iy, iz) = cached[j + 1];
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }

    if waypoints.len() < 2 {
        return Vec::new();
    }

    // Hold the standoff only when blocked; a clean run to the goal has nothing
    // to stand off from.
    if blocked {
        return back_off_tail(&waypoints, BEST_EFFORT_DISTANCE_M);
    }
    waypoints
}

/// Trim `distance` off the goal end, measured in the ground plane.
fn back_off_tail(waypoints: &[(f32, f32, f32)], distance: f32) -> Vec<(f32, f32, f32)> {
    let mut remaining = distance;
    for i in (1..waypoints.len()).rev() {
        let (b, a) = (waypoints[i], waypoints[i - 1]);
        let seg = (b.0 - a.0).hypot(b.1 - a.1);
        if seg < remaining {
            remaining -= seg;
            continue;
        }
        let t = if seg == 0.0 {
            0.0
        } else {
            (seg - remaining) / seg
        };
        let cut = (
            a.0 + (b.0 - a.0) * t,
            a.1 + (b.1 - a.1) * t,
            a.2 + (b.2 - a.2) * t,
        );
        let mut out = waypoints[..i].to_vec();
        if out.last() != Some(&cut) {
            out.push(cut);
        }
        return if out.len() >= 2 { out } else { Vec::new() };
    }
    Vec::new()
}

/// Walk the straight chord a -> b at surface resolution, the same sampling the
/// segment validator uses. Returns the world point of the last traversable
/// surface cell reached and whether the chord was cut short of b. The point is
/// None only when the chord's start column is already off the surface.
fn last_safe_on_chord(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    wc: &WallCost,
    voxel_size: f32,
) -> (Option<(f32, f32, f32)>, bool) {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        // Same column: traversable only if it is not a pure vertical move.
        return if dz == 0 {
            (Some(surface_point_xyz(a.0, a.1, a.2, voxel_size)), false)
        } else {
            (None, true)
        };
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    let mut prev_iz: Option<i32> = None;
    let mut last_safe: Option<(f32, f32, f32)> = None;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let iz_line = a.2 as f32 + t * dz as f32;
        let Some(zs) = plg.surface_lookup.get(&(ix, iy)) else {
            return (last_safe, true);
        };
        // Surface cell in this column nearest the interpolated segment height.
        let mut nearest: Option<(f32, i32)> = None;
        for &iz in zs {
            let d = (iz as f32 - iz_line).abs();
            if nearest.is_none_or(|(bd, _)| d < bd) {
                nearest = Some((d, iz));
            }
        }
        let Some((d, iz)) = nearest else {
            return (last_safe, true);
        };
        if d > step_cells as f32 {
            return (last_safe, true);
        }
        if prev_iz.is_some_and(|p| (iz - p).abs() > step_cells) {
            return (last_safe, true);
        }
        let pen = match plg.cells.id((ix, iy, iz)) {
            Some(id) => {
                let wall_dist = plg
                    .wall_state
                    .dist
                    .get(id as usize)
                    .copied()
                    .unwrap_or(f32::INFINITY);
                penalty_of(wall_dist, wc.clearance_m, wc.buffer_m, wc.buffer_weight)
            }
            None => 1.0,
        };
        if !pen.is_finite() {
            return (last_safe, true);
        }
        prev_iz = Some(iz);
        last_safe = Some(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    (last_safe, false)
}

/// Index of the cached segment the robot is on, by nearest-point projection in
/// the ground plane. The route ahead resumes at the following cell.
fn resume_segment(cached: &[VoxelKey], start: (f32, f32, f32), voxel_size: f32) -> usize {
    let p = (start.0, start.1);
    let mut best = 0usize;
    let mut best_d2 = f32::INFINITY;
    for i in 0..cached.len() - 1 {
        let a = surface_point_xyz(cached[i].0, cached[i].1, cached[i].2, voxel_size);
        let b = surface_point_xyz(
            cached[i + 1].0,
            cached[i + 1].1,
            cached[i + 1].2,
            voxel_size,
        );
        let d2 = point_segment_dist2((a.0, a.1), (b.0, b.1), p);
        if d2 < best_d2 {
            best_d2 = d2;
            best = i;
        }
    }
    best
}

/// Squared distance from point p to segment a-b in the plane.
fn point_segment_dist2(a: (f32, f32), b: (f32, f32), p: (f32, f32)) -> f32 {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let denom = abx * abx + aby * aby;
    let t = if denom == 0.0 {
        0.0
    } else {
        (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / denom).clamp(0.0, 1.0)
    };
    let (cx, cy) = (a.0 + t * abx, a.1 + t * aby);
    let (dx, dy) = (p.0 - cx, p.1 - cy);
    dx * dx + dy * dy
}

/// Pick the entry node by connect cost plus cost-to-go, with its on-surface
/// lead-in and the node sequence to the goal.
fn select_entry(
    plg: &PlannerGraph,
    start_cell: CellId,
    goal_node: NodeId,
    cost_to_go: &AHashMap<NodeId, f32>,
    pred_to_goal: &AHashMap<NodeId, NodeId>,
    node_cells: &AHashSet<NodeId>,
    radius_m: f32,
) -> Option<(Vec<CellId>, Vec<NodeId>)> {
    let (connect_dist, connect_pred) = robot_search(&plg.cells, start_cell, radius_m);

    let mut entry_node = NO_NODE;
    let mut best_score = f32::INFINITY;
    // Scan the bounded reachable set, not every node. Tie-break by cell id for
    // deterministic order.
    for (&cell, &connect) in &connect_dist {
        if !node_cells.contains(&cell) {
            continue;
        }
        let Some(&ctg) = cost_to_go.get(&cell) else {
            continue;
        };
        let score = connect + ctg;
        let better = match score.partial_cmp(&best_score) {
            Some(std::cmp::Ordering::Less) => true,
            Some(std::cmp::Ordering::Equal) => cell < entry_node,
            _ => false,
        };
        if better {
            best_score = score;
            entry_node = cell;
        }
    }

    if best_score.is_finite() {
        let mut lead = walk_local_preds(&connect_pred, entry_node);
        lead.reverse();
        return Some((lead, follow_preds(entry_node, goal_node, pred_to_goal)?));
    }

    let start_segment = walk_preds(&plg.cell_state, start_cell);
    let region_node = *start_segment.last()?;
    if !node_cells.contains(&region_node)
        || !cost_to_go.get(&region_node).is_some_and(|c| c.is_finite())
    {
        return None;
    }
    Some((
        start_segment,
        follow_preds(region_node, goal_node, pred_to_goal)?,
    ))
}

/// Bounded Dijkstra from the robot cell. Cost is wall-penalized for steering,
/// but the radius bounds metric distance, not penalized cost.
fn robot_search(
    cells: &SurfaceCells,
    source: CellId,
    radius_m: f32,
) -> (AHashMap<CellId, f32>, AHashMap<CellId, CellId>) {
    let mut dist: AHashMap<CellId, f32> = AHashMap::new();
    let mut geo: AHashMap<CellId, f32> = AHashMap::new();
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    dist.insert(source, 0.0);
    geo.insert(source, 0.0);
    heap.push(Scored(0.0, source));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        // Stop expanding past the metric radius.
        if geo.get(&u).copied().unwrap_or(f32::INFINITY) > radius_m {
            continue;
        }
        for edge in cells.neighbors(u) {
            let nd = d + edge.cost;
            if nd < dist.get(&edge.dest).copied().unwrap_or(f32::INFINITY) {
                dist.insert(edge.dest, nd);
                geo.insert(edge.dest, geo[&u] + edge.base_cost);
                pred.insert(edge.dest, u);
                heap.push(Scored(nd, edge.dest));
            }
        }
    }
    (dist, pred)
}

/// Nearest node to `from` by hops, ignoring edge cost so it reaches a node
/// across cells the wall penalty makes impassable. Returns the node and the
/// path from `from`.
fn nearest_node(
    cells: &SurfaceCells,
    from: CellId,
    node_cells: &AHashSet<NodeId>,
) -> Option<(NodeId, Vec<CellId>)> {
    if node_cells.contains(&from) {
        return Some((from, vec![from]));
    }
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut seen: AHashSet<CellId> = AHashSet::new();
    let mut queue: VecDeque<CellId> = VecDeque::new();
    seen.insert(from);
    queue.push_back(from);

    while let Some(u) = queue.pop_front() {
        for edge in cells.neighbors(u) {
            let v = edge.dest;
            if !seen.insert(v) {
                continue;
            }
            pred.insert(v, u);
            if node_cells.contains(&v) {
                let mut path = vec![v];
                let mut cur = v;
                while let Some(&p) = pred.get(&cur) {
                    cur = p;
                    path.push(cur);
                }
                path.reverse();
                return Some((v, path));
            }
            queue.push_back(v);
        }
    }
    None
}

/// Walk predecessors back to the search source.
fn walk_local_preds(pred: &AHashMap<CellId, CellId>, from: CellId) -> Vec<CellId> {
    let mut path = vec![from];
    let mut cur = from;
    while let Some(&p) = pred.get(&cur) {
        cur = p;
        path.push(cur);
    }
    path
}

/// Cost-to-go to source for every reachable node, with a predecessor pointing
/// one hop toward it. Nodes are keyed by their CellId. Unreachable nodes are
/// simply absent from the maps.
fn node_dijkstra(
    plg: &PlannerGraph,
    source: NodeId,
) -> (AHashMap<NodeId, f32>, AHashMap<NodeId, NodeId>) {
    let mut dist: AHashMap<NodeId, f32> = AHashMap::new();
    let mut pred: AHashMap<NodeId, NodeId> = AHashMap::new();
    dist.insert(source, 0.0);
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    heap.push(Scored(0.0, source));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        let Some(adj) = plg.node_adj.get(&u) else {
            continue;
        };
        for &edge_idx in adj {
            let edge = &plg.node_edges[edge_idx as usize];
            let neighbor = if edge.a == u { edge.b } else { edge.a };
            let nd = d + edge.cost;
            if nd < dist.get(&neighbor).copied().unwrap_or(f32::INFINITY) {
                dist.insert(neighbor, nd);
                pred.insert(neighbor, u);
                heap.push(Scored(nd, neighbor));
            }
        }
    }
    (dist, pred)
}

/// Build the node sequence by following goal-pointing predecessors.
fn follow_preds(
    from: NodeId,
    goal: NodeId,
    pred: &AHashMap<NodeId, NodeId>,
) -> Option<Vec<NodeId>> {
    let mut seq = vec![from];
    let mut cur = from;
    while cur != goal {
        let &next = pred.get(&cur)?;
        cur = next;
        seq.push(cur);
    }
    Some(seq)
}

/// Append a cell, cancelling an out-and-back spur when the next cell retraces
/// the second-to-last.
fn push_cell(cells: &mut Vec<CellId>, c: CellId) {
    if cells.len() >= 2 && cells[cells.len() - 2] == c {
        cells.pop();
    } else if cells.last() != Some(&c) {
        cells.push(c);
    }
}

/// Build the cell path from the entry lead-in through the node edges to the goal.
fn assemble_cells(
    plg: &PlannerGraph,
    node_seq: &[NodeId],
    lead_in: &[CellId],
    goal_segment: &[CellId],
) -> Vec<CellId> {
    let mut cells: Vec<CellId> = Vec::new();
    for &c in lead_in {
        push_cell(&mut cells, c);
    }

    for pair in node_seq.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let edge_idx =
            edge_between(plg, a, b).expect("consecutive nodes in path must share an edge");
        let edge = &plg.node_edges[edge_idx as usize];
        let (start_side, end_side) = if a == edge.a {
            (edge.boundary_u, edge.boundary_v)
        } else {
            (edge.boundary_v, edge.boundary_u)
        };

        let mut from_a = walk_preds(&plg.cell_state, start_side);
        from_a.reverse();
        let to_b = walk_preds(&plg.cell_state, end_side);

        for c in from_a.into_iter().chain(to_b) {
            push_cell(&mut cells, c);
        }
    }

    for &c in goal_segment.iter().rev() {
        push_cell(&mut cells, c);
    }

    cells
}

/// Convert the cell path to world waypoints, with the raw start and goal poses
/// as the endpoints.
fn cells_to_waypoints(
    plg: &PlannerGraph,
    cells: &[CellId],
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
) -> Vec<(f32, f32, f32)> {
    let mut waypoints: Vec<(f32, f32, f32)> = Vec::with_capacity(cells.len() + 2);
    waypoints.push(start_pose);
    for &id in cells {
        let (ix, iy, iz) = plg.cells.coord(id);
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    waypoints.push(goal_pose);
    waypoints
}

/// Clearance and step limits the smoother holds the path to.
struct WallCost {
    clearance_m: f32,
    buffer_m: f32,
    buffer_weight: f32,
    voxel_size: f32,
}

/// Replace runs of cells with straight chords that come no closer to a wall and
/// climb no more than the run they replace.
fn string_pull(
    plg: &PlannerGraph,
    cells: &[CellId],
    step_cells: i32,
    wc: &WallCost,
) -> Vec<CellId> {
    if cells.len() <= 2 {
        return cells.to_vec();
    }
    let metrics = |from: CellId, to: CellId| {
        segment_metrics(
            plg,
            plg.cells.coord(from),
            plg.cells.coord(to),
            step_cells,
            wc,
        )
    };
    let mut out = vec![cells[0]];
    let mut anchor = 0;
    while anchor + 1 < cells.len() {
        let mut best = anchor + 1;
        let mut rough_pen = 1.0_f32;
        let mut rough_rise = 0.0_f32;
        let mut j = anchor + 1;
        while j < cells.len() {
            match metrics(cells[j - 1], cells[j]) {
                Some((pen, rise)) => {
                    rough_pen = rough_pen.max(pen);
                    rough_rise += rise;
                }
                // Infeasible step. Raise the baseline so the chord breaks.
                None => {
                    rough_pen = f32::INFINITY;
                    rough_rise = f32::INFINITY;
                }
            }
            match metrics(cells[anchor], cells[j]) {
                Some((pen, rise)) if pen <= rough_pen + 1e-3 && rise <= rough_rise + 1e-3 => {
                    best = j
                }
                _ => break,
            }
            j += 1;
        }
        out.push(cells[best]);
        anchor = best;
    }
    out
}

/// Worst wall penalty and total climb along the straight segment a -> b. None if
/// it leaves the surface, exceeds step_cells, or enters the hard clearance.
fn segment_metrics(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    wc: &WallCost,
) -> Option<(f32, f32)> {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        // A same-column vertical chord is not traversable.
        return (dz == 0).then_some((1.0, 0.0));
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
    let mut prev_iz: Option<i32> = None;
    let mut max_pen = 1.0_f32;
    let mut rise_cells = 0i32;
    for k in 0..=samples {
        let t = k as f32 / samples as f32;
        let ix = (a.0 as f32 + t * dx as f32).round() as i32;
        let iy = (a.1 as f32 + t * dy as f32).round() as i32;
        if ix == last_ix && iy == last_iy {
            continue;
        }
        last_ix = ix;
        last_iy = iy;
        let iz_line = a.2 as f32 + t * dz as f32;
        let zs = plg.surface_lookup.get(&(ix, iy))?;
        // Surface cell in this column nearest the interpolated segment height.
        let mut nearest: Option<(f32, i32)> = None;
        for &iz in zs {
            let d = (iz as f32 - iz_line).abs();
            if nearest.is_none_or(|(bd, _)| d < bd) {
                nearest = Some((d, iz));
            }
        }
        let (d, iz) = nearest?;
        if d > step_cells as f32 {
            return None;
        }
        // Tally climb and reject an untraversable step between columns.
        if let Some(p) = prev_iz {
            let step = (iz - p).abs();
            if step > step_cells {
                return None;
            }
            rise_cells += step;
        }
        prev_iz = Some(iz);
        // Columns on the surface but not in the graph carry no wall penalty.
        let p = match plg.cells.id((ix, iy, iz)) {
            Some(id) => {
                let wall_dist = plg
                    .wall_state
                    .dist
                    .get(id as usize)
                    .copied()
                    .unwrap_or(f32::INFINITY);
                penalty_of(wall_dist, wc.clearance_m, wc.buffer_m, wc.buffer_weight)
            }
            None => 1.0,
        };
        if !p.is_finite() {
            return None;
        }
        max_pen = max_pen.max(p);
    }
    Some((max_pen, rise(rise_cells, wc.voxel_size)))
}

fn edge_between(plg: &PlannerGraph, a: NodeId, b: NodeId) -> Option<NodeEdgeIdx> {
    for &edge_idx in plg.node_adj.get(&a)? {
        let edge = &plg.node_edges[edge_idx as usize];
        let other = if edge.a == a { edge.b } else { edge.a };
        if other == b {
            return Some(edge_idx);
        }
    }
    None
}

struct Scored(f32, NodeId);

impl PartialEq for Scored {
    fn eq(&self, other: &Self) -> bool {
        self.0.total_cmp(&other.0) == Ordering::Equal && self.1 == other.1
    }
}
impl Eq for Scored {}
impl PartialOrd for Scored {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Scored {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0).then(self.1.cmp(&other.1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::edges::build_node_edges;
    use crate::nodes::NodeData;

    const VOXEL: f32 = 0.1;
    const Z_TOL: f32 = 1.5;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(surface_cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg.nodes = node_cells
            .iter()
            .map(|&c| {
                let id = plg.cells.id(c).expect("node cell must be in surface");
                NodeData {
                    cell_id: id,
                    pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
                }
            })
            .collect();
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );
        plg
    }

    fn strip(n: i32) -> Vec<VoxelKey> {
        (0..n).map(|x| (x, 0, 0)).collect()
    }

    fn plan_simple(
        plg: &PlannerGraph,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
    ) -> Option<Vec<(f32, f32, f32)>> {
        let config = Config {
            world_frame: "world".into(),
            voxel_size: VOXEL,
            robot_height: Z_TOL,
            surface_closing_radius: 0.0,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.2,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 4.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 4.0,
            goal_tolerance: 0.3,
            viz_publish_hz: 2.0,
        };
        plan(plg, start, goal, &config).map(|(wp, _)| wp)
    }

    fn surface_graph(cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        plg
    }

    fn truncate_config() -> Config {
        Config {
            world_frame: "world".into(),
            voxel_size: VOXEL,
            robot_height: Z_TOL,
            surface_closing_radius: 0.0,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.2,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 4.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 4.0,
            goal_tolerance: 0.3,
            viz_publish_hz: 2.0,
        }
    }

    #[test]
    fn truncate_keeps_the_full_clear_route_ahead() {
        let cfg = truncate_config();
        // Cached route 0 -> 39 (cells), still fully traversable, robot at start.
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let start = surface_point_xyz(0, 0, 0, VOXEL);

        // No blockage, so no standoff: start pose plus every cell ahead, where
        // the robot's own cell 0 is dropped.
        let full = truncate_to_safe(&surface_graph(&cached), &cached, start, &cfg);
        assert_eq!(full.len(), cached.len(), "start pose + cells 1..=39");
        assert_eq!(full[1], surface_point_xyz(1, 0, 0, VOXEL));
        assert_eq!(*full.last().unwrap(), surface_point_xyz(39, 0, 0, VOXEL));
    }

    #[test]
    fn truncate_holds_standoff_ahead_of_advanced_robot() {
        let cfg = truncate_config();
        // Robot has advanced to x=20 along a 0 -> 39 route, and the surface is
        // now gone at x=35 (a door closed ahead).
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(20, 0, 0, VOXEL);
        let blocked: Vec<VoxelKey> = (0..40).filter(|&x| x != 35).map(|x| (x, 0, 0)).collect();

        let wp = truncate_to_safe(&surface_graph(&blocked), &cached, robot, &cfg);
        assert_eq!(wp[0], robot);

        // Never behind the robot, always forward toward the goal.
        let xs: Vec<f32> = wp.iter().map(|w| w.0).collect();
        assert!(
            xs.iter().all(|&x| x >= robot.0 - 1e-4),
            "backtracked: {xs:?}"
        );
        assert!(xs.windows(2).all(|p| p[1] >= p[0]), "not forward: {xs:?}");

        // Stops a standoff short of the last traversable cell (x=34).
        let last_safe = surface_point_xyz(34, 0, 0, VOXEL);
        let last = *wp.last().unwrap();
        let gap = (last_safe.0 - last.0).hypot(last_safe.1 - last.1);
        assert!(
            (gap - BEST_EFFORT_DISTANCE_M).abs() < VOXEL,
            "standoff is {gap} m, expected ~{BEST_EFFORT_DISTANCE_M}"
        );
    }

    #[test]
    fn truncate_walks_into_a_sparse_chord_to_the_blockage() {
        let cfg = truncate_config();
        // Sparse cached route: one 0 -> 39 chord, as string_pull leaves a
        // straight approach. The surface is gone at x=20, mid-chord, where the
        // old anchor-level cut would have discarded the whole chord and stopped
        // back at x=0.
        let cached: Vec<VoxelKey> = vec![(0, 0, 0), (39, 0, 0)];
        let surface: Vec<VoxelKey> = (0..40).filter(|&x| x != 20).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(0, 0, 0, VOXEL);

        let wp = truncate_to_safe(&surface_graph(&surface), &cached, robot, &cfg);

        // It advances well into the chord and stops a standoff short of the gap.
        let last = *wp.last().unwrap();
        assert!(last.0 > 0.5, "did not walk into the chord: {last:?}");
        let last_safe = surface_point_xyz(19, 0, 0, VOXEL);
        let gap = (last_safe.0 - last.0).hypot(last_safe.1 - last.1);
        assert!(
            (gap - BEST_EFFORT_DISTANCE_M).abs() < VOXEL,
            "standoff is {gap} m from the last safe cell, expected ~{BEST_EFFORT_DISTANCE_M}"
        );
    }

    #[test]
    fn truncate_stops_inside_standoff_or_at_blockage() {
        let cfg = truncate_config();
        let cached: Vec<VoxelKey> = (0..40).map(|x| (x, 0, 0)).collect();
        let robot = surface_point_xyz(0, 0, 0, VOXEL);

        // Blockage at the next step: nothing safe ahead, stop.
        let at_robot: Vec<VoxelKey> = (0..40).filter(|&x| x != 1).map(|x| (x, 0, 0)).collect();
        assert!(
            truncate_to_safe(&surface_graph(&at_robot), &cached, robot, &cfg).is_empty(),
            "blockage at the next step -> stop"
        );

        // Blockage only ~0.4 m ahead, inside the standoff: the best-effort point
        // is behind the robot, so stop.
        let near: Vec<VoxelKey> = (0..40).filter(|&x| x != 5).map(|x| (x, 0, 0)).collect();
        assert!(
            truncate_to_safe(&surface_graph(&near), &cached, robot, &cfg).is_empty(),
            "inside the standoff -> stop"
        );
    }

    #[test]
    fn back_off_tail_trims_from_the_goal_end() {
        let path = vec![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
        ];
        // Trim within the last segment.
        assert_eq!(*back_off_tail(&path, 0.5).last().unwrap(), (2.5, 0.0, 0.0));
        // Trim exactly to a vertex without leaving a duplicate point.
        let to_vertex = back_off_tail(&path, 1.0);
        assert_eq!(*to_vertex.last().unwrap(), (2.0, 0.0, 0.0));
        assert_eq!(to_vertex.len(), 3);
        // Trimming more than the path length stops (empty).
        assert!(back_off_tail(&path, 5.0).is_empty());
    }

    #[test]
    fn snap_picks_in_column_cell() {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&strip(20), &mut lookup);
        let cell = snap_pose_to_cell(&lookup, (0.5, 0.0, 0.1), VOXEL, Z_TOL).unwrap();
        assert_eq!(cell, (5, 0, 0));
    }

    #[test]
    fn snap_falls_back_to_nearby_column() {
        let mut cells = strip(20);
        cells.retain(|c| c.0 != 2);
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&cells, &mut lookup);
        let cell = snap_pose_to_cell(&lookup, (0.25, 0.0, 0.1), VOXEL, Z_TOL).unwrap();
        assert!(cell == (1, 0, 0) || cell == (3, 0, 0));
    }

    #[test]
    fn snap_rejects_outside_z_tolerance() {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&strip(20), &mut lookup);
        assert!(snap_pose_to_cell(&lookup, (0.5, 0.0, 2.0), VOXEL, 1.5).is_none());
    }

    #[test]
    fn plan_returns_none_if_start_cant_snap() {
        let plg = graph_with_nodes(&strip(20), &[(10, 0, 0)]);
        let result = plan_simple(&plg, (0.5, 0.0, 10.0), (1.0, 0.0, 0.1));
        assert!(result.is_none());
    }

    #[test]
    fn plan_returns_none_if_disconnected() {
        // The gap must exceed SNAP_SEARCH_RADIUS_M so no start candidate
        // can relocate onto the goal island.
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((30..35).map(|x| (x, 0, 0)));
        let plg = graph_with_nodes(&cells, &[(2, 0, 0), (32, 0, 0)]);
        let result = plan_simple(&plg, (0.25, 0.0, 0.1), (3.25, 0.0, 0.1));
        assert!(result.is_none());
    }

    #[test]
    fn plan_same_start_and_goal_passes_through_snap_cell() {
        let plg = graph_with_nodes(&strip(20), &[(10, 0, 0)]);
        let wp = plan_simple(&plg, (1.0, 0.0, 0.05), (1.0, 0.0, 0.05)).unwrap();
        assert_eq!(wp.first(), Some(&(1.0, 0.0, 0.05)));
        assert_eq!(wp.last(), Some(&(1.0, 0.0, 0.05)));
        let snap = surface_point_xyz(10, 0, 0, VOXEL);
        assert!(wp.contains(&snap));
    }

    #[test]
    fn plan_traces_surface_from_pose_to_first_node() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (1.7, 0.0, 0.05)).unwrap();
        // First waypoint is the robot's own snapped cell, not a jump ahead.
        let start_cell_pos = surface_point_xyz(2, 0, 0, VOXEL);
        let goal_cell_pos = surface_point_xyz(17, 0, 0, VOXEL);
        assert_eq!(wp[1], start_cell_pos);
        assert_eq!(wp[wp.len() - 2], goal_cell_pos);
    }

    #[test]
    fn plan_lead_in_does_not_backtrack_to_region_node() {
        // Robot at cell 5 is in node 3's region but sits between it and node 15.
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.55, 0.0, 0.05), (1.7, 0.0, 0.05)).unwrap();
        let xs: Vec<i32> = wp[1..wp.len() - 1]
            .iter()
            .map(|w| (w.0 / VOXEL).floor() as i32)
            .collect();
        assert_eq!(xs.first(), Some(&5));
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "lead-in walked backward: {xs:?}"
        );
    }

    fn waypoint_key(w: &(f32, f32, f32)) -> VoxelKey {
        (
            (w.0 / VOXEL).floor() as i32,
            (w.1 / VOXEL).floor() as i32,
            (w.2 / VOXEL).round() as i32 - 1,
        )
    }

    #[test]
    fn plan_path_segments_stay_on_the_surface() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (1.9, 0.0, 0.05)).unwrap();
        // Smoothed waypoints are no longer cell-adjacent, but each segment
        // between them must still stay on the surface.
        let step_cells = (0.25f32 / VOXEL).floor() as i32;
        for w in &wp[1..wp.len() - 1] {
            assert!(
                plg.cells.id(waypoint_key(w)).is_some(),
                "waypoint {w:?} is off the surface"
            );
        }
        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        for pair in wp[1..wp.len() - 1].windows(2) {
            assert!(
                segment_metrics(
                    &plg,
                    waypoint_key(&pair[0]),
                    waypoint_key(&pair[1]),
                    step_cells,
                    &wc
                )
                .is_some(),
                "segment {:?} -> {:?} leaves the surface",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn string_pull_straightens_open_area() {
        // Filled rectangle: every straight segment is on-surface, so the diagonal
        // path collapses instead of staircasing through the nodes.
        let mut cells: Vec<VoxelKey> = Vec::new();
        for x in 0..10 {
            for y in 0..6 {
                cells.push((x, y, 0));
            }
        }
        let plg = graph_with_nodes(&cells, &[(2, 2, 0), (7, 3, 0)]);
        let wp = plan_simple(&plg, (0.05, 0.05, 0.05), (0.85, 0.55, 0.05)).unwrap();
        let interior = wp.len() - 2;
        assert!(
            interior <= 4,
            "path not straightened: {interior} interior points"
        );
    }

    #[test]
    fn segment_metrics_rejects_vertical_chord() {
        let plg = PlannerGraph::new();
        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.3,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        assert!(segment_metrics(&plg, (5, 5, 0), (5, 5, 4), 2, &wc).is_none());
        assert_eq!(
            segment_metrics(&plg, (5, 5, 0), (5, 5, 0), 2, &wc),
            Some((1.0, 0.0))
        );
    }

    #[test]
    fn string_pull_refuses_shortcut_through_sub_clearance_cell() {
        // Straight strip: with open clearance the run collapses to its
        // endpoints. Drop one mid cell below the hard clearance and the shortcut
        // spanning it is refused, so the smoothed path retains that cell.
        let mut plg = PlannerGraph::new();
        build_surface_lookup(&strip(10), &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let path: Vec<CellId> = (0..10).map(|x| plg.cells.id((x, 0, 0)).unwrap()).collect();

        let wc = WallCost {
            clearance_m: 0.2,
            buffer_m: 0.5,
            buffer_weight: 4.0,
            voxel_size: VOXEL,
        };
        plg.wall_state.dist = vec![f32::INFINITY; plg.cells.slot_capacity()];
        let open = string_pull(&plg, &path, 1, &wc);
        assert_eq!(open.len(), 2, "open strip should collapse to its endpoints");

        let mid = plg.cells.id((5, 0, 0)).unwrap();
        plg.wall_state.dist[mid as usize] = 0.1; // below the 0.2 clearance
        let guarded = string_pull(&plg, &path, 1, &wc);
        assert!(
            guarded.len() > 2,
            "shortcut across a sub-clearance cell must be refused: {guarded:?}"
        );
        assert!(
            guarded.contains(&mid),
            "smoothed path must still traverse the low-clearance cell"
        );
    }

    #[test]
    fn plan_enters_on_goalward_node_not_nearest() {
        // Robot sits past node 2 toward the goal. Entry must skip it for node 10.
        let plg = graph_with_nodes(&strip(20), &[(2, 0, 0), (10, 0, 0)]);
        let wp = plan_simple(&plg, (0.45, 0.0, 0.05), (1.25, 0.0, 0.05)).unwrap();
        let nearest = surface_point_xyz(2, 0, 0, VOXEL);
        assert!(
            !wp.iter().any(|w| (w.0 - nearest.0).abs() < 1e-5),
            "path doubled back to the nearest node: {wp:?}"
        );
        let xs: Vec<i32> = wp[1..wp.len() - 1]
            .iter()
            .map(|w| (w.0 / VOXEL).floor() as i32)
            .collect();
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "path stepped backward: {xs:?}"
        );
    }
}
