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

use std::collections::{BinaryHeap, VecDeque};

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{rise, CellId, SurfaceCells, SurfaceLookup};
use crate::dijkstra::{walk_preds, Scored};
use crate::edges::PlannerGraph;
use crate::mls_planner::Config;
use crate::nodes::penalty_of;
use crate::nodes::{NodeId, NO_NODE};
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Robot-rooted candidate search radius, in multiples of node spacing.
const CANDIDATE_RADIUS_FACTOR: f32 = 3.0;

/// Horizontal search radius when snapping a pose to the surface.
const SNAP_SEARCH_RADIUS_M: f32 = 1.5;

/// Max snap candidates tried when connecting the start.
const MAX_SNAP_ATTEMPTS: usize = 64;

/// World-frame waypoints paired with the string-pulled cell path that produced
/// them. The cell path is cached so a warp can re-key along it.
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
        tracing::debug!(
            ?start_pose,
            "plan failed: start does not snap to any surface cell"
        );
        return None;
    }
    let Some(goal_coord) =
        snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)
    else {
        tracing::debug!(
            ?goal_pose,
            "plan failed: goal does not snap to any surface cell"
        );
        return None;
    };
    let Some(goal_cell) = plg.cells.id(goal_coord) else {
        tracing::debug!(?goal_coord, "plan failed: goal cell is not in the graph");
        return None;
    };
    let start_cells: Vec<CellId> = start_candidates
        .iter()
        .take(MAX_SNAP_ATTEMPTS)
        .filter_map(|&candidate| plg.cells.id(candidate))
        .collect();

    let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();

    // The penalized Voronoi cannot own sub-clearance goals, so fall back to the
    // nearest node by hops. A real failure then reports as disconnected below.
    let mut goal_segment = walk_live_preds(plg, goal_cell);
    let mut goal_node = *goal_segment
        .last()
        .expect("the goal cell is live, so the walk keeps at least it");
    if !node_cells.contains(&goal_node) {
        let Some((node, path)) = nearest_node(&plg.cells, goal_cell, &node_cells) else {
            tracing::debug!(
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
    // Nearest candidate first so the lead-in anchors at the robot's own cell.
    // The rest are only a fallback when it cannot connect.
    let (near, rest) = start_cells.split_at(start_cells.len().min(1));
    let entry = [near, rest].into_iter().find_map(|group| {
        select_entry(
            plg,
            group,
            goal_cell,
            goal_node,
            &cost_to_go,
            &pred_to_goal,
            &node_cells,
            radius,
        )
    });
    let Some((lead_in, node_seq)) = entry else {
        tracing::debug!(
            candidates = start_cells.len(),
            reachable_nodes = cost_to_go.len(),
            total_nodes = plg.nodes.len(),
            "plan failed: start and goal lie on separate connected surface components",
        );
        return None;
    };

    // Max traversable step in cells, the hard bound shared with the graph.
    let step_cells = config.step_cells();

    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };
    // A direct goal connection carries the whole route in its lead-in.
    let goal_segment: &[CellId] = if node_seq.is_empty() {
        &[]
    } else {
        &goal_segment
    };
    let cells = assemble_cells(plg, &node_seq, &lead_in, goal_segment);
    let cells = string_pull(plg, &cells, step_cells, &wall_cost);
    let waypoints = cells_to_waypoints(plg, &cells, start_pose, goal_pose, voxel_size);
    let waypoints = crate::smoother::smooth_path(
        plg,
        waypoints,
        step_cells,
        &wall_cost,
        config.step_penalty_weight,
    );
    let path_cells: Vec<VoxelKey> = cells.iter().map(|&id| plg.cells.coord(id)).collect();
    Some((waypoints, path_cells))
}

/// Whether each leg of `waypoints` walks over traversable surface (the chord walk
/// at surface resolution), each waypoint snapped to its surface cell.
pub fn legs_traversable(
    plg: &PlannerGraph,
    waypoints: &[(f32, f32, f32)],
    config: &Config,
) -> Vec<bool> {
    let voxel_size = config.voxel_size;
    let wall_cost = WallCost {
        clearance_m: config.wall_clearance_m,
        buffer_m: config.wall_buffer_m,
        buffer_weight: config.wall_buffer_weight,
        voxel_size,
    };
    let keys: Vec<Option<VoxelKey>> = waypoints
        .iter()
        .map(|&w| snap_pose_to_cell(&plg.surface_lookup, w, voxel_size, config.robot_height))
        .collect();
    keys.windows(2)
        .map(|pair| match (pair[0], pair[1]) {
            (Some(a), Some(b)) => walkable(plg, a, b, config.step_cells(), &wall_cost, voxel_size),
            _ => false,
        })
        .collect()
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

/// Whether a -> b is walkable: a step within one column (two surface cells a
/// step apart, as smoothed waypoints can land), else the chord walk reaches b.
fn walkable(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    step_cells: i32,
    wc: &WallCost,
    voxel_size: f32,
) -> bool {
    if (a.0, a.1) == (b.0, b.1) {
        return (a.2 - b.2).abs() <= step_cells;
    }
    !last_safe_on_chord(plg, a, b, step_cells, wc, voxel_size).1
}

/// Pick the entry node by connect cost plus cost-to-go, with its on-surface
/// lead-in and the node sequence to the goal. A goal cell inside the search
/// radius connects directly instead, signalled by an empty node sequence.
#[allow(clippy::too_many_arguments)]
fn select_entry(
    plg: &PlannerGraph,
    start_cells: &[CellId],
    goal_cell: CellId,
    goal_node: NodeId,
    cost_to_go: &AHashMap<NodeId, f32>,
    pred_to_goal: &AHashMap<NodeId, NodeId>,
    node_cells: &AHashSet<NodeId>,
    radius_m: f32,
) -> Option<(Vec<CellId>, Vec<NodeId>)> {
    let (connect_dist, connect_pred) = robot_search(&plg.cells, start_cells, radius_m);

    if connect_dist.contains_key(&goal_cell) {
        let mut lead = walk_local_preds(&connect_pred, goal_cell);
        lead.reverse();
        return Some((lead, Vec::new()));
    }

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

    for &start_cell in start_cells {
        let start_segment = walk_live_preds(plg, start_cell);
        let region_node = *start_segment
            .last()
            .expect("the start cell is live, so the walk keeps at least it");
        if !node_cells.contains(&region_node)
            || !cost_to_go.get(&region_node).is_some_and(|c| c.is_finite())
        {
            continue;
        }
        let Some(node_seq) = follow_preds(region_node, goal_node, pred_to_goal) else {
            continue;
        };
        return Some((start_segment, node_seq));
    }
    None
}

/// Bounded Dijkstra from the robot cell. Cost is wall-penalized for steering,
/// but the radius bounds metric distance, not penalized cost.
fn robot_search(
    cells: &SurfaceCells,
    sources: &[CellId],
    radius_m: f32,
) -> (AHashMap<CellId, f32>, AHashMap<CellId, CellId>) {
    let mut dist: AHashMap<CellId, f32> = AHashMap::new();
    let mut geo: AHashMap<CellId, f32> = AHashMap::new();
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut heap: BinaryHeap<Scored<CellId>> = BinaryHeap::new();
    for &source in sources {
        dist.insert(source, 0.0);
        geo.insert(source, 0.0);
        heap.push(Scored(0.0, source));
    }

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

/// walk_preds through the persistent Voronoi state, truncated at the first
/// cell that died since the state was written. Regional updates leave
/// out-of-window pred chains stale, and a freed slot's coord is garbage.
fn walk_live_preds(plg: &PlannerGraph, from: CellId) -> Vec<CellId> {
    let mut chain = walk_preds(&plg.cell_state, from);
    if let Some(cut) = chain.iter().position(|&c| !plg.cells.is_live(c)) {
        chain.truncate(cut);
    }
    chain
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
    let mut heap: BinaryHeap<Scored<NodeId>> = BinaryHeap::new();
    heap.push(Scored(0.0, source));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        for &edge_idx in plg.node_edges.adjacent(u) {
            let edge = &plg.node_edges.edges()[edge_idx as usize];
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
        let edge_idx = plg
            .node_edges
            .between(a, b)
            .expect("consecutive nodes in path must share an edge");
        let edge = &plg.node_edges.edges()[edge_idx as usize];

        // The stored corridor runs a to b. Dead cells are skipped: the update
        // safety walk drops broken corridors, so gaps here are rare races.
        let resolve = |coord: &VoxelKey| plg.cells.id(*coord);
        if a == edge.a {
            for c in edge.chain.iter().filter_map(resolve) {
                push_cell(&mut cells, c);
            }
        } else {
            for c in edge.chain.iter().rev().filter_map(resolve) {
                push_cell(&mut cells, c);
            }
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
pub(crate) struct WallCost {
    pub(crate) clearance_m: f32,
    pub(crate) buffer_m: f32,
    pub(crate) buffer_weight: f32,
    pub(crate) voxel_size: f32,
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::edges::build_node_edges;
    use crate::nodes::{NodeData, Nodes};

    const VOXEL: f32 = 0.1;
    const Z_TOL: f32 = 1.5;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(surface_cells, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let list = node_cells
            .iter()
            .map(|&c| {
                let id = plg.cells.id(c).expect("node cell must be in surface");
                NodeData {
                    cell_id: id,
                    pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
                }
            })
            .collect();
        let params = truncate_config().placement_params();
        plg.nodes = Nodes::from_list(&plg.cells, list, &params);
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
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
            base_frame: "base_link".into(),
            voxel_size: VOXEL,
            robot_height: Z_TOL,
            start_z_offset_m: 0.0,
            max_overhead_m: 2.0,
            surface_closing_radius: 0.0,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.2,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 4.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 4.0,
            goal_tolerance: 0.3,
            viz_publish_hz: 2.0,
            viz_cell_m: 4.0,
            viz_sweep_cells: 0,
            worker_threads: 4,
        };
        plan(plg, start, goal, &config).map(|(wp, _)| wp)
    }

    fn truncate_config() -> Config {
        Config {
            world_frame: "world".into(),
            base_frame: "base_link".into(),
            voxel_size: VOXEL,
            robot_height: Z_TOL,
            start_z_offset_m: 0.0,
            max_overhead_m: 2.0,
            surface_closing_radius: 0.0,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.2,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 4.0,
            step_threshold_m: 0.25,
            step_penalty_weight: 4.0,
            goal_tolerance: 0.3,
            viz_publish_hz: 2.0,
            viz_cell_m: 4.0,
            viz_sweep_cells: 0,
            worker_threads: 4,
        }
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
    fn plan_traces_surface_from_pose_to_first_node() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan_simple(&plg, (0.2, 0.0, 0.05), (1.7, 0.0, 0.05)).unwrap();
        // The path leaves from the robot's own cell, not a jump ahead, and
        // walks x monotonically to the goal.
        assert!(
            (wp[1].0 - 0.2).abs() < 2.0 * VOXEL,
            "jumped ahead: {:?}",
            wp[1]
        );
        assert!(
            wp.windows(2).all(|p| p[1].0 >= p[0].0 - 1e-4),
            "walked backward"
        );
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
        assert!(
            *xs.first().unwrap() >= 5,
            "backtracked to the region node: {xs:?}"
        );
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
        let length: f32 = wp
            .windows(2)
            .map(|w| ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt())
            .sum();
        let direct = (0.8f32.powi(2) + 0.5f32.powi(2)).sqrt();
        assert!(
            length <= direct * 1.2,
            "path not straightened: length {length} vs direct {direct}"
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
    fn select_entry_connects_straight_to_in_radius_goal() {
        // The only node is behind the robot and the goal is just ahead.
        // Entry must connect straight to the goal, not dogleg via the node.
        let plg = graph_with_nodes(&strip(20), &[(2, 0, 0)]);
        let start = plg.cells.id((10, 0, 0)).unwrap();
        let goal = plg.cells.id((15, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node);

        let (lead, node_seq) = select_entry(
            &plg,
            &[start],
            goal,
            goal_node,
            &ctg,
            &pred,
            &node_cells,
            3.0,
        )
        .unwrap();

        assert!(
            node_seq.is_empty(),
            "endgame routed via nodes: {node_seq:?}"
        );
        assert_eq!(lead.first(), Some(&start));
        assert_eq!(lead.last(), Some(&goal));
        let xs: Vec<i32> = lead.iter().map(|&c| plg.cells.coord(c).0).collect();
        assert!(
            xs.windows(2).all(|p| p[1] >= p[0]),
            "lead-in walked backward: {xs:?}"
        );
    }

    #[test]
    fn select_entry_anchors_lead_at_the_nearest_candidate() {
        // The nearest candidate connects, so the lead-in starts at the
        // robot's own cell even though other candidates exist.
        let plg = graph_with_nodes(&strip(20), &[(12, 0, 0)]);
        let near = plg.cells.id((10, 0, 0)).unwrap();
        let goal = plg.cells.id((19, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node);

        let (lead, node_seq) = select_entry(
            &plg,
            &[near],
            goal,
            goal_node,
            &ctg,
            &pred,
            &node_cells,
            0.3,
        )
        .unwrap();
        assert_eq!(
            lead.first(),
            Some(&near),
            "lead must anchor at the robot's cell"
        );
        assert_eq!(lead.last(), Some(&goal_node));
        assert_eq!(node_seq, vec![goal_node]);
    }

    #[test]
    fn select_entry_falls_back_to_merged_rest_candidates() {
        // The nearest candidate sits on a disconnected fragment. The fallback
        // runs one merged wavefront over the rest, and only the candidate
        // that actually reaches the node roots the lead-in.
        let mut surface = strip(20);
        surface.extend((0..3).map(|x| (x, 10, 0)));
        let plg = graph_with_nodes(&surface, &[(12, 0, 0)]);
        let near = plg.cells.id((1, 10, 0)).unwrap();
        let far_rest = plg.cells.id((5, 0, 0)).unwrap();
        let reaching_rest = plg.cells.id((14, 0, 0)).unwrap();
        let goal = plg.cells.id((19, 0, 0)).unwrap();
        let goal_node = plg.nodes[0].cell_id;
        let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();
        let (ctg, pred) = node_dijkstra(&plg, goal_node);

        let entry = select_entry(
            &plg,
            &[near],
            goal,
            goal_node,
            &ctg,
            &pred,
            &node_cells,
            0.3,
        );
        assert!(entry.is_none(), "disconnected candidate must not connect");

        let (lead, node_seq) = select_entry(
            &plg,
            &[far_rest, reaching_rest],
            goal,
            goal_node,
            &ctg,
            &pred,
            &node_cells,
            0.3,
        )
        .unwrap();
        assert_eq!(
            lead.first(),
            Some(&reaching_rest),
            "lead must root at the candidate that reaches the node"
        );
        assert_eq!(lead.last(), Some(&goal_node));
        assert_eq!(node_seq, vec![goal_node]);
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
}
