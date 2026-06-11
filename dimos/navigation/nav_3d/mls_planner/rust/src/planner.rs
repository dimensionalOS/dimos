// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{CellId, SurfaceCells, SurfaceLookup};
use crate::dijkstra::walk_preds;
use crate::edges::{NodeEdgeIdx, NodeId, PlannerGraph, NO_NODE};
use crate::voxel::{surface_point_xyz, VoxelKey};

/// Robot-rooted candidate search radius, in multiples of node spacing.
const CANDIDATE_RADIUS_FACTOR: f32 = 3.0;

/// How far to search horizontally when snapping a pose to the surface.
/// A downward-pitched lidar cannot see the floor under the robot, so the
/// nearest mapped surface can start well outside the robot footprint.
const SNAP_SEARCH_RADIUS_M: f32 = 1.5;

/// Snap a pose to the best surface cell.
pub fn snap_pose_to_cell(
    surface_lookup: &SurfaceLookup,
    pose: (f32, f32, f32),
    voxel_size: f32,
    tolerance_m: f32,
) -> Option<VoxelKey> {
    let ix = (pose.0 / voxel_size).floor() as i32;
    let iy = (pose.1 / voxel_size).floor() as i32;
    let target_iz = (pose.2 / voxel_size).floor() as i32 - 1;
    let tol_cells = (tolerance_m / voxel_size).ceil() as i32;

    if let Some(cell) = best_iz_in_column(surface_lookup, ix, iy, target_iz, tol_cells) {
        return Some(cell);
    }

    let search_radius = (SNAP_SEARCH_RADIUS_M / voxel_size).ceil() as i32;
    let mut best: Option<(i32, VoxelKey)> = None;
    for dix in -search_radius..=search_radius {
        for diy in -search_radius..=search_radius {
            if dix == 0 && diy == 0 {
                continue;
            }
            let Some(cell) =
                best_iz_in_column(surface_lookup, ix + dix, iy + diy, target_iz, tol_cells)
            else {
                continue;
            };
            let d2 = dix * dix + diy * diy;
            if best.is_none_or(|(bd, _)| d2 < bd) {
                best = Some((d2, cell));
            }
        }
    }
    best.map(|(_, c)| c)
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
/// Returns none if either of the poses can't be snapped to surface or if
/// there is no valid path.
#[allow(clippy::too_many_arguments)]
pub fn plan(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
    z_tolerance_m: f32,
    node_spacing_m: f32,
    node_step_threshold_m: f32,
    node_wall_buffer_m: f32,
) -> Option<Vec<(f32, f32, f32)>> {
    let Some(start_coord) =
        snap_pose_to_cell(&plg.surface_lookup, start_pose, voxel_size, z_tolerance_m)
    else {
        tracing::warn!(
            ?start_pose,
            "plan failed: start does not snap to any surface cell"
        );
        return None;
    };
    let Some(goal_coord) =
        snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)
    else {
        tracing::warn!(
            ?goal_pose,
            "plan failed: goal does not snap to any surface cell"
        );
        return None;
    };
    let Some(start_cell) = plg.cells.id(start_coord) else {
        tracing::warn!(?start_coord, "plan failed: start cell is not in the graph");
        return None;
    };
    let Some(goal_cell) = plg.cells.id(goal_coord) else {
        tracing::warn!(?goal_coord, "plan failed: goal cell is not in the graph");
        return None;
    };

    let node_cells: AHashSet<NodeId> = plg.nodes.iter().map(|n| n.cell_id).collect();

    let goal_segment = walk_preds(&plg.cell_state, goal_cell);
    let Some(&goal_node) = goal_segment.last() else {
        tracing::warn!(?goal_coord, "plan failed: goal has no predecessor chain");
        return None;
    };
    if !node_cells.contains(&goal_node) {
        tracing::warn!(
            ?goal_coord,
            "plan failed: goal region does not reach a graph node"
        );
        return None;
    }

    // Rooted at the goal so one pass covers every node's cost-to-go.
    let (cost_to_go, pred_to_goal) = node_dijkstra(plg, goal_node);

    let radius = (node_spacing_m * CANDIDATE_RADIUS_FACTOR).max(voxel_size);
    let Some((lead_in, node_seq)) = select_entry(
        plg,
        start_cell,
        goal_node,
        &cost_to_go,
        &pred_to_goal,
        &node_cells,
        radius,
    ) else {
        tracing::warn!(
            ?start_coord,
            reachable_nodes = cost_to_go.len(),
            total_nodes = plg.nodes.len(),
            "plan failed: no entry node connects the start to the goal component",
        );
        return None;
    };

    // Shortcut height tolerance in cells, tied to the traversable step.
    let smooth_tol_cells = ((node_step_threshold_m / voxel_size).round() as i32).max(1);

    let cells = assemble_cells(plg, &node_seq, &lead_in, &goal_segment);
    let cells = string_pull(plg, &cells, smooth_tol_cells, node_wall_buffer_m);
    Some(cells_to_waypoints(
        plg, &cells, start_pose, goal_pose, voxel_size,
    ))
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
    for node in &plg.nodes {
        let Some(&connect) = connect_dist.get(&node.cell_id) else {
            continue;
        };
        let Some(&ctg) = cost_to_go.get(&node.cell_id) else {
            continue;
        };
        let score = connect + ctg;
        if score < best_score {
            best_score = score;
            entry_node = node.cell_id;
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

/// Bounded Dijkstra from the robot cell, visiting cells within the radius.
/// Returns per-cell distance and predecessor maps.
fn robot_search(
    cells: &SurfaceCells,
    source: CellId,
    radius_m: f32,
) -> (AHashMap<CellId, f32>, AHashMap<CellId, CellId>) {
    let mut dist: AHashMap<CellId, f32> = AHashMap::new();
    let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    dist.insert(source, 0.0);
    heap.push(Scored(0.0, source));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > radius_m {
            break;
        }
        if d > dist.get(&u).copied().unwrap_or(f32::INFINITY) {
            continue;
        }
        for edge in cells.neighbors(u) {
            let nd = d + edge.cost;
            if nd < dist.get(&edge.dest).copied().unwrap_or(f32::INFINITY) {
                dist.insert(edge.dest, nd);
                pred.insert(edge.dest, u);
                heap.push(Scored(nd, edge.dest));
            }
        }
    }
    (dist, pred)
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

/// Shortcut runs of cells with straight on-surface segments, keeping the
/// farthest cell in line of sight from each anchor. A shortcut is only taken
/// when it never passes closer than buffer_m to a wall, so smoothing cannot
/// erode the wall clearance the penalized routing built in.
fn string_pull(plg: &PlannerGraph, cells: &[CellId], tol_cells: i32, buffer_m: f32) -> Vec<CellId> {
    if cells.len() <= 2 {
        return cells.to_vec();
    }
    let mut out = vec![cells[0]];
    let mut anchor = 0;
    while anchor + 1 < cells.len() {
        let anchor_coord = plg.cells.coord(cells[anchor]);
        let mut last_ok = anchor + 1;
        let mut j = anchor + 1;
        while j < cells.len() {
            let coord = plg.cells.coord(cells[j]);
            if !los_on_surface(plg, anchor_coord, coord, tol_cells, buffer_m) {
                break;
            }
            last_ok = j;
            j += 1;
        }
        out.push(cells[last_ok]);
        anchor = last_ok;
    }
    out
}

/// True if every column the segment crosses holds a surface cell within
/// tol_cells of the interpolated segment height, and that cell stays at least
/// buffer_m from the nearest wall. Cells without a wall-distance value are
/// treated as open, so an unpopulated field reduces to a pure on-surface test.
fn los_on_surface(
    plg: &PlannerGraph,
    a: VoxelKey,
    b: VoxelKey,
    tol_cells: i32,
    buffer_m: f32,
) -> bool {
    let (dx, dy, dz) = (b.0 - a.0, b.1 - a.1, b.2 - a.2);
    let samples = dx.abs().max(dy.abs()) * 2;
    if samples == 0 {
        return true;
    }
    let (mut last_ix, mut last_iy) = (i32::MIN, i32::MIN);
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
            return false;
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
            return false;
        };
        if d > tol_cells as f32 {
            return false;
        }
        if let Some(id) = plg.cells.id((ix, iy, iz)) {
            let wall_dist = plg
                .wall_state
                .dist
                .get(id as usize)
                .copied()
                .unwrap_or(f32::INFINITY);
            if wall_dist < buffer_m {
                return false;
            }
        }
    }
    true
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
        plan(plg, start, goal, VOXEL, Z_TOL, 1.0, 0.25, 0.3)
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
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let plg = graph_with_nodes(&cells, &[(2, 0, 0), (12, 0, 0)]);
        let result = plan_simple(&plg, (0.25, 0.0, 0.1), (1.25, 0.0, 0.1));
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
        let tol = ((0.25f32 / VOXEL).round() as i32).max(1);
        for w in &wp[1..wp.len() - 1] {
            assert!(
                plg.cells.id(waypoint_key(w)).is_some(),
                "waypoint {w:?} is off the surface"
            );
        }
        for pair in wp[1..wp.len() - 1].windows(2) {
            assert!(
                los_on_surface(
                    &plg,
                    waypoint_key(&pair[0]),
                    waypoint_key(&pair[1]),
                    tol,
                    0.3
                ),
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
    fn string_pull_refuses_shortcut_through_sub_buffer_cell() {
        // Straight strip: with open clearance the run collapses to its
        // endpoints. Drop one mid cell below the buffer and the shortcut
        // spanning it is refused, so the smoothed path retains that cell.
        let mut plg = PlannerGraph::new();
        build_surface_lookup(&strip(10), &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);
        let path: Vec<CellId> = (0..10).map(|x| plg.cells.id((x, 0, 0)).unwrap()).collect();

        plg.wall_state.dist = vec![f32::INFINITY; plg.cells.slot_capacity()];
        let open = string_pull(&plg, &path, 1, 0.3);
        assert_eq!(open.len(), 2, "open strip should collapse to its endpoints");

        let mid = plg.cells.id((5, 0, 0)).unwrap();
        plg.wall_state.dist[mid as usize] = 0.1;
        let guarded = string_pull(&plg, &path, 1, 0.3);
        assert!(
            guarded.len() > 2,
            "shortcut across a sub-buffer cell must be refused: {guarded:?}"
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
