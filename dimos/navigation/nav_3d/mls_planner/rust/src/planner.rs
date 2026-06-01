// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use ahash::AHashMap;

use crate::adjacency::{CellId, SurfaceLookup};
use crate::dijkstra::walk_preds;
use crate::edges::{NodeEdgeIdx, NodeId, PlannerGraph, NO_NODE};
use crate::voxel::{surface_point_xyz, VoxelKey};

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

    const SEARCH_RADIUS: i32 = 5;
    let mut best: Option<(i32, VoxelKey)> = None;
    for dix in -SEARCH_RADIUS..=SEARCH_RADIUS {
        for diy in -SEARCH_RADIUS..=SEARCH_RADIUS {
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
pub fn plan(
    plg: &PlannerGraph,
    start_pose: (f32, f32, f32),
    goal_pose: (f32, f32, f32),
    voxel_size: f32,
    z_tolerance_m: f32,
) -> Option<Vec<(f32, f32, f32)>> {
    let start_coord =
        snap_pose_to_cell(&plg.surface_lookup, start_pose, voxel_size, z_tolerance_m)?;
    let goal_coord = snap_pose_to_cell(&plg.surface_lookup, goal_pose, voxel_size, z_tolerance_m)?;
    let start_cell = plg.cells.id(start_coord)?;
    let goal_cell = plg.cells.id(goal_coord)?;

    let node_idx_by_cell: AHashMap<CellId, NodeId> = plg
        .nodes
        .iter()
        .enumerate()
        .map(|(i, n)| (n.cell_id, i as NodeId))
        .collect();

    let start_segment = walk_preds(&plg.cell_state, start_cell);
    let goal_segment = walk_preds(&plg.cell_state, goal_cell);
    let start_node = *node_idx_by_cell.get(start_segment.last()?)?;
    let goal_node = *node_idx_by_cell.get(goal_segment.last()?)?;

    let node_seq = shortest_path_nodes(plg, start_node, goal_node)?;
    Some(assemble_waypoints(
        plg,
        &node_seq,
        start_pose,
        &start_segment,
        goal_pose,
        &goal_segment,
        voxel_size,
    ))
}

pub fn shortest_path_nodes(plg: &PlannerGraph, start: NodeId, goal: NodeId) -> Option<Vec<NodeId>> {
    if start == goal {
        return Some(vec![start]);
    }
    let n = plg.nodes.len();
    let mut dist = vec![f32::INFINITY; n];
    let mut pred = vec![NO_NODE; n];
    dist[start as usize] = 0.0;
    let mut heap: BinaryHeap<Scored> = BinaryHeap::new();
    heap.push(Scored(0.0, start));

    while let Some(Scored(d, u)) = heap.pop() {
        if d > dist[u as usize] {
            continue;
        }
        if u == goal {
            break;
        }
        for &edge_idx in &plg.node_adj[u as usize] {
            let edge = &plg.node_edges[edge_idx as usize];
            let neighbor = if edge.a == u { edge.b } else { edge.a };
            let nd = d + edge.cost;
            if nd < dist[neighbor as usize] {
                dist[neighbor as usize] = nd;
                pred[neighbor as usize] = u;
                heap.push(Scored(nd, neighbor));
            }
        }
    }

    if !dist[goal as usize].is_finite() {
        return None;
    }
    let mut path = vec![goal];
    let mut cur = goal;
    while pred[cur as usize] != NO_NODE {
        cur = pred[cur as usize];
        path.push(cur);
    }
    path.reverse();
    Some(path)
}

fn assemble_waypoints(
    plg: &PlannerGraph,
    node_seq: &[NodeId],
    start_pose: (f32, f32, f32),
    start_segment: &[CellId],
    goal_pose: (f32, f32, f32),
    goal_segment: &[CellId],
    voxel_size: f32,
) -> Vec<(f32, f32, f32)> {
    let mut cells: Vec<CellId> = Vec::new();
    cells.extend_from_slice(start_segment);

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
            if cells.last() != Some(&c) {
                cells.push(c);
            }
        }
    }

    for &c in goal_segment.iter().rev() {
        if cells.last() != Some(&c) {
            cells.push(c);
        }
    }

    let mut waypoints: Vec<(f32, f32, f32)> = Vec::with_capacity(cells.len() + 2);
    waypoints.push(start_pose);
    for id in cells {
        let (ix, iy, iz) = plg.cells.coord(id);
        waypoints.push(surface_point_xyz(ix, iy, iz, voxel_size));
    }
    waypoints.push(goal_pose);
    waypoints
}

fn edge_between(plg: &PlannerGraph, a: NodeId, b: NodeId) -> Option<NodeEdgeIdx> {
    for &edge_idx in &plg.node_adj[a as usize] {
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
        let result = plan(&plg, (0.5, 0.0, 10.0), (1.0, 0.0, 0.1), VOXEL, Z_TOL);
        assert!(result.is_none());
    }

    #[test]
    fn plan_returns_none_if_disconnected() {
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let plg = graph_with_nodes(&cells, &[(2, 0, 0), (12, 0, 0)]);
        let result = plan(&plg, (0.25, 0.0, 0.1), (1.25, 0.0, 0.1), VOXEL, Z_TOL);
        assert!(result.is_none());
    }

    #[test]
    fn plan_same_start_and_goal_passes_through_snap_cell() {
        let plg = graph_with_nodes(&strip(20), &[(10, 0, 0)]);
        let wp = plan(&plg, (1.0, 0.0, 0.05), (1.0, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        assert_eq!(wp.first(), Some(&(1.0, 0.0, 0.05)));
        assert_eq!(wp.last(), Some(&(1.0, 0.0, 0.05)));
        let snap = surface_point_xyz(10, 0, 0, VOXEL);
        assert!(wp.contains(&snap));
    }

    #[test]
    fn plan_traces_surface_from_pose_to_first_node() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (15, 0, 0)]);
        let wp = plan(&plg, (0.2, 0.0, 0.05), (1.7, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        let start_cell_pos = surface_point_xyz(2, 0, 0, VOXEL);
        let goal_cell_pos = surface_point_xyz(17, 0, 0, VOXEL);
        assert_eq!(wp[1], start_cell_pos);
        assert_eq!(wp[wp.len() - 2], goal_cell_pos);
    }

    #[test]
    fn plan_three_nodes_visits_them_all() {
        let plg = graph_with_nodes(&strip(20), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let wp = plan(&plg, (0.2, 0.0, 0.05), (1.9, 0.0, 0.05), VOXEL, Z_TOL).unwrap();
        let node_xy: Vec<(f32, f32)> = plg.nodes.iter().map(|n| (n.pos.0, n.pos.1)).collect();
        for &(nx, ny) in &node_xy {
            assert!(
                wp.iter()
                    .any(|w| (w.0 - nx).abs() < 1e-5 && (w.1 - ny).abs() < 1e-5),
                "node ({nx}, {ny}) should appear among waypoints"
            );
        }
    }
}
