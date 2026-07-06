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

//! Node-graph edge construction.
//!
//! Multi-source Dijkstra from the start nodes labels each cell with its closest
//! source, partitioning the surface into Voronoi regions. Edges between nodes
//! come from the boundaries between those regions.

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

use crate::adjacency::{CellId, SurfaceCells, SurfaceLookup, NO_CELL};
use crate::dijkstra::{dijkstra, dijkstra_region, walk_preds, DijkstraState, Weight};
use crate::nodes::NodeData;
use crate::voxel::VoxelKey;

/// A node is identified by the CellId it sits on. Stable across incremental
/// updates so cached edges and the Voronoi forest survive a regional rebuild.
pub type NodeId = CellId;
pub const NO_NODE: NodeId = NO_CELL;

/// Index into the planner graph node edges.
pub type NodeEdgeIdx = u32;

#[derive(Clone, Copy, Debug)]
pub struct NodeEdge {
    pub a: NodeId,
    pub b: NodeId,
    pub cost: f32,
    /// Cell on a's side of the cheapest Voronoi boundary crossing.
    pub boundary_u: CellId,
    /// Cell on b's side.
    pub boundary_v: CellId,
}

#[derive(Default)]
pub struct PlannerGraph {
    pub cells: SurfaceCells,
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
    pub node_edges: Vec<NodeEdge>,
    pub node_adj: AHashMap<NodeId, Vec<NodeEdgeIdx>>,
    /// Each cell's nearest node and the predecessor back toward it. The planner
    /// walks these to expand a node-to-node edge into its cell path.
    pub cell_state: DijkstraState,
    /// Each cell's distance to the nearest wall.
    pub wall_state: DijkstraState,
}

impl PlannerGraph {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Assemble the cheapest edges between neighboring source nodes from their
/// Voronoi region boundaries.
pub fn build_node_edges(
    cells: &SurfaceCells,
    nodes: &[NodeData],
    state: &mut DijkstraState,
    out_edges: &mut Vec<NodeEdge>,
    out_adj: &mut AHashMap<NodeId, Vec<NodeEdgeIdx>>,
) {
    out_edges.clear();
    out_adj.clear();

    if nodes.is_empty() {
        state.reset(cells.slot_capacity());
        return;
    }

    let source_cells: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();
    dijkstra(cells, &source_cells, state, Weight::Penalized);

    best_boundary_edges(cells, state, out_edges);

    rebuild_node_adj(out_edges, out_adj);
}

/// Rebuild the per-node edge index from the edge list.
fn rebuild_node_adj(edges: &[NodeEdge], out_adj: &mut AHashMap<NodeId, Vec<NodeEdgeIdx>>) {
    out_adj.clear();
    for (edge_idx, edge) in edges.iter().enumerate() {
        out_adj
            .entry(edge.a)
            .or_default()
            .push(edge_idx as NodeEdgeIdx);
        out_adj
            .entry(edge.b)
            .or_default()
            .push(edge_idx as NodeEdgeIdx);
    }
}

/// Incremental build_node_edges. Redo the Voronoi inside the window, keep cached
/// edges whose boundary lies outside it, and rescan the window for new
/// node-to-node crossings.
pub fn build_node_edges_region(
    cells: &SurfaceCells,
    nodes: &[NodeData],
    window: &AHashSet<CellId>,
    state: &mut DijkstraState,
    out_edges: &mut Vec<NodeEdge>,
    out_adj: &mut AHashMap<NodeId, Vec<NodeEdgeIdx>>,
) {
    let source_cells: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();
    if source_cells.is_empty() {
        state.reset(cells.slot_capacity());
        out_edges.clear();
        out_adj.clear();
        return;
    }
    dijkstra_region(cells, &source_cells, window, state, Weight::Penalized);

    let live_node: AHashSet<NodeId> = source_cells.iter().copied().collect();

    let mut merged: AHashMap<(NodeId, NodeId), NodeEdge> = AHashMap::new();
    for e in out_edges.iter() {
        if window.contains(&e.boundary_u) || window.contains(&e.boundary_v) {
            continue;
        }
        if !live_node.contains(&e.a) || !live_node.contains(&e.b) {
            continue;
        }
        merged.insert((e.a, e.b), *e);
    }

    let win_cells: Vec<CellId> = window.iter().copied().collect();
    let mut new_edges = boundary_edge_map(cells, state, &win_cells);
    new_edges.retain(|_, e| live_node.contains(&e.a) && live_node.contains(&e.b));
    merge_min(&mut merged, new_edges);

    out_edges.clear();
    out_edges.extend(merged.into_values());
    out_edges.par_sort_unstable_by_key(|e| (e.a, e.b));
    rebuild_node_adj(out_edges, out_adj);
}

fn best_boundary_edges(cells: &SurfaceCells, state: &DijkstraState, out: &mut Vec<NodeEdge>) {
    let scan: Vec<CellId> = cells.ids().collect();
    let merged = boundary_edge_map(cells, state, &scan);
    out.clear();
    out.extend(merged.into_values());
    out.par_sort_unstable_by_key(|e| (e.a, e.b));
}

/// Cheapest Voronoi-boundary crossing per adjacent node pair over the scanned cells.
fn boundary_edge_map(
    cells: &SurfaceCells,
    state: &DijkstraState,
    scan: &[CellId],
) -> AHashMap<(NodeId, NodeId), NodeEdge> {
    scan.par_iter()
        .fold(
            AHashMap::<(NodeId, NodeId), NodeEdge>::new,
            |mut local, &u| {
                let du = state.dist[u as usize];
                if !du.is_finite() {
                    return local;
                }
                let sa = state.source[u as usize];
                for edge in cells.neighbors(u) {
                    let v = edge.dest;
                    let dv = state.dist[v as usize];
                    if !dv.is_finite() {
                        continue;
                    }
                    let sb = state.source[v as usize];
                    if sa == sb {
                        continue;
                    }
                    let cost = du + edge.cost + dv;
                    // Skip impassable crossings.
                    if !cost.is_finite() {
                        continue;
                    }
                    let (key_a, key_b, bu, bv) = if sa < sb {
                        (sa, sb, u, v)
                    } else {
                        (sb, sa, v, u)
                    };
                    let entry = local.entry((key_a, key_b)).or_insert(NodeEdge {
                        a: key_a,
                        b: key_b,
                        cost: f32::INFINITY,
                        boundary_u: NO_CELL,
                        boundary_v: NO_CELL,
                    });
                    if cost < entry.cost {
                        entry.cost = cost;
                        entry.boundary_u = bu;
                        entry.boundary_v = bv;
                    }
                }
                local
            },
        )
        .reduce(AHashMap::new, |mut a, b| {
            merge_min(&mut a, b);
            a
        })
}

/// Keep the lower-cost edge for each node pair when merging two maps.
fn merge_min(
    into: &mut AHashMap<(NodeId, NodeId), NodeEdge>,
    from: AHashMap<(NodeId, NodeId), NodeEdge>,
) {
    for (k, edge) in from {
        let entry = into.entry(k).or_insert(edge);
        if edge.cost < entry.cost {
            *entry = edge;
        }
    }
}

/// Expand each node-graph edge into VoxelKey segments along its cell path.
pub fn edges_to_segments(
    cells: &SurfaceCells,
    state: &DijkstraState,
    node_edges: &[NodeEdge],
) -> Vec<(VoxelKey, VoxelKey, f32)> {
    node_edges
        .par_iter()
        .flat_map_iter(|edge| {
            let mut from_a = walk_preds(state, edge.boundary_u);
            from_a.reverse();
            let to_b = walk_preds(state, edge.boundary_v);
            let path: Vec<CellId> = from_a.into_iter().chain(to_b).collect();
            let cost = edge.cost;
            path.windows(2)
                .map(|pair| (cells.coord(pair[0]), cells.coord(pair[1]), cost))
                .collect::<Vec<_>>()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::nodes::NodeData;
    use crate::voxel::surface_point_xyz;

    const VOXEL: f32 = 0.1;

    fn setup(surface: &[VoxelKey], node_cells: &[VoxelKey]) -> PlannerGraph {
        let mut plg = PlannerGraph::new();
        build_surface_lookup(surface, &mut plg.surface_lookup);
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

    fn strip_cells() -> Vec<VoxelKey> {
        (0..20).map(|x| (x, 0, 0)).collect()
    }

    #[test]
    fn two_nodes_on_strip_have_one_edge() {
        let pg = setup(&strip_cells(), &[(3, 0, 0), (15, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];
        let a = pg.cells.id((3, 0, 0)).unwrap();
        let b = pg.cells.id((15, 0, 0)).unwrap();
        assert_eq!((e.a.min(e.b), e.a.max(e.b)), (a.min(b), a.max(b)));
        assert_eq!(pg.node_adj[&a], vec![0]);
        assert_eq!(pg.node_adj[&b], vec![0]);
    }

    #[test]
    fn three_nodes_in_line_form_a_chain() {
        let pg = setup(&strip_cells(), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let c = |k| pg.cells.id(k).unwrap();
        let pairs: Vec<(NodeId, NodeId)> = pg.node_edges.iter().map(|e| (e.a, e.b)).collect();
        assert_eq!(
            pairs,
            vec![
                (c((3, 0, 0)), c((10, 0, 0))),
                (c((10, 0, 0)), c((17, 0, 0)))
            ]
        );
    }

    #[test]
    fn infinite_crossing_is_not_an_edge() {
        // The only crossing between the two nodes is impassable, so no edge.
        let surface: Vec<VoxelKey> = (0..6).map(|x| (x, 0, 0)).collect();
        let mut plg = PlannerGraph::new();
        build_surface_lookup(&surface, &mut plg.surface_lookup);
        build_surface_cells(&mut plg.cells, &plg.surface_lookup, VOXEL, 2);

        let c2 = plg.cells.id((2, 0, 0)).unwrap();
        let c3 = plg.cells.id((3, 0, 0)).unwrap();
        for e in plg.cells.edges_mut(c2) {
            if e.dest == c3 {
                e.cost = f32::INFINITY;
            }
        }
        for e in plg.cells.edges_mut(c3) {
            if e.dest == c2 {
                e.cost = f32::INFINITY;
            }
        }

        plg.nodes = [(0, 0, 0), (5, 0, 0)]
            .iter()
            .map(|&c| NodeData {
                cell_id: plg.cells.id(c).unwrap(),
                pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
            })
            .collect();
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
            &mut plg.node_adj,
        );

        assert!(
            plg.node_edges.is_empty(),
            "an infinite crossing is not an edge"
        );
        // Walking boundaries must not panic on an unset boundary cell.
        edges_to_segments(&plg.cells, &plg.cell_state, &plg.node_edges);
    }

    #[test]
    fn disconnected_components_have_no_edge() {
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let pg = setup(&cells, &[(2, 0, 0), (12, 0, 0)]);
        assert!(pg.node_edges.is_empty());
    }

    #[test]
    fn predecessor_walk_recovers_cell_path() {
        let pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];

        let cell_a = pg.nodes[0].cell_id;
        let cell_b = pg.nodes[1].cell_id;

        let chain_u = walk_preds(&pg.cell_state, e.boundary_u);
        let chain_v = walk_preds(&pg.cell_state, e.boundary_v);
        assert_eq!(chain_u.last(), Some(&cell_a));
        assert_eq!(chain_v.last(), Some(&cell_b));
    }
}
