// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node-graph edge construction.
//!
//! Build edges by running multi-source Dijkstra from all the start nodes.
//! This labels the surface with each cells closest source, also known as
//! the Voronoi region. We use the boundaries of these regions to build the
//! edges between start nodes.

use ahash::AHashMap;
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells, SurfaceLookup, NO_CELL};
use crate::dijkstra::{dijkstra, walk_preds, DijkstraState};
use crate::nodes::NodeData;
use crate::voxel::VoxelKey;

/// Index into planner graph nodes
pub type NodeId = u32;
pub const NO_NODE: NodeId = u32::MAX;

/// Index into planner graph node edges
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
    pub node_adj: Vec<Vec<NodeEdgeIdx>>,
    pub cell_state: DijkstraState,
}

impl PlannerGraph {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Assemble the cheapest paths between neighboring source nodes.
///
/// Runs multi-source dijkstra from the sources, then adds the cheapest edges
/// between Voronoi region boundaries.
pub fn build_node_edges(
    cells: &SurfaceCells,
    nodes: &[NodeData],
    state: &mut DijkstraState,
    out_edges: &mut Vec<NodeEdge>,
    out_adj: &mut Vec<Vec<NodeEdgeIdx>>,
) {
    out_edges.clear();
    out_adj.clear();

    if nodes.is_empty() {
        state.reset(cells.slot_capacity());
        return;
    }

    let source_cells: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();
    dijkstra(cells, &source_cells, state);

    best_boundary_edges(cells, state, out_edges);

    out_adj.resize_with(nodes.len(), Vec::new);
    for v in out_adj.iter_mut() {
        v.clear();
    }
    for (edge_idx, edge) in out_edges.iter().enumerate() {
        out_adj[edge.a as usize].push(edge_idx as NodeEdgeIdx);
        out_adj[edge.b as usize].push(edge_idx as NodeEdgeIdx);
    }
}

fn best_boundary_edges(cells: &SurfaceCells, state: &DijkstraState, out: &mut Vec<NodeEdge>) {
    let cell_entries: Vec<(CellId, &[Edge])> = cells.iter().collect();

    let merged: AHashMap<(NodeId, NodeId), NodeEdge> = cell_entries
        .par_iter()
        .fold(
            AHashMap::<(NodeId, NodeId), NodeEdge>::new,
            |mut local, (u, edges)| {
                let du = state.dist[*u as usize];
                if !du.is_finite() {
                    return local;
                }
                let sa = state.source[*u as usize];
                for edge in *edges {
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

                    let (key_a, key_b, bu, bv) = if sa < sb {
                        (sa, sb, *u, v)
                    } else {
                        (sb, sa, v, *u)
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
        .reduce(AHashMap::<(NodeId, NodeId), NodeEdge>::new, |mut a, b| {
            for (k, v_edge) in b {
                let entry = a.entry(k).or_insert(v_edge);
                if v_edge.cost < entry.cost {
                    *entry = v_edge;
                }
            }
            a
        });

    out.clear();
    out.extend(merged.into_values());
    out.par_sort_unstable_by_key(|e| (e.a, e.b));
}

/// Walk every node-graph edge and emit one segment per consecutive cell
/// pair along the reconstructed cell path. Output coords are in VoxelKey
/// space.
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
        assert_eq!((e.a, e.b), (0, 1));
        assert_eq!(pg.node_adj[0], vec![0]);
        assert_eq!(pg.node_adj[1], vec![0]);
    }

    #[test]
    fn three_nodes_in_line_form_a_chain() {
        let pg = setup(&strip_cells(), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let pairs: Vec<(NodeId, NodeId)> = pg.node_edges.iter().map(|e| (e.a, e.b)).collect();
        assert_eq!(pairs, vec![(0, 1), (1, 2)]);
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
