// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node-graph edge construction.
//!
//! Build edges by running multi-source Dijkstra from all the start nodes.
//! This labels the surface with each cells closest source, also known as
//! the Voronoi region. We use the boundaries of these regions to build the
//! edges between start nodes.

use ahash::AHashMap;

use crate::adjacency::{SurfaceAdjacency, SurfaceLookup};
use crate::dijkstra::{dijkstra, CellState};
use crate::nodes::{NodeData, SurfaceGraph};
use crate::voxel::VoxelKey;

pub struct NodeEdge {
    pub a: u32,
    pub b: u32,
    pub cost: f32,
    /// Cell on a's side of the cheapest Voronoi boundary crossing.
    pub boundary_u: VoxelKey,
    /// Cell on b's side.
    pub boundary_v: VoxelKey,
}

pub struct PlannerGraph {
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
    pub node_edges: Vec<NodeEdge>,
    pub node_adj: Vec<Vec<u32>>,

    pub cell_state: AHashMap<VoxelKey, CellState>,
}

pub fn add_node_edges(sg: SurfaceGraph) -> PlannerGraph {
    let SurfaceGraph {
        adj,
        surface_lookup,
        nodes,
    } = sg;

    if nodes.is_empty() {
        return PlannerGraph {
            surface_lookup,
            nodes,
            node_edges: Vec::new(),
            node_adj: Vec::new(),
            cell_state: AHashMap::new(),
        };
    }

    let source_cells: Vec<VoxelKey> = nodes.iter().map(|n| n.cell).collect();
    let cell_state = dijkstra(&adj, &source_cells).state;
    let node_edges = best_boundary_edges(&adj, &cell_state);

    let mut node_adj: Vec<Vec<u32>> = vec![Vec::new(); nodes.len()];
    for (edge_idx, edge) in node_edges.iter().enumerate() {
        node_adj[edge.a as usize].push(edge_idx as u32);
        node_adj[edge.b as usize].push(edge_idx as u32);
    }

    PlannerGraph {
        surface_lookup,
        nodes,
        node_edges,
        node_adj,
        cell_state,
    }
}

/// Walk every node-graph edge and emit one segment per consecutive cell pair
/// along the reconstructed cell path.
pub fn edges_to_segments(plg: &PlannerGraph, _voxel_size: f32) -> Vec<(VoxelKey, VoxelKey, f32)> {
    let mut segments = Vec::new();
    for edge in &plg.node_edges {
        let mut from_a = walk_preds_to_source(plg, edge.boundary_u);
        from_a.reverse();
        let to_b = walk_preds_to_source(plg, edge.boundary_v);
        let mut path: Vec<VoxelKey> = from_a;
        path.extend(to_b);
        for pair in path.windows(2) {
            segments.push((pair[0], pair[1], edge.cost));
        }
    }
    segments
}

pub fn walk_preds_to_source(plg: &PlannerGraph, start_cell: VoxelKey) -> Vec<VoxelKey> {
    let mut cells = vec![start_cell];
    let mut cur = start_cell;
    while let Some(p) = plg.cell_state.get(&cur).and_then(|s| s.pred) {
        cur = p;
        cells.push(cur);
    }
    cells
}

fn best_boundary_edges(
    adj: &SurfaceAdjacency,
    state: &AHashMap<VoxelKey, CellState>,
) -> Vec<NodeEdge> {
    let mut best: AHashMap<(u32, u32), NodeEdge> = AHashMap::new();

    for (u, edges) in adj.iter() {
        let Some(su) = state.get(&u) else {
            continue;
        };
        let sa = su.source;
        let du = su.dist;
        for edge in edges {
            let v = edge.dst;
            let Some(sv) = state.get(&v) else {
                continue;
            };
            let sb = sv.source;
            if sa == sb {
                continue;
            }
            let dv = sv.dist;
            let cost = du + edge.cost + dv;

            let (key_a, key_b, bu, bv) = if sa < sb {
                (sa, sb, u, v)
            } else {
                (sb, sa, v, u)
            };

            let entry = best.entry((key_a, key_b)).or_insert(NodeEdge {
                a: key_a,
                b: key_b,
                cost: f32::INFINITY,
                boundary_u: (0, 0, 0),
                boundary_v: (0, 0, 0),
            });
            if cost < entry.cost {
                entry.cost = cost;
                entry.boundary_u = bu;
                entry.boundary_v = bv;
            }
        }
    }

    let mut out: Vec<NodeEdge> = best.into_values().collect();
    out.sort_by_key(|e| (e.a, e.b));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_adjacency, build_surface_lookup};
    use crate::voxel::surface_point_xyz;

    const VOXEL: f32 = 0.1;

    fn graph_with_nodes(surface_cells: &[VoxelKey], node_cells: &[VoxelKey]) -> SurfaceGraph {
        let surface_lookup = build_surface_lookup(surface_cells);
        let adj = build_surface_adjacency(&surface_lookup, VOXEL, 2);
        let nodes: Vec<NodeData> = node_cells
            .iter()
            .map(|&c| NodeData {
                cell: c,
                pos: surface_point_xyz(c.0, c.1, c.2, VOXEL),
            })
            .collect();
        SurfaceGraph {
            adj,
            surface_lookup,
            nodes,
        }
    }

    fn strip_cells() -> Vec<VoxelKey> {
        (0..20).map(|x| (x, 0, 0)).collect()
    }

    #[test]
    fn two_nodes_on_strip_have_one_edge() {
        let sg = graph_with_nodes(&strip_cells(), &[(3, 0, 0), (15, 0, 0)]);
        let pg = add_node_edges(sg);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];
        assert_eq!((e.a, e.b), (0, 1));
        assert_eq!(pg.node_adj[0], vec![0]);
        assert_eq!(pg.node_adj[1], vec![0]);
    }

    #[test]
    fn three_nodes_in_line_form_a_chain() {
        // Nodes at 3, 10, 17 in a strip 0..20. Voronoi boundaries are
        // around 6-7 and 13-14, so we get edges (0,1) and (1,2) but no (0,2).
        let sg = graph_with_nodes(&strip_cells(), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let pg = add_node_edges(sg);
        let pairs: Vec<(u32, u32)> = pg.node_edges.iter().map(|e| (e.a, e.b)).collect();
        assert_eq!(pairs, vec![(0, 1), (1, 2)]);
    }

    #[test]
    fn disconnected_components_have_no_edge() {
        // Two strips with a gap, one node in each.
        let mut cells: Vec<VoxelKey> = (0..5).map(|x| (x, 0, 0)).collect();
        cells.extend((10..15).map(|x| (x, 0, 0)));
        let sg = graph_with_nodes(&cells, &[(2, 0, 0), (12, 0, 0)]);
        let pg = add_node_edges(sg);
        assert!(pg.node_edges.is_empty());
    }

    #[test]
    fn predecessor_walk_recovers_cell_path() {
        // Two nodes at strip ends. Walk preds from each boundary cell back to
        // its owning node cell and verify the chain reaches the node.
        let sg = graph_with_nodes(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let pg = add_node_edges(sg);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];

        let cell_a = pg.nodes[0].cell;
        let cell_b = pg.nodes[1].cell;

        let mut cur = e.boundary_u;
        let mut hops = 0;
        while let Some(p) = pg.cell_state.get(&cur).and_then(|s| s.pred) {
            cur = p;
            hops += 1;
            assert!(hops < 1000, "predecessor walk did not terminate");
        }
        assert_eq!(cur, cell_a, "u-side preds must reach node a");

        let mut cur = e.boundary_v;
        while let Some(p) = pg.cell_state.get(&cur).and_then(|s| s.pred) {
            cur = p;
        }
        assert_eq!(cur, cell_b, "v-side preds must reach node b");
    }
}
