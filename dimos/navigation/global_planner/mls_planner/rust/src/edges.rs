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

use std::collections::hash_map::Entry;

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

use crate::adjacency::{CellId, SurfaceCells, SurfaceLookup, NO_CELL};
use crate::dijkstra::{dijkstra, dijkstra_region, walk_preds, DijkstraState, Weight};
use crate::nodes::{NodeData, NodeScratch};
use crate::voxel::VoxelKey;

/// A node is identified by the CellId it sits on. Stable across incremental
/// updates so cached edges and the Voronoi forest survive a regional rebuild.
pub type NodeId = CellId;
pub const NO_NODE: NodeId = NO_CELL;

/// Index into the planner graph node edges.
pub type NodeEdgeIdx = u32;

#[derive(Clone, Debug)]
pub struct NodeEdge {
    pub a: NodeId,
    pub b: NodeId,
    pub cost: f32,
    /// Cell on a's side of the cheapest Voronoi boundary crossing.
    pub boundary_u: CellId,
    /// Cell on b's side.
    pub boundary_v: CellId,
    /// The corridor: cell coordinates from a toward b, captured when the edge
    /// was built. Coordinates, not ids, so slot recycling cannot alias it.
    pub chain: Vec<VoxelKey>,
    /// Bounding box of the chain, (min, max) inclusive. Regional updates only
    /// re-walk corridors whose box touches the update window.
    pub bbox: (VoxelKey, VoxelKey),
}

/// A fresh crossing replaces a valid cached corridor only when clearly
/// cheaper. Stability: the graph should change when the world does, not when
/// a Voronoi boundary drifts a cell.
const CORRIDOR_ADOPT_FRAC: f32 = 0.7;

/// Fill an edge's corridor from the current Voronoi field by walking preds
/// from both boundary cells. False when either walk fails to reach the edge's
/// own endpoint: stale out-of-window state can label a cell with one node
/// while its pred chain walks back to another, and freezing that corridor
/// would corrupt the edge for good.
fn capture_chain(cells: &SurfaceCells, state: &DijkstraState, edge: &mut NodeEdge) -> bool {
    let mut from_a = walk_live_chain(cells, state, edge.boundary_u);
    let to_b = walk_live_chain(cells, state, edge.boundary_v);
    if from_a.last() != Some(&edge.a) || to_b.last() != Some(&edge.b) {
        return false;
    }
    from_a.reverse();
    edge.chain = from_a
        .into_iter()
        .chain(to_b)
        .map(|c| cells.coord(c))
        .collect();
    edge.bbox = chain_bbox(edge.chain.iter().copied());
    true
}

/// walk_preds truncated at the first dead cell or non-adjacent hop. Regional
/// updates leave out-of-window pred chains stale. A freed slot's coord is
/// garbage, and a recycled slot can alias an unrelated cell far away.
fn walk_live_chain(cells: &SurfaceCells, state: &DijkstraState, from: CellId) -> Vec<CellId> {
    let mut chain = walk_preds(state, from);
    let mut keep = 0;
    for (i, &c) in chain.iter().enumerate() {
        if !cells.is_live(c) {
            break;
        }
        if i > 0 && !cells.neighbors(chain[i - 1]).iter().any(|e| e.dest == c) {
            break;
        }
        keep = i + 1;
    }
    chain.truncate(keep);
    chain
}

fn chain_bbox(chain: impl IntoIterator<Item = VoxelKey>) -> (VoxelKey, VoxelKey) {
    let mut min = (i32::MAX, i32::MAX, i32::MAX);
    let mut max = (i32::MIN, i32::MIN, i32::MIN);
    for (x, y, z) in chain {
        min = (min.0.min(x), min.1.min(y), min.2.min(z));
        max = (max.0.max(x), max.1.max(y), max.2.max(z));
    }
    (min, max)
}

fn bbox_intersects(a: (VoxelKey, VoxelKey), b: (VoxelKey, VoxelKey)) -> bool {
    a.0 .0 <= b.1 .0
        && b.0 .0 <= a.1 .0
        && a.0 .1 <= b.1 .1
        && b.0 .1 <= a.1 .1
        && a.0 .2 <= b.1 .2
        && b.0 .2 <= a.1 .2
}

/// The corridor still runs from a's cell to b's cell. Slot recycling can hand
/// an edge's node ids to unrelated cells, which this catches.
fn endpoints_match(cells: &SurfaceCells, edge: &NodeEdge) -> bool {
    let first = edge.chain.first().and_then(|&c| cells.id(c));
    let last = edge.chain.last().and_then(|&c| cells.id(c));
    first == Some(edge.a) && last == Some(edge.b)
}

/// Walk a corridor on the current surface and price it at current costs.
/// None when it no longer connects the edge's own endpoints, any cell died,
/// or any hop is impassable, meaning the corridor is no longer known safe.
fn corridor_cost(cells: &SurfaceCells, edge: &NodeEdge) -> Option<f32> {
    let chain = &edge.chain;
    if chain.len() < 2 || !endpoints_match(cells, edge) {
        return None;
    }
    let mut total = 0.0f32;
    let mut prev = cells.id(chain[0])?;
    for &coord in &chain[1..] {
        let cur = cells.id(coord)?;
        let hop = cells.neighbors(prev).iter().find(|e| e.dest == cur)?.cost;
        if !hop.is_finite() {
            return None;
        }
        total += hop;
        prev = cur;
    }
    Some(total)
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
    /// Reusable dense scratch for node placement, shared across region frames.
    pub node_scratch: NodeScratch,
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

/// Incremental build_node_edges. Redo the Voronoi inside the window, keep
/// cached edges whose corridors are still intact, and rescan the window for
/// new node-to-node crossings.
pub fn build_node_edges_region(
    cells: &SurfaceCells,
    nodes: &[NodeData],
    window: &[CellId],
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
    let window_bbox = chain_bbox(window.iter().map(|&c| cells.coord(c)));

    // Corridors clear of the update window are provably untouched and keep
    // their cost. Touched ones are re-priced, or dropped when broken.
    let mut merged: AHashMap<(NodeId, NodeId), NodeEdge> = AHashMap::new();
    for e in out_edges.drain(..) {
        if !live_node.contains(&e.a) || !live_node.contains(&e.b) {
            continue;
        }
        if !bbox_intersects(e.bbox, window_bbox) {
            if endpoints_match(cells, &e) {
                merged.insert((e.a, e.b), e);
            }
            continue;
        }
        let Some(cost) = corridor_cost(cells, &e) else {
            continue;
        };
        let mut e = e;
        e.cost = cost;
        merged.insert((e.a, e.b), e);
    }

    let mut new_edges = boundary_edge_map(cells, state, window);
    new_edges.retain(|_, e| live_node.contains(&e.a) && live_node.contains(&e.b));
    for ((a, b), mut e) in new_edges {
        match merged.entry((a, b)) {
            Entry::Occupied(mut o) => {
                if e.cost < CORRIDOR_ADOPT_FRAC * o.get().cost
                    && capture_chain(cells, state, &mut e)
                {
                    o.insert(e);
                }
            }
            Entry::Vacant(v) => {
                if capture_chain(cells, state, &mut e) {
                    v.insert(e);
                }
            }
        }
    }

    out_edges.extend(merged.into_values());
    out_edges.par_sort_unstable_by_key(|e| (e.a, e.b));
    rebuild_node_adj(out_edges, out_adj);
}

fn best_boundary_edges(cells: &SurfaceCells, state: &DijkstraState, out: &mut Vec<NodeEdge>) {
    let scan: Vec<CellId> = cells.ids().collect();
    let merged = boundary_edge_map(cells, state, &scan);
    out.clear();
    out.extend(merged.into_values());
    out.retain_mut(|e| capture_chain(cells, state, e));
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
                    let entry = local.entry((key_a, key_b)).or_insert_with(|| NodeEdge {
                        a: key_a,
                        b: key_b,
                        cost: f32::INFINITY,
                        boundary_u: NO_CELL,
                        boundary_v: NO_CELL,
                        chain: Vec::new(),
                        bbox: ((0, 0, 0), (0, 0, 0)),
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
        match into.entry(k) {
            Entry::Occupied(mut o) => {
                if edge.cost < o.get().cost {
                    o.insert(edge);
                }
            }
            Entry::Vacant(v) => {
                v.insert(edge);
            }
        }
    }
}

/// Expand each node-graph edge into VoxelKey segments along its corridor.
pub fn edges_to_segments(node_edges: &[NodeEdge]) -> Vec<(VoxelKey, VoxelKey, f32)> {
    node_edges
        .par_iter()
        .flat_map_iter(|edge| {
            edge.chain
                .windows(2)
                .map(|pair| (pair[0], pair[1], edge.cost))
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
        edges_to_segments(&plg.node_edges);
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

    #[test]
    fn corridor_cost_none_on_impassable_hop() {
        let mut pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let edge = pg.node_edges[0].clone();
        assert!(corridor_cost(&pg.cells, &edge).is_some());

        let c9 = pg.cells.id((9, 0, 0)).unwrap();
        let c10 = pg.cells.id((10, 0, 0)).unwrap();
        for e in pg.cells.edges_mut(c9) {
            if e.dest == c10 {
                e.cost = f32::INFINITY;
            }
        }
        assert!(
            corridor_cost(&pg.cells, &edge).is_none(),
            "an impassable hop invalidates the corridor"
        );
    }

    #[test]
    fn corridor_cost_none_when_a_chain_cell_dies() {
        let mut pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let edge = pg.node_edges[0].clone();
        pg.cells.remove((10, 0, 0));
        assert!(
            corridor_cost(&pg.cells, &edge).is_none(),
            "a dead cell invalidates the corridor"
        );
    }

    #[test]
    fn corridor_cost_none_when_the_chain_misses_an_endpoint() {
        let pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let mut edge = pg.node_edges[0].clone();
        assert!(corridor_cost(&pg.cells, &edge).is_some());
        // A corridor that loops back to a instead of reaching b is corrupt
        // even though every cell is live and every hop is feasible.
        edge.chain.pop();
        edge.chain.push(edge.chain[edge.chain.len() - 2]);
        assert!(
            corridor_cost(&pg.cells, &edge).is_none(),
            "a corridor not ending at b must not be priced"
        );
    }

    #[test]
    fn capture_chain_rejects_a_walk_that_misses_the_endpoint() {
        let mut pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let mut edge = pg.node_edges[0].clone();
        assert!(capture_chain(&pg.cells, &pg.cell_state, &mut edge));
        // Kill a cell between the boundary and node b: the live walk truncates
        // before its endpoint, so the corridor must be refused, not stored.
        pg.cells.remove((15, 0, 0));
        assert!(!capture_chain(&pg.cells, &pg.cell_state, &mut edge));
    }

    fn parallel_strips() -> Vec<VoxelKey> {
        let mut v: Vec<VoxelKey> = (0..20).map(|x| (x, 0, 0)).collect();
        v.extend((0..20).map(|x| (x, 1, 0)));
        v
    }

    fn rebuild_region_all(pg: &mut PlannerGraph) {
        let window: Vec<CellId> = pg.cells.ids().collect();
        let PlannerGraph {
            cells,
            nodes,
            node_edges,
            node_adj,
            cell_state,
            ..
        } = pg;
        build_node_edges_region(cells, nodes, &window, cell_state, node_edges, node_adj);
    }

    #[test]
    fn cached_corridor_survives_an_equal_cost_rescan() {
        let mut pg = setup(&parallel_strips(), &[(0, 0, 0), (19, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 1);
        let cached = pg.node_edges[0].chain.clone();

        rebuild_region_all(&mut pg);

        assert_eq!(pg.node_edges.len(), 1);
        assert_eq!(
            pg.node_edges[0].chain, cached,
            "a crossing that is not clearly cheaper must not displace the corridor"
        );
    }

    #[test]
    fn clearly_cheaper_crossing_replaces_the_cached_corridor() {
        let mut pg = setup(&parallel_strips(), &[(0, 0, 0), (19, 0, 0)]);
        let cached = pg.node_edges[0].chain.clone();
        let old_cost = pg.node_edges[0].cost;

        // The y=1 row becomes a highway two orders of magnitude cheaper.
        let ids: Vec<CellId> = pg.cells.ids().collect();
        let into_row: Vec<(CellId, Vec<bool>)> = ids
            .iter()
            .map(|&id| {
                let marks = pg
                    .cells
                    .neighbors(id)
                    .iter()
                    .map(|e| pg.cells.coord(e.dest).1 == 1)
                    .collect();
                (id, marks)
            })
            .collect();
        for (id, marks) in into_row {
            for (e, cheap) in pg.cells.edges_mut(id).iter_mut().zip(marks) {
                if cheap {
                    e.cost *= 0.01;
                }
            }
        }

        rebuild_region_all(&mut pg);

        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges[0];
        assert!(e.cost < CORRIDOR_ADOPT_FRAC * old_cost);
        assert_ne!(e.chain, cached, "the cheaper crossing is adopted");
        assert!(
            e.chain.iter().any(|&(_, y, _)| y == 1),
            "the adopted corridor routes through the cheap row"
        );
    }
}
