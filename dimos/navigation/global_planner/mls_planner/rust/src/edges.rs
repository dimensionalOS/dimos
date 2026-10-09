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

use ahash::AHashMap;
use rayon::prelude::*;
use smallvec::SmallVec;

use crate::adjacency::{CellId, SurfaceCells, SurfaceLookup, NO_CELL};
use crate::dijkstra::{
    dijkstra, dijkstra_clusters, walk_preds, ClusterIndex, DijkstraState, Weight,
};
use crate::nodes::{NodeData, NodeIndex, NodeScratch};
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

/// The cells a repair re-labels, split into the connected clusters the
/// searches run over, with the cluster index assigned for them.
pub struct RepairWindow<'a> {
    pub cells: &'a [CellId],
    pub clusters: &'a [Vec<CellId>],
    pub index: &'a ClusterIndex,
}

/// The node graph's edges, their per-node adjacency, and per surface cell
/// slot the edges whose corridor runs through it.
#[derive(Default)]
pub struct NodeEdges {
    pub edges: Vec<NodeEdge>,
    pub adj: AHashMap<NodeId, Vec<NodeEdgeIdx>>,
    through: Vec<SmallVec<[NodeEdgeIdx; 2]>>,
}

impl NodeEdges {
    pub fn len(&self) -> usize {
        self.edges.len()
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    pub fn clear(&mut self) {
        self.edges.clear();
        self.adj.clear();
        for list in &mut self.through {
            list.clear();
        }
    }

    /// The edge between two nodes, if any.
    pub fn between(&self, a: NodeId, b: NodeId) -> Option<NodeEdgeIdx> {
        self.adj.get(&a)?.iter().copied().find(|&i| {
            let e = &self.edges[i as usize];
            e.a == b || e.b == b
        })
    }

    fn ensure_capacity(&mut self, slots: usize) {
        if self.through.len() < slots {
            self.through.resize_with(slots, SmallVec::new);
        }
    }

    /// Rebuild the adjacency and the per-cell index from the edge list.
    fn reindex(&mut self, cells: &SurfaceCells) {
        self.adj.clear();
        for list in &mut self.through {
            list.clear();
        }
        self.ensure_capacity(cells.slot_capacity());
        for (i, e) in self.edges.iter().enumerate() {
            let i = i as NodeEdgeIdx;
            self.adj.entry(e.a).or_default().push(i);
            self.adj.entry(e.b).or_default().push(i);
            Self::index_chain(&mut self.through, cells, i, &e.chain);
        }
    }

    fn push(&mut self, cells: &SurfaceCells, edge: NodeEdge) {
        let i = self.edges.len() as NodeEdgeIdx;
        self.adj.entry(edge.a).or_default().push(i);
        self.adj.entry(edge.b).or_default().push(i);
        Self::index_chain(&mut self.through, cells, i, &edge.chain);
        self.edges.push(edge);
    }

    /// Swap in a new corridor for the same node pair.
    fn replace(&mut self, cells: &SurfaceCells, i: NodeEdgeIdx, edge: NodeEdge) {
        let old = std::mem::replace(&mut self.edges[i as usize], edge);
        Self::unindex_chain(&mut self.through, cells, i, &old.chain);
        Self::index_chain(&mut self.through, cells, i, &self.edges[i as usize].chain);
    }

    fn swap_remove(&mut self, cells: &SurfaceCells, i: NodeEdgeIdx) {
        let last = (self.edges.len() - 1) as NodeEdgeIdx;
        let gone = self.edges.swap_remove(i as usize);
        Self::drop_adj(&mut self.adj, gone.a, i);
        Self::drop_adj(&mut self.adj, gone.b, i);
        Self::unindex_chain(&mut self.through, cells, i, &gone.chain);
        if i != last {
            let moved = &self.edges[i as usize];
            Self::renumber_adj(&mut self.adj, moved.a, last, i);
            Self::renumber_adj(&mut self.adj, moved.b, last, i);
            for &c in &moved.chain {
                if let Some(slot) = cells.id(c) {
                    if let Some(x) = self.through[slot as usize].iter_mut().find(|x| **x == last) {
                        *x = i;
                    }
                }
            }
        }
    }

    fn drop_adj(adj: &mut AHashMap<NodeId, Vec<NodeEdgeIdx>>, node: NodeId, i: NodeEdgeIdx) {
        if let Entry::Occupied(mut o) = adj.entry(node) {
            o.get_mut().retain(|&x| x != i);
            if o.get().is_empty() {
                o.remove();
            }
        }
    }

    fn renumber_adj(
        adj: &mut AHashMap<NodeId, Vec<NodeEdgeIdx>>,
        node: NodeId,
        from: NodeEdgeIdx,
        to: NodeEdgeIdx,
    ) {
        if let Some(x) = adj
            .get_mut(&node)
            .and_then(|list| list.iter_mut().find(|x| **x == from))
        {
            *x = to;
        }
    }

    fn index_chain(
        through: &mut [SmallVec<[NodeEdgeIdx; 2]>],
        cells: &SurfaceCells,
        i: NodeEdgeIdx,
        chain: &[VoxelKey],
    ) {
        for &c in chain {
            if let Some(slot) = cells.id(c) {
                through[slot as usize].push(i);
            }
        }
    }

    /// Cells the corridor lost no longer list the edge, so a dead coordinate
    /// is skipped here and its slot cleared by the region repair.
    fn unindex_chain(
        through: &mut [SmallVec<[NodeEdgeIdx; 2]>],
        cells: &SurfaceCells,
        i: NodeEdgeIdx,
        chain: &[VoxelKey],
    ) {
        for &c in chain {
            if let Some(slot) = cells.id(c) {
                let list = &mut through[slot as usize];
                if let Some(p) = list.iter().position(|&x| x == i) {
                    list.swap_remove(p);
                }
            }
        }
    }
}

#[derive(Default)]
pub struct PlannerGraph {
    pub cells: SurfaceCells,
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
    pub node_edges: NodeEdges,
    /// Each cell's nearest node and the predecessor back toward it. The planner
    /// walks these to expand a node-to-node edge into its cell path.
    pub cell_state: DijkstraState,
    /// Each cell's distance to the nearest wall.
    pub wall_state: DijkstraState,
    /// Reusable dense scratch for node placement, shared across region frames.
    pub node_scratch: NodeScratch,
    /// Which cell holds which node, and node cells by spacing bin.
    pub node_index: NodeIndex,
    /// The repair window's clusters, assigned for the duration of a repair.
    pub cluster_index: ClusterIndex,
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
    out: &mut NodeEdges,
) {
    out.clear();

    if nodes.is_empty() {
        state.reset(cells.slot_capacity());
        return;
    }

    let source_cells: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();
    dijkstra(cells, &source_cells, state, Weight::Penalized);

    best_boundary_edges(cells, state, &mut out.edges);
    out.reindex(cells);
}

/// Incremental build_node_edges. Redo the Voronoi inside the window, drop the
/// edges of gone nodes, re-price or drop the corridors running through the
/// window or a removed cell, and rescan the window for new crossings.
#[allow(clippy::too_many_arguments)]
pub fn build_node_edges_region(
    cells: &SurfaceCells,
    nodes: &[NodeData],
    index: &NodeIndex,
    window: &RepairWindow,
    removed: &[CellId],
    gone: &[NodeId],
    state: &mut DijkstraState,
    out: &mut NodeEdges,
) {
    if nodes.is_empty() {
        state.reset(cells.slot_capacity());
        out.clear();
        return;
    }
    let sources: Vec<CellId> = window
        .cells
        .iter()
        .copied()
        .filter(|&w| index.has(w))
        .collect();
    dijkstra_clusters(
        cells,
        &sources,
        window.clusters,
        window.index,
        state,
        Weight::Penalized,
    );
    let window = &window.cells;
    out.ensure_capacity(cells.slot_capacity());

    // A gone node's edges go whatever their corridors say. The rest are
    // visited in descending order so swap_remove never moves an edge still
    // to be visited.
    let mut work: Vec<(NodeEdgeIdx, bool)> = Vec::new();
    for g in gone {
        if let Some(list) = out.adj.remove(g) {
            work.extend(list.into_iter().map(|i| (i, true)));
        }
    }
    for &c in window.iter().chain(removed) {
        work.extend(out.through[c as usize].iter().map(|&i| (i, false)));
    }
    for &r in removed {
        out.through[r as usize].clear();
    }
    work.sort_unstable_by(|x, y| y.cmp(x));
    work.dedup_by_key(|x| x.0);
    for (i, doomed) in work {
        if doomed {
            out.swap_remove(cells, i);
            continue;
        }
        match corridor_cost(cells, &out.edges[i as usize]) {
            Some(cost) => out.edges[i as usize].cost = cost,
            None => out.swap_remove(cells, i),
        }
    }

    let mut crossings: Vec<NodeEdge> = boundary_edge_map(cells, state, window)
        .into_values()
        .filter(|e| index.has(e.a) && index.has(e.b))
        .collect();
    crossings.sort_unstable_by_key(|e| (e.a, e.b));
    for mut e in crossings {
        match out.between(e.a, e.b) {
            Some(i) => {
                if e.cost < CORRIDOR_ADOPT_FRAC * out.edges[i as usize].cost
                    && capture_chain(cells, state, &mut e)
                {
                    out.replace(cells, i, e);
                }
            }
            None => {
                if capture_chain(cells, state, &mut e) {
                    out.push(cells, e);
                }
            }
        }
    }
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
    use crate::dijkstra::window_clusters;
    use crate::nodes::{NodeData, PlacementParams};
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
        plg.node_index.rebuild(&plg.cells, &plg.nodes, &params());
        build_node_edges(
            &plg.cells,
            &plg.nodes,
            &mut plg.cell_state,
            &mut plg.node_edges,
        );
        plg
    }

    fn params() -> PlacementParams {
        PlacementParams {
            clearance_cells: 2,
            step_cells: 2,
            voxel_size: VOXEL,
            node_spacing_m: 1.0,
            wall_clearance_m: 0.1,
            wall_buffer_m: 0.5,
            wall_buffer_weight: 10.0,
            step_penalty_weight: 1.0,
        }
    }

    fn strip_cells() -> Vec<VoxelKey> {
        (0..20).map(|x| (x, 0, 0)).collect()
    }

    #[test]
    fn two_nodes_on_strip_have_one_edge() {
        let pg = setup(&strip_cells(), &[(3, 0, 0), (15, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 1);
        let e = &pg.node_edges.edges[0];
        let a = pg.cells.id((3, 0, 0)).unwrap();
        let b = pg.cells.id((15, 0, 0)).unwrap();
        assert_eq!((e.a.min(e.b), e.a.max(e.b)), (a.min(b), a.max(b)));
        assert_eq!(pg.node_edges.adj[&a], vec![0]);
        assert_eq!(pg.node_edges.adj[&b], vec![0]);
    }

    #[test]
    fn three_nodes_in_line_form_a_chain() {
        let pg = setup(&strip_cells(), &[(3, 0, 0), (10, 0, 0), (17, 0, 0)]);
        let c = |k| pg.cells.id(k).unwrap();
        let pairs: Vec<(NodeId, NodeId)> = pg.node_edges.edges.iter().map(|e| (e.a, e.b)).collect();
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
        );

        assert!(
            plg.node_edges.is_empty(),
            "an infinite crossing is not an edge"
        );
        // Walking boundaries must not panic on an unset boundary cell.
        edges_to_segments(&plg.node_edges.edges);
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
        let e = &pg.node_edges.edges[0];

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
        let edge = pg.node_edges.edges[0].clone();
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
        let edge = pg.node_edges.edges[0].clone();
        pg.cells.remove((10, 0, 0));
        assert!(
            corridor_cost(&pg.cells, &edge).is_none(),
            "a dead cell invalidates the corridor"
        );
    }

    #[test]
    fn corridor_cost_none_when_the_chain_misses_an_endpoint() {
        let pg = setup(&strip_cells(), &[(0, 0, 0), (19, 0, 0)]);
        let mut edge = pg.node_edges.edges[0].clone();
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
        let mut edge = pg.node_edges.edges[0].clone();
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
        repair_region(pg, &window, &[], &[]);
    }

    fn repair_region(
        pg: &mut PlannerGraph,
        window: &[CellId],
        removed: &[CellId],
        gone: &[NodeId],
    ) {
        let PlannerGraph {
            cells,
            nodes,
            node_index,
            node_edges,
            cell_state,
            node_scratch,
            cluster_index,
            ..
        } = pg;
        let clusters = window_clusters(cells, window, &mut node_scratch.seen);
        cluster_index.assign(cells.slot_capacity(), &clusters);
        let repair = RepairWindow {
            cells: window,
            clusters: &clusters,
            index: cluster_index,
        };
        build_node_edges_region(
            cells, nodes, node_index, &repair, removed, gone, cell_state, node_edges,
        );
        cluster_index.clear(&clusters);
        assert_indexed(cells, node_edges);
    }

    /// The adjacency and per-cell index match what a rebuild from the edge
    /// list would give.
    fn assert_indexed(cells: &SurfaceCells, ne: &NodeEdges) {
        let mut fresh = NodeEdges {
            edges: ne.edges.clone(),
            ..NodeEdges::default()
        };
        fresh.reindex(cells);
        let sorted = |adj: &AHashMap<NodeId, Vec<NodeEdgeIdx>>| {
            let mut v: Vec<(NodeId, Vec<NodeEdgeIdx>)> = adj
                .iter()
                .map(|(k, l)| {
                    let mut l = l.clone();
                    l.sort_unstable();
                    (*k, l)
                })
                .collect();
            v.sort_unstable();
            v
        };
        assert_eq!(sorted(&ne.adj), sorted(&fresh.adj));
        for slot in 0..cells.slot_capacity() {
            let mut have: Vec<NodeEdgeIdx> =
                ne.through.get(slot).map_or(Vec::new(), |l| l.to_vec());
            let mut want: Vec<NodeEdgeIdx> = fresh.through[slot].to_vec();
            have.sort_unstable();
            want.sort_unstable();
            assert_eq!(have, want, "edges through slot {slot}");
        }
    }

    #[test]
    fn gone_node_loses_its_edges_and_the_survivors_bridge_the_gap() {
        let mut pg = setup(&strip_cells(), &[(0, 0, 0), (10, 0, 0), (19, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 2);
        let gone = pg.cells.id((10, 0, 0)).unwrap();
        pg.nodes.retain(|n| n.cell_id != gone);
        pg.node_index.rebuild(&pg.cells, &pg.nodes, &params());

        let window: Vec<CellId> = pg.cells.ids().collect();
        repair_region(&mut pg, &window, &[], &[gone]);

        let (a, b) = (
            pg.cells.id((0, 0, 0)).unwrap(),
            pg.cells.id((19, 0, 0)).unwrap(),
        );
        assert_eq!(pg.node_edges.len(), 1);
        assert!(pg.node_edges.between(a, b).is_some());
        assert!(!pg.node_edges.adj.contains_key(&gone));
    }

    #[test]
    fn removed_cell_under_a_corridor_drops_the_edge_outside_the_window() {
        let mut pg = setup(
            &strip_cells(),
            &[(0, 0, 0), (6, 0, 0), (12, 0, 0), (19, 0, 0)],
        );
        assert_eq!(pg.node_edges.len(), 3);
        let removed = pg.cells.remove((3, 0, 0)).unwrap();

        repair_region(&mut pg, &[], &[removed], &[]);

        let ids = |x: i32| pg.cells.id((x, 0, 0)).unwrap();
        assert_eq!(pg.node_edges.len(), 2);
        assert!(pg.node_edges.between(ids(0), ids(6)).is_none());
        assert!(pg.node_edges.between(ids(6), ids(12)).is_some());
        assert!(pg.node_edges.between(ids(12), ids(19)).is_some());
    }

    #[test]
    fn cached_corridor_survives_an_equal_cost_rescan() {
        let mut pg = setup(&parallel_strips(), &[(0, 0, 0), (19, 0, 0)]);
        assert_eq!(pg.node_edges.len(), 1);
        let cached = pg.node_edges.edges[0].chain.clone();

        rebuild_region_all(&mut pg);

        assert_eq!(pg.node_edges.len(), 1);
        assert_eq!(
            pg.node_edges.edges[0].chain, cached,
            "a crossing that is not clearly cheaper must not displace the corridor"
        );
    }

    #[test]
    fn clearly_cheaper_crossing_replaces_the_cached_corridor() {
        let mut pg = setup(&parallel_strips(), &[(0, 0, 0), (19, 0, 0)]);
        let cached = pg.node_edges.edges[0].chain.clone();
        let old_cost = pg.node_edges.edges[0].cost;

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
        let e = &pg.node_edges.edges[0];
        assert!(e.cost < CORRIDOR_ADOPT_FRAC * old_cost);
        assert_ne!(e.chain, cached, "the cheaper crossing is adopted");
        assert!(
            e.chain.iter().any(|&(_, y, _)| y == 1),
            "the adopted corridor routes through the cheap row"
        );
    }
}
