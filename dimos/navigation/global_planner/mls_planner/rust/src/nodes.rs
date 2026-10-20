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

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

use std::cmp::Ordering;

use ahash::AHashMap;
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells, SurfaceLookup, NO_CELL};
use crate::columns::ColumnIz;
use crate::dijkstra::{
    chain_source, dijkstra, dijkstra_clusters, window_clusters, ClusterIndex, DijkstraState,
    Frontier, RepairWindow, Weight,
};
use crate::surfaces::is_standable;
use crate::voxel::{surface_point_xyz, VoxelKey};

const NEIGHBORS_4: [(i32, i32, u8); 4] = [(-1, 0, 1), (1, 0, 2), (0, -1, 4), (0, 1, 8)];

/// A node is identified by the CellId it sits on. Stable across incremental
/// updates so cached edges and the Voronoi forest survive a regional rebuild.
pub type NodeId = CellId;
pub const NO_NODE: NodeId = NO_CELL;

#[derive(Clone, Copy, Debug)]
pub struct NodeData {
    pub cell_id: CellId,
    pub pos: (f32, f32, f32),
}

/// Config-derived scalars shared by the node placement passes.
pub struct PlacementParams {
    pub clearance_cells: i32,
    pub step_cells: i32,
    pub voxel_size: f32,
    pub node_spacing_m: f32,
    pub wall_clearance_m: f32,
    pub wall_buffer_m: f32,
    pub wall_buffer_weight: f32,
    pub step_penalty_weight: f32,
}

/// A relocation may not land within this fraction of the node spacing of
/// another node.
const RELOCATION_CROWDING_FRAC: f32 = 0.9;

const NO_NODE_AT: u32 = u32::MAX;

/// The graph nodes with where they are: the node each cell holds, and the
/// node cells by spacing bin, so placement and relocation never walk the list.
#[derive(Default)]
pub struct Nodes {
    list: Vec<NodeData>,
    /// Index into the list per cell slot, NO_NODE_AT for none.
    at: Vec<u32>,
    bins: AHashMap<(i32, i32, i32), Vec<CellId>>,
}

impl std::ops::Deref for Nodes {
    type Target = [NodeData];

    fn deref(&self) -> &[NodeData] {
        &self.list
    }
}

fn bin_of(c: VoxelKey, params: &PlacementParams) -> (i32, i32, i32) {
    let bin_cells = ((params.node_spacing_m / params.voxel_size) as i32).max(1);
    (
        c.0.div_euclid(bin_cells),
        c.1.div_euclid(bin_cells),
        c.2.div_euclid(bin_cells),
    )
}

impl Nodes {
    pub fn clear(&mut self) {
        self.list.clear();
        self.at.clear();
        self.bins.clear();
    }

    /// Whether a cell holds a node.
    pub fn has(&self, cell: CellId) -> bool {
        self.node_at(cell).is_some()
    }

    /// The list index of the node a cell holds.
    pub fn node_at(&self, cell: CellId) -> Option<usize> {
        self.at
            .get(cell as usize)
            .copied()
            .filter(|&i| i != NO_NODE_AT)
            .map(|i| i as usize)
    }

    /// Whether a node sits within `radius_m` of a coordinate. Radii up to one
    /// spacing are exact.
    pub fn near(
        &self,
        cells: &SurfaceCells,
        params: &PlacementParams,
        coord: VoxelKey,
        radius_m: f32,
    ) -> bool {
        let r_sq = (radius_m as f64) * (radius_m as f64);
        let v = params.voxel_size as f64;
        let (bx, by, bz) = bin_of(coord, params);
        for dbx in -1..=1 {
            for dby in -1..=1 {
                for dbz in -1..=1 {
                    let Some(nearby) = self.bins.get(&(bx + dbx, by + dby, bz + dbz)) else {
                        continue;
                    };
                    for &n_id in nearby {
                        let n = cells.coord(n_id);
                        let dx = (coord.0 - n.0) as f64 * v;
                        let dy = (coord.1 - n.1) as f64 * v;
                        let dz = (coord.2 - n.2) as f64 * v;
                        if dx * dx + dy * dy + dz * dz <= r_sq {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Record where the node at a list index sits.
    fn index(&mut self, cells: &SurfaceCells, params: &PlacementParams, i: usize) {
        let cell = self.list[i].cell_id;
        if self.at.len() < cells.slot_capacity() {
            self.at.resize(cells.slot_capacity(), NO_NODE_AT);
        }
        self.at[cell as usize] = i as u32;
        let bin = bin_of(cells.coord(cell), params);
        self.bins.entry(bin).or_default().push(cell);
    }

    /// Forget the node a cell held. The coordinate is given, since a dead
    /// cell's slot may already belong to another cell.
    fn unindex(&mut self, params: &PlacementParams, cell: CellId, coord: VoxelKey) {
        if let Some(slot) = self.at.get_mut(cell as usize) {
            *slot = NO_NODE_AT;
        }
        let bin = bin_of(coord, params);
        if let Some(near) = self.bins.get_mut(&bin) {
            near.retain(|&c| c != cell);
            if near.is_empty() {
                self.bins.remove(&bin);
            }
        }
    }

    pub(crate) fn push(&mut self, cells: &SurfaceCells, params: &PlacementParams, node: NodeData) {
        self.list.push(node);
        self.index(cells, params, self.list.len() - 1);
    }

    /// Drop the node at a list index.
    pub(crate) fn swap_remove(
        &mut self,
        cells: &SurfaceCells,
        params: &PlacementParams,
        i: usize,
    ) -> NodeData {
        let gone = self.list.swap_remove(i);
        if gone.cell_id != NO_CELL {
            self.unindex(params, gone.cell_id, cells.coord(gone.cell_id));
        }
        if let Some(moved) = self.list.get(i).filter(|n| n.cell_id != NO_CELL) {
            self.at[moved.cell_id as usize] = i as u32;
        }
        gone
    }

    /// A node list as it stands when some of its cells have just died.
    #[cfg(test)]
    pub(crate) fn from_list(
        cells: &SurfaceCells,
        list: Vec<NodeData>,
        params: &PlacementParams,
    ) -> Self {
        let mut nodes = Self {
            list,
            ..Self::default()
        };
        for i in 0..nodes.list.len() {
            let cell = nodes.list[i].cell_id;
            if cell != NO_CELL && cells.is_live(cell) {
                nodes.index(cells, params, i);
            }
        }
        nodes
    }
}

/// Move each dead node to a live neighbor cell, or drop it when none is
/// free or the move would crowd another node. Returns the ids the dead nodes
/// gave up, so their edges can go.
pub fn relocate_dead_nodes(
    cells: &SurfaceCells,
    lookup: &SurfaceLookup,
    nodes: &mut Nodes,
    dead_nodes: &[(usize, VoxelKey)],
    params: &PlacementParams,
) -> Vec<NodeId> {
    let mut gone: Vec<NodeId> = Vec::with_capacity(dead_nodes.len());
    let step = params.step_cells;
    let crowd_m = RELOCATION_CROWDING_FRAC * params.node_spacing_m;
    // Dead nodes leave the index before any relocates. The old id is what
    // the edges still reference.
    for &(ni, coord) in dead_nodes {
        let old = nodes.list[ni].cell_id;
        gone.push(old);
        nodes.unindex(params, old, coord);
    }
    let mut unplaced: Vec<usize> = Vec::new();
    for &(ni, (ix, iy, iz)) in dead_nodes {
        let mut best: Option<(i32, CellId, VoxelKey)> = None;
        for (dx, dy) in [(0_i32, 0_i32), (-1, 0), (1, 0), (0, -1), (0, 1)] {
            let Some(zs) = lookup.get(&(ix + dx, iy + dy)) else {
                continue;
            };
            for &nz in zs {
                let dz = (nz - iz).abs();
                if dz > step {
                    continue;
                }
                let k = (ix + dx, iy + dy, nz);
                let Some(id) = cells.id(k) else {
                    continue;
                };
                if nodes.has(id) {
                    continue;
                }
                let rank = 2 * dz + dx.abs() + dy.abs();
                if best.is_none_or(|(r, _, _)| rank < r) {
                    best = Some((rank, id, k));
                }
            }
        }
        match best {
            Some((_, id, k)) if !nodes.near(cells, params, k, crowd_m) => {
                nodes.list[ni] = NodeData {
                    cell_id: id,
                    pos: surface_point_xyz(k.0, k.1, k.2, params.voxel_size),
                };
                nodes.index(cells, params, ni);
            }
            _ => {
                nodes.list[ni].cell_id = NO_CELL;
                unplaced.push(ni);
            }
        }
    }
    // Descending order keeps swap_remove from moving a node still to go.
    unplaced.sort_unstable_by(|a, b| b.cmp(a));
    for ni in unplaced {
        nodes.swap_remove(cells, params, ni);
    }
    gone
}

/// Place graph nodes across the surface, spaced out and biased away from walls.
pub fn place_nodes(
    cells: &mut SurfaceCells,
    by_col: &ColumnIz,
    params: &PlacementParams,
    state: &mut DijkstraState,
    scratch: &mut NodeScratch,
    nodes: &mut Nodes,
) {
    nodes.clear();
    if cells.is_empty() {
        return;
    }

    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_cells(
        cells,
        by_col,
        params.clearance_cells,
        params.step_cells,
        &mut wall_seeds,
    );
    dijkstra(cells, &wall_seeds, state, Weight::Base);

    // Floor is the hard clearance. NMS already prefers the clearest cells.
    let node_floor = params.wall_clearance_m;
    let candidates: Vec<CellId> = cells
        .ids()
        .filter(|&id| state.dist[id as usize] >= node_floor)
        .collect();
    place_from_candidates(cells, candidates, &state.dist, params, nodes);

    let live: Vec<CellId> = cells.ids().collect();
    let clusters = window_clusters(cells, &live, &mut scratch.seen);
    ensure_node_per_component(cells, &state.dist, params, &clusters, None, nodes);

    apply_wall_safe_penalty(cells, &state.dist, params);
}

/// Thin candidates with NMS, clearest-first, against every node already
/// placed, and append the survivors as nodes.
fn place_from_candidates(
    cells: &SurfaceCells,
    mut candidates: Vec<CellId>,
    dist: &[f32],
    params: &PlacementParams,
    nodes: &mut Nodes,
) {
    candidates.par_sort_unstable_by(|&a, &b| {
        dist[b as usize]
            .total_cmp(&dist[a as usize])
            .then(cells.coord(a).cmp(&cells.coord(b)))
    });
    for id in candidates {
        let (ix, iy, iz) = cells.coord(id);
        if nodes.near(cells, params, (ix, iy, iz), params.node_spacing_m) {
            continue;
        }
        nodes.push(
            cells,
            params,
            NodeData {
                cell_id: id,
                pos: surface_point_xyz(ix, iy, iz, params.voxel_size),
            },
        );
    }
}

/// Repair the wall distance field inside the window.
pub fn wall_distances_region(
    cells: &SurfaceCells,
    by_col: &ColumnIz,
    params: &PlacementParams,
    window: &RepairWindow,
    wall_state: &mut DijkstraState,
) {
    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_in_window(
        cells,
        by_col,
        params.clearance_cells,
        params.step_cells,
        window.cells(),
        &mut wall_seeds,
    );
    dijkstra_clusters(
        cells,
        &wall_seeds,
        window.clusters(),
        window.index(),
        wall_state,
        Weight::Base,
        Frontier::Any,
    );
}

/// Regional counterpart to the placement in place_nodes, run on fresh wall
/// distances. Nodes are sticky: survivors keep their placement, and new nodes
/// are placed only on cells added by this update, spaced by NMS against every
/// existing node. Returns the ids of the nodes it dropped.
pub fn place_nodes_region(
    cells: &SurfaceCells,
    params: &PlacementParams,
    added: &[CellId],
    window: &RepairWindow,
    wall_dist: &[f32],
    voronoi: &DijkstraState,
    nodes: &mut Nodes,
) -> Vec<NodeId> {
    let dropped = drop_walled_in_nodes(cells, params, window.cells(), wall_dist, nodes);

    // New nodes only in comfortably open freshly-seen space: transient fringe
    // cells near walls must not spawn graph structure every frame.
    let spawn_floor = params.wall_clearance_m + SPAWN_FLOOR_BUFFER_FRAC * params.wall_buffer_m;
    let candidates: Vec<CellId> = added
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id) && wall_dist[id as usize] >= spawn_floor)
        .collect();
    place_from_candidates(cells, candidates, wall_dist, params, nodes);
    ensure_node_per_component(
        cells,
        wall_dist,
        params,
        window.clusters(),
        Some(Beyond {
            cluster_index: window.index(),
            owner: voronoi,
        }),
        nodes,
    );
    dropped
}

/// Drop the window nodes whose fresh wall distance marks their cell
/// impassable. The field is stale outside the window, so only in-window
/// nodes are judged by it.
fn drop_walled_in_nodes(
    cells: &SurfaceCells,
    params: &PlacementParams,
    window: &[CellId],
    wall_dist: &[f32],
    nodes: &mut Nodes,
) -> Vec<NodeId> {
    let mut doomed: Vec<usize> = window
        .iter()
        .filter(|&&w| wall_dist[w as usize] < params.wall_clearance_m)
        .filter_map(|&w| nodes.node_at(w))
        .collect();
    doomed.sort_unstable_by(|a, b| b.cmp(a));
    doomed.dedup();
    doomed
        .into_iter()
        .map(|i| nodes.swap_remove(cells, params, i).cell_id)
        .collect()
}

/// Wall-adjacency over a cell subset, matching collect_wall_adjacent_cells.
fn collect_wall_adjacent_in_window(
    cells: &SurfaceCells,
    by_col: &ColumnIz,
    clearance_cells: i32,
    step_cells: i32,
    window: &[CellId],
    out: &mut Vec<CellId>,
) {
    *out = window
        .par_iter()
        .filter(|&&id| {
            cells.is_live(id) && real_wall_adjacent(cells, by_col, id, clearance_cells, step_cells)
        })
        .copied()
        .collect();
}

/// Empty columns a gap may span before it counts as a real edge, not a hole.
pub(crate) const HOLE_SPAN_CELLS: i32 = 4;

/// Smallest unserved component the fallback seeding still gives a node.
const MIN_COMPONENT_CELLS: u32 = 4;

/// Fraction of the wall buffer added to the clearance before a fresh cell may
/// spawn a node.
const SPAWN_FLOOR_BUFFER_FRAC: f32 = 0.5;

/// True when any missing 4-neighbor opens onto a real edge rather than a hole.
fn real_wall_adjacent(
    cells: &SurfaceCells,
    by_col: &ColumnIz,
    id: CellId,
    clearance_cells: i32,
    step_cells: i32,
) -> bool {
    let (cx, cy, cz) = cells.coord(id);
    let mut mask: u8 = 0;
    for e in cells.neighbors(id) {
        let (nx, ny, _) = cells.coord(e.dest);
        mask |= match (nx - cx, ny - cy) {
            (-1, 0) => 1,
            (1, 0) => 2,
            (0, -1) => 4,
            (0, 1) => 8,
            _ => 0,
        };
    }
    for (dx, dy, bit) in NEIGHBORS_4 {
        if mask & bit != 0 {
            continue; // a surface neighbor already connects this direction
        }
        if edge_in_direction(by_col, cx, cy, cz, dx, dy, clearance_cells, step_cells) {
            return true; // wall, cliff, or drop in this direction
        }
    }
    false
}

/// True when the missing neighbor in this direction is a real edge (wall, cliff,
/// or drop) rather than a small sensor hole that surface bridges within the span.
#[allow(clippy::too_many_arguments)]
fn edge_in_direction(
    by_col: &ColumnIz,
    cx: i32,
    cy: i32,
    cz: i32,
    dx: i32,
    dy: i32,
    clearance_cells: i32,
    step_cells: i32,
) -> bool {
    for k in 1..=HOLE_SPAN_CELLS {
        let (nx, ny) = (cx + dx * k, cy + dy * k);
        let Some(zs) = by_col.get((nx, ny)) else {
            continue; // empty column: keep scanning across the hole
        };
        let reachable = zs.iter().any(|&oz| {
            (oz - cz).abs() <= step_cells && is_standable(nx, ny, oz, by_col, clearance_cells)
        });
        return !reachable;
    }
    true
}

/// Rescale edge costs for the window and its neighbors, whose wall distance may
/// have changed. Idempotent via base_cost.
pub fn apply_wall_safe_penalty_region(
    cells: &mut SurfaceCells,
    dist: &[f32],
    params: &PlacementParams,
    window: &[CellId],
    scratch: &mut NodeScratch,
) {
    // The window and its boundary, deduped via the dense seen mask.
    scratch.ensure_capacity(cells.slot_capacity());
    let mut affected: Vec<CellId> = Vec::with_capacity(window.len() * 2);
    {
        let seen = &mut scratch.seen;
        for &w in window {
            if !seen[w as usize] {
                seen[w as usize] = true;
                affected.push(w);
            }
            for e in cells.neighbors(w) {
                if !seen[e.dest as usize] {
                    seen[e.dest as usize] = true;
                    affected.push(e.dest);
                }
            }
        }
    }
    for &id in &affected {
        scratch.seen[id as usize] = false;
    }
    for id in affected {
        scale_edges(cells.edges_mut(id), id, dist, params);
    }
}

/// Wall-adjacent cells over the whole graph. Falls back to a single cell so a
/// fully-enclosed map still seeds the wall-distance field.
fn collect_wall_adjacent_cells(
    cells: &SurfaceCells,
    by_col: &ColumnIz,
    clearance_cells: i32,
    step_cells: i32,
    out: &mut Vec<CellId>,
) {
    let ids: Vec<CellId> = cells.ids().collect();
    *out = ids
        .par_iter()
        .filter(|&&id| real_wall_adjacent(cells, by_col, id, clearance_cells, step_cells))
        .copied()
        .collect();
    if out.is_empty() {
        if let Some(c) = cells.ids().next() {
            out.push(c);
        }
    }
}

/// Scale each edge by its endpoints' average wall penalty and add the step
/// penalty. Unreached cells (dist +INFINITY) collapse the wall penalty to 1.0.
fn apply_wall_safe_penalty(cells: &mut SurfaceCells, dist: &[f32], params: &PlacementParams) {
    let mut edge_lists: Vec<(CellId, &mut Vec<Edge>)> = cells.iter_edges_mut().collect();
    edge_lists
        .par_iter_mut()
        .for_each(|(src, edges)| scale_edges(edges, *src, dist, params));
}

/// Rescale one cell's outgoing edges from base_cost. Idempotent, so a regional
/// repass cannot compound the penalty.
#[inline]
fn scale_edges(edges: &mut [Edge], src: CellId, dist: &[f32], params: &PlacementParams) {
    let penalty = |d: f32| {
        penalty_of(
            d,
            params.wall_clearance_m,
            params.wall_buffer_m,
            params.wall_buffer_weight,
        )
    };
    let pu = penalty(dist[src as usize]);
    for edge in edges.iter_mut() {
        let pv = penalty(dist[edge.dest as usize]);
        edge.cost = edge.base_cost * (pu + pv) / 2.0 + params.step_penalty_weight * edge.rise;
    }
}

/// Lateral wall multiplier: infinite inside clearance, ramping convexly from
/// 1 + weight at the clearance edge down to 1 at clearance_m + buffer_m.
#[inline]
pub(crate) fn penalty_of(d: f32, clearance_m: f32, buffer_m: f32, weight: f32) -> f32 {
    if d < clearance_m {
        return f32::INFINITY;
    }
    let outer = clearance_m + buffer_m;
    if d >= outer {
        return 1.0;
    }
    let band = buffer_m.max(1e-3);
    let t = (outer - d) / band; // 0 at the outer edge, 1 at the clearance edge
    1.0 + weight * t * t
}

/// The surface beyond a repair window: which cells are outside it, and the
/// node each of them was last owned by.
#[derive(Clone, Copy)]
pub struct Beyond<'a> {
    pub cluster_index: &'a ClusterIndex,
    pub owner: &'a DijkstraState,
}

impl Beyond<'_> {
    /// Whether a cell outside the window is reachable from a live node along
    /// a cached chain that stays outside the window, so this repair cannot
    /// cut it.
    fn has_node(&self, cells: &SurfaceCells, id: CellId, nodes: &Nodes) -> bool {
        chain_source(cells, self.owner, self.cluster_index, id, &|c| nodes.has(c)).is_some()
    }
}

/// Seed a node in every cluster the clearance floor left empty, so a thin or
/// sparse component is still reachable.
fn ensure_node_per_component(
    cells: &SurfaceCells,
    dist: &[f32],
    params: &PlacementParams,
    clusters: &[Vec<CellId>],
    beyond: Option<Beyond>,
    nodes: &mut Nodes,
) {
    // A cluster is served when it holds or borders a node. Fragments below
    // the size floor are sensor noise, not places to grow graph structure.
    let seeds: Vec<CellId> = {
        let nodes = &*nodes;
        clusters
            .par_iter()
            .filter_map(|cluster| {
                if (cluster.len() as u32) < MIN_COMPONENT_CELLS {
                    return None;
                }
                let served = cluster.iter().any(|&id| {
                    nodes.has(id)
                        || cells.neighbors(id).iter().any(|e| {
                            nodes.has(e.dest)
                                || beyond.is_some_and(|b| b.has_node(cells, e.dest, nodes))
                        })
                });
                if served {
                    return None;
                }
                cluster.iter().copied().reduce(|best, id| {
                    if is_clearer(cells, dist, id, best) {
                        id
                    } else {
                        best
                    }
                })
            })
            .collect()
    };
    for id in seeds {
        let (ix, iy, iz) = cells.coord(id);
        nodes.push(
            cells,
            params,
            NodeData {
                cell_id: id,
                pos: surface_point_xyz(ix, iy, iz, params.voxel_size),
            },
        );
    }
}

/// Better fallback seed: farther from a wall, ties broken by coordinate.
fn is_clearer(cells: &SurfaceCells, dist: &[f32], a: CellId, b: CellId) -> bool {
    match dist[a as usize].total_cmp(&dist[b as usize]) {
        Ordering::Greater => true,
        Ordering::Less => false,
        Ordering::Equal => cells.coord(a) < cells.coord(b),
    }
}

/// Reusable dense visited mask for the window walks, left all-false between
/// calls.
#[derive(Default)]
pub struct NodeScratch {
    pub(crate) seen: Vec<bool>,
}

impl NodeScratch {
    pub(crate) fn ensure_capacity(&mut self, n: usize) {
        if self.seen.len() < n {
            self.seen.resize(n, false);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup, SurfaceLookup};
    use crate::dijkstra::ClusterIndex;

    const VOXEL: f32 = 0.1;

    fn open_patch(ix0: i32, iy0: i32, size: i32) -> Vec<VoxelKey> {
        let mut c = Vec::new();
        for dx in 0..size {
            for dy in 0..size {
                c.push((ix0 + dx, iy0 + dy, 0));
            }
        }
        c
    }

    fn params(wall_clearance_m: f32, step_penalty_weight: f32) -> PlacementParams {
        PlacementParams {
            clearance_cells: 5,
            step_cells: 2,
            voxel_size: VOXEL,
            node_spacing_m: 1.0,
            wall_clearance_m,
            wall_buffer_m: 0.3,
            wall_buffer_weight: 1.0,
            step_penalty_weight,
        }
    }

    fn build_cells(surface: &[VoxelKey], step_cells: i32) -> SurfaceCells {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(surface, &mut lookup);
        let mut sc = SurfaceCells::default();
        build_surface_cells(&mut sc, &lookup, VOXEL, step_cells);
        sc
    }

    #[test]
    fn relocation_moves_node_onto_replacement_cell() {
        // The node's cell (5, 5, 0) died and the surviving surface is the
        // same column one voxel up. The node must move there, not vanish.
        let surface = [(5, 5, 1)];
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&surface, &mut lookup);
        let sc = build_cells(&surface, 2);
        let list = vec![NodeData {
            cell_id: 0,
            pos: surface_point_xyz(5, 5, 0, VOXEL),
        }];
        let mut nodes = Nodes::from_list(&sc, list, &params(0.0, 0.0));
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0))],
            &params(0.0, 0.0),
        );
        let id = sc.id((5, 5, 1)).unwrap();
        assert_eq!(nodes[0].cell_id, id, "node must move to the raised cell");
        assert!(nodes.has(id));
        assert_eq!(nodes[0].pos, surface_point_xyz(5, 5, 1, VOXEL));
    }

    #[test]
    fn crowded_relocation_drops_the_node() {
        // A live node already sits next to the dead node's only free
        // relocation target, so the relocation is rejected and the node
        // dropped.
        let surface = [(5, 5, 1), (5, 6, 0)];
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&surface, &mut lookup);
        let sc = build_cells(&surface, 2);
        let live_id = sc.id((5, 6, 0)).unwrap();
        let list = vec![
            NodeData {
                cell_id: NO_CELL,
                pos: surface_point_xyz(5, 5, 0, VOXEL),
            },
            NodeData {
                cell_id: live_id,
                pos: surface_point_xyz(5, 6, 0, VOXEL),
            },
        ];
        let mut nodes = Nodes::from_list(&sc, list, &params(0.0, 0.0));
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0))],
            &params(0.0, 0.0),
        );
        assert_eq!(nodes.len(), 1, "crowded relocation must drop the node");
        assert_eq!(nodes[0].cell_id, live_id, "live node untouched");
        assert!(nodes.has(live_id));
    }

    #[test]
    fn co_relocating_nodes_crowd_each_other() {
        // Two nodes die in the same update with adjacent relocation targets.
        // The second relocation must see the first and die instead of landing
        // within the crowding radius of it.
        let surface = [(5, 5, 1), (5, 6, 1)];
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&surface, &mut lookup);
        let sc = build_cells(&surface, 2);
        let list = vec![
            NodeData {
                cell_id: 0,
                pos: surface_point_xyz(5, 5, 0, VOXEL),
            },
            NodeData {
                cell_id: 1,
                pos: surface_point_xyz(5, 6, 0, VOXEL),
            },
        ];
        let mut nodes = Nodes::from_list(&sc, list, &params(0.0, 0.0));
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0)), (1, (5, 6, 0))],
            &params(0.0, 0.0),
        );
        let relocated = sc.id((5, 5, 1)).unwrap();
        assert_eq!(nodes[0].cell_id, relocated, "first relocation lands");
        assert!(nodes.has(relocated));
        assert_eq!(
            nodes.len(),
            1,
            "second relocation must be crowded out by the first"
        );
    }

    #[test]
    fn spawn_floor_blocks_new_nodes_near_walls() {
        // A cell inside the spawn floor must not spawn a node, while an open
        // cell that clears it must.
        let surface = open_patch(0, 0, 10);
        let sc = build_cells(&surface, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let corner = sc.id((0, 0, 0)).unwrap();
        let list = vec![NodeData {
            cell_id: corner,
            pos: surface_point_xyz(0, 0, 0, VOXEL),
        }];
        let window: Vec<CellId> = sc.ids().collect();
        let near_wall = sc.id((1, 5, 0)).unwrap();
        let open = sc.id((5, 5, 0)).unwrap();
        let p = PlacementParams {
            node_spacing_m: 0.2,
            ..params(0.0, 0.0)
        };
        let mut nodes = Nodes::from_list(&sc, list, &p);
        let mut cluster_index = ClusterIndex::default();
        let repair = RepairWindow::new(&sc, &window, &mut cluster_index, &mut scratch.seen);
        wall_distances_region(&sc, &ColumnIz::default(), &p, &repair, &mut state);
        place_nodes_region(
            &sc,
            &p,
            &[near_wall, open],
            &repair,
            &state.dist,
            &DijkstraState::default(),
            &mut nodes,
        );
        let ids: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();
        assert!(ids.contains(&open), "open added cell must spawn a node");
        assert!(
            !ids.contains(&near_wall),
            "added cell under the spawn floor must not spawn a node"
        );
    }

    #[test]
    fn tiny_component_gets_no_fallback_node() {
        // A 3-cell fragment is below MIN_COMPONENT_CELLS, so only the 8-cell
        // strip earns a fallback node.
        let mut cells_in: Vec<VoxelKey> = (0..8).map(|ix| (ix, 0, 0)).collect();
        cells_in.extend((0..3).map(|ix| (ix, 20, 0)));
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Nodes::default();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.5, 0.0),
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        assert_eq!(nodes.len(), 1, "only the big strip gets a node");
        assert_eq!(sc.coord(nodes[0].cell_id).1, 0);
    }

    #[test]
    fn open_patch_places_at_least_one_node() {
        let mut sc = build_cells(&open_patch(0, 0, 10), 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Nodes::default();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.0, 0.0),
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        assert!(!nodes.is_empty());
        for n in nodes.iter() {
            let (ix, iy, _) = sc.coord(n.cell_id);
            assert!((0..10).contains(&ix) && (0..10).contains(&iy));
        }
    }

    #[test]
    fn each_disconnected_component_gets_a_node() {
        // Two 1-wide strips far apart: every cell is wall-adjacent so none
        // clears the 0.5 m clearance floor, yet each disconnected strip must
        // still get exactly one node.
        let mut cells_in: Vec<VoxelKey> = (0..8).map(|ix| (ix, 0, 0)).collect();
        cells_in.extend((0..8).map(|ix| (ix, 20, 0)));
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Nodes::default();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.5, 0.0),
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        assert_eq!(
            nodes.len(),
            2,
            "each disconnected component needs its own node"
        );
        let ys: Vec<i32> = nodes.iter().map(|n| sc.coord(n.cell_id).1).collect();
        assert!(ys.contains(&0) && ys.contains(&20));
    }

    #[test]
    fn nms_enforces_spacing() {
        let mut cells_in = open_patch(0, 0, 10);
        cells_in.extend(open_patch(20, 0, 10));
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Nodes::default();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.0, 0.0),
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        assert!(nodes.len() >= 2);
        for i in 0..nodes.len() {
            for j in (i + 1)..nodes.len() {
                let a = nodes[i].pos;
                let b = nodes[j].pos;
                let dx = a.0 - b.0;
                let dy = a.1 - b.1;
                let dz = a.2 - b.2;
                let d_sq = dx * dx + dy * dy + dz * dz;
                assert!(d_sq > 1.0 * 1.0 - 1e-4);
            }
        }
    }

    #[test]
    fn penalty_ramps_across_buffer_zone() {
        // clearance 0.1, soft zone 0.4 wide, so the outer edge is at 0.5.
        let (clearance, buffer, w) = (0.1, 0.4, 4.0);
        assert!(penalty_of(0.05, clearance, buffer, w).is_infinite());
        assert!((penalty_of(0.1, clearance, buffer, w) - 5.0).abs() < 1e-6);
        assert!((penalty_of(0.5, clearance, buffer, w) - 1.0).abs() < 1e-6);
        assert!((penalty_of(1.0, clearance, buffer, w) - 1.0).abs() < 1e-6);
        assert!((penalty_of(0.3, clearance, buffer, w) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn wall_penalty_doubles_cost_at_the_wall() {
        // On a 1-wide strip every cell is wall-adjacent (d = 0), so with zero
        // clearance the ramp peaks at 2 and edge cost is twice the geometric.
        let cells_in: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Nodes::default();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.0, 0.0),
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        let id = sc.id((5, 0, 0)).unwrap();
        assert!((sc.neighbors(id)[0].cost - 2.0 * VOXEL).abs() < 1e-5);
    }

    #[test]
    fn step_penalty_adds_to_vertical_edges() {
        // A 2-cell rise (0.2 m) between adjacent cells. With weight 10 the edge
        // gains 10 * 0.2 = 2.0 on top of its geometric and wall cost.
        let cells_in: Vec<VoxelKey> = vec![(0, 0, 0), (1, 0, 2), (2, 0, 2)];
        let cost_with = |step_weight: f32| {
            let mut sc = build_cells(&cells_in, 2);
            let mut state = DijkstraState::default();
            let mut scratch = NodeScratch::default();
            let mut nodes = Nodes::default();
            place_nodes(
                &mut sc,
                &ColumnIz::default(),
                &params(0.0, step_weight),
                &mut state,
                &mut scratch,
                &mut nodes,
            );
            let id = sc.id((0, 0, 0)).unwrap();
            sc.neighbors(id)
                .iter()
                .find(|e| sc.coord(e.dest) == (1, 0, 2))
                .unwrap()
                .cost
        };
        assert!(
            (cost_with(10.0) - cost_with(0.0) - 10.0 * 0.2).abs() < 1e-4,
            "step penalty must add weight * rise"
        );
    }

    /// An isolated cell whose four neighbor columns each hold an occupied,
    /// standable voxel within a step: sparse real surface, not a wall.
    #[test]
    fn sparse_step_neighbors_do_not_seed_a_wall() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        for col in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            by_col.add((col.0, col.1, 0));
        }
        assert!(!real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }

    /// The same cell with one empty neighbor column: a cliff edge that must seed
    /// the wall-clearance field even though the other three sides are steps.
    #[test]
    fn empty_neighbor_column_seeds_as_drop() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        for col in [(1, 0), (0, -1), (0, 1)] {
            by_col.add((col.0, col.1, 0));
        }
        assert!(real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }

    /// An occupied neighbor that rises well beyond the step threshold reads as a
    /// wall, not a traversable step.
    #[test]
    fn step_taller_than_threshold_seeds_as_wall() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        by_col.add((1, 0, 10));
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.add((col.0, col.1, 0));
        }
        assert!(real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }

    /// Immediate neighbor empty but surface resumes within the span at the same
    /// height: a sensor hole to cross, not an edge.
    #[test]
    fn small_hole_bridged_by_surface_is_not_an_edge() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        by_col.add((2, 0, 0));
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.add((col.0, col.1, 0));
        }
        assert!(!real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }

    /// Empty for more than the span before surface resumes: a real cliff.
    #[test]
    fn gap_wider_than_span_still_seeds() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        by_col.add((10, 0, 0));
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.add((col.0, col.1, 0));
        }
        assert!(real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }
}
