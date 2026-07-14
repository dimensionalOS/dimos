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

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells, NO_CELL};
use crate::dijkstra::{dijkstra, dijkstra_region, DijkstraState, Weight};
use crate::surfaces::{is_standable, ColumnIz};
use crate::voxel::{surface_point_xyz, VoxelKey};

const NEIGHBORS_4: [(i32, i32, u8); 4] = [(-1, 0, 1), (1, 0, 2), (0, -1, 4), (0, 1, 8)];

#[derive(Clone, Copy, Debug)]
pub struct NodeData {
    pub cell_id: CellId,
    pub pos: (f32, f32, f32),
}

/// Place graph nodes across the surface, spaced out and biased away from walls.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes(
    cells: &mut SurfaceCells,
    by_col: &ColumnIz,
    clearance_cells: i32,
    step_cells: i32,
    voxel_size: f32,
    node_spacing_m: f32,
    wall_clearance_m: f32,
    wall_buffer_m: f32,
    wall_buffer_weight: f32,
    step_penalty_weight: f32,
    state: &mut DijkstraState,
    scratch: &mut NodeScratch,
    out_nodes: &mut Vec<NodeData>,
) {
    out_nodes.clear();
    if cells.is_empty() {
        return;
    }

    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_cells(cells, by_col, clearance_cells, step_cells, &mut wall_seeds);
    dijkstra(cells, &wall_seeds, state, Weight::Base);

    // Floor is the hard clearance. NMS already prefers the clearest cells.
    let node_floor = wall_clearance_m;
    let candidates: Vec<CellId> = cells
        .ids()
        .filter(|&id| state.dist[id as usize] >= node_floor)
        .collect();
    place_from_candidates(
        cells,
        candidates,
        &state.dist,
        &[],
        voxel_size,
        node_spacing_m,
        out_nodes,
    );

    let domain: Vec<CellId> = cells.ids().collect();
    ensure_node_per_component(cells, &state.dist, voxel_size, &domain, scratch, out_nodes);

    apply_wall_safe_penalty(
        cells,
        &state.dist,
        wall_clearance_m,
        wall_buffer_m,
        wall_buffer_weight,
        step_penalty_weight,
    );
}

/// Thin candidates with NMS, clearest-first, against the seed nodes.
fn place_from_candidates(
    cells: &SurfaceCells,
    mut candidates: Vec<CellId>,
    dist: &[f32],
    seeds: &[CellId],
    voxel_size: f32,
    node_spacing_m: f32,
    out_nodes: &mut Vec<NodeData>,
) {
    candidates.par_sort_unstable_by(|&a, &b| {
        dist[b as usize]
            .total_cmp(&dist[a as usize])
            .then(cells.coord(a).cmp(&cells.coord(b)))
    });
    let survivors = nms_grid(cells, &candidates, seeds, voxel_size, node_spacing_m);
    out_nodes.reserve(survivors.len());
    for &id in &survivors {
        let (ix, iy, iz) = cells.coord(id);
        out_nodes.push(NodeData {
            cell_id: id,
            pos: surface_point_xyz(ix, iy, iz, voxel_size),
        });
    }
}

/// Regional counterpart to place_nodes: recompute the wall-distance field and
/// node placement inside the window, keeping cached nodes outside it as NMS
/// seeds so spacing holds across the seam.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes_region(
    cells: &mut SurfaceCells,
    by_col: &ColumnIz,
    clearance_cells: i32,
    step_cells: i32,
    window: &AHashSet<CellId>,
    voxel_size: f32,
    node_spacing_m: f32,
    wall_clearance_m: f32,
    wall_buffer_m: f32,
    wall_buffer_weight: f32,
    step_penalty_weight: f32,
    wall_state: &mut DijkstraState,
    scratch: &mut NodeScratch,
    nodes: &mut Vec<NodeData>,
) {
    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_in_window(
        cells,
        by_col,
        clearance_cells,
        step_cells,
        window,
        &mut wall_seeds,
    );
    dijkstra_region(cells, &wall_seeds, window, wall_state, Weight::Base);

    nodes.retain(|n| cells.is_live(n.cell_id) && !window.contains(&n.cell_id));
    let kept: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();

    let node_floor = wall_clearance_m;
    let candidates: Vec<CellId> = window
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id) && wall_state.dist[id as usize] >= node_floor)
        .collect();
    place_from_candidates(
        cells,
        candidates,
        &wall_state.dist,
        &kept,
        voxel_size,
        node_spacing_m,
        nodes,
    );

    let domain: Vec<CellId> = window
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id))
        .collect();
    ensure_node_per_component(cells, &wall_state.dist, voxel_size, &domain, scratch, nodes);

    apply_wall_safe_penalty_region(
        cells,
        &wall_state.dist,
        wall_clearance_m,
        wall_buffer_m,
        wall_buffer_weight,
        step_penalty_weight,
        window,
        scratch,
    );
}

/// Wall-adjacency over a cell subset, matching collect_wall_adjacent_cells.
fn collect_wall_adjacent_in_window(
    cells: &SurfaceCells,
    by_col: &ColumnIz,
    clearance_cells: i32,
    step_cells: i32,
    window: &AHashSet<CellId>,
    out: &mut Vec<CellId>,
) {
    let win: Vec<CellId> = window.iter().copied().collect();
    *out = win
        .par_iter()
        .filter(|&&id| {
            cells.is_live(id) && real_wall_adjacent(cells, by_col, id, clearance_cells, step_cells)
        })
        .copied()
        .collect();
}

/// Empty columns a gap may span before it counts as a real edge, not a hole.
const HOLE_SPAN_CELLS: i32 = 4;

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
        let Some(zs) = by_col.get(&(nx, ny)) else {
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
#[allow(clippy::too_many_arguments)]
fn apply_wall_safe_penalty_region(
    cells: &mut SurfaceCells,
    dist: &[f32],
    clearance_m: f32,
    buffer_m: f32,
    buffer_weight: f32,
    step_weight: f32,
    window: &AHashSet<CellId>,
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
    for &id in &affected {
        scale_edges(
            cells.edges_mut(id),
            id,
            dist,
            clearance_m,
            buffer_m,
            buffer_weight,
            step_weight,
        );
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

/// Keep nodes at least node_spacing_m apart. Seeds suppress nearby candidates
/// without being emitted, so regional re-placement respects cached nodes
/// outside the window.
fn nms_grid(
    cells: &SurfaceCells,
    candidates_sorted: &[CellId],
    seeds: &[CellId],
    voxel_size: f32,
    node_spacing_m: f32,
) -> Vec<CellId> {
    let bin_size = ((node_spacing_m / voxel_size) as i32).max(1);
    let r_sq = (node_spacing_m as f64) * (node_spacing_m as f64);
    let v = voxel_size as f64;
    let bin_of = |c: VoxelKey| {
        (
            c.0.div_euclid(bin_size),
            c.1.div_euclid(bin_size),
            c.2.div_euclid(bin_size),
        )
    };

    let mut bins: AHashMap<(i32, i32, i32), Vec<CellId>> = AHashMap::new();
    for &s in seeds {
        bins.entry(bin_of(cells.coord(s))).or_default().push(s);
    }
    let mut survivors: Vec<CellId> = Vec::new();
    for &id in candidates_sorted {
        let coord = cells.coord(id);
        let (bx, by, bz) = bin_of(coord);
        let mut killed = false;
        'outer: for dbx in -1..=1 {
            for dby in -1..=1 {
                for dbz in -1..=1 {
                    if let Some(nearby) = bins.get(&(bx + dbx, by + dby, bz + dbz)) {
                        for &n_id in nearby {
                            let n = cells.coord(n_id);
                            let dx = (coord.0 - n.0) as f64 * v;
                            let dy = (coord.1 - n.1) as f64 * v;
                            let dz = (coord.2 - n.2) as f64 * v;
                            if dx * dx + dy * dy + dz * dz <= r_sq {
                                killed = true;
                                break 'outer;
                            }
                        }
                    }
                }
            }
        }
        if !killed {
            survivors.push(id);
            bins.entry((bx, by, bz)).or_default().push(id);
        }
    }
    survivors
}

/// Scale each edge by its endpoints' average wall penalty and add the step
/// penalty. Unreached cells (dist +INFINITY) collapse the wall penalty to 1.0.
fn apply_wall_safe_penalty(
    cells: &mut SurfaceCells,
    dist: &[f32],
    clearance_m: f32,
    buffer_m: f32,
    buffer_weight: f32,
    step_weight: f32,
) {
    let mut edge_lists: Vec<(CellId, &mut Vec<Edge>)> = cells.iter_edges_mut().collect();
    edge_lists.par_iter_mut().for_each(|(src, edges)| {
        scale_edges(
            edges,
            *src,
            dist,
            clearance_m,
            buffer_m,
            buffer_weight,
            step_weight,
        );
    });
}

/// Rescale one cell's outgoing edges from base_cost. Idempotent, so a regional
/// repass cannot compound the penalty.
#[inline]
fn scale_edges(
    edges: &mut [Edge],
    src: CellId,
    dist: &[f32],
    clearance_m: f32,
    buffer_m: f32,
    buffer_weight: f32,
    step_weight: f32,
) {
    let pu = penalty_of(dist[src as usize], clearance_m, buffer_m, buffer_weight);
    for edge in edges.iter_mut() {
        let pv = penalty_of(
            dist[edge.dest as usize],
            clearance_m,
            buffer_m,
            buffer_weight,
        );
        edge.cost = edge.base_cost * (pu + pv) / 2.0 + step_weight * edge.rise;
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

/// Seed a node in every connected component in `domain` that the clearance
/// floor left empty, so a thin or sparse component is still reachable. `domain`
/// is every live cell for a full rebuild, or the window for an incremental one.
fn ensure_node_per_component(
    cells: &SurfaceCells,
    dist: &[f32],
    voxel_size: f32,
    domain: &[CellId],
    scratch: &mut NodeScratch,
    out_nodes: &mut Vec<NodeData>,
) {
    if domain.is_empty() {
        return;
    }
    scratch.ensure_capacity(cells.slot_capacity());

    // Union the domain into components. make() also marks in-domain membership,
    // which contains() below tests.
    for &id in domain {
        scratch.uf.make(id);
    }
    for &id in domain {
        for e in cells.neighbors(id) {
            if scratch.uf.contains(e.dest) {
                scratch.uf.union(id, e.dest);
            }
        }
    }

    // Flag cells that already hold a node, including nodes outside the domain.
    for nd in out_nodes.iter() {
        scratch.node_flag[nd.cell_id as usize] = true;
    }

    // A component is served when it holds or borders a node. Indexed by root.
    for &id in domain {
        let touches_node = scratch.node_flag[id as usize]
            || cells
                .neighbors(id)
                .iter()
                .any(|e| scratch.node_flag[e.dest as usize]);
        if touches_node {
            let root = scratch.uf.find(id) as usize;
            scratch.served[root] = true;
        }
    }

    // Clearest cell per still-unserved component, indexed by root.
    for &id in domain {
        let root = scratch.uf.find(id) as usize;
        if scratch.served[root] {
            continue;
        }
        let cur = scratch.best[root];
        if cur == NO_CELL || is_clearer(cells, dist, id, cur) {
            scratch.best[root] = id;
        }
    }

    // Emit one node per unserved component: the cell that won its root's slot.
    for &id in domain {
        let root = scratch.uf.find(id) as usize;
        if !scratch.served[root] && scratch.best[root] == id {
            let (ix, iy, iz) = cells.coord(id);
            out_nodes.push(NodeData {
                cell_id: id,
                pos: surface_point_xyz(ix, iy, iz, voxel_size),
            });
        }
    }

    // Leave every buffer all-default for the next call by resetting only the
    // slots this pass touched.
    for &id in domain {
        scratch.uf.clear(id);
        scratch.served[id as usize] = false;
        scratch.best[id as usize] = NO_CELL;
    }
    for nd in out_nodes.iter() {
        scratch.node_flag[nd.cell_id as usize] = false;
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

/// Reusable dense scratch for node placement, left all-default between calls.
#[derive(Default)]
pub struct NodeScratch {
    uf: UnionFind,
    node_flag: Vec<bool>,
    served: Vec<bool>,
    best: Vec<CellId>,
    seen: Vec<bool>,
}

impl NodeScratch {
    fn ensure_capacity(&mut self, n: usize) {
        self.uf.ensure_capacity(n);
        if self.node_flag.len() < n {
            self.node_flag.resize(n, false);
            self.served.resize(n, false);
            self.best.resize(n, NO_CELL);
            self.seen.resize(n, false);
        }
    }
}

/// Array-backed union-find indexed by CellId. Unenrolled slots are NO_CELL.
#[derive(Default)]
struct UnionFind {
    parent: Vec<CellId>,
    rank: Vec<u8>,
}

impl UnionFind {
    fn ensure_capacity(&mut self, n: usize) {
        if self.parent.len() < n {
            self.parent.resize(n, NO_CELL);
            self.rank.resize(n, 0);
        }
    }

    fn clear(&mut self, x: CellId) {
        let i = x as usize;
        self.parent[i] = NO_CELL;
        self.rank[i] = 0;
    }

    fn make(&mut self, x: CellId) {
        let i = x as usize;
        if self.parent[i] == NO_CELL {
            self.parent[i] = x;
        }
    }

    fn contains(&self, x: CellId) -> bool {
        self.parent[x as usize] != NO_CELL
    }

    fn find(&mut self, x: CellId) -> CellId {
        let mut root = x;
        while self.parent[root as usize] != root {
            root = self.parent[root as usize];
        }
        let mut cur = x;
        while cur != root {
            let next = self.parent[cur as usize];
            self.parent[cur as usize] = root;
            cur = next;
        }
        root
    }

    fn union(&mut self, a: CellId, b: CellId) {
        let mut ra = self.find(a);
        let mut rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.rank[ra as usize] < self.rank[rb as usize] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb as usize] = ra;
        if self.rank[ra as usize] == self.rank[rb as usize] {
            self.rank[ra as usize] += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adjacency::{build_surface_cells, build_surface_lookup, SurfaceLookup};

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

    fn build_cells(surface: &[VoxelKey], step_cells: i32) -> SurfaceCells {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(surface, &mut lookup);
        let mut sc = SurfaceCells::default();
        build_surface_cells(&mut sc, &lookup, VOXEL, step_cells);
        sc
    }

    #[test]
    fn open_patch_places_at_least_one_node() {
        let mut sc = build_cells(&open_patch(0, 0, 10), 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            5,
            2,
            VOXEL,
            1.0,
            0.0,
            0.3,
            1.0,
            0.0,
            &mut state,
            &mut scratch,
            &mut nodes,
        );
        assert!(!nodes.is_empty());
        for n in &nodes {
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            5,
            2,
            VOXEL,
            1.0,
            0.5,
            0.3,
            1.0,
            0.0,
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            5,
            2,
            VOXEL,
            1.0,
            0.0,
            0.3,
            1.0,
            0.0,
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            5,
            2,
            VOXEL,
            1.0,
            0.0,
            0.3,
            1.0,
            0.0,
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
            let mut nodes = Vec::new();
            place_nodes(
                &mut sc,
                &ColumnIz::default(),
                5,
                2,
                VOXEL,
                1.0,
                0.0,
                0.3,
                1.0,
                step_weight,
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
            by_col.insert(col, vec![0]);
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
            by_col.insert(col, vec![0]);
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
        by_col.insert((1, 0), vec![10]);
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.insert(col, vec![0]);
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
        by_col.insert((2, 0), vec![0]);
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.insert(col, vec![0]);
        }
        assert!(!real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }

    /// Empty for more than the span before surface resumes: a real cliff.
    #[test]
    fn gap_wider_than_span_still_seeds() {
        let sc = build_cells(&[(0, 0, 0)], 2);
        let id = sc.id((0, 0, 0)).unwrap();
        let mut by_col = ColumnIz::default();
        by_col.insert((10, 0), vec![0]);
        for col in [(-1, 0), (0, -1), (0, 1)] {
            by_col.insert(col, vec![0]);
        }
        assert!(real_wall_adjacent(&sc, &by_col, id, 5, 2));
    }
}
