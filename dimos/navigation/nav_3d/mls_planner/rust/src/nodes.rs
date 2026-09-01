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
use crate::dijkstra::{dijkstra, dijkstra_region, DijkstraState, Weight};
use crate::surfaces::{is_standable, ColumnIz};
use crate::voxel::{surface_point_xyz, VoxelKey};

const NEIGHBORS_4: [(i32, i32, u8); 4] = [(-1, 0, 1), (1, 0, 2), (0, -1, 4), (0, 1, 8)];

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

type BinKey = (i32, i32, i32);

/// Spacing bins of node positions for crowding checks during relocation.
struct NodeBins {
    spacing: f32,
    crowd_sq: f32,
    bins: AHashMap<BinKey, Vec<(f32, f32, f32)>>,
}

impl NodeBins {
    fn new(spacing: f32) -> Self {
        let crowd = RELOCATION_CROWDING_FRAC * spacing;
        Self {
            spacing,
            crowd_sq: crowd * crowd,
            bins: AHashMap::new(),
        }
    }

    fn bin_of(&self, p: (f32, f32, f32)) -> (i32, i32, i32) {
        (
            (p.0 / self.spacing).floor() as i32,
            (p.1 / self.spacing).floor() as i32,
            (p.2 / self.spacing).floor() as i32,
        )
    }

    fn insert(&mut self, p: (f32, f32, f32)) {
        let bin = self.bin_of(p);
        self.bins.entry(bin).or_default().push(p);
    }

    fn crowded(&self, p: (f32, f32, f32)) -> bool {
        let (bx, by, bz) = self.bin_of(p);
        for dx in -1..=1_i32 {
            for dy in -1..=1_i32 {
                for dz in -1..=1_i32 {
                    let Some(near) = self.bins.get(&(bx + dx, by + dy, bz + dz)) else {
                        continue;
                    };
                    for q in near {
                        let d = (p.0 - q.0, p.1 - q.1, p.2 - q.2);
                        if d.0 * d.0 + d.1 * d.1 + d.2 * d.2 < self.crowd_sq {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}

/// Move nodes whose cell died onto a nearby live cell instead of dropping
/// them, so transient surface flicker cannot delete and respawn graph
/// structure. Dead nodes are keyed by their captured coordinate because their
/// ids were freed and may now alias recycled live cells. Unrelocatable nodes
/// are marked NO_CELL and dropped by the sticky retention pass.
pub fn relocate_dead_nodes(
    cells: &SurfaceCells,
    lookup: &SurfaceLookup,
    nodes: &mut [NodeData],
    dead_nodes: &[(usize, VoxelKey)],
    params: &PlacementParams,
) {
    if dead_nodes.is_empty() {
        return;
    }
    let step = params.step_cells;
    let mut dead = vec![false; nodes.len()];
    for &(i, _) in dead_nodes {
        dead[i] = true;
    }
    let mut taken = vec![false; cells.slot_capacity()];
    // Fragment fallbacks whose relocation would crowd the main surface die
    // instead of moving.
    let mut bins = NodeBins::new(params.node_spacing_m);
    for (i, n) in nodes.iter().enumerate() {
        if !dead[i] {
            taken[n.cell_id as usize] = true;
            bins.insert(n.pos);
        }
    }
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
                if taken[id as usize] {
                    continue;
                }
                let rank = 2 * dz + dx.abs() + dy.abs();
                if best.is_none_or(|(r, _, _)| rank < r) {
                    best = Some((rank, id, k));
                }
            }
        }
        let n = &mut nodes[ni];
        match best {
            Some((_, id, k)) => {
                let pos = surface_point_xyz(k.0, k.1, k.2, params.voxel_size);
                if bins.crowded(pos) {
                    n.cell_id = NO_CELL;
                    continue;
                }
                taken[id as usize] = true;
                bins.insert(pos);
                n.cell_id = id;
                n.pos = pos;
            }
            None => n.cell_id = NO_CELL,
        }
    }
}

/// Place graph nodes across the surface, spaced out and biased away from walls.
pub fn place_nodes(
    cells: &mut SurfaceCells,
    by_col: &ColumnIz,
    params: &PlacementParams,
    state: &mut DijkstraState,
    scratch: &mut NodeScratch,
    out_nodes: &mut Vec<NodeData>,
) {
    out_nodes.clear();
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
    place_from_candidates(
        cells,
        candidates,
        &state.dist,
        &[],
        params.voxel_size,
        params.node_spacing_m,
        out_nodes,
    );

    let domain: Vec<CellId> = cells.ids().collect();
    ensure_node_per_component(
        cells,
        &state.dist,
        params.voxel_size,
        &domain,
        scratch,
        out_nodes,
    );

    apply_wall_safe_penalty(
        cells,
        &state.dist,
        params.wall_clearance_m,
        params.wall_buffer_m,
        params.wall_buffer_weight,
        params.step_penalty_weight,
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

/// Regional counterpart to place_nodes. Nodes are sticky: survivors keep
/// their placement, and new nodes are placed only on cells added by this
/// update, spaced by NMS against every existing node.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes_region(
    cells: &mut SurfaceCells,
    by_col: &ColumnIz,
    params: &PlacementParams,
    added: &[CellId],
    window: &[CellId],
    wall_state: &mut DijkstraState,
    scratch: &mut NodeScratch,
    nodes: &mut Vec<NodeData>,
) {
    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_in_window(
        cells,
        by_col,
        params.clearance_cells,
        params.step_cells,
        window,
        &mut wall_seeds,
    );
    dijkstra_region(cells, &wall_seeds, window, wall_state, Weight::Base);

    let node_floor = params.wall_clearance_m;
    // Drop only nodes whose cell died (marked NO_CELL by relocation) or whose
    // fresh wall distance marks the cell impassable. The distance field is
    // stale outside the window, so only in-window nodes are judged by it.
    scratch.ensure_capacity(cells.slot_capacity());
    for &w in window {
        scratch.seen[w as usize] = true;
    }
    let in_window = &scratch.seen;
    nodes.retain(|n| {
        n.cell_id != NO_CELL
            && cells.is_live(n.cell_id)
            && !(in_window[n.cell_id as usize] && wall_state.dist[n.cell_id as usize] < node_floor)
    });
    for &w in window {
        scratch.seen[w as usize] = false;
    }
    let kept: Vec<CellId> = nodes.iter().map(|n| n.cell_id).collect();

    // New nodes only in comfortably open freshly-seen space: transient fringe
    // cells near walls must not spawn graph structure every frame.
    let spawn_floor = params.wall_clearance_m + SPAWN_FLOOR_BUFFER_FRAC * params.wall_buffer_m;
    let candidates: Vec<CellId> = added
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id) && wall_state.dist[id as usize] >= spawn_floor)
        .collect();
    place_from_candidates(
        cells,
        candidates,
        &wall_state.dist,
        &kept,
        params.voxel_size,
        params.node_spacing_m,
        nodes,
    );

    let domain: Vec<CellId> = window
        .iter()
        .copied()
        .filter(|&id| cells.is_live(id))
        .collect();
    ensure_node_per_component(
        cells,
        &wall_state.dist,
        params.voxel_size,
        &domain,
        scratch,
        nodes,
    );

    apply_wall_safe_penalty_region(
        cells,
        &wall_state.dist,
        params.wall_clearance_m,
        params.wall_buffer_m,
        params.wall_buffer_weight,
        params.step_penalty_weight,
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

    // Clearest cell and size per still-unserved component, indexed by root.
    for &id in domain {
        let root = scratch.uf.find(id) as usize;
        if scratch.served[root] {
            continue;
        }
        scratch.size[root] += 1;
        let cur = scratch.best[root];
        if cur == NO_CELL || is_clearer(cells, dist, id, cur) {
            scratch.best[root] = id;
        }
    }

    // Emit one node per unserved component: the cell that won its root's slot.
    // Fragments below the size floor are transient sensor noise, not places to
    // grow graph structure.
    for &id in domain {
        let root = scratch.uf.find(id) as usize;
        if !scratch.served[root]
            && scratch.best[root] == id
            && scratch.size[root] >= MIN_COMPONENT_CELLS
        {
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
        scratch.size[id as usize] = 0;
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
    size: Vec<u32>,
    pub(crate) seen: Vec<bool>,
}

impl NodeScratch {
    pub(crate) fn ensure_capacity(&mut self, n: usize) {
        self.uf.ensure_capacity(n);
        if self.node_flag.len() < n {
            self.node_flag.resize(n, false);
            self.served.resize(n, false);
            self.best.resize(n, NO_CELL);
            self.size.resize(n, 0);
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
        let mut nodes = vec![NodeData {
            cell_id: 0,
            pos: surface_point_xyz(5, 5, 0, VOXEL),
        }];
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0))],
            &params(0.0, 0.0),
        );
        let id = sc.id((5, 5, 1)).unwrap();
        assert_eq!(nodes[0].cell_id, id, "node must move to the raised cell");
        assert_eq!(nodes[0].pos, surface_point_xyz(5, 5, 1, VOXEL));
    }

    #[test]
    fn crowded_relocation_drops_the_node() {
        // A live node already sits next to the dead node's only free
        // relocation target, so the relocation is rejected and the node
        // marked dead.
        let surface = [(5, 5, 1), (5, 6, 0)];
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(&surface, &mut lookup);
        let sc = build_cells(&surface, 2);
        let live_id = sc.id((5, 6, 0)).unwrap();
        let mut nodes = vec![
            NodeData {
                cell_id: NO_CELL,
                pos: surface_point_xyz(5, 5, 0, VOXEL),
            },
            NodeData {
                cell_id: live_id,
                pos: surface_point_xyz(5, 6, 0, VOXEL),
            },
        ];
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0))],
            &params(0.0, 0.0),
        );
        assert_eq!(
            nodes[0].cell_id, NO_CELL,
            "crowded relocation must mark the node dead"
        );
        assert_eq!(nodes[1].cell_id, live_id, "live node untouched");
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
        let mut nodes = vec![
            NodeData {
                cell_id: 0,
                pos: surface_point_xyz(5, 5, 0, VOXEL),
            },
            NodeData {
                cell_id: 1,
                pos: surface_point_xyz(5, 6, 0, VOXEL),
            },
        ];
        relocate_dead_nodes(
            &sc,
            &lookup,
            &mut nodes,
            &[(0, (5, 5, 0)), (1, (5, 6, 0))],
            &params(0.0, 0.0),
        );
        let relocated = sc.id((5, 5, 1)).unwrap();
        assert_eq!(nodes[0].cell_id, relocated, "first relocation lands");
        assert_eq!(
            nodes[1].cell_id, NO_CELL,
            "second relocation must be crowded out by the first"
        );
    }

    #[test]
    fn spawn_floor_blocks_new_nodes_near_walls() {
        // A cell inside the spawn floor must not spawn a node, while an open
        // cell that clears it must.
        let surface = open_patch(0, 0, 10);
        let mut sc = build_cells(&surface, 2);
        let mut state = DijkstraState::default();
        let mut scratch = NodeScratch::default();
        let corner = sc.id((0, 0, 0)).unwrap();
        let mut nodes = vec![NodeData {
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
        place_nodes_region(
            &mut sc,
            &ColumnIz::default(),
            &p,
            &[near_wall, open],
            &window,
            &mut state,
            &mut scratch,
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
        let mut nodes = Vec::new();
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc,
            &ColumnIz::default(),
            &params(0.0, 0.0),
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
        let mut nodes = Vec::new();
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
        let mut nodes = Vec::new();
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
            let mut nodes = Vec::new();
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
