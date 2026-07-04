// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

use std::cmp::Ordering;

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells};
use crate::dijkstra::{dijkstra, dijkstra_region, DijkstraState, Weight};
use crate::voxel::{surface_point_xyz, VoxelKey};

#[derive(Clone, Copy, Debug)]
pub struct NodeData {
    pub cell_id: CellId,
    pub pos: (f32, f32, f32),
}

/// Place graph nodes across the surface, spaced out and biased away from walls.
#[allow(clippy::too_many_arguments)]
pub fn place_nodes(
    cells: &mut SurfaceCells,
    voxel_size: f32,
    node_spacing_m: f32,
    wall_clearance_m: f32,
    wall_buffer_m: f32,
    wall_buffer_weight: f32,
    step_penalty_weight: f32,
    state: &mut DijkstraState,
    out_nodes: &mut Vec<NodeData>,
) {
    out_nodes.clear();
    if cells.is_empty() {
        return;
    }

    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_cells(cells, &mut wall_seeds);
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
    ensure_node_per_component(cells, &state.dist, voxel_size, &domain, out_nodes);

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
    window: &AHashSet<CellId>,
    voxel_size: f32,
    node_spacing_m: f32,
    wall_clearance_m: f32,
    wall_buffer_m: f32,
    wall_buffer_weight: f32,
    step_penalty_weight: f32,
    wall_state: &mut DijkstraState,
    nodes: &mut Vec<NodeData>,
) {
    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_in_window(cells, window, &mut wall_seeds);
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
    ensure_node_per_component(cells, &wall_state.dist, voxel_size, &domain, nodes);

    apply_wall_safe_penalty_region(
        cells,
        &wall_state.dist,
        wall_clearance_m,
        wall_buffer_m,
        wall_buffer_weight,
        step_penalty_weight,
        window,
    );
}

/// Wall-adjacency over a cell subset, matching collect_wall_adjacent_cells.
fn collect_wall_adjacent_in_window(
    cells: &SurfaceCells,
    window: &AHashSet<CellId>,
    out: &mut Vec<CellId>,
) {
    out.clear();
    for &id in window {
        if cells.is_live(id) && is_wall_adjacent(cells, id) {
            out.push(id);
        }
    }
}

/// A cell is wall-adjacent when it lacks at least one of its 4 xy neighbors.
fn is_wall_adjacent(cells: &SurfaceCells, id: CellId) -> bool {
    let (cx, cy, _) = cells.coord(id);
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
    mask != 0b1111
}

/// Rescale edge costs for the window and its neighbors, whose wall distance may
/// have changed. Idempotent via base_cost.
fn apply_wall_safe_penalty_region(
    cells: &mut SurfaceCells,
    dist: &[f32],
    clearance_m: f32,
    buffer_m: f32,
    buffer_weight: f32,
    step_weight: f32,
    window: &AHashSet<CellId>,
) {
    let mut affected: AHashSet<CellId> = AHashSet::with_capacity(window.len() * 2);
    for &w in window {
        affected.insert(w);
        for e in cells.neighbors(w) {
            affected.insert(e.dest);
        }
    }
    for id in affected {
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
fn collect_wall_adjacent_cells(cells: &SurfaceCells, out: &mut Vec<CellId>) {
    out.clear();
    for id in cells.ids() {
        if is_wall_adjacent(cells, id) {
            out.push(id);
        }
    }
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
    out_nodes: &mut Vec<NodeData>,
) {
    if domain.is_empty() {
        return;
    }
    let node_cells: AHashSet<CellId> = out_nodes.iter().map(|n| n.cell_id).collect();
    let in_domain: AHashSet<CellId> = domain.iter().copied().collect();

    let mut uf = UnionFind::default();
    for &id in domain {
        uf.make(id);
    }
    for &id in domain {
        for e in cells.neighbors(id) {
            if in_domain.contains(&e.dest) {
                uf.union(id, e.dest);
            }
        }
    }

    // Served: the component holds or borders a node, including one outside domain.
    let mut served: AHashSet<CellId> = AHashSet::new();
    for &id in domain {
        let touches_node = node_cells.contains(&id)
            || cells
                .neighbors(id)
                .iter()
                .any(|e| node_cells.contains(&e.dest));
        if touches_node {
            served.insert(uf.find(id));
        }
    }

    // Clearest cell per still-unserved component.
    let mut best: AHashMap<CellId, CellId> = AHashMap::new();
    for &id in domain {
        let root = uf.find(id);
        if served.contains(&root) {
            continue;
        }
        match best.get(&root) {
            Some(&cur) if !is_clearer(cells, dist, id, cur) => {}
            _ => {
                best.insert(root, id);
            }
        }
    }

    out_nodes.reserve(best.len());
    for &id in best.values() {
        let (ix, iy, iz) = cells.coord(id);
        out_nodes.push(NodeData {
            cell_id: id,
            pos: surface_point_xyz(ix, iy, iz, voxel_size),
        });
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

/// Union-find keyed by CellId so the incremental path pays for the window only.
#[derive(Default)]
struct UnionFind {
    parent: AHashMap<CellId, CellId>,
}

impl UnionFind {
    fn make(&mut self, x: CellId) {
        self.parent.entry(x).or_insert(x);
    }

    fn find(&mut self, x: CellId) -> CellId {
        let mut root = x;
        while let Some(&p) = self.parent.get(&root) {
            if p == root {
                break;
            }
            root = p;
        }
        let mut cur = x;
        while let Some(&p) = self.parent.get(&cur) {
            if p == root {
                break;
            }
            self.parent.insert(cur, root);
            cur = p;
        }
        root
    }

    fn union(&mut self, a: CellId, b: CellId) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra != rb {
            self.parent.insert(ra, rb);
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc, VOXEL, 1.0, 0.0, 0.3, 1.0, 0.0, &mut state, &mut nodes,
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc, VOXEL, 1.0, 0.5, 0.3, 1.0, 0.0, &mut state, &mut nodes,
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc, VOXEL, 1.0, 0.0, 0.3, 1.0, 0.0, &mut state, &mut nodes,
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
        let mut nodes = Vec::new();
        place_nodes(
            &mut sc, VOXEL, 1.0, 0.0, 0.3, 1.0, 0.0, &mut state, &mut nodes,
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
            let mut nodes = Vec::new();
            place_nodes(
                &mut sc,
                VOXEL,
                1.0,
                0.0,
                0.3,
                1.0,
                step_weight,
                &mut state,
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
}
