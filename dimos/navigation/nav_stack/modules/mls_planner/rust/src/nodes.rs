// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

use ahash::AHashMap;
use rayon::prelude::*;

use crate::adjacency::{CellId, Edge, SurfaceCells};
use crate::dijkstra::{dijkstra, DijkstraState};
use crate::voxel::{surface_point_xyz, VoxelKey};

#[derive(Clone, Copy, Debug)]
pub struct NodeData {
    pub cell_id: CellId,
    pub pos: (f32, f32, f32),
}

/// Run wall-distance Dijkstra, NMS-pick node cells, write them into
/// `out_nodes`, and apply wall-safe edge-cost scaling to `cells`. The
/// DijkstraState is left holding the wall-distance result so callers can
/// reuse it before the next Dijkstra overwrites it.
pub fn place_nodes(
    cells: &mut SurfaceCells,
    voxel_size: f32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
    state: &mut DijkstraState,
    out_nodes: &mut Vec<NodeData>,
) {
    out_nodes.clear();
    if cells.is_empty() {
        return;
    }

    let mut wall_seeds: Vec<CellId> = Vec::new();
    collect_wall_adjacent_cells(cells, &mut wall_seeds);
    dijkstra(cells, &wall_seeds, state);

    let mut candidates: Vec<CellId> = cells
        .ids()
        .filter(|&id| state.dist[id as usize] >= node_wall_buffer_m)
        .collect();
    candidates.par_sort_unstable_by(|&a, &b| {
        state.dist[b as usize]
            .total_cmp(&state.dist[a as usize])
            .then(a.cmp(&b))
    });

    let survivors = nms_grid(cells, &candidates, voxel_size, node_spacing_m);

    out_nodes.reserve(survivors.len());
    for &id in &survivors {
        let (ix, iy, iz) = cells.coord(id);
        out_nodes.push(NodeData {
            cell_id: id,
            pos: surface_point_xyz(ix, iy, iz, voxel_size),
        });
    }

    apply_wall_safe_penalty(cells, &state.dist, node_wall_buffer_m);
}

/// Cells missing any of their 4 xy-direction neighbors are treated as
/// boundaries. Direction membership is tracked with a 4-bit mask so the
/// 349k-cell case avoids per-cell hashset allocation.
fn collect_wall_adjacent_cells(cells: &SurfaceCells, out: &mut Vec<CellId>) {
    out.clear();
    for (id, edges) in cells.iter() {
        let (cx, cy, _) = cells.coord(id);
        let mut mask: u8 = 0;
        for e in edges {
            let (nx, ny, _) = cells.coord(e.dst);
            mask |= match (nx - cx, ny - cy) {
                (-1, 0) => 1,
                (1, 0) => 2,
                (0, -1) => 4,
                (0, 1) => 8,
                _ => 0,
            };
        }
        if mask != 0b1111 {
            out.push(id);
        }
    }
    if out.is_empty() {
        if let Some(c) = cells.ids().next() {
            out.push(c);
        }
    }
}

/// Space out nodes based on minimum distance.
fn nms_grid(
    cells: &SurfaceCells,
    candidates_sorted: &[CellId],
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

/// Scale every edge cost by the average of its endpoint penalties, which
/// pushes shortest paths away from walls. Unreached cells have
/// dist == +INFINITY which collapses to penalty 1.0.
fn apply_wall_safe_penalty(cells: &mut SurfaceCells, dist: &[f32], buffer_m: f32) {
    let mut edge_lists: Vec<(CellId, &mut Vec<Edge>)> = cells.iter_edges_mut().collect();
    edge_lists.par_iter_mut().for_each(|(src, edges)| {
        let pu = penalty_of(dist[*src as usize], buffer_m);
        for edge in edges.iter_mut() {
            let pv = penalty_of(dist[edge.dst as usize], buffer_m);
            edge.cost *= (pu + pv) / 2.0;
        }
    });
}

#[inline]
fn penalty_of(d: f32, buffer_m: f32) -> f32 {
    (1.0 + (buffer_m - d) / buffer_m).max(1.0)
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
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, &mut state, &mut nodes);
        assert!(!nodes.is_empty());
        for n in &nodes {
            let (ix, iy, _) = sc.coord(n.cell_id);
            assert!((0..10).contains(&ix) && (0..10).contains(&iy));
        }
    }

    #[test]
    fn sloped_patch_places_interior_nodes() {
        let mut cells_in = Vec::new();
        for ix in 0..10 {
            for iy in 0..10 {
                cells_in.push((ix, iy, ix));
            }
        }
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, &mut state, &mut nodes);
        assert!(!nodes.is_empty());
    }

    #[test]
    fn nms_enforces_spacing() {
        let mut cells_in = open_patch(0, 0, 10);
        cells_in.extend(open_patch(20, 0, 10));
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, &mut state, &mut nodes);
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
    fn wall_cells_scale_outbound_cost() {
        let cells_in: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let mut sc = build_cells(&cells_in, 2);
        let mut state = DijkstraState::default();
        let mut nodes = Vec::new();
        place_nodes(&mut sc, VOXEL, 1.0, 0.3, &mut state, &mut nodes);
        let id0 = sc.id((0, 0, 0)).unwrap();
        let outbound = sc.neighbors(id0);
        assert!(!outbound.is_empty());
        for edge in outbound {
            assert!(edge.cost >= 1.5 * VOXEL - 1e-5);
        }
    }
}
