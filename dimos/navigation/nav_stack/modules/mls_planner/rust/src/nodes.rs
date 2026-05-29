// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Node placement: identify standable cells far from any wall, place graph
//! nodes at local maxima via NMS, and rescale cell-edge costs to push paths
//! toward corridor centers.

use ahash::{AHashMap, AHashSet};

use crate::adjacency::{
    build_surface_adjacency, build_surface_lookup, SurfaceAdjacency, SurfaceLookup,
};
use crate::dijkstra::dijkstra;
use crate::voxel::{surface_point_xyz, VoxelKey};

pub struct NodeData {
    pub cell: VoxelKey,
    pub pos: (f32, f32, f32),
}

pub struct SurfaceGraph {
    pub adj: SurfaceAdjacency,
    pub surface_lookup: SurfaceLookup,
    pub nodes: Vec<NodeData>,
}

pub fn place_nodes(
    surface_cells: &[VoxelKey],
    voxel_size: f32,
    maximum_step_cells: i32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
) -> SurfaceGraph {
    let surface_lookup = build_surface_lookup(surface_cells);
    let mut adj = build_surface_adjacency(&surface_lookup, voxel_size, maximum_step_cells);

    if adj.is_empty() {
        return SurfaceGraph {
            adj,
            surface_lookup,
            nodes: Vec::new(),
        };
    }

    let wall_seeds = wall_adjacent_cells(&adj);
    let dist = dijkstra(&adj, &wall_seeds).dist_map();

    let mut candidates: Vec<VoxelKey> = dist
        .iter()
        .filter_map(|(&c, &d)| {
            if d >= node_wall_buffer_m {
                Some(c)
            } else {
                None
            }
        })
        .collect();
    candidates.sort_by(|a, b| dist[b].total_cmp(&dist[a]).then(a.cmp(b)));

    let survivors = nms_grid(&candidates, voxel_size, node_spacing_m);

    let nodes: Vec<NodeData> = survivors
        .iter()
        .map(|&cell| NodeData {
            cell,
            pos: surface_point_xyz(cell.0, cell.1, cell.2, voxel_size),
        })
        .collect();

    apply_wall_safe_penalty(&mut adj, &dist, node_wall_buffer_m);

    SurfaceGraph {
        adj,
        surface_lookup,
        nodes,
    }
}

/// Cells missing any of their 4 xy-direction neighbors are treated as boundaries.
fn wall_adjacent_cells(adj: &SurfaceAdjacency) -> Vec<VoxelKey> {
    let mut wall: Vec<VoxelKey> = adj
        .iter()
        .filter(|(c, edges)| {
            let mut dirs: AHashSet<(i32, i32)> = AHashSet::new();
            for e in *edges {
                dirs.insert((e.dst.0 - c.0, e.dst.1 - c.1));
            }
            dirs.len() < 4
        })
        .map(|(c, _)| c)
        .collect();
    wall.sort();
    if wall.is_empty() {
        if let Some(c) = adj.cells().min() {
            wall.push(c);
        }
    }
    wall
}

/// Space out nodes based on minimum distance
fn nms_grid(candidates_sorted: &[VoxelKey], voxel_size: f32, node_spacing_m: f32) -> Vec<VoxelKey> {
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

    let mut bins: AHashMap<(i32, i32, i32), Vec<VoxelKey>> = AHashMap::new();
    let mut survivors = Vec::new();
    for &cell in candidates_sorted {
        let (bx, by, bz) = bin_of(cell);
        let mut killed = false;
        'outer: for dbx in -1..=1 {
            for dby in -1..=1 {
                for dbz in -1..=1 {
                    if let Some(nearby) = bins.get(&(bx + dbx, by + dby, bz + dbz)) {
                        for &n in nearby {
                            let dx = (cell.0 - n.0) as f64 * v;
                            let dy = (cell.1 - n.1) as f64 * v;
                            let dz = (cell.2 - n.2) as f64 * v;
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
            survivors.push(cell);
            bins.entry((bx, by, bz)).or_default().push(cell);
        }
    }
    survivors
}

/// Scales every edge cost by the average of its endpoint penalties, which
/// pushes shortest paths away from walls.
fn apply_wall_safe_penalty(
    adj: &mut SurfaceAdjacency,
    dist: &AHashMap<VoxelKey, f32>,
    buffer_m: f32,
) {
    let penalty: AHashMap<VoxelKey, f32> = adj
        .cells()
        .map(|c| {
            let p = match dist.get(&c) {
                Some(&d) => (1.0 + (buffer_m - d) / buffer_m).max(1.0),
                None => 1.0,
            };
            (c, p)
        })
        .collect();

    for (src, edges) in adj.iter_edges_mut() {
        let pu = penalty.get(&src).copied().unwrap_or(1.0);
        for edge in edges {
            let pv = penalty.get(&edge.dst).copied().unwrap_or(1.0);
            edge.cost *= (pu + pv) / 2.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn open_patch_places_at_least_one_node() {
        // 10x10 at voxel=0.1 is 1m x 1m. Center is ~0.5m from any wall, well above buffer=0.3m.
        let sg = place_nodes(&open_patch(0, 0, 10), VOXEL, 2, 1.0, 0.3);
        assert!(!sg.nodes.is_empty());
        for n in &sg.nodes {
            let (ix, iy, _) = n.cell;
            assert!((0..10).contains(&ix) && (0..10).contains(&iy));
        }
    }

    #[test]
    fn sloped_patch_places_interior_nodes() {
        // 10x10 plane sloped 1 cell of z per cell of x. With step_threshold=2
        // every interior cell still has all 4 xy-direction neighbors in-graph,
        // so it must not be flagged as wall-adjacent.
        let mut cells = Vec::new();
        for ix in 0..10 {
            for iy in 0..10 {
                cells.push((ix, iy, ix));
            }
        }
        let sg = place_nodes(&cells, VOXEL, 2, 1.0, 0.3);
        assert!(!sg.nodes.is_empty());
    }

    #[test]
    fn nms_enforces_spacing() {
        // Two 10x10 patches separated by 1m gap; each places at least one node, no pair within 1m.
        let mut cells = open_patch(0, 0, 10);
        cells.extend(open_patch(20, 0, 10));
        let sg = place_nodes(&cells, VOXEL, 2, 1.0, 0.3);
        assert!(sg.nodes.len() >= 2);
        for i in 0..sg.nodes.len() {
            for j in (i + 1)..sg.nodes.len() {
                let a = sg.nodes[i].pos;
                let b = sg.nodes[j].pos;
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
        let cells: Vec<VoxelKey> = (0..10).map(|ix| (ix, 0, 0)).collect();
        let sg = place_nodes(&cells, VOXEL, 2, 1.0, 0.3);
        let outbound: Vec<_> = sg.adj.neighbors((0, 0, 0)).collect();
        assert!(!outbound.is_empty());
        // End cell penalty=2, neighbor penalty>=1, so outbound cost >= 1.5 * VOXEL.
        for edge in &outbound {
            assert!(edge.cost >= 1.5 * VOXEL - 1e-5);
        }
    }
}
