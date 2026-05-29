// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use ahash::AHashMap;

use crate::voxel::VoxelKey;

pub type SurfaceLookup = AHashMap<(i32, i32), Vec<i32>>;

const NEIGHBORS_4: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub dst: VoxelKey,
    pub cost: f32,
}

#[derive(Default)]
pub struct SurfaceAdjacency {
    cells: AHashMap<VoxelKey, Vec<Edge>>,
}

impl SurfaceAdjacency {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_cell(&mut self, cell: VoxelKey) {
        self.cells.entry(cell).or_default();
    }

    pub fn add_edge(&mut self, src: VoxelKey, dst: VoxelKey, cost: f32) {
        self.cells.entry(src).or_default().push(Edge { dst, cost });
    }

    pub fn neighbors(&self, cell: VoxelKey) -> impl Iterator<Item = Edge> + '_ {
        self.cells.get(&cell).into_iter().flatten().copied()
    }

    pub fn cells(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.cells.keys().copied()
    }

    /// Per-cell iteration that yields the cell and a borrowed view of its
    /// edges in a single hashmap probe.
    pub fn iter(&self) -> impl Iterator<Item = (VoxelKey, &[Edge])> + '_ {
        self.cells.iter().map(|(&k, v)| (k, v.as_slice()))
    }

    /// Mutable per-cell edge iterator.
    pub fn iter_edges_mut(&mut self) -> impl Iterator<Item = (VoxelKey, &mut Vec<Edge>)> + '_ {
        self.cells.iter_mut().map(|(&k, v)| (k, v))
    }

    pub fn contains(&self, cell: VoxelKey) -> bool {
        self.cells.contains_key(&cell)
    }

    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

/// Group cells by XY column with sorted unique iz per column.
pub fn build_surface_lookup(cells: &[VoxelKey]) -> SurfaceLookup {
    let mut lookup: SurfaceLookup = AHashMap::new();
    for &(ix, iy, iz) in cells {
        lookup.entry((ix, iy)).or_default().push(iz);
    }
    for zs in lookup.values_mut() {
        zs.sort_unstable();
        zs.dedup();
    }
    lookup
}

/// 4 way L1 adjacency, limited by step height threshold.
pub fn build_surface_adjacency(
    surface_lookup: &SurfaceLookup,
    voxel_size: f32,
    step_threshold_cells: i32,
) -> SurfaceAdjacency {
    let mut adj = SurfaceAdjacency::new();
    for (&(ix, iy), zs) in surface_lookup {
        for &iz in zs {
            adj.add_cell((ix, iy, iz));
        }
    }
    for (&(ix, iy), zs) in surface_lookup {
        for &iz in zs {
            for (dx, dy) in NEIGHBORS_4 {
                let Some(nzs) = surface_lookup.get(&(ix + dx, iy + dy)) else {
                    continue;
                };
                for &nz in nzs {
                    let dz = nz - iz;
                    if dz.abs() > step_threshold_cells {
                        continue;
                    }
                    let cost = ((dx * dx + dy * dy + dz * dz) as f32).sqrt() * voxel_size;
                    adj.add_edge((ix, iy, iz), (ix + dx, iy + dy, nz), cost);
                }
            }
        }
    }
    adj
}

#[cfg(test)]
mod tests {
    use super::*;

    const VOXEL: f32 = 0.1;

    fn approx_eq(a: f32, b: f32) {
        let eps = 1e-5;
        assert!((a - b).abs() < eps, "{a} != {b} (eps {eps})");
    }

    fn neighbors_of(adj: &SurfaceAdjacency, cell: VoxelKey) -> Vec<Edge> {
        adj.neighbors(cell).collect()
    }

    #[test]
    fn same_z_neighbors_are_bidirectional() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 0)]);
        let adj = build_surface_adjacency(&lookup, VOXEL, 2);
        let a = neighbors_of(&adj, (0, 0, 0));
        let b = neighbors_of(&adj, (1, 0, 0));
        assert_eq!(a.len(), 1);
        assert_eq!(b.len(), 1);
        assert_eq!(a[0].dst, (1, 0, 0));
        assert_eq!(b[0].dst, (0, 0, 0));
        approx_eq(a[0].cost, VOXEL);
        approx_eq(b[0].cost, VOXEL);
    }

    #[test]
    fn diagonal_not_connected_under_4_connectivity() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 1, 0)]);
        let adj = build_surface_adjacency(&lookup, VOXEL, 2);
        assert!(neighbors_of(&adj, (0, 0, 0)).is_empty());
        assert!(neighbors_of(&adj, (1, 1, 0)).is_empty());
    }

    #[test]
    fn step_threshold_blocks_large_dz() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 5)]);
        let adj = build_surface_adjacency(&lookup, VOXEL, 2);
        assert!(neighbors_of(&adj, (0, 0, 0)).is_empty());
        assert!(neighbors_of(&adj, (1, 0, 5)).is_empty());
    }

    #[test]
    fn step_within_threshold_uses_3d_distance() {
        let lookup = build_surface_lookup(&[(0, 0, 0), (1, 0, 1)]);
        let adj = build_surface_adjacency(&lookup, VOXEL, 2);
        let expected = (2.0_f32).sqrt() * VOXEL;
        let a = neighbors_of(&adj, (0, 0, 0));
        let b = neighbors_of(&adj, (1, 0, 1));
        assert_eq!(a.len(), 1);
        assert_eq!(b.len(), 1);
        approx_eq(a[0].cost, expected);
        approx_eq(b[0].cost, expected);
    }

    #[test]
    fn plus_pattern_center_has_four_neighbors() {
        let cells = vec![(0, 0, 0), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)];
        let lookup = build_surface_lookup(&cells);
        let adj = build_surface_adjacency(&lookup, VOXEL, 2);
        assert_eq!(neighbors_of(&adj, (0, 0, 0)).len(), 4);
    }
}
