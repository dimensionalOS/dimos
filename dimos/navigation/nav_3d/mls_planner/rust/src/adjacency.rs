// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Surface cells indexed by dense CellId.
//!
//! Uses a "slot map" to store cells. When inserting, either expand the map
//! or reuse a freed location marked with a tombstone.

use ahash::AHashMap;
use rayon::prelude::*;

use crate::voxel::VoxelKey;

pub type SurfaceLookup = AHashMap<(i32, i32), Vec<i32>>;

/// Index of surface voxel
pub type CellId = u32;
pub const NO_CELL: CellId = u32::MAX;

/// Represent a deleted cell that can be reincarnated on an insertion.
const TOMBSTONE: VoxelKey = (i32::MIN, i32::MIN, i32::MIN);
const NEIGHBORS_4: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub dest: CellId,
    pub cost: f32,
}

#[derive(Default)]
pub struct SurfaceCells {
    coord: Vec<VoxelKey>,
    edges: Vec<Vec<Edge>>,
    by_coord: AHashMap<VoxelKey, CellId>,
    free: Vec<CellId>,
}

impl SurfaceCells {
    pub fn is_empty(&self) -> bool {
        self.by_coord.is_empty()
    }

    /// Total slot count, including tombstoned cells.
    pub fn slot_capacity(&self) -> usize {
        self.coord.len()
    }

    /// Clear all vecs but keeps space allocated.
    pub fn clear(&mut self) {
        self.coord.clear();
        self.by_coord.clear();
        self.free.clear();
        for e in self.edges.iter_mut() {
            e.clear();
        }
    }

    #[inline]
    pub fn is_live(&self, id: CellId) -> bool {
        self.coord[id as usize] != TOMBSTONE
    }

    /// Get or insert a new cell.
    ///
    /// Only expand the list if there are no available dead cells.
    pub fn insert(&mut self, k: VoxelKey) -> CellId {
        debug_assert_ne!(k, TOMBSTONE, "voxel coord collides with tombstone sentinel");
        if let Some(&id) = self.by_coord.get(&k) {
            return id;
        }
        let id = if let Some(free_id) = self.free.pop() {
            self.coord[free_id as usize] = k;
            free_id
        } else {
            let id = self.coord.len() as CellId;
            self.coord.push(k);
            self.edges.push(Vec::new());
            id
        };
        self.by_coord.insert(k, id);
        id
    }

    /// Remove a cell.
    ///
    /// Mark the cell as available with a tombstone and remove the output edges.
    #[allow(dead_code)]
    pub fn remove(&mut self, k: VoxelKey) -> Option<CellId> {
        let id = self.by_coord.remove(&k)?;
        let outbound = std::mem::take(&mut self.edges[id as usize]);
        for e in &outbound {
            let neigh = &mut self.edges[e.dest as usize];
            neigh.retain(|x| x.dest != id);
        }
        self.coord[id as usize] = TOMBSTONE;
        self.free.push(id);
        Some(id)
    }

    /// XYZ coord to cell ID.
    #[inline]
    pub fn id(&self, k: VoxelKey) -> Option<CellId> {
        self.by_coord.get(&k).copied()
    }

    /// Cell ID to XYZ coord.
    #[inline]
    pub fn coord(&self, id: CellId) -> VoxelKey {
        self.coord[id as usize]
    }

    #[inline]
    pub fn neighbors(&self, id: CellId) -> &[Edge] {
        &self.edges[id as usize]
    }

    #[cfg(test)]
    pub fn add_edge(&mut self, src: CellId, dest: CellId, cost: f32) {
        self.edges[src as usize].push(Edge { dest, cost });
    }

    /// Iterate live cells: (id, outgoing edges).
    pub fn iter(&self) -> impl Iterator<Item = (CellId, &[Edge])> + '_ {
        self.coord.iter().enumerate().filter_map(move |(i, c)| {
            if *c != TOMBSTONE {
                Some((i as CellId, self.edges[i].as_slice()))
            } else {
                None
            }
        })
    }

    /// Mutable per-cell edge iterator over live cells.
    pub fn iter_edges_mut(&mut self) -> impl Iterator<Item = (CellId, &mut Vec<Edge>)> + '_ {
        self.coord
            .iter()
            .zip(self.edges.iter_mut())
            .enumerate()
            .filter_map(|(i, (c, e))| {
                if *c != TOMBSTONE {
                    Some((i as CellId, e))
                } else {
                    None
                }
            })
    }

    pub fn ids(&self) -> impl Iterator<Item = CellId> + '_ {
        self.coord.iter().enumerate().filter_map(|(i, c)| {
            if *c != TOMBSTONE {
                Some(i as CellId)
            } else {
                None
            }
        })
    }
}

/// Group cells by XY column with sorted unique iz per column.
pub fn build_surface_lookup(cells: &[VoxelKey], out: &mut SurfaceLookup) {
    out.clear();
    for &(ix, iy, iz) in cells {
        out.entry((ix, iy)).or_default().push(iz);
    }
    for zs in out.values_mut() {
        zs.sort_unstable();
        zs.dedup();
    }
}

/// Populate cells with surface adjacency from the lookup. Deletes any
/// existing contents
pub fn build_surface_cells(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    voxel_size: f32,
    step_threshold_cells: i32,
) {
    cells.clear();

    let mut keys: Vec<(i32, i32)> = surface_lookup.keys().copied().collect();
    keys.sort_unstable();
    for &(ix, iy) in &keys {
        for &iz in &surface_lookup[&(ix, iy)] {
            cells.insert((ix, iy, iz));
        }
    }

    let n = cells.coord.len();
    cells.edges.resize_with(n, Vec::new);
    let coord: &[VoxelKey] = &cells.coord;
    let by_coord: &AHashMap<VoxelKey, CellId> = &cells.by_coord;
    cells
        .edges
        .par_iter_mut()
        .enumerate()
        .for_each(|(src_id, local)| {
            let (ix, iy, iz) = coord[src_id];
            for (dx, dy) in NEIGHBORS_4 {
                let Some(nzs) = surface_lookup.get(&(ix + dx, iy + dy)) else {
                    continue;
                };
                for &nz in nzs {
                    let dz = nz - iz;
                    if dz.abs() > step_threshold_cells {
                        continue;
                    }
                    let dest = *by_coord
                        .get(&(ix + dx, iy + dy, nz))
                        .expect("neighbor cell exists in lookup");
                    let cost = ((dx * dx + dy * dy + dz * dz) as f32).sqrt() * voxel_size;
                    local.push(Edge { dest, cost });
                }
            }
        });
}

#[cfg(test)]
mod tests {
    use super::*;

    const VOXEL: f32 = 0.1;

    fn approx_eq(a: f32, b: f32) {
        let eps = 1e-5;
        assert!((a - b).abs() < eps, "{a} != {b} (eps {eps})");
    }

    fn build(cells: &[VoxelKey]) -> (SurfaceLookup, SurfaceCells) {
        let mut lookup = SurfaceLookup::new();
        build_surface_lookup(cells, &mut lookup);
        let mut sc = SurfaceCells::default();
        build_surface_cells(&mut sc, &lookup, VOXEL, 2);
        (lookup, sc)
    }

    fn neighbors_of(sc: &SurfaceCells, k: VoxelKey) -> Vec<(VoxelKey, f32)> {
        let id = sc.id(k).expect("cell should exist");
        sc.neighbors(id)
            .iter()
            .map(|e| (sc.coord(e.dest), e.cost))
            .collect()
    }

    #[test]
    fn same_z_neighbors_are_bidirectional() {
        let (_, sc) = build(&[(0, 0, 0), (1, 0, 0)]);
        let a = neighbors_of(&sc, (0, 0, 0));
        let b = neighbors_of(&sc, (1, 0, 0));
        assert_eq!(a.len(), 1);
        assert_eq!(b.len(), 1);
        assert_eq!(a[0].0, (1, 0, 0));
        assert_eq!(b[0].0, (0, 0, 0));
        approx_eq(a[0].1, VOXEL);
        approx_eq(b[0].1, VOXEL);
    }

    #[test]
    fn diagonal_not_connected_under_4_connectivity() {
        let (_, sc) = build(&[(0, 0, 0), (1, 1, 0)]);
        assert!(neighbors_of(&sc, (0, 0, 0)).is_empty());
        assert!(neighbors_of(&sc, (1, 1, 0)).is_empty());
    }

    #[test]
    fn step_threshold_blocks_large_dz() {
        let (_, sc) = build(&[(0, 0, 0), (1, 0, 5)]);
        assert!(neighbors_of(&sc, (0, 0, 0)).is_empty());
        assert!(neighbors_of(&sc, (1, 0, 5)).is_empty());
    }

    #[test]
    fn step_within_threshold_uses_3d_distance() {
        let (_, sc) = build(&[(0, 0, 0), (1, 0, 1)]);
        let expected = (2.0_f32).sqrt() * VOXEL;
        let a = neighbors_of(&sc, (0, 0, 0));
        let b = neighbors_of(&sc, (1, 0, 1));
        assert_eq!(a.len(), 1);
        assert_eq!(b.len(), 1);
        approx_eq(a[0].1, expected);
        approx_eq(b[0].1, expected);
    }

    #[test]
    fn plus_pattern_center_has_four_neighbors() {
        let cells = vec![(0, 0, 0), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)];
        let (_, sc) = build(&cells);
        assert_eq!(neighbors_of(&sc, (0, 0, 0)).len(), 4);
    }

    #[test]
    fn clear_keeps_edge_vec_capacity() {
        let (_, mut sc) = build(&[(0, 0, 0), (1, 0, 0), (2, 0, 0)]);
        let edge_vec_count = sc.edges.len();
        sc.clear();
        assert!(sc.is_empty());
        assert_eq!(sc.edges.len(), edge_vec_count);
    }

    #[test]
    fn remove_keeps_neighbor_cell_ids_stable() {
        let (_, mut sc) = build(&[(0, 0, 0), (1, 0, 0), (2, 0, 0)]);
        let id0 = sc.id((0, 0, 0)).unwrap();
        let id2 = sc.id((2, 0, 0)).unwrap();
        sc.remove((1, 0, 0));
        assert_eq!(sc.id((0, 0, 0)), Some(id0));
        assert_eq!(sc.id((2, 0, 0)), Some(id2));
        assert_eq!(sc.id((1, 0, 0)), None);
        assert!(
            sc.neighbors(id0).is_empty(),
            "back-edge from 0 to 1 must be dropped"
        );
        assert!(
            sc.neighbors(id2).is_empty(),
            "back-edge from 2 to 1 must be dropped"
        );
    }

    #[test]
    fn alloc_after_remove_reuses_freed_slot() {
        let (_, mut sc) = build(&[(0, 0, 0), (1, 0, 0)]);
        let removed_id = sc.remove((1, 0, 0)).unwrap();
        let new_id = sc.insert((5, 5, 0));
        assert_eq!(new_id, removed_id);
        assert_eq!(sc.coord(new_id), (5, 5, 0));
        assert!(sc.is_live(new_id));
    }

    #[test]
    fn live_iter_skips_tombstones() {
        let (_, mut sc) = build(&[(0, 0, 0), (1, 0, 0), (2, 0, 0)]);
        sc.remove((1, 0, 0));
        let live: Vec<VoxelKey> = sc.ids().map(|id| sc.coord(id)).collect();
        assert_eq!(live, vec![(0, 0, 0), (2, 0, 0)]);
    }
}
