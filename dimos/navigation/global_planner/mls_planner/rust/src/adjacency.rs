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

//! Surface cells indexed by dense CellId.
//!
//! A slot map: inserts reuse freed slots marked with a tombstone, or grow.

use ahash::AHashMap;
use rayon::prelude::*;

use crate::terrain::{climb, surface_gap, within_step, Gap, Slope, Surface, Terrain};
use crate::voxel::VoxelKey;

pub type SurfaceLookup = AHashMap<(i32, i32), Vec<i32>>;

/// Index of a surface voxel.
pub type CellId = u32;
pub const NO_CELL: CellId = u32::MAX;

/// Marks a freed slot reusable by the next insert.
const TOMBSTONE: VoxelKey = (i32::MIN, i32::MIN, i32::MIN);
const NEIGHBORS_4: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub dest: CellId,
    /// Geometric cost, set at build time and never mutated.
    pub base_cost: f32,
    /// Vertical change of the edge in meters.
    pub rise: f32,
    /// base_cost scaled by the wall penalty plus the step penalty.
    pub cost: f32,
}

#[derive(Default)]
pub struct SurfaceCells {
    coord: Vec<VoxelKey>,
    /// Surface height of each cell, from its plane or its top face.
    z: Vec<f32>,
    /// Each cell's surface slope when a plane gave its height.
    slope: Vec<Option<Slope>>,
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

    /// Clear all vecs but keep allocated capacity.
    pub fn clear(&mut self) {
        self.coord.clear();
        self.z.clear();
        self.slope.clear();
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

    /// Get or insert a cell with its surface height `z` and plane slope,
    /// reusing a freed slot before growing. An existing cell keeps its id and
    /// takes the new surface.
    pub fn insert(&mut self, k: VoxelKey, (z, slope): Surface) -> CellId {
        debug_assert_ne!(k, TOMBSTONE, "voxel coord collides with tombstone sentinel");
        if let Some(&id) = self.by_coord.get(&k) {
            self.z[id as usize] = z;
            self.slope[id as usize] = slope;
            return id;
        }
        let id = if let Some(free_id) = self.free.pop() {
            self.coord[free_id as usize] = k;
            self.z[free_id as usize] = z;
            self.slope[free_id as usize] = slope;
            free_id
        } else {
            let id = self.coord.len() as CellId;
            self.coord.push(k);
            self.z.push(z);
            self.slope.push(slope);
            self.edges.push(Vec::new());
            id
        };
        self.by_coord.insert(k, id);
        id
    }

    /// Remove a cell, tombstoning its slot and dropping its edges.
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

    /// Coord to cell ID.
    #[inline]
    pub fn id(&self, k: VoxelKey) -> Option<CellId> {
        self.by_coord.get(&k).copied()
    }

    /// Cell ID to coord.
    #[inline]
    pub fn coord(&self, id: CellId) -> VoxelKey {
        self.coord[id as usize]
    }

    /// Cell ID to its surface point: the column center at the surface height.
    #[inline]
    pub fn point(&self, id: CellId, voxel_size: f32) -> (f32, f32, f32) {
        let (ix, iy, _) = self.coord[id as usize];
        (
            (ix as f32 + 0.5) * voxel_size,
            (iy as f32 + 0.5) * voxel_size,
            self.z[id as usize],
        )
    }

    #[inline]
    pub fn z(&self, id: CellId) -> f32 {
        self.z[id as usize]
    }

    #[inline]
    pub fn surface(&self, id: CellId) -> Surface {
        (self.z[id as usize], self.slope[id as usize])
    }

    /// The height change from cell `a`'s center to cell `b`'s: between their
    /// planes when both have one, else between whole-voxel levels.
    pub fn rise(&self, a: CellId, b: CellId, voxel_size: f32) -> f32 {
        self.view().rise(a, b, voxel_size)
    }

    /// The height gap the robot steps from cell `a` to cell `b`.
    pub fn gap(&self, a: CellId, b: CellId, voxel_size: f32) -> Gap {
        self.view().gap(a, b, voxel_size)
    }

    fn view(&self) -> CellView<'_> {
        CellView {
            coord: &self.coord,
            z: &self.z,
            slope: &self.slope,
            by_coord: &self.by_coord,
        }
    }

    #[inline]
    pub fn neighbors(&self, id: CellId) -> &[Edge] {
        &self.edges[id as usize]
    }

    #[inline]
    pub fn edges_mut(&mut self, id: CellId) -> &mut [Edge] {
        &mut self.edges[id as usize]
    }

    #[cfg(test)]
    pub fn add_edge(&mut self, src: CellId, dest: CellId, cost: f32) {
        self.edges[src as usize].push(Edge {
            dest,
            base_cost: cost,
            rise: 0.0,
            cost,
        });
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

/// The cell store's read-only half, borrowed apart from its edges.
struct CellView<'a> {
    coord: &'a [VoxelKey],
    z: &'a [f32],
    slope: &'a [Option<Slope>],
    by_coord: &'a AHashMap<VoxelKey, CellId>,
}

impl CellView<'_> {
    /// See `SurfaceCells::gap`.
    fn gap(&self, a: CellId, b: CellId, voxel_size: f32) -> Gap {
        let (a, b) = (a as usize, b as usize);
        surface_gap(
            (self.z[a], self.slope[a]),
            (self.z[b], self.slope[b]),
            (self.coord[a], self.coord[b]),
            voxel_size,
        )
    }

    /// See `SurfaceCells::rise`.
    fn rise(&self, a: CellId, b: CellId, voxel_size: f32) -> f32 {
        let (a, b) = (a as usize, b as usize);
        if self.slope[a].is_some() && self.slope[b].is_some() {
            (self.z[b] - self.z[a]).abs()
        } else {
            (self.coord[b].2 - self.coord[a].2).unsigned_abs() as f32 * voxel_size
        }
    }
}

/// Fill `out` with the edges from `src` to every 4-neighbor cell its surface
/// steps onto, each costed by the 3D distance between their surface points.
fn edges_of(
    view: &CellView,
    surface_lookup: &SurfaceLookup,
    terrain: &Terrain,
    src: CellId,
    out: &mut Vec<Edge>,
) {
    out.clear();
    let reach = terrain.step_reach_cells();
    let (ix, iy, iz) = view.coord[src as usize];
    for (dx, dy) in NEIGHBORS_4 {
        let Some(nzs) = surface_lookup.get(&(ix + dx, iy + dy)) else {
            continue;
        };
        for &nz in nzs {
            let dest_key = (ix + dx, iy + dy, nz);
            if (nz - iz).abs() > reach {
                continue;
            }
            let dest = *view
                .by_coord
                .get(&dest_key)
                .expect("neighbor cell exists in lookup");
            let gap = view.gap(src, dest, terrain.voxel_size);
            if !within_step(gap, terrain.voxel_size, terrain.step_m, terrain.noise_m) {
                continue;
            }
            let rise = climb(view.rise(src, dest, terrain.voxel_size), terrain.noise_m);
            let cost = terrain.voxel_size.hypot(rise);
            out.push(Edge {
                dest,
                base_cost: cost,
                rise,
                cost,
            });
        }
    }
}

/// Build surface adjacency from the lookup, replacing any existing contents.
pub fn build_surface_cells(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    terrain: &Terrain,
) {
    cells.clear();

    let mut keys: Vec<(i32, i32)> = surface_lookup.keys().copied().collect();
    keys.sort_unstable();
    for &(ix, iy) in &keys {
        for &iz in &surface_lookup[&(ix, iy)] {
            cells.insert((ix, iy, iz), terrain.surface((ix, iy, iz)));
        }
    }

    let n = cells.coord.len();
    cells.edges.resize_with(n, Vec::new);
    let view = CellView {
        coord: &cells.coord,
        z: &cells.z,
        slope: &cells.slope,
        by_coord: &cells.by_coord,
    };
    cells
        .edges
        .par_iter_mut()
        .enumerate()
        .for_each(|(src, out)| edges_of(&view, surface_lookup, terrain, src as CellId, out));
}

/// `build_surface_cells` over whole voxels with no map around them, for a
/// robot that steps `step_cells` voxels.
#[cfg(test)]
pub(crate) fn build_whole_voxel_cells(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    voxel_size: f32,
    step_cells: i32,
) {
    let (by_col, planes) = (
        crate::columns::ColumnIz::default(),
        crate::terrain::PlaneMap::new(),
    );
    let terrain = Terrain {
        by_col: &by_col,
        planes: &planes,
        voxel_size,
        robot_height: voxel_size,
        step_m: step_cells as f32 * voxel_size,
        noise_m: 0.0,
        max_slope: 0.0,
    };
    build_surface_cells(cells, surface_lookup, &terrain);
}

/// Recompute outgoing edges for the affected cells. Call after an incremental
/// insert, remove, or height change with the changed cells and their surface
/// neighbors so the region matches a full rebuild.
pub fn rebuild_edges_around(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    affected: &[CellId],
    terrain: &Terrain,
) {
    let view = cells.view();
    let rebuilt: Vec<(CellId, Vec<Edge>)> = affected
        .par_iter()
        .map(|&id| {
            let mut edges = Vec::new();
            edges_of(&view, surface_lookup, terrain, id, &mut edges);
            (id, edges)
        })
        .collect();
    for (id, edges) in rebuilt {
        cells.edges[id as usize] = edges;
    }
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
        build_whole_voxel_cells(&mut sc, &lookup, VOXEL, 2);
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
        let new_id = sc.insert((5, 5, 0), (VOXEL, None));
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
