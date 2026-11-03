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

use crate::voxel::VoxelKey;

pub type SurfaceLookup = AHashMap<(i32, i32), Vec<i32>>;

/// Index of a surface voxel.
pub type CellId = u32;
pub const NO_CELL: CellId = u32::MAX;

/// Marks a freed slot reusable by the next insert.
const TOMBSTONE: VoxelKey = (i32::MIN, i32::MIN, i32::MIN);
const NEIGHBORS_4: [(i32, i32); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];

/// Vertical extent of a dz-cell change in meters, the step-penalty input.
#[inline]
pub fn rise(dz: i32, voxel_size: f32) -> f32 {
    dz.unsigned_abs() as f32 * voxel_size
}

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

/// Coordinate to cell id, split into shards by hash so a full build can index
/// every cell in parallel; lookups and edits touch one shard.
struct CoordIndex {
    hasher: ahash::RandomState,
    shards: Vec<AHashMap<VoxelKey, CellId>>,
}

const COORD_SHARDS: usize = 64;

impl Default for CoordIndex {
    fn default() -> Self {
        Self {
            hasher: ahash::RandomState::with_seeds(1, 2, 3, 4),
            shards: (0..COORD_SHARDS).map(|_| AHashMap::new()).collect(),
        }
    }
}

impl CoordIndex {
    #[inline]
    fn shard(&self, k: &VoxelKey) -> usize {
        (self.hasher.hash_one(k) >> 58) as usize
    }

    #[inline]
    fn get(&self, k: &VoxelKey) -> Option<CellId> {
        self.shards[self.shard(k)].get(k).copied()
    }

    fn insert(&mut self, k: VoxelKey, id: CellId) {
        let s = self.shard(&k);
        self.shards[s].insert(k, id);
    }

    fn remove(&mut self, k: &VoxelKey) -> Option<CellId> {
        let s = self.shard(k);
        self.shards[s].remove(k)
    }

    fn len(&self) -> usize {
        self.shards.iter().map(|s| s.len()).sum()
    }

    fn is_empty(&self) -> bool {
        self.shards.iter().all(|s| s.is_empty())
    }

    fn keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.shards.iter().flat_map(|s| s.keys().copied())
    }

    fn clear(&mut self) {
        for s in &mut self.shards {
            s.clear();
        }
    }

    /// Index every cell, its id its position in `coord`, each shard in parallel.
    fn fill(&mut self, coord: &[VoxelKey]) {
        let mut by_shard: Vec<Vec<(VoxelKey, CellId)>> = vec![Vec::new(); COORD_SHARDS];
        let tagged: Vec<(u8, VoxelKey, CellId)> = coord
            .par_iter()
            .enumerate()
            .map(|(id, k)| (self.shard(k) as u8, *k, id as CellId))
            .collect();
        for (s, k, id) in tagged {
            by_shard[s as usize].push((k, id));
        }
        self.shards
            .par_iter_mut()
            .zip(by_shard)
            .for_each(|(shard, entries)| {
                shard.reserve(entries.len());
                shard.extend(entries);
            });
    }
}

#[derive(Default)]
pub struct SurfaceCells {
    coord: Vec<VoxelKey>,
    /// Per slot: where a warp moved the cell's point off its key's surface
    /// point. Outside the local area a cell moves without being re-keyed.
    offset: Vec<[f32; 3]>,
    /// Every cell's outgoing edges in one buffer, a cell's at its span. A full
    /// build lays them out in id order; an edit that grows a cell's list moves it
    /// to the end, and the buffer compacts once the stranded edges outweigh the live.
    flat: Vec<Edge>,
    /// Per slot: (start, len) into `flat`.
    spans: Vec<(u32, u32)>,
    /// Edges in `flat` no span covers.
    stranded: usize,
    by_coord: CoordIndex,
    free: Vec<CellId>,
    /// Whether every id was appended in coordinate order, as a full build does,
    /// so ids order cells the way coordinates do. A reused slot breaks it.
    coord_ordered: bool,
}

impl SurfaceCells {
    pub fn is_empty(&self) -> bool {
        self.by_coord.is_empty()
    }

    /// Forget that ids follow coordinates, so searches take the general path.
    #[cfg(test)]
    pub fn unorder(&mut self) {
        self.coord_ordered = false;
    }

    /// Whether ids order live cells the way their coordinates do.
    pub fn coord_ordered(&self) -> bool {
        self.coord_ordered
    }

    /// Total slot count, including tombstoned cells.
    pub fn slot_capacity(&self) -> usize {
        self.coord.len()
    }

    /// Every live cell's voxel.
    pub fn live_coords(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.by_coord.keys()
    }

    /// Live cells, excluding tombstones.
    pub fn live_len(&self) -> usize {
        self.by_coord.len()
    }

    /// Edges held across every slot, tombstones included.
    pub fn edge_count(&self) -> usize {
        self.flat.len() - self.stranded
    }

    /// Clear all vecs but keep allocated capacity.
    pub fn clear(&mut self) {
        self.coord.clear();
        self.offset.clear();
        self.by_coord.clear();
        self.free.clear();
        self.coord_ordered = true;
        self.flat.clear();
        self.spans.clear();
        self.stranded = 0;
    }

    #[inline]
    fn range(&self, id: CellId) -> std::ops::Range<usize> {
        let (start, len) = self.spans[id as usize];
        start as usize..(start + len) as usize
    }

    /// Replace a cell's outgoing edges, in place when they fit.
    pub fn set_edges(&mut self, id: CellId, edges: &[Edge]) {
        let (start, len) = self.spans[id as usize];
        if edges.len() <= len as usize {
            let start = start as usize;
            self.flat[start..start + edges.len()].copy_from_slice(edges);
            self.stranded += len as usize - edges.len();
            self.spans[id as usize].1 = edges.len() as u32;
            return;
        }
        self.stranded += len as usize;
        self.spans[id as usize] = (self.flat.len() as u32, edges.len() as u32);
        self.flat.extend_from_slice(edges);
        if self.stranded > self.flat.len() / 2 && self.flat.len() > 4096 {
            self.compact();
        }
    }

    /// Lay the live edges out in id order again, dropping the stranded ones.
    fn compact(&mut self) {
        let mut flat: Vec<Edge> = Vec::with_capacity(self.flat.len() - self.stranded);
        for span in self.spans.iter_mut() {
            let (start, len) = *span;
            *span = (flat.len() as u32, len);
            flat.extend_from_slice(&self.flat[start as usize..(start + len) as usize]);
        }
        self.flat = flat;
        self.stranded = 0;
    }

    #[inline]
    pub fn is_live(&self, id: CellId) -> bool {
        self.coord[id as usize] != TOMBSTONE
    }

    /// Get or insert a cell, reusing a freed slot before growing.
    pub fn insert(&mut self, k: VoxelKey) -> CellId {
        debug_assert_ne!(k, TOMBSTONE, "voxel coord collides with tombstone sentinel");
        if let Some(id) = self.by_coord.get(&k) {
            return id;
        }
        let id = if let Some(free_id) = self.free.pop() {
            self.coord[free_id as usize] = k;
            self.offset[free_id as usize] = [0.0; 3];
            self.coord_ordered = false;
            free_id
        } else {
            let id = self.coord.len() as CellId;
            if self
                .coord
                .last()
                .is_some_and(|&last| last == TOMBSTONE || last >= k)
            {
                self.coord_ordered = false;
            }
            self.coord.push(k);
            self.offset.push([0.0; 3]);
            self.spans.push((self.flat.len() as u32, 0));
            id
        };
        self.by_coord.insert(k, id);
        id
    }

    /// Remove a cell, tombstoning its slot and dropping its edges.
    pub fn remove(&mut self, k: VoxelKey) -> Option<CellId> {
        let id = self.by_coord.remove(&k)?;
        let outbound: Vec<CellId> = self.neighbors(id).iter().map(|e| e.dest).collect();
        self.stranded += self.spans[id as usize].1 as usize;
        self.spans[id as usize].1 = 0;
        for dest in outbound {
            // Drop the edges back to the removed cell, keeping the rest in order.
            let range = self.range(dest);
            let list = &mut self.flat[range.clone()];
            let mut kept = 0;
            for i in 0..list.len() {
                if list[i].dest != id {
                    list[kept] = list[i];
                    kept += 1;
                }
            }
            self.stranded += list.len() - kept;
            self.spans[dest as usize].1 = kept as u32;
        }
        self.coord[id as usize] = TOMBSTONE;
        self.offset[id as usize] = [0.0; 3];
        self.free.push(id);
        Some(id)
    }

    /// Coord to cell ID.
    #[inline]
    pub fn id(&self, k: VoxelKey) -> Option<CellId> {
        self.by_coord.get(&k)
    }

    /// Cell ID to coord.
    #[inline]
    pub fn coord(&self, id: CellId) -> VoxelKey {
        self.coord[id as usize]
    }

    /// How far a warp moved a cell's point off its key's surface point.
    #[inline]
    pub fn offset(&self, id: CellId) -> [f32; 3] {
        self.offset[id as usize]
    }

    /// The key a cell's point sits in now.
    pub fn placed(&self, id: CellId, voxel_size: f32) -> VoxelKey {
        let (ix, iy, iz) = self.coord(id);
        let o = self.offset(id);
        crate::voxel::voxelize(
            (
                (ix as f32 + 0.5) * voxel_size + o[0],
                (iy as f32 + 0.5) * voxel_size + o[1],
                (iz as f32 + 0.5) * voxel_size + o[2],
            ),
            voxel_size,
        )
    }

    /// Move cells' points by `deltas`, keeping their keys.
    pub fn shift(&mut self, ids: &[CellId], deltas: &[[f32; 3]]) {
        for (&id, d) in ids.iter().zip(deltas) {
            let o = &mut self.offset[id as usize];
            for a in 0..3 {
                o[a] += d[a];
            }
        }
    }

    /// Re-key cells, each to a free key, keeping their ids and edges; the
    /// offset left is what the new key does not account for.
    pub fn rekey(&mut self, moves: &[(CellId, VoxelKey)], voxel_size: f32) {
        for &(id, _) in moves {
            self.by_coord.remove(&self.coord[id as usize]);
        }
        for &(id, to) in moves {
            let from = self.coord[id as usize];
            let o = &mut self.offset[id as usize];
            o[0] -= (to.0 - from.0) as f32 * voxel_size;
            o[1] -= (to.1 - from.1) as f32 * voxel_size;
            o[2] -= (to.2 - from.2) as f32 * voxel_size;
            self.coord[id as usize] = to;
            let previous = self.by_coord.get(&to);
            debug_assert!(previous.is_none(), "rekey into a taken key");
            self.by_coord.insert(to, id);
        }
        if !moves.is_empty() {
            self.coord_ordered = false;
        }
    }

    #[inline]
    pub fn neighbors(&self, id: CellId) -> &[Edge] {
        &self.flat[self.range(id)]
    }

    #[inline]
    pub fn edges_mut(&mut self, id: CellId) -> &mut [Edge] {
        let range = self.range(id);
        &mut self.flat[range]
    }

    #[cfg(test)]
    pub fn add_edge(&mut self, src: CellId, dest: CellId, cost: f32) {
        let mut edges = self.neighbors(src).to_vec();
        edges.push(Edge {
            dest,
            base_cost: cost,
            rise: 0.0,
            cost,
        });
        self.set_edges(src, &edges);
    }

    /// Iterate live cells: (id, outgoing edges).
    pub fn iter(&self) -> impl Iterator<Item = (CellId, &[Edge])> + '_ {
        self.coord.iter().enumerate().filter_map(move |(i, c)| {
            if *c != TOMBSTONE {
                Some((i as CellId, self.neighbors(i as CellId)))
            } else {
                None
            }
        })
    }

    /// Every live cell's outgoing edges, mutable and disjoint, for parallel passes.
    pub fn edge_lists_mut(&mut self) -> Vec<(CellId, &mut [Edge])> {
        let mut live: Vec<(u32, u32, CellId)> = self
            .coord
            .iter()
            .zip(&self.spans)
            .enumerate()
            .filter(|(_, (c, (_, len)))| **c != TOMBSTONE && *len > 0)
            .map(|(i, (_, &(start, len)))| (start, len, i as CellId))
            .collect();
        if !live.windows(2).all(|w| w[0].0 <= w[1].0) {
            live.sort_unstable();
        }
        let mut out = Vec::with_capacity(live.len());
        let mut rest: &mut [Edge] = &mut self.flat;
        let mut at = 0u32;
        for (start, len, id) in live {
            let (_, tail) = rest.split_at_mut((start - at) as usize);
            let (list, tail) = tail.split_at_mut(len as usize);
            out.push((id, list));
            rest = tail;
            at = start + len;
        }
        out
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

/// Build surface adjacency from the lookup, replacing any existing contents.
pub fn build_surface_cells(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    voxel_size: f32,
    step_threshold_cells: i32,
) {
    cells.clear();

    let mut keys: Vec<(i32, i32)> = surface_lookup.keys().copied().collect();
    keys.par_sort_unstable();
    let total: usize = surface_lookup.values().map(Vec::len).sum();
    cells.coord.reserve(total);
    for &(ix, iy) in &keys {
        for &iz in &surface_lookup[&(ix, iy)] {
            cells.coord.push((ix, iy, iz));
        }
    }
    // Ids in coordinate order, indexed in parallel.
    cells.spans.resize(cells.coord.len(), (0, 0));
    cells.offset.resize(cells.coord.len(), [0.0; 3]);
    cells.by_coord.fill(&cells.coord);

    // Count each cell's edges, lay the spans out in id order, then fill every
    // block of cells' slice of the buffer in parallel, with no temporaries.
    const BLOCK: usize = 4096;
    let coord: &[VoxelKey] = &cells.coord;
    let neighbors = |(ix, iy, iz): VoxelKey| {
        NEIGHBORS_4.into_iter().flat_map(move |(dx, dy)| {
            surface_lookup
                .get(&(ix + dx, iy + dy))
                .into_iter()
                .flatten()
                .filter(move |&&nz| (nz - iz).abs() <= step_threshold_cells)
                .map(move |&nz| (dx, dy, nz))
        })
    };
    cells
        .spans
        .par_iter_mut()
        .zip(coord.par_iter())
        .for_each(|(span, &k)| span.1 = neighbors(k).count() as u32);
    let mut total = 0u32;
    for span in cells.spans.iter_mut() {
        span.0 = total;
        total += span.1;
    }
    let blank = Edge {
        dest: 0,
        base_cost: 0.0,
        rise: 0.0,
        cost: 0.0,
    };
    cells.flat.resize(total as usize, blank);
    let mut slices: Vec<&mut [Edge]> = Vec::with_capacity(coord.len().div_ceil(BLOCK));
    let mut rest: &mut [Edge] = &mut cells.flat;
    for block in cells.spans.chunks(BLOCK) {
        let len: u32 = block.iter().map(|s| s.1).sum();
        let (head, tail) = rest.split_at_mut(len as usize);
        slices.push(head);
        rest = tail;
    }
    let by_coord: &CoordIndex = &cells.by_coord;
    slices
        .into_par_iter()
        .zip(coord.par_chunks(BLOCK))
        .for_each(|(slice, block)| {
            let mut at = 0;
            for &(ix, iy, iz) in block {
                for (dx, dy, nz) in neighbors((ix, iy, iz)) {
                    let dz = nz - iz;
                    let dest = by_coord
                        .get(&(ix + dx, iy + dy, nz))
                        .expect("neighbor cell exists in lookup");
                    let cost = ((dx * dx + dy * dy + dz * dz) as f32).sqrt() * voxel_size;
                    slice[at] = Edge {
                        dest,
                        base_cost: cost,
                        rise: rise(dz, voxel_size),
                        cost,
                    };
                    at += 1;
                }
            }
        });
}

/// Recompute outgoing edges for the affected cells. Call after an incremental
/// insert or remove with the changed cells and their surface neighbors so the
/// region matches a full rebuild.
pub fn rebuild_edges_around(
    cells: &mut SurfaceCells,
    surface_lookup: &SurfaceLookup,
    affected: &[CellId],
    voxel_size: f32,
    step_threshold_cells: i32,
) {
    let rebuilt: Vec<(CellId, Vec<Edge>)> = affected
        .par_iter()
        .map(|&id| {
            let (ix, iy, iz) = cells.coord(id);
            let mut edges: Vec<Edge> = Vec::new();
            for (dx, dy) in NEIGHBORS_4 {
                let Some(nzs) = surface_lookup.get(&(ix + dx, iy + dy)) else {
                    continue;
                };
                for &nz in nzs {
                    let dz = nz - iz;
                    if dz.abs() > step_threshold_cells {
                        continue;
                    }
                    let dest = cells
                        .id((ix + dx, iy + dy, nz))
                        .expect("neighbor cell exists in lookup");
                    let cost = ((dx * dx + dy * dy + dz * dz) as f32).sqrt() * voxel_size;
                    edges.push(Edge {
                        dest,
                        base_cost: cost,
                        rise: rise(dz, voxel_size),
                        cost,
                    });
                }
            }
            (id, edges)
        })
        .collect();
    for (id, edges) in rebuilt {
        cells.set_edges(id, &edges);
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
        let capacity = sc.flat.capacity();
        sc.clear();
        assert!(sc.is_empty());
        assert_eq!(sc.flat.capacity(), capacity);
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
