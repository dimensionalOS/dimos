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

use crate::fixed_hash::AHashMap;
use rayon::prelude::*;

use crate::lazy::Version;
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

/// What fills the unused tail of an edge block.
const UNUSED_EDGE: Edge = Edge {
    dest: NO_CELL,
    base_cost: f32::INFINITY,
    rise: 0.0,
    cost: f32::INFINITY,
};

/// Coordinate to cell id, sharded by hash so a full build indexes cells in parallel and lookups touch one shard.
pub struct CoordIndex {
    hasher: ahash::RandomState,
    shards: Vec<AHashMap<VoxelKey, CellId>>,
}

const COORD_SHARDS: usize = 64;

impl Default for CoordIndex {
    fn default() -> Self {
        Self {
            hasher: ahash::RandomState::with_seeds(1, 2, 3, 4),
            shards: (0..COORD_SHARDS).map(|_| AHashMap::default()).collect(),
        }
    }
}

impl CoordIndex {
    #[inline]
    fn shard(&self, k: &VoxelKey) -> usize {
        (self.hasher.hash_one(k) >> 58) as usize
    }

    #[inline]
    pub fn get(&self, k: &VoxelKey) -> Option<CellId> {
        self.shards[self.shard(k)].get(k).copied()
    }

    pub fn insert(&mut self, k: VoxelKey, id: CellId) {
        let s = self.shard(&k);
        self.shards[s].insert(k, id);
    }

    pub fn remove(&mut self, k: &VoxelKey) -> Option<CellId> {
        let s = self.shard(k);
        self.shards[s].remove(k)
    }

    pub fn len(&self) -> usize {
        self.shards.iter().map(|s| s.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.shards.iter().all(|s| s.is_empty())
    }

    pub fn keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
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
    /// Surface height of each cell, from its plane or its top face.
    z: Vec<f32>,
    /// Each cell's surface slope when a plane gave its height.
    slope: Vec<Option<Slope>>,
    /// Every cell's outgoing edges in one buffer, each slot's in a block of it; a list that outgrows its block moves
    /// to a free block of a big enough size class, or a new one at the end, so no change rewrites the whole buffer.
    flat: Vec<Edge>,
    /// Per slot: (start, len) into `flat`.
    spans: Vec<(u32, u32)>,
    /// Per slot: the edges its block holds.
    caps: Vec<u32>,
    /// Blocks no slot holds, (start, cap), by size class: floor(log2(cap)).
    free_blocks: Vec<Vec<(u32, u32)>>,
    /// Edges the spans cover.
    live_edges: usize,
    /// The current layer's cells by key; a stale layer's cells keep their slot, edges and key but leave this index.
    by_coord: CoordIndex,
    /// Per slot: the layer the cell's key is in.
    layer: Vec<Version>,
    /// The layer inserts land in.
    current_layer: Version,
    /// The stale layers' cells by key, each in its own version's frame.
    stale: std::collections::BTreeMap<Version, CoordIndex>,
    free: Vec<CellId>,
    /// Slots removed since the last repair finished; corridors may still name them, so they are not reused yet.
    freed: Vec<CellId>,
    /// Whether every id was appended in coordinate order, as a full build does; a reused slot breaks it.
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
        self.live_edges
    }

    /// Clear all vecs but keep allocated capacity.
    pub fn clear(&mut self) {
        self.coord.clear();
        self.z.clear();
        self.slope.clear();
        self.layer.clear();
        self.current_layer = 0;
        self.stale.clear();
        self.by_coord.clear();
        self.free.clear();
        self.freed.clear();
        self.coord_ordered = true;
        self.flat.clear();
        self.spans.clear();
        self.caps.clear();
        self.free_blocks.clear();
        self.live_edges = 0;
    }

    #[inline]
    fn range(&self, id: CellId) -> std::ops::Range<usize> {
        let (start, len) = self.spans[id as usize];
        start as usize..(start + len) as usize
    }

    /// Replace a cell's outgoing edges, in place when they fit its block, else in a block of a size class that fits.
    pub fn set_edges(&mut self, id: CellId, edges: &[Edge]) {
        let slot = id as usize;
        let ((start, len), cap) = (self.spans[slot], self.caps[slot]);
        let n = edges.len();
        self.live_edges = self.live_edges + n - len as usize;
        if n <= cap as usize {
            let start = start as usize;
            self.flat[start..start + n].copy_from_slice(edges);
            self.spans[slot].1 = n as u32;
            return;
        }
        if cap > 0 {
            let class = cap.ilog2() as usize;
            if self.free_blocks.len() <= class {
                self.free_blocks.resize_with(class + 1, Vec::new);
            }
            self.free_blocks[class].push((start, cap));
        }
        // Any block of class k or above holds 2^k >= n edges.
        let k = n.next_power_of_two().ilog2() as usize;
        let reused = (k..self.free_blocks.len()).find_map(|c| self.free_blocks[c].pop());
        let (at, cap) = reused.unwrap_or_else(|| {
            let at = self.flat.len() as u32;
            let cap = 1u32 << k;
            self.flat
                .resize(self.flat.len() + cap as usize, UNUSED_EDGE);
            (at, cap)
        });
        self.flat[at as usize..at as usize + n].copy_from_slice(edges);
        self.spans[slot] = (at, n as u32);
        self.caps[slot] = cap;
    }

    #[inline]
    pub fn is_live(&self, id: CellId) -> bool {
        self.coord[id as usize] != TOMBSTONE
    }

    /// Get or insert a cell with surface height `z` and slope, reusing a freed slot; an existing cell keeps its id.
    pub fn insert(&mut self, k: VoxelKey, (z, slope): Surface) -> CellId {
        debug_assert_ne!(k, TOMBSTONE, "voxel coord collides with tombstone sentinel");
        if let Some(id) = self.by_coord.get(&k) {
            self.z[id as usize] = z;
            self.slope[id as usize] = slope;
            return id;
        }
        let id = if let Some(free_id) = self.free.pop() {
            self.coord[free_id as usize] = k;
            self.z[free_id as usize] = z;
            self.slope[free_id as usize] = slope;
            self.layer[free_id as usize] = self.current_layer;
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
            self.z.push(z);
            self.slope.push(slope);
            self.layer.push(self.current_layer);
            self.spans.push((self.flat.len() as u32, 0));
            self.caps.push(0);
            id
        };
        self.by_coord.insert(k, id);
        id
    }

    /// Remove a current-layer cell, tombstoning its slot and dropping its edges.
    pub fn remove(&mut self, k: VoxelKey) -> Option<CellId> {
        let id = self.by_coord.remove(&k)?;
        self.drop_slot(id);
        Some(id)
    }

    /// Remove a cell of any layer by id; a stale one must already be out of its layer's index.
    pub fn remove_id(&mut self, id: CellId) {
        let (layer, k) = (self.layer[id as usize], self.coord[id as usize]);
        if layer == self.current_layer {
            if self.by_coord.get(&k) == Some(id) {
                self.by_coord.remove(&k);
            }
        } else {
            self.leave_stale(id);
        }
        self.drop_slot(id);
    }

    /// Take a stale cell out of its layer's index, dropping an emptied layer.
    fn leave_stale(&mut self, id: CellId) {
        let (layer, k) = (self.layer[id as usize], self.coord[id as usize]);
        if let Some(index) = self.stale.get_mut(&layer) {
            if index.get(&k) == Some(id) {
                index.remove(&k);
            }
            if index.is_empty() {
                self.stale.remove(&layer);
            }
        }
    }

    /// A cell by key in any layer: the current one first, then the stale ones newest first. For stored corridors, whose hops are checked.
    pub fn resolve(&self, k: VoxelKey) -> Option<CellId> {
        self.by_coord
            .get(&k)
            .or_else(|| self.stale.values().rev().find_map(|index| index.get(&k)))
    }

    /// A stale layer's cell by key.
    pub fn stale_id(&self, layer: Version, k: VoxelKey) -> Option<CellId> {
        self.stale.get(&layer)?.get(&k)
    }

    fn drop_slot(&mut self, id: CellId) {
        let outbound: Vec<CellId> = self.neighbors(id).iter().map(|e| e.dest).collect();
        self.live_edges -= self.spans[id as usize].1 as usize;
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
            self.live_edges -= list.len() - kept;
            self.spans[dest as usize].1 = kept as u32;
        }
        self.coord[id as usize] = TOMBSTONE;
        self.freed.push(id);
    }

    /// Let inserts reuse the slots removed since the last call; called once a repair has dropped their corridors.
    pub fn recycle_freed(&mut self) {
        self.free.append(&mut self.freed);
    }

    /// Whether an edge from `a` to `b` is one key adjacency would not rebuild: across layers, or not between 4-neighbor columns.
    pub fn stretched(&self, a: CellId, b: CellId) -> bool {
        if !self.is_live(b) {
            return false;
        }
        if self.layer(a) != self.layer(b) {
            return true;
        }
        let (ka, kb) = (self.coord(a), self.coord(b));
        (ka.0 - kb.0).abs() + (ka.1 - kb.1).abs() != 1
    }

    /// The layer a cell's key is in.
    #[inline]
    pub fn layer(&self, id: CellId) -> Version {
        self.layer[id as usize]
    }

    pub fn current_layer(&self) -> Version {
        self.current_layer
    }

    /// Start a new current layer; the old one's cells keep their ids, edges and keys, indexed as stale.
    pub fn begin_layer(&mut self, layer: Version) {
        let old = std::mem::take(&mut self.by_coord);
        if !old.is_empty() {
            self.stale.insert(self.current_layer, old);
        }
        self.current_layer = layer;
    }

    /// Move a cell into the current layer at a free key, keeping its id and edges.
    pub fn move_to_layer(&mut self, id: CellId, to: VoxelKey) {
        debug_assert!(self.by_coord.get(&to).is_none(), "move into a taken key");
        if self.layer[id as usize] != self.current_layer {
            self.leave_stale(id);
        } else {
            self.by_coord.remove(&self.coord[id as usize]);
        }
        self.coord[id as usize] = to;
        self.layer[id as usize] = self.current_layer;
        self.by_coord.insert(to, id);
        self.coord_ordered = false;
    }

    /// Add an edge unless `src` already has one to `dest`.
    pub fn link(&mut self, src: CellId, edge: Edge) {
        if self.neighbors(src).iter().any(|e| e.dest == edge.dest) {
            return;
        }
        let mut edges = self.neighbors(src).to_vec();
        edges.push(edge);
        self.set_edges(src, &edges);
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

    /// The height change between cells' centers: by their planes when both have one, else by whole-voxel levels.
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

/// The cell store's read-only half, borrowed apart from its edges.
struct CellView<'a> {
    coord: &'a [VoxelKey],
    z: &'a [f32],
    slope: &'a [Option<Slope>],
    by_coord: &'a CoordIndex,
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

/// Fill `out` with `src`'s edges to each 4-neighbor its surface steps onto, costed by 3D surface distance.
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
            let dest = view
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
    keys.par_sort_unstable();
    let total: usize = surface_lookup.values().map(Vec::len).sum();
    cells.coord.reserve(total);
    for &(ix, iy) in &keys {
        for &iz in &surface_lookup[&(ix, iy)] {
            cells.coord.push((ix, iy, iz));
        }
    }
    // Ids in coordinate order, surfaces and the index filled in parallel.
    let n = cells.coord.len();
    let surfaces: Vec<Surface> = cells
        .coord
        .par_iter()
        .map(|&k| terrain.surface(k))
        .collect();
    (cells.z, cells.slope) = surfaces.into_iter().unzip();
    cells.layer.resize(n, cells.current_layer);
    cells.by_coord.fill(&cells.coord);

    // Each block of ids builds its edges in parallel, then the blocks are laid out in id order.
    const BLOCK: usize = 4096;
    let view = cells.view();
    let blocks: Vec<(Vec<u32>, Vec<Edge>)> = (0..n)
        .collect::<Vec<usize>>()
        .par_chunks(BLOCK)
        .map(|ids| {
            let (mut lens, mut flat, mut out) =
                (Vec::with_capacity(ids.len()), Vec::new(), Vec::new());
            for &id in ids {
                edges_of(&view, surface_lookup, terrain, id as CellId, &mut out);
                lens.push(out.len() as u32);
                flat.extend_from_slice(&out);
            }
            (lens, flat)
        })
        .collect();
    cells.spans.clear();
    cells.spans.reserve(n);
    cells.caps.clear();
    cells.caps.reserve(n);
    cells.free_blocks.clear();
    let mut at = 0u32;
    for (lens, flat) in blocks {
        for len in lens {
            cells.spans.push((at, len));
            cells.caps.push(len);
            at += len;
        }
        cells.flat.extend(flat);
    }
    cells.live_edges = cells.flat.len();
}

/// `build_surface_cells` over whole voxels alone, for a robot that steps `step_cells` voxels.
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
            // Links to another layer, or past the 4-neighborhood a reform stretched them across, are kept as they are.
            edges.extend(
                cells
                    .neighbors(id)
                    .iter()
                    .filter(|e| cells.stretched(id, e.dest))
                    .copied(),
            );
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
        let mut lookup = SurfaceLookup::default();
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
    fn edge_lists_that_grow_and_shrink_reuse_blocks_instead_of_growing_the_buffer() {
        let keys: Vec<VoxelKey> = (0..200).map(|i| (i, 0, 0)).collect();
        let (_, mut sc) = build(&keys);
        let edge = |dest: CellId| Edge {
            dest,
            base_cost: 1.0,
            rise: 0.0,
            cost: 1.0,
        };
        let mut high_water = 0;
        for round in 0..50u32 {
            for id in 0..200u32 {
                let n = ((id + round) % 9) as usize;
                let edges: Vec<Edge> = (0..n as u32).map(|k| edge((id + k) % 200)).collect();
                sc.set_edges(id, &edges);
                let dests: Vec<CellId> = sc.neighbors(id).iter().map(|e| e.dest).collect();
                assert_eq!(dests, edges.iter().map(|e| e.dest).collect::<Vec<_>>());
            }
            if round == 10 {
                high_water = sc.flat.len();
            }
        }
        let live: usize = (0..200).map(|id| sc.neighbors(id).len()).sum();
        assert_eq!(sc.edge_count(), live);
        // Once every size has been through every slot, freed blocks cover the moves.
        assert!(
            sc.flat.len() <= high_water + 16 * 9,
            "{} edges held against {high_water} at round 10",
            sc.flat.len()
        );
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
    fn a_freed_slot_is_reused_only_after_the_repair_recycles_it() {
        let (_, mut sc) = build(&[(0, 0, 0), (1, 0, 0)]);
        let removed_id = sc.remove((1, 0, 0)).unwrap();
        let before = sc.insert((6, 6, 0), (VOXEL, None));
        assert_ne!(
            before, removed_id,
            "a corridor may still name the removed slot"
        );
        sc.recycle_freed();
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
