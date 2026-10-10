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

//! The occupied z values of every xy column, stored in square tiles so a
//! column lookup is one hash probe per tile and an array index, and a scan
//! over a region walks contiguous memory.

use ahash::AHashMap;
use rayon::prelude::*;
use smallvec::SmallVec;

use crate::voxel::VoxelKey;

const TILE_BITS: i32 = 5;
/// Columns per tile edge.
pub const TILE_EDGE: i32 = 1 << TILE_BITS;
const TILE_MASK: i32 = TILE_EDGE - 1;
const TILE_COLUMNS: usize = (TILE_EDGE * TILE_EDGE) as usize;

/// A column's sorted z values. Most columns hold a floor and little else, so
/// a few fit without a heap allocation.
type Column = SmallVec<[i32; 4]>;

pub type Tile = (i32, i32);

#[inline]
fn tile_of((ix, iy): (i32, i32)) -> Tile {
    (ix >> TILE_BITS, iy >> TILE_BITS)
}

#[inline]
fn slot_of((ix, iy): (i32, i32)) -> usize {
    (((iy & TILE_MASK) << TILE_BITS) | (ix & TILE_MASK)) as usize
}

#[inline]
fn column_of(tile: Tile, slot: usize) -> (i32, i32) {
    let s = slot as i32;
    (
        (tile.0 << TILE_BITS) | (s & TILE_MASK),
        (tile.1 << TILE_BITS) | (s >> TILE_BITS),
    )
}

/// Occupied z per column, by tile.
#[derive(Default)]
pub struct ColumnIz {
    tiles: AHashMap<Tile, Vec<Column>>,
}

impl ColumnIz {
    /// The sorted z values of a column, None when it holds no voxel.
    pub fn get(&self, col: (i32, i32)) -> Option<&[i32]> {
        let zs = self.tiles.get(&tile_of(col))?[slot_of(col)].as_slice();
        (!zs.is_empty()).then_some(zs)
    }

    fn column_mut(&mut self, col: (i32, i32)) -> &mut Column {
        let tile = self
            .tiles
            .entry(tile_of(col))
            .or_insert_with(|| vec![Column::new(); TILE_COLUMNS]);
        &mut tile[slot_of(col)]
    }

    /// Add one voxel, keeping its column sorted.
    pub fn add(&mut self, (ix, iy, iz): VoxelKey) {
        let zs = self.column_mut((ix, iy));
        if let Err(pos) = zs.binary_search(&iz) {
            zs.insert(pos, iz);
        }
    }

    /// Remove one voxel if present.
    pub fn remove(&mut self, (ix, iy, iz): VoxelKey) {
        if let Some(tile) = self.tiles.get_mut(&tile_of((ix, iy))) {
            let zs = &mut tile[slot_of((ix, iy))];
            if let Ok(pos) = zs.binary_search(&iz) {
                zs.remove(pos);
            }
        }
    }

    pub fn clear(&mut self) {
        self.tiles.clear();
    }

    /// Rebuild from a set of voxels.
    pub fn from_voxels<'a>(voxels: impl IntoIterator<Item = &'a VoxelKey>) -> Self {
        let mut out = Self::default();
        for &(ix, iy, iz) in voxels {
            out.column_mut((ix, iy)).push(iz);
        }
        out.tiles.par_iter_mut().for_each(|(_, tile)| {
            for zs in tile.iter_mut() {
                zs.sort_unstable();
                zs.dedup();
            }
        });
        out
    }

    /// Every occupied column with its z values, tile by tile in parallel.
    pub fn par_columns(&self) -> impl ParallelIterator<Item = ((i32, i32), &[i32])> + '_ {
        self.tiles.par_iter().flat_map_iter(|(&tile, columns)| {
            columns
                .iter()
                .enumerate()
                .filter(|(_, zs)| !zs.is_empty())
                .map(move |(slot, zs)| (column_of(tile, slot), zs.as_slice()))
        })
    }

    /// The tiles covering an inclusive column box, present or not.
    pub fn tiles_covering((x0, x1, y0, y1): (i32, i32, i32, i32)) -> Vec<Tile> {
        let (tx0, tx1) = (x0 >> TILE_BITS, x1 >> TILE_BITS);
        let (ty0, ty1) = (y0 >> TILE_BITS, y1 >> TILE_BITS);
        let mut tiles = Vec::with_capacity(((tx1 - tx0 + 1) * (ty1 - ty0 + 1)).max(0) as usize);
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                tiles.push((tx, ty));
            }
        }
        tiles
    }

    /// Every column of one tile inside the box, with its z values, empty for
    /// columns holding no voxel.
    pub fn tile_columns_in(
        &self,
        tile: Tile,
        (x0, x1, y0, y1): (i32, i32, i32, i32),
    ) -> impl Iterator<Item = ((i32, i32), &[i32])> + '_ {
        let columns = self.tiles.get(&tile).map(Vec::as_slice);
        let (ox, oy) = (tile.0 << TILE_BITS, tile.1 << TILE_BITS);
        let xs = x0.max(ox)..=x1.min(ox + TILE_MASK);
        let ys = y0.max(oy)..=y1.min(oy + TILE_MASK);
        ys.flat_map(move |iy| {
            xs.clone().map(move |ix| {
                let zs = columns.map_or(&[][..], |c| c[slot_of((ix, iy))].as_slice());
                ((ix, iy), zs)
            })
        })
    }
}
