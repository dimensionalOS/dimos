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

use crate::bit_grid::BitGrid;
use crate::voxel::VoxelKey;

/// Inclusive column window, as x0, x1, y0, y1.
pub type ColumnWindow = (i32, i32, i32, i32);

const TILE_BITS: i32 = 5;
/// Columns per tile edge.
const TILE_EDGE: i32 = 1 << TILE_BITS;
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
    let slot = slot as i32;
    (
        (tile.0 << TILE_BITS) | (slot & TILE_MASK),
        (tile.1 << TILE_BITS) | (slot >> TILE_BITS),
    )
}

/// One tile's columns and how many voxels they hold.
struct TileColumns {
    columns: Vec<Column>,
    voxels: usize,
}

impl TileColumns {
    fn new() -> Self {
        Self {
            columns: vec![Column::new(); TILE_COLUMNS],
            voxels: 0,
        }
    }
}

/// Occupied z per column, by tile. A tile is dropped with its last voxel.
#[derive(Default)]
pub struct ColumnIz {
    tiles: AHashMap<Tile, TileColumns>,
}

impl ColumnIz {
    /// The sorted z values of a column, None when it holds no voxel.
    pub fn get(&self, col: (i32, i32)) -> Option<&[i32]> {
        let zs = self.tiles.get(&tile_of(col))?.columns[slot_of(col)].as_slice();
        (!zs.is_empty()).then_some(zs)
    }

    fn tile_mut(&mut self, col: (i32, i32)) -> &mut TileColumns {
        self.tiles
            .entry(tile_of(col))
            .or_insert_with(TileColumns::new)
    }

    /// Add one voxel, keeping its column sorted.
    pub fn add(&mut self, (ix, iy, iz): VoxelKey) {
        let tile = self.tile_mut((ix, iy));
        let zs = &mut tile.columns[slot_of((ix, iy))];
        if let Err(pos) = zs.binary_search(&iz) {
            zs.insert(pos, iz);
            tile.voxels += 1;
        }
    }

    /// Remove one voxel if present.
    pub fn remove(&mut self, (ix, iy, iz): VoxelKey) {
        let key = tile_of((ix, iy));
        let Some(tile) = self.tiles.get_mut(&key) else {
            return;
        };
        let zs = &mut tile.columns[slot_of((ix, iy))];
        let Ok(pos) = zs.binary_search(&iz) else {
            return;
        };
        zs.remove(pos);
        tile.voxels -= 1;
        if tile.voxels == 0 {
            self.tiles.remove(&key);
        }
    }

    pub fn clear(&mut self) {
        self.tiles.clear();
    }

    /// Rebuild from a set of voxels.
    pub fn from_voxels<'a>(voxels: impl IntoIterator<Item = &'a VoxelKey>) -> Self {
        let mut out = Self::default();
        for &(ix, iy, iz) in voxels {
            out.tile_mut((ix, iy)).columns[slot_of((ix, iy))].push(iz);
        }
        out.tiles.par_iter_mut().for_each(|(_, tile)| {
            for zs in tile.columns.iter_mut() {
                zs.sort_unstable();
                zs.dedup();
                tile.voxels += zs.len();
            }
        });
        out
    }

    /// Every occupied column with its z values, tile by tile in parallel.
    pub fn par_columns(&self) -> impl ParallelIterator<Item = ((i32, i32), &[i32])> + '_ {
        self.tiles.par_iter().flat_map_iter(|(&tile, held)| {
            held.columns
                .iter()
                .enumerate()
                .filter(|(_, zs)| !zs.is_empty())
                .map(move |(slot, zs)| (column_of(tile, slot), zs.as_slice()))
        })
    }

    /// The tiles covering an inclusive column box, present or not.
    pub fn tiles_covering((x0, x1, y0, y1): ColumnWindow) -> Vec<Tile> {
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
        (x0, x1, y0, y1): ColumnWindow,
    ) -> impl Iterator<Item = ((i32, i32), &[i32])> + '_ {
        let columns = self.tiles.get(&tile).map(|held| held.columns.as_slice());
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

/// Dense set of columns over an inclusive box, for change footprints that
/// dilate by the morphology reach.
#[derive(Clone)]
pub struct ColumnMask {
    x0: i32,
    y0: i32,
    grid: BitGrid,
    /// Inclusive row range holding set bits, so scans skip the rest.
    rows_set: Option<(usize, usize)>,
}

impl ColumnMask {
    /// An empty mask over the box widened by `margin` on every side.
    pub fn new((x0, x1, y0, y1): ColumnWindow, margin: i32) -> Self {
        let width = (x1 - x0 + 1 + 2 * margin).max(0) as usize;
        let height = (y1 - y0 + 1 + 2 * margin).max(0) as usize;
        Self {
            x0: x0 - margin,
            y0: y0 - margin,
            grid: BitGrid::new(width, height),
            rows_set: None,
        }
    }

    fn local(&self, (ix, iy): (i32, i32)) -> Option<(usize, usize)> {
        let (x, y) = (ix - self.x0, iy - self.y0);
        (x >= 0 && y >= 0 && (x as usize) < self.grid.width() && (y as usize) < self.grid.height())
            .then_some((x as usize, y as usize))
    }

    /// Mark a column. Columns outside the box are ignored.
    pub fn set(&mut self, col: (i32, i32)) {
        if let Some((x, y)) = self.local(col) {
            self.grid.set(x, y);
            self.rows_set = Some(match self.rows_set {
                None => (y, y),
                Some((lo, hi)) => (lo.min(y), hi.max(y)),
            });
        }
    }

    pub fn contains(&self, col: (i32, i32)) -> bool {
        self.local(col).is_some_and(|(x, y)| self.grid.get(x, y))
    }

    /// Every column within `r` of a set column, clipped to the box.
    pub fn dilated(&self, r: i32) -> ColumnMask {
        let Some((lo, hi)) = self.rows_set else {
            return self.clone();
        };
        let r = r.max(0) as usize;
        let last_row = self.grid.height() - 1;
        ColumnMask {
            x0: self.x0,
            y0: self.y0,
            grid: self.grid.square_dilated(r, (lo, hi)),
            rows_set: Some((lo.saturating_sub(r), (hi + r).min(last_row))),
        }
    }

    /// Inclusive bbox of the set columns, None when empty.
    pub fn bounds(&self) -> Option<ColumnWindow> {
        let mut bb: Option<ColumnWindow> = None;
        for y in self.rows() {
            let Some((xmin, xmax)) = self.grid.row_span(y) else {
                continue;
            };
            let (ix0, ix1, iy) = (
                self.x0 + xmin as i32,
                self.x0 + xmax as i32,
                self.y0 + y as i32,
            );
            bb = Some(match bb {
                None => (ix0, ix1, iy, iy),
                Some((bx0, bx1, by0, by1)) => {
                    (bx0.min(ix0), bx1.max(ix1), by0.min(iy), by1.max(iy))
                }
            });
        }
        bb
    }

    /// Row indices that may hold set columns.
    pub fn rows(&self) -> std::ops::Range<usize> {
        match self.rows_set {
            Some((lo, hi)) => lo..hi + 1,
            None => 0..0,
        }
    }

    /// The set columns of one row.
    pub fn row_columns(&self, row: usize) -> impl Iterator<Item = (i32, i32)> + '_ {
        let iy = self.y0 + row as i32;
        self.grid
            .row_cells(row)
            .map(move |x| (self.x0 + x as i32, iy))
    }
}

/// The heights a repair's edits added or removed per column, sorted, and the
/// columns that gained their first voxel or lost their last.
#[derive(Default)]
pub struct VoxelEdits {
    pub changed: Vec<((i32, i32), Vec<i32>)>,
    pub flipped: Vec<(i32, i32)>,
}

impl VoxelEdits {
    pub fn is_empty(&self) -> bool {
        self.changed.is_empty() && self.flipped.is_empty()
    }

    /// Every column with a voxel edit.
    pub fn columns(&self) -> impl Iterator<Item = (i32, i32)> + '_ {
        let changed = self.changed.iter().map(|&(col, _)| col);
        changed.chain(self.flipped.iter().copied())
    }
}

/// One column's voxel changes from a region update.
pub struct ColumnEdit {
    pub col: (i32, i32),
    pub removed: Vec<i32>,
    pub added: Vec<i32>,
}

/// Incoming voxels bucketed by column over a column bbox, each column's z
/// values sorted and deduped.
pub struct ColumnBuckets {
    x0: i32,
    y0: i32,
    width: usize,
    height: usize,
    bands: Vec<Band>,
}

/// The columns of TILE_EDGE consecutive rows of a bbox.
struct Band {
    starts: Vec<usize>,
    lens: Vec<usize>,
    zs: Vec<i32>,
}

const BAND_ROWS: usize = TILE_EDGE as usize;

impl ColumnBuckets {
    pub fn new(keys: &[VoxelKey], (x0, x1, y0, y1): ColumnWindow) -> Self {
        let width = (x1 - x0 + 1).max(0) as usize;
        let height = (y1 - y0 + 1).max(0) as usize;
        let n_bands = height.div_ceil(BAND_ROWS);
        let band_of = |&(_, iy, _): &VoxelKey| {
            let y = iy - y0;
            debug_assert!(y >= 0 && (y as usize) < height);
            y as usize / BAND_ROWS
        };
        let grouped: Vec<Vec<VoxelKey>> = keys
            .par_iter()
            .fold(
                || vec![Vec::new(); n_bands],
                |mut groups, k| {
                    groups[band_of(k)].push(*k);
                    groups
                },
            )
            .reduce(
                || vec![Vec::new(); n_bands],
                |mut a, b| {
                    for (into, from) in a.iter_mut().zip(b) {
                        into.extend(from);
                    }
                    a
                },
            );
        let bands: Vec<Band> = grouped
            .into_par_iter()
            .enumerate()
            .map(|(bi, band_keys)| {
                let rows = BAND_ROWS.min(height - bi * BAND_ROWS);
                Band::new(&band_keys, x0, y0 + (bi * BAND_ROWS) as i32, width, rows)
            })
            .collect();
        Self {
            x0,
            y0,
            width,
            height,
            bands,
        }
    }

    /// The sorted z values a column received, empty outside the bbox.
    pub fn column(&self, (ix, iy): (i32, i32)) -> &[i32] {
        let (x, y) = (ix - self.x0, iy - self.y0);
        if x < 0 || y < 0 || x as usize >= self.width || y as usize >= self.height {
            return &[];
        }
        let (x, y) = (x as usize, y as usize);
        self.bands[y / BAND_ROWS].column(x, y % BAND_ROWS, self.width)
    }
}

impl Band {
    fn new(keys: &[VoxelKey], x0: i32, y0: i32, width: usize, rows: usize) -> Self {
        let n = width * rows;
        let index = |&(ix, iy, _): &VoxelKey| (iy - y0) as usize * width + (ix - x0) as usize;
        let mut starts = vec![0usize; n + 1];
        for k in keys {
            starts[index(k) + 1] += 1;
        }
        for i in 0..n {
            starts[i + 1] += starts[i];
        }
        let mut fill = starts.clone();
        let mut zs = vec![0i32; keys.len()];
        for k in keys {
            let i = index(k);
            zs[fill[i]] = k.2;
            fill[i] += 1;
        }
        let mut lens = vec![0usize; n];
        for i in 0..n {
            let col = &mut zs[starts[i]..starts[i + 1]];
            col.sort_unstable();
            let mut kept = 0;
            for j in 0..col.len() {
                if kept == 0 || col[j] != col[kept - 1] {
                    col[kept] = col[j];
                    kept += 1;
                }
            }
            lens[i] = kept;
        }
        Self { starts, lens, zs }
    }

    fn column(&self, x: usize, row: usize, width: usize) -> &[i32] {
        let i = row * width + x;
        &self.zs[self.starts[i]..self.starts[i] + self.lens[i]]
    }
}

/// Merge a column's current voxels inside the region against its new ones.
/// None when nothing changed.
pub fn diff_column(
    col: (i32, i32),
    old: &[i32],
    new: &[i32],
    in_region: impl Fn(VoxelKey) -> bool,
) -> Option<ColumnEdit> {
    let (ix, iy) = col;
    let mut removed: Vec<i32> = Vec::new();
    let mut added: Vec<i32> = Vec::new();
    let mut new_iter = new.iter().copied().peekable();
    for &iz in old {
        if !in_region((ix, iy, iz)) {
            continue;
        }
        while let Some(&nz) = new_iter.peek() {
            if nz >= iz {
                break;
            }
            added.push(nz);
            new_iter.next();
        }
        if new_iter.peek() == Some(&iz) {
            new_iter.next();
        } else {
            removed.push(iz);
        }
    }
    added.extend(new_iter);
    (!removed.is_empty() || !added.is_empty()).then_some(ColumnEdit {
        col,
        removed,
        added,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_tile_is_dropped_with_its_last_voxel() {
        let mut columns = ColumnIz::default();
        columns.add((-1, -1, 0));
        columns.add((-1, -1, 3));
        columns.add((-2, -1, 0));
        columns.add((0, 0, 0));
        assert_eq!(columns.tiles.len(), 2);

        columns.remove((-1, -1, 0));
        columns.remove((-2, -1, 0));
        columns.remove((-2, -1, 0));
        columns.remove((-1, -1, 7));
        assert_eq!(columns.tiles.len(), 2);
        assert_eq!(columns.get((-1, -1)), Some(&[3][..]));

        columns.remove((-1, -1, 3));
        assert_eq!(columns.tiles.len(), 1);
        assert_eq!(columns.get((-1, -1)), None);
        assert_eq!(columns.get((0, 0)), Some(&[0][..]));
    }

    #[test]
    fn a_rebuilt_store_counts_each_voxel_once() {
        let voxels = [(0, 0, 2), (0, 0, 2), (0, 0, 1), (40, 0, 0)];
        let mut columns = ColumnIz::from_voxels(voxels.iter());
        assert_eq!(columns.get((0, 0)), Some(&[1, 2][..]));

        columns.remove((0, 0, 1));
        assert_eq!(columns.tiles.len(), 2);
        columns.remove((0, 0, 2));
        assert_eq!(columns.tiles.len(), 1);
    }
}
