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

//! Surface extraction: mark cells with robot-height clearance above as
//! standable, then morphologically close per-z-level holes without bridging
//! across walls.

use ahash::{AHashMap, AHashSet};
use rayon::prelude::*;

pub use crate::columns::ColumnIz;
use crate::voxel::VoxelKey;

const INF: u16 = u16::MAX - 1;

/// A cell is standable if it has at least the robot's height of clear space
/// above it.
pub(crate) fn is_standable(
    ix: i32,
    iy: i32,
    iz: i32,
    by_col: &ColumnIz,
    clearance_cells: i32,
) -> bool {
    let Some(zs) = by_col.get((ix, iy)) else {
        return true;
    };
    let idx = zs.partition_point(|&z| z <= iz);
    match zs.get(idx) {
        Some(&next) => next - iz > clearance_cells,
        None => true,
    }
}

/// Extract standable cells from the voxelized global map, then close small
/// holes.
pub fn extract_surfaces(
    voxel_map: &AHashSet<VoxelKey>,
    clearance_cells: i32,
    closing_passes: u32,
    by_col: &mut ColumnIz,
    out: &mut Vec<VoxelKey>,
) {
    out.clear();
    by_col.clear();
    if voxel_map.is_empty() {
        return;
    }
    *by_col = ColumnIz::from_voxels(voxel_map.iter());

    let standable: Vec<VoxelKey> = by_col
        .par_columns()
        .fold(Vec::new, |mut local, ((ix, iy), zs)| {
            standable_in_column(ix, iy, zs, clearance_cells, &mut local);
            local
        })
        .flatten_iter()
        .collect();

    close_surface_holes(standable, by_col, closing_passes, clearance_cells, out);
}

/// Standable cells in one column: any cell with robot clearance above, plus
/// the topmost cell.
fn standable_in_column(
    ix: i32,
    iy: i32,
    zs: &[i32],
    clearance_cells: i32,
    out: &mut Vec<VoxelKey>,
) {
    for w in zs.windows(2) {
        if w[1] - w[0] > clearance_cells {
            out.push((ix, iy, w[0]));
        }
    }
    if let Some(&last_iz) = zs.last() {
        out.push((ix, iy, last_iz));
    }
}

/// Dense set of columns over an inclusive box, for change footprints that
/// dilate by the morphology reach. One bit per column, so a dilation is a
/// few word shifts per row and a scan skips empty words.
#[derive(Clone)]
pub struct ColumnMask {
    x0: i32,
    y0: i32,
    w: usize,
    h: usize,
    words_per_row: usize,
    bits: Vec<u64>,
    /// Inclusive row range holding set bits, so scans skip the rest.
    rows_set: Option<(usize, usize)>,
}

impl ColumnMask {
    /// An empty mask over the box widened by `margin` on every side.
    pub fn new((x0, x1, y0, y1): (i32, i32, i32, i32), margin: i32) -> Self {
        let w = (x1 - x0 + 1 + 2 * margin).max(0) as usize;
        let h = (y1 - y0 + 1 + 2 * margin).max(0) as usize;
        let words_per_row = w.div_ceil(64);
        Self {
            x0: x0 - margin,
            y0: y0 - margin,
            w,
            h,
            words_per_row,
            bits: vec![0; words_per_row * h],
            rows_set: None,
        }
    }

    fn local(&self, (ix, iy): (i32, i32)) -> Option<(usize, usize)> {
        let x = ix - self.x0;
        let y = iy - self.y0;
        (x >= 0 && y >= 0 && (x as usize) < self.w && (y as usize) < self.h)
            .then_some((x as usize, y as usize))
    }

    /// Mark a column. Columns outside the box are ignored.
    pub fn set(&mut self, col: (i32, i32)) {
        if let Some((x, y)) = self.local(col) {
            self.bits[y * self.words_per_row + x / 64] |= 1 << (x % 64);
            self.rows_set = Some(match self.rows_set {
                None => (y, y),
                Some((lo, hi)) => (lo.min(y), hi.max(y)),
            });
        }
    }

    pub fn contains(&self, col: (i32, i32)) -> bool {
        self.local(col)
            .is_some_and(|(x, y)| self.bits[y * self.words_per_row + x / 64] >> (x % 64) & 1 == 1)
    }

    /// Every column within `r` of a set column, clipped to the box.
    pub fn dilated(&self, r: i32) -> ColumnMask {
        let r = r.max(0) as usize;
        let mut out = self.clone();
        let Some((lo, hi)) = self.rows_set else {
            return out;
        };
        let wpr = self.words_per_row;
        let mut row = vec![0u64; wpr];
        for y in lo..=hi {
            let src = &self.bits[y * wpr..(y + 1) * wpr];
            row.copy_from_slice(src);
            for _ in 0..r {
                shift_or(&mut row, self.w);
            }
            out.bits[y * wpr..(y + 1) * wpr].copy_from_slice(&row);
        }
        let (lo2, hi2) = (lo.saturating_sub(r), (hi + r).min(self.h.saturating_sub(1)));
        let widened = out.bits.clone();
        for y in lo2..=hi2 {
            let dst = &mut out.bits[y * wpr..(y + 1) * wpr];
            for sy in y.saturating_sub(r)..=(y + r).min(self.h.saturating_sub(1)) {
                if sy == y {
                    continue;
                }
                for (d, &w) in dst.iter_mut().zip(&widened[sy * wpr..(sy + 1) * wpr]) {
                    *d |= w;
                }
            }
        }
        out.rows_set = (self.h > 0).then_some((lo2, hi2));
        out
    }

    /// Inclusive bbox of the set columns, None when empty.
    pub fn bounds(&self) -> Option<(i32, i32, i32, i32)> {
        let (lo, hi) = self.rows_set?;
        let mut bb: Option<(i32, i32, i32, i32)> = None;
        for y in lo..=hi {
            let row = &self.bits[y * self.words_per_row..(y + 1) * self.words_per_row];
            let Some(first) = row.iter().position(|&w| w != 0) else {
                continue;
            };
            let last = row
                .iter()
                .rposition(|&w| w != 0)
                .expect("a set word exists");
            let xmin = first * 64 + row[first].trailing_zeros() as usize;
            let xmax = last * 64 + 63 - row[last].leading_zeros() as usize;
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

    /// Row indices that may hold set columns, for splitting a scan across
    /// threads.
    pub fn rows(&self) -> std::ops::Range<usize> {
        match self.rows_set {
            Some((lo, hi)) => lo..hi + 1,
            None => 0..0,
        }
    }

    /// The set columns of one row.
    pub fn row_columns(&self, row: usize) -> impl Iterator<Item = (i32, i32)> + '_ {
        let iy = self.y0 + row as i32;
        self.bits[row * self.words_per_row..(row + 1) * self.words_per_row]
            .iter()
            .enumerate()
            .flat_map(move |(wi, &word)| {
                SetBits(word).map(move |bit| (self.x0 + (wi * 64 + bit) as i32, iy))
            })
    }

    #[cfg(test)]
    pub fn columns(&self) -> impl Iterator<Item = (i32, i32)> + '_ {
        self.rows().flat_map(move |row| self.row_columns(row))
    }
}

/// Mark every cell within one of a set cell along a row of words, clipped
/// to `width` columns.
fn shift_or(row: &mut [u64], width: usize) {
    let n = row.len();
    let mut carry_left = 0u64;
    let mut out = vec![0u64; n];
    for i in 0..n {
        let w = row[i];
        out[i] = w | (w << 1) | carry_left | (w >> 1);
        carry_left = w >> 63;
    }
    for i in 0..n {
        let next_low = if i + 1 < n { row[i + 1] & 1 } else { 0 };
        out[i] |= next_low << 63;
    }
    if !width.is_multiple_of(64) {
        out[n - 1] &= (1u64 << (width % 64)) - 1;
    }
    row.copy_from_slice(&out);
}

/// Positions of the set bits of one word, ascending.
struct SetBits(u64);

impl Iterator for SetBits {
    type Item = usize;

    fn next(&mut self) -> Option<usize> {
        if self.0 == 0 {
            return None;
        }
        let bit = self.0.trailing_zeros() as usize;
        self.0 &= self.0 - 1;
        Some(bit)
    }
}

/// Re-extract surface cells in the write mask. Reads a morphology halo
/// around it so boundary closing matches a full rebuild, then filters back
/// to the mask. by_col must already be current.
pub fn extract_surfaces_region(
    by_col: &ColumnIz,
    clearance_cells: i32,
    closing_passes: u32,
    write: &ColumnMask,
) -> Vec<VoxelKey> {
    let pad = (2 * closing_passes) as i32;
    let read = write.dilated(pad);

    let standable: Vec<VoxelKey> = read
        .rows()
        .into_par_iter()
        .flat_map_iter(|row| {
            let mut local: Vec<VoxelKey> = Vec::new();
            for (ix, iy) in read.row_columns(row) {
                if let Some(zs) = by_col.get((ix, iy)) {
                    standable_in_column(ix, iy, zs, clearance_cells, &mut local);
                }
            }
            local
        })
        .collect();

    let mut closed: Vec<VoxelKey> = Vec::new();
    close_surface_holes(
        standable,
        by_col,
        closing_passes,
        clearance_cells,
        &mut closed,
    );
    closed
        .into_iter()
        .filter(|&(ix, iy, _)| write.contains((ix, iy)))
        .collect()
}

/// Dilate then erode every xy slice to fill small holes.
fn close_surface_holes(
    standable: Vec<VoxelKey>,
    by_col: &ColumnIz,
    closing_passes: u32,
    clearance_cells: i32,
    out: &mut Vec<VoxelKey>,
) {
    if standable.is_empty() || closing_passes == 0 {
        out.extend(standable);
        return;
    }

    let mut by_z: AHashMap<i32, Vec<(i32, i32)>> = AHashMap::new();
    for &(ix, iy, iz) in &standable {
        by_z.entry(iz).or_default().push((ix, iy));
    }

    let slices: Vec<(i32, Vec<(i32, i32)>)> = by_z.into_iter().collect();
    let tasks: Vec<(i32, Vec<(i32, i32)>)> = slices
        .into_par_iter()
        .flat_map_iter(|(iz, xys)| {
            interaction_clusters(&xys, closing_passes)
                .into_iter()
                .map(move |cluster| (iz, cluster))
        })
        .collect();
    out.par_extend(
        tasks.par_iter().flat_map_iter(|(iz, xys)| {
            close_at_z(xys, *iz, by_col, closing_passes, clearance_cells)
        }),
    );
}

/// Split a slice into clusters that closing cannot connect.
fn interaction_clusters(xys: &[(i32, i32)], closing_passes: u32) -> Vec<Vec<(i32, i32)>> {
    const MIN_TILE_SIDE: i32 = 16;
    // Separate clusters end up more than 2 closing passes apart, so each closes
    // to the same cells it would as part of the whole slice.
    let side = (4 * closing_passes as i32).max(MIN_TILE_SIDE);
    let mut tiles: AHashMap<(i32, i32), Vec<(i32, i32)>> = AHashMap::new();
    for &(x, y) in xys {
        tiles
            .entry((x.div_euclid(side), y.div_euclid(side)))
            .or_default()
            .push((x, y));
    }

    let mut clusters = Vec::new();
    let mut stack = Vec::new();
    let keys: Vec<(i32, i32)> = tiles.keys().copied().collect();
    for key in keys {
        if !tiles.contains_key(&key) {
            continue;
        }
        let mut cluster = Vec::new();
        stack.push(key);
        while let Some((tx, ty)) = stack.pop() {
            let Some(cells) = tiles.remove(&(tx, ty)) else {
                continue;
            };
            cluster.extend(cells);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let neighbor = (tx + dx, ty + dy);
                    if tiles.contains_key(&neighbor) {
                        stack.push(neighbor);
                    }
                }
            }
        }
        clusters.push(cluster);
    }
    clusters
}

/// Whether an occupied voxel lies near this cell at a compatible height.
fn has_support(by_col: &ColumnIz, ix: i32, iy: i32, iz: i32) -> bool {
    const R: i32 = 3;
    const Z_TOL: i32 = 3;
    for dx in -R..=R {
        for dy in -R..=R {
            if let Some(zs) = by_col.get((ix + dx, iy + dy)) {
                if zs.iter().any(|&oz| (oz - iz).abs() <= Z_TOL) {
                    return true;
                }
            }
        }
    }
    false
}

/// Close holes in one cluster of an xy slice.
fn close_at_z(
    xys: &[(i32, i32)],
    iz: i32,
    by_col: &ColumnIz,
    closing_passes: u32,
    clearance_cells: i32,
) -> Vec<VoxelKey> {
    let pad = closing_passes as i64;
    let mut min_x = i64::MAX;
    let mut max_x = i64::MIN;
    let mut min_y = i64::MAX;
    let mut max_y = i64::MIN;
    for &(ix, iy) in xys {
        min_x = min_x.min(ix as i64);
        max_x = max_x.max(ix as i64);
        min_y = min_y.min(iy as i64);
        max_y = max_y.max(iy as i64);
    }

    let w = (max_x - min_x + 1 + 2 * pad) as usize;
    let h = (max_y - min_y + 1 + 2 * pad) as usize;
    let x0 = min_x - pad;
    let y0 = min_y - pad;

    let r = closing_passes.min(INF as u32 - 1) as u16;
    let mut dist = vec![INF; w * h];
    for &(ix, iy) in xys {
        dist[(iy as i64 - y0) as usize * w + (ix as i64 - x0) as usize] = 0;
    }
    chamfer(&mut dist, w, h, Border::Empty);
    // Reseeding from the dilation's complement turns the second pass into the
    // erosion.
    for v in dist.iter_mut() {
        *v = if *v <= r { INF } else { 0 };
    }
    chamfer(&mut dist, w, h, Border::Source);

    let original: AHashSet<(i32, i32)> = xys.iter().copied().collect();
    let mut out = Vec::new();
    for py in 0..h {
        for px in 0..w {
            if dist[py * w + px] <= r {
                continue;
            }
            let (Ok(ix), Ok(iy)) = (i32::try_from(x0 + px as i64), i32::try_from(y0 + py as i64))
            else {
                continue;
            };

            if !is_standable(ix, iy, iz, by_col, clearance_cells) {
                continue;
            }
            // Keep a filled cell only with nearby occupied evidence.
            if !original.contains(&(ix, iy)) && !has_support(by_col, ix, iy, iz) {
                continue;
            }
            out.push((ix, iy, iz));
        }
    }
    out
}

/// What lies beyond the grid edge for the distance transform.
#[derive(Clone, Copy)]
enum Border {
    Empty,
    Source,
}

/// Two-pass L1 distance transform to the zero cells.
fn chamfer(dist: &mut [u16], w: usize, h: usize, border: Border) {
    let edge = match border {
        Border::Empty => INF,
        Border::Source => 0,
    };
    for y in 0..h {
        for x in 0..w {
            let left = if x > 0 { dist[y * w + x - 1] } else { edge };
            let up = if y > 0 { dist[(y - 1) * w + x] } else { edge };
            let best = left.min(up).saturating_add(1);
            let i = y * w + x;
            if best < dist[i] {
                dist[i] = best;
            }
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            let right = if x + 1 < w { dist[y * w + x + 1] } else { edge };
            let down = if y + 1 < h {
                dist[(y + 1) * w + x]
            } else {
                edge
            };
            let best = right.min(down).saturating_add(1);
            let i = y * w + x;
            if best < dist[i] {
                dist[i] = best;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voxel_map(cells: &[VoxelKey]) -> AHashSet<VoxelKey> {
        cells.iter().copied().collect()
    }

    #[test]
    fn column_mask_dilates_by_chebyshev_distance_and_clips_to_its_box() {
        let mut mask = ColumnMask::new((0, 4, 0, 4), 1);
        mask.set((0, 0));
        mask.set((4, 4));
        mask.set((9, 9));
        assert!(!mask.contains((9, 9)), "outside the box is ignored");
        let d = mask.dilated(1);
        assert_eq!(d.columns().count(), 18, "two 3x3 blocks");
        assert!(d.contains((-1, -1)) && d.contains((1, 1)));
        assert!(d.contains((5, 5)) && !d.contains((6, 6)));
        assert_eq!(d.bounds(), Some((-1, 5, -1, 5)));
        assert_eq!(ColumnMask::new((0, 4, 0, 4), 0).bounds(), None);
    }

    #[test]
    fn region_extraction_over_a_mask_matches_a_full_extraction_inside_it() {
        let mut cells: Vec<VoxelKey> = Vec::new();
        for ix in 0..20 {
            for iy in 0..20 {
                if (ix, iy) != (7, 7) {
                    cells.push((ix, iy, 0));
                }
            }
        }
        let full = run(&cells, 3, 1);
        let mut by_col = ColumnIz::default();
        for &k in &cells {
            by_col.add(k);
        }
        let mut write = ColumnMask::new((0, 19, 0, 19), 2);
        write.set((7, 7));
        let write = write.dilated(2);
        let region = extract_surfaces_region(&by_col, 3, 1, &write);
        let want: AHashSet<VoxelKey> = full
            .iter()
            .copied()
            .filter(|&(ix, iy, _)| write.contains((ix, iy)))
            .collect();
        assert_eq!(region.iter().copied().collect::<AHashSet<_>>(), want);
        assert!(want.contains(&(7, 7, 0)), "the hole closes");
    }

    fn run(cells: &[VoxelKey], clearance: i32, closing: u32) -> Vec<VoxelKey> {
        let map = voxel_map(cells);
        let mut by_col = ColumnIz::default();
        let mut out = Vec::new();
        extract_surfaces(&map, clearance, closing, &mut by_col, &mut out);
        out
    }

    #[test]
    fn empty_input() {
        assert!(run(&[], 5, 0).is_empty());
    }

    #[test]
    fn stacked_cells_within_headroom_only_topmost_is_surface() {
        let cells: Vec<VoxelKey> = (0..5).map(|z| (0, 0, z)).collect();
        let s = run(&cells, 5, 0);
        assert_eq!(s, vec![(0, 0, 4)]);
    }

    #[test]
    fn gap_larger_than_headroom_makes_lower_cell_standable() {
        let mut s = run(&[(0, 0, 0), (0, 0, 10)], 5, 0);
        s.sort();
        assert_eq!(s, vec![(0, 0, 0), (0, 0, 10)]);
    }

    #[test]
    fn morphological_closing_fills_center_hole() {
        let cells: Vec<VoxelKey> = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]
        .into_iter()
        .map(|(dx, dy)| (dx, dy, 0))
        .collect();
        let s = run(&cells, 5, 3);
        assert!(
            s.contains(&(0, 0, 0)),
            "closing should fill the center hole"
        );
    }

    #[test]
    fn closing_does_not_fill_unsupported_void() {
        // A ring with a large empty center: closing reaches it geometrically but
        // has no occupied support there, so it must stay a hole.
        let mut cells = Vec::new();
        for d in -5..=5 {
            cells.push((d, -5, 0));
            cells.push((d, 5, 0));
            cells.push((-5, d, 0));
            cells.push((5, d, 0));
        }
        let s = run(&cells, 5, 6);
        assert!(
            !s.contains(&(0, 0, 0)),
            "unsupported void center must not be filled"
        );
        assert!(s.contains(&(0, -5, 0)), "the real ring stays");
    }

    #[test]
    fn closing_keeps_solid_block_exact() {
        let cells: Vec<VoxelKey> = (0..8)
            .flat_map(|x| (0..8).map(move |y| (x, y, 0)))
            .collect();
        let mut s = run(&cells, 5, 3);
        s.sort();
        let mut expected = cells;
        expected.sort();
        assert_eq!(s, expected, "closing must not grow the block outward");
    }

    #[test]
    fn boundary_coordinates_do_not_overflow() {
        let cells = [(i32::MIN, i32::MIN, 0), (i32::MAX, i32::MAX, 0)];
        let s = run(&cells, 5, 3);
        assert!(s.contains(&cells[0]), "min-corner cell survives");
        assert!(s.contains(&cells[1]), "max-corner cell survives");
    }

    #[test]
    fn closing_keeps_a_thin_row_exact() {
        for axis_is_x in [true, false] {
            let cells: Vec<VoxelKey> = (0..8)
                .map(|t| if axis_is_x { (t, 0, 0) } else { (0, t, 0) })
                .collect();
            let mut s = run(&cells, 5, 3);
            s.sort();
            let mut expected = cells;
            expected.sort();
            assert_eq!(s, expected, "closing must not grow a one-cell-wide row");
        }
    }

    #[test]
    fn stray_cell_does_not_affect_distant_cluster() {
        let mut cells: Vec<VoxelKey> = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]
        .into_iter()
        .map(|(dx, dy)| (dx, dy, 0))
        .collect();
        cells.push((10000, 5000, 0));
        let s = run(&cells, 5, 3);
        assert!(s.contains(&(0, 0, 0)), "hole still closes");
        assert!(s.contains(&(10000, 5000, 0)), "stray cell survives");
    }

    #[test]
    fn closing_does_not_bridge_voxel_in_headroom() {
        let mut cells: Vec<VoxelKey> = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]
        .into_iter()
        .map(|(dx, dy)| (dx, dy, 0))
        .collect();
        cells.push((0, 0, 1));
        let s = run(&cells, 5, 3);
        assert!(!s.contains(&(0, 0, 0)));
    }
}
