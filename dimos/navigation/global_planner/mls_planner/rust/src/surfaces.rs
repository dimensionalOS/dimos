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

use crate::bit_grid::BitGrid;
use crate::columns::{ColumnIz, ColumnMask};
use crate::voxel::VoxelKey;

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

/// Re-extract surface cells in the write mask, matching a full rebuild.
/// by_col must already be current.
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

    let mut grid = BitGrid::new(w, h);
    for &(ix, iy) in xys {
        grid.set((ix as i64 - x0) as usize, (iy as i64 - y0) as usize);
    }
    let original = grid.clone();
    let mut scratch = BitGrid::new(w, h);
    for _ in 0..closing_passes {
        grid.dilate(&mut scratch);
    }
    for _ in 0..closing_passes {
        grid.erode(&mut scratch);
    }

    // Closing never removes a cell, so the originals pass straight through.
    let mut out: Vec<VoxelKey> = xys.iter().map(|&(ix, iy)| (ix, iy, iz)).collect();
    for (px, py) in grid.set_cells() {
        if original.get(px, py) {
            continue;
        }
        let (Ok(ix), Ok(iy)) = (i32::try_from(x0 + px as i64), i32::try_from(y0 + py as i64))
        else {
            continue;
        };
        if !is_standable(ix, iy, iz, by_col, clearance_cells) {
            continue;
        }
        if !has_support(by_col, ix, iy, iz) {
            continue;
        }
        out.push((ix, iy, iz));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voxel_map(cells: &[VoxelKey]) -> AHashSet<VoxelKey> {
        cells.iter().copied().collect()
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
