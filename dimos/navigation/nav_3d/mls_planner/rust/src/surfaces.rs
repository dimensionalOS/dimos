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

use crate::voxel::VoxelKey;

const INF: u16 = u16::MAX - 1;

pub type ColumnIz = AHashMap<(i32, i32), Vec<i32>>;

/// A cell is standable if it has at least the robot's height of clear space
/// above it.
pub(crate) fn is_standable(
    ix: i32,
    iy: i32,
    iz: i32,
    by_col: &ColumnIz,
    clearance_cells: i32,
) -> bool {
    let Some(zs) = by_col.get(&(ix, iy)) else {
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

    for &(ix, iy, iz) in voxel_map {
        by_col.entry((ix, iy)).or_default().push(iz);
    }

    let mut entries: Vec<((i32, i32), &mut Vec<i32>)> =
        by_col.iter_mut().map(|(&k, v)| (k, v)).collect();
    entries
        .par_iter_mut()
        .for_each(|(_, zs)| zs.sort_unstable());

    let standable: Vec<VoxelKey> = entries
        .par_iter()
        .flat_map_iter(|((ix, iy), zs)| {
            let mut local: Vec<VoxelKey> = Vec::new();
            standable_in_column(*ix, *iy, zs, clearance_cells, &mut local);
            local
        })
        .collect();
    drop(entries);

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

/// Insert a voxel into the per-column index, keeping each column sorted.
pub fn add_to_by_col(by_col: &mut ColumnIz, (ix, iy, iz): VoxelKey) {
    let zs = by_col.entry((ix, iy)).or_default();
    if let Err(pos) = zs.binary_search(&iz) {
        zs.insert(pos, iz);
    }
}

/// Remove a voxel from the per-column index, dropping emptied columns.
pub fn remove_from_by_col(by_col: &mut ColumnIz, (ix, iy, iz): VoxelKey) {
    if let Some(zs) = by_col.get_mut(&(ix, iy)) {
        if let Ok(pos) = zs.binary_search(&iz) {
            zs.remove(pos);
        }
        if zs.is_empty() {
            by_col.remove(&(ix, iy));
        }
    }
}

/// Re-extract surface cells in the inclusive write box. Reads a morphology
/// halo around the box so boundary closing matches a full rebuild, then
/// filters back to the box. by_col must already be current.
pub fn extract_surfaces_region(
    by_col: &ColumnIz,
    clearance_cells: i32,
    closing_passes: u32,
    write: (i32, i32, i32, i32),
) -> Vec<VoxelKey> {
    let (wx0, wx1, wy0, wy1) = write;
    let pad = (2 * closing_passes) as i32;

    let standable: Vec<VoxelKey> = ((wx0 - pad)..(wx1 + pad + 1))
        .into_par_iter()
        .flat_map_iter(|ix| {
            let mut local: Vec<VoxelKey> = Vec::new();
            for iy in (wy0 - pad)..=(wy1 + pad) {
                if let Some(zs) = by_col.get(&(ix, iy)) {
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
        .filter(|&(ix, iy, _)| ix >= wx0 && ix <= wx1 && iy >= wy0 && iy <= wy1)
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

    let tasks: Vec<(i32, Vec<(i32, i32)>)> = by_z
        .into_iter()
        .flat_map(|(iz, xys)| {
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

/// Split a slice into clusters that closing cannot connect. Cells go into
/// tiles, then 8-adjacent occupied tiles flood fill into clusters.
fn interaction_clusters(xys: &[(i32, i32)], closing_passes: u32) -> Vec<Vec<(i32, i32)>> {
    // Cells in different clusters are at least the tile side plus one apart.
    // Beyond 4r their dilations stay more than r apart, so closing each
    // cluster alone gives the same result as closing the whole slice.
    let side = (4 * closing_passes as i32).max(16);
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
            if let Some(zs) = by_col.get(&(ix + dx, iy + dy)) {
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
    let pad = closing_passes as i32;
    let mut min_x = i32::MAX;
    let mut max_x = i32::MIN;
    let mut min_y = i32::MAX;
    let mut max_y = i32::MIN;
    for &(ix, iy) in xys {
        min_x = min_x.min(ix);
        max_x = max_x.max(ix);
        min_y = min_y.min(iy);
        max_y = max_y.max(iy);
    }

    let w = (max_x - min_x + 1 + 2 * pad) as usize;
    let h = (max_y - min_y + 1 + 2 * pad) as usize;
    let x0 = min_x - pad;
    let y0 = min_y - pad;

    let r = closing_passes.min(INF as u32 - 1) as u16;
    let mut dist = vec![INF; w * h];
    for &(ix, iy) in xys {
        dist[(iy - y0) as usize * w + (ix - x0) as usize] = 0;
    }
    chamfer(&mut dist, w, h, false);
    // Cells within r of an occupied cell are the dilation. Reseeding from its
    // complement measures distance to background, and cells farther than r
    // survive the erosion.
    for v in dist.iter_mut() {
        *v = if *v <= r { INF } else { 0 };
    }
    chamfer(&mut dist, w, h, true);

    let original: AHashSet<(i32, i32)> = xys.iter().copied().collect();
    let mut out = Vec::new();
    for py in 0..h {
        for px in 0..w {
            if dist[py * w + px] <= r {
                continue;
            }
            let ix = x0 + px as i32;
            let iy = y0 + py as i32;

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

/// Two-pass L1 distance transform to the zero cells. Cells beyond the grid
/// edge count as sources when outside_is_source is set.
fn chamfer(dist: &mut [u16], w: usize, h: usize, outside_is_source: bool) {
    let edge = if outside_is_source { 0 } else { INF };
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

    fn run(cells: &[VoxelKey], clearance: i32, closing: u32) -> Vec<VoxelKey> {
        let map = voxel_map(cells);
        let mut by_col = ColumnIz::new();
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
