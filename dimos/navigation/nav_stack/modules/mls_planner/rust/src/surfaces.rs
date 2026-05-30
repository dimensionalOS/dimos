// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Surface extraction: from a voxel map, mark cells with robot-height
//! clearance above as standable, then morphologically close per-z-level
//! holes without letting closing bridge across walls.

use ahash::{AHashMap, AHashSet};
use image::{GrayImage, Luma};
use imageproc::distance_transform::Norm;
use imageproc::morphology::{dilate, erode};
use rayon::prelude::*;

use crate::voxel::VoxelKey;

const ON: Luma<u8> = Luma([255]);
const OFF: Luma<u8> = Luma([0]);

pub type ColumnIz = AHashMap<(i32, i32), Vec<i32>>;

/// A cell is standable if it has at least the robot's height of clear
/// space above it.
fn is_standable(ix: i32, iy: i32, iz: i32, by_col: &ColumnIz, clearance_cells: i32) -> bool {
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
/// holes. `by_col` is a scratch buffer cleared on entry; its capacity is
/// reused across calls. Results land in `out` (cleared first).
pub fn extract_surfaces(
    voxel_map: &AHashSet<VoxelKey>,
    clearance_cells: i32,
    dilation_passes: u32,
    erosion_passes: u32,
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
            for w in zs.windows(2) {
                if w[1] - w[0] > clearance_cells {
                    local.push((*ix, *iy, w[0]));
                }
            }
            if let Some(&last_iz) = zs.last() {
                local.push((*ix, *iy, last_iz));
            }
            local
        })
        .collect();
    drop(entries);

    close_surface_holes(
        standable,
        by_col,
        dilation_passes,
        erosion_passes,
        clearance_cells,
        out,
    );
}

/// Dilation and erosion on all xy slices of the extracted surfaces
/// to fill in small holes.
fn close_surface_holes(
    standable: Vec<VoxelKey>,
    by_col: &ColumnIz,
    dilation_passes: u32,
    erosion_passes: u32,
    clearance_cells: i32,
    out: &mut Vec<VoxelKey>,
) {
    if standable.is_empty() || (dilation_passes == 0 && erosion_passes == 0) {
        out.extend(standable);
        return;
    }

    let mut by_z: AHashMap<i32, Vec<(i32, i32)>> = AHashMap::new();
    for &(ix, iy, iz) in &standable {
        by_z.entry(iz).or_default().push((ix, iy));
    }

    let slices: Vec<(i32, Vec<(i32, i32)>)> = by_z.into_iter().collect();
    out.par_extend(slices.par_iter().flat_map_iter(|(iz, xys)| {
        close_at_z(
            xys,
            *iz,
            by_col,
            dilation_passes,
            erosion_passes,
            clearance_cells,
        )
    }));
}

/// Close holes on an xy slice of the surfaces.
fn close_at_z(
    xys: &[(i32, i32)],
    iz: i32,
    by_col: &ColumnIz,
    dilation_passes: u32,
    erosion_passes: u32,
    clearance_cells: i32,
) -> Vec<VoxelKey> {
    let pad = (dilation_passes + erosion_passes) as i32;
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

    let w = (max_x - min_x + 1 + 2 * pad) as u32;
    let h = (max_y - min_y + 1 + 2 * pad) as u32;
    let x0 = min_x - pad;
    let y0 = min_y - pad;

    let mut img = GrayImage::from_pixel(w, h, OFF);
    for &(ix, iy) in xys {
        img.put_pixel((ix - x0) as u32, (iy - y0) as u32, ON);
    }

    if dilation_passes > 0 {
        img = dilate(&img, Norm::L1, dilation_passes as u8);
    }
    if erosion_passes > 0 {
        img = erode(&img, Norm::L1, erosion_passes as u8);
    }

    let mut out = Vec::new();
    for py in 0..h {
        for px in 0..w {
            if img.get_pixel(px, py).0[0] == 0 {
                continue;
            }
            let ix = x0 + px as i32;
            let iy = y0 + py as i32;

            if !is_standable(ix, iy, iz, by_col, clearance_cells) {
                continue;
            }
            out.push((ix, iy, iz));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voxel_map(cells: &[VoxelKey]) -> AHashSet<VoxelKey> {
        cells.iter().copied().collect()
    }

    fn run(cells: &[VoxelKey], clearance: i32, dil: u32, ero: u32) -> Vec<VoxelKey> {
        let map = voxel_map(cells);
        let mut by_col = ColumnIz::new();
        let mut out = Vec::new();
        extract_surfaces(&map, clearance, dil, ero, &mut by_col, &mut out);
        out
    }

    #[test]
    fn empty_input() {
        assert!(run(&[], 5, 0, 0).is_empty());
    }

    #[test]
    fn single_cell_is_topmost_surface() {
        let s = run(&[(0, 0, 0)], 5, 0, 0);
        assert_eq!(s, vec![(0, 0, 0)]);
    }

    #[test]
    fn stacked_cells_within_headroom_only_topmost_is_surface() {
        let cells: Vec<VoxelKey> = (0..5).map(|z| (0, 0, z)).collect();
        let s = run(&cells, 5, 0, 0);
        assert_eq!(s, vec![(0, 0, 4)]);
    }

    #[test]
    fn gap_larger_than_headroom_makes_lower_cell_standable() {
        let mut s = run(&[(0, 0, 0), (0, 0, 10)], 5, 0, 0);
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
        let s = run(&cells, 5, 3, 3);
        assert!(
            s.contains(&(0, 0, 0)),
            "closing should fill the center hole"
        );
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
        let s = run(&cells, 5, 3, 3);
        assert!(!s.contains(&(0, 0, 0)));
    }
}
