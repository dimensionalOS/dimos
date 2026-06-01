// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Voxel-grid coordinate math.

pub type VoxelKey = (i32, i32, i32);

#[inline]
pub fn voxelize(p: (f32, f32, f32), voxel_size: f32) -> VoxelKey {
    let inv = 1.0 / voxel_size;
    (
        (p.0 * inv).floor() as i32,
        (p.1 * inv).floor() as i32,
        (p.2 * inv).floor() as i32,
    )
}

/// XY centered in the cell, Z at the cell's top face.
#[inline]
pub fn surface_point_xyz(ix: i32, iy: i32, iz: i32, voxel_size: f32) -> (f32, f32, f32) {
    (
        (ix as f32 + 0.5) * voxel_size,
        (iy as f32 + 0.5) * voxel_size,
        (iz as f32 + 1.0) * voxel_size,
    )
}
