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
