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

//
// Rust ports of the go2 mapping stack (issue #2820):
//   - voxel_grid: dimos/mapping/voxels.py `VoxelGrid` (activate + column carve)
//   - occupancy:  dimos/mapping/pointclouds/occupancy.py `height_cost_occupancy`
//   - cloud:      PointCloud2 <-> xyz packing shared by both binaries
//
// The ports are behavior-preserving: same algorithm steps and config semantics
// as the Python originals, golden-tested against them on recorded frames.

pub mod cloud;
pub mod occupancy;
pub mod voxel_grid;
