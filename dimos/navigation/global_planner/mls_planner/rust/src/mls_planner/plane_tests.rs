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

//! The voxel problem scenes, each read from whole voxels and from planes.

#[path = "voxel_problem.rs"]
mod voxel_problem;

use super::*;
use voxel_problem::{plan, scenes, voxels, whole_voxels};

/// The scene at `index` in `scenes` crosses from planes and not from whole voxels, or the reverse.
fn check(index: usize) {
    let scene = &scenes()[index];
    let points = voxels(scene);
    let whole = plan(&whole_voxels(&points), scene).1.is_some();
    let planes = plan(&points, scene).1.is_some();
    assert_eq!(
        (whole, planes),
        (!scene.planes_cross, scene.planes_cross),
        "{}: {}",
        scene.name,
        scene.caption
    );
}

#[test]
fn a_tiny_rise_across_a_grid_line_is_no_step() {
    check(0);
}

#[test]
fn a_tall_rise_read_as_two_voxels_is_still_a_wall() {
    check(1);
}

#[test]
fn a_ramp_is_not_a_staircase() {
    check(2);
}

#[test]
fn a_floor_straddling_a_grid_line_stays_one_level() {
    check(3);
}

#[test]
fn headroom_counts_from_the_surface_not_the_top_face() {
    check(4);
}
