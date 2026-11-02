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

use super::tests::{basic_config, map_of_points};
use super::*;

/// A thin level patch filling voxel (5, 0, 5) at height `z`, sampled every cm.
fn thin_patch(z: f32) -> VoxelMap {
    let points: Vec<(f32, f32, f32)> = (0..10)
        .flat_map(|i| (0..10).map(move |j| (0.505 + 0.01 * i as f32, 0.005 + 0.01 * j as f32, z)))
        .collect();
    map_of_points(&points, 0.1)
}

// A wide pooled band, so a test passing proves the thin rules decided.
fn thin_config(thin_surface_m: f32, thin_graze_cos: f32) -> Config {
    Config {
        voxel_size: 0.1,
        shadow_depth: 0.1,
        graze_cos: 0.9,
        thin_surface_m,
        thin_graze_cos,
        ..basic_config()
    }
}

/// A ray through a thin surface's voxel that stays on one side of its plane passes beside it.
#[test]
fn thin_surface_spares_a_ray_that_never_crosses_its_plane() {
    let patch = (5, 0, 5);
    // Level ray 6 cm above the patch's plane, inside its voxel, ending past it.
    let (origin, hit) = ((0.05, 0.05, 0.58), (1.55, 0.05, 0.58));
    let mut map = thin_patch(0.52);
    let off = Config {
        graze_cos: 0.0,
        ..thin_config(0.0, 0.0)
    };
    update_map(&mut map, origin, &[hit], &off);
    assert!(
        !map.voxels.contains_key(&patch),
        "without the rule the ray carves it"
    );

    let mut map = thin_patch(0.52);
    let on = Config {
        thin_surface_m: 0.01,
        ..off
    };
    update_map(&mut map, origin, &[hit], &on);
    assert!(
        map.voxels.contains_key(&patch),
        "a ray beside a thin surface carved it"
    );
}

/// A ray crossing a thin surface is spared only within the thin grazing band.
#[test]
fn thin_surface_grazing_band_spares_only_shallow_crossings() {
    let patch = (5, 0, 5);
    let crossing = |deg: f32| {
        let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
        let at = (0.55, 0.05, 0.52);
        let origin = (at.0 - c, at.1, at.2 + s);
        let hit = (at.0 + c, at.1, at.2 - s);
        let mut map = thin_patch(0.52);
        update_map(&mut map, origin, &[hit], &thin_config(0.01, 0.26));
        map.voxels.contains_key(&patch)
    };
    assert!(
        crossing(10.0),
        "a 10 degree crossing is a graze and must be spared"
    );
    assert!(
        !crossing(30.0),
        "a 30 degree crossing sees through the patch and must carve it"
    );
}

/// Sparse points above a dense floor share its plane but are not a thin surface of their own.
#[test]
fn points_above_a_floor_are_not_thin_through_its_fit() {
    let mut points: Vec<(f32, f32, f32)> = (0..10)
        .flat_map(|i| {
            (0..10).map(move |j| (0.505 + 0.01 * i as f32, 0.005 + 0.01 * j as f32, 0.42))
        })
        .collect();
    points.extend((0..12).map(|i| (0.51 + 0.007 * i as f32, 0.05, 0.58 + 0.003 * i as f32)));
    let mut map = map_of_points(&points, 0.1);
    let ghost = (5, 0, 5);
    // A level ray through the stray points, beside the floor's plane, must clear them.
    update_map(
        &mut map,
        (0.05, 0.05, 0.56),
        &[(1.55, 0.05, 0.56)],
        &Config {
            graze_cos: 0.0,
            ..thin_config(0.03, 0.26)
        },
    );
    assert!(
        !map.voxels.contains_key(&ghost),
        "stray points kept by the floor below's fit"
    );
}

/// Points along one scan line leave their plane open, so they never count as thin.
#[test]
fn scan_line_voxel_is_not_thin() {
    let points: Vec<(f32, f32, f32)> = (0..4)
        .map(|i| (0.52 + 0.02 * i as f32, 0.05, 0.52))
        .collect();
    let mut map = map_of_points(&points, 0.1);
    update_map(
        &mut map,
        (0.05, 0.05, 0.58),
        &[(1.55, 0.05, 0.58)],
        &Config {
            graze_cos: 0.0,
            ..thin_config(0.01, 0.0)
        },
    );
    assert!(
        !map.voxels.contains_key(&(5, 0, 5)),
        "a scan line was treated as a thin surface"
    );
}

/// Three points fit any plane exactly, so they never count as thin.
#[test]
fn three_point_voxel_is_not_thin() {
    let points = [(0.52, 0.02, 0.52), (0.58, 0.02, 0.52), (0.55, 0.08, 0.52)];
    let mut map = map_of_points(&points, 0.1);
    let cfg = Config {
        graze_cos: 0.0,
        ..thin_config(0.01, 0.0)
    };
    update_map(&mut map, (0.05, 0.05, 0.58), &[(1.55, 0.05, 0.58)], &cfg);
    assert!(
        !map.voxels.contains_key(&(5, 0, 5)),
        "three points were treated as a thin surface"
    );
}
