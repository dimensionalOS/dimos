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

use super::*;

/// A 4x4 floor of healthy voxels, with one point folded into each.
fn floor() -> VoxelMap {
    let mut map = VoxelMap::with_support_min(2);
    for x in 0..4 {
        for y in 0..4 {
            map.set_health((x, y, 0), 3);
            map.voxels
                .get_mut(&(x, y, 0))
                .unwrap()
                .observe(Vector3::new(0.1, 0.0, 0.0));
        }
    }
    map
}

/// Every voxel's key, health and support, sorted.
fn snapshot(map: &VoxelMap) -> Vec<(VoxelKey, VoxelHealth, u8)> {
    let mut out: Vec<_> = map
        .voxels
        .iter()
        .map(|(k, v)| (k, v.health, map.voxels.support(&k).unwrap()))
        .collect();
    out.sort();
    out
}

/// Supports as a fresh map with the same voxels would count them.
fn recounted(map: &VoxelMap) -> Vec<(VoxelKey, u8)> {
    let mut out: Vec<_> = map
        .voxels
        .iter()
        .map(|(k, _)| (k, map.count_healthy_neighbors(k)))
        .collect();
    out.sort();
    out
}

fn shift_all(map: &VoxelMap, by: VoxelKey) -> Vec<VoxelMove> {
    map.voxels
        .iter()
        .map(|(k, _)| VoxelMove {
            from: k,
            to: (k.0 + by.0, k.1 + by.1, k.2 + by.2),
            offset: Vector3::zeros(),
        })
        .collect()
}

#[test]
fn moved_voxels_keep_health_and_exact_support() {
    let mut map = floor();
    let moves = shift_all(&map, (1, 2, 0));
    map.move_voxels(&moves, 1.0);
    let got = snapshot(&map);
    assert_eq!(got.len(), 16);
    assert!(got
        .iter()
        .all(|(k, h, _)| *h == 3 && (1..5).contains(&k.0) && (2..6).contains(&k.1)));
    let supports: Vec<(VoxelKey, u8)> = got.iter().map(|(k, _, s)| (*k, *s)).collect();
    assert_eq!(supports, recounted(&map));
    assert_eq!(map.healthy_count(), 16);
}

#[test]
fn landing_on_a_voxel_merges_moments_and_takes_the_higher_health() {
    let mut map = VoxelMap::default();
    map.set_health((0, 0, 0), 1);
    map.set_health((1, 0, 0), 4);
    map.voxels
        .get_mut(&(0, 0, 0))
        .unwrap()
        .observe(Vector3::new(0.25, 0.0, 0.0));
    map.voxels
        .get_mut(&(1, 0, 0))
        .unwrap()
        .observe(Vector3::new(-0.25, 0.0, 0.0));
    map.move_voxels(
        &[VoxelMove {
            from: (0, 0, 0),
            to: (1, 0, 0),
            offset: Vector3::new(-0.5, 0.0, 0.0),
        }],
        0.5,
    );
    let v = map.voxels.get(&(1, 0, 0)).unwrap();
    assert_eq!((map.voxels.len(), v.health, v.num_pts), (1, 4, 2));
    // The moved point sits 0.25 - 0.5 from the new center, the other at -0.25.
    assert!((v.sum.x + 0.5).abs() < 1e-6);
    assert!((v.m2[(0, 0)] - 0.125).abs() < 1e-6);
}

#[test]
fn a_move_in_place_only_recenters_the_moments() {
    let mut map = floor();
    let before = snapshot(&map);
    map.move_voxels(
        &[VoxelMove {
            from: (1, 1, 0),
            to: (1, 1, 0),
            offset: Vector3::new(0.2, 0.0, 0.0),
        }],
        1.0,
    );
    assert_eq!(snapshot(&map), before);
    assert!((map.voxels.get(&(1, 1, 0)).unwrap().sum.x - 0.3).abs() < 1e-6);
}

#[test]
fn moves_land_by_the_warped_center() {
    let keys = [(0, 0, 0), (-1, 2, 3)];
    let centers = voxel_centers(&keys, 0.5);
    assert_eq!(centers[1], [-0.25, 1.25, 1.75]);
    let warped = [[0.6, 0.2, 0.2], [-0.25, 1.25, 1.75]];
    let moves = voxel_moves(&keys, &centers, &warped, 0.5, |_| false);
    assert_eq!(moves[0].to, (1, 0, 0));
    assert!((moves[0].offset - Vector3::new(-0.15, -0.05, -0.05)).norm() < 1e-6);
    assert_eq!((moves[1].from, moves[1].to), ((-1, 2, 3), (-1, 2, 3)));
}

/// One healthy voxel holding a flat patch of points at height `z` off its center.
fn flat_patch(z: f32) -> VoxelMap {
    let mut map = VoxelMap::default();
    map.set_health((0, 0, 0), 3);
    let v = map.voxels.get_mut(&(0, 0, 0)).unwrap();
    for (x, y) in [
        (-0.2, -0.2),
        (0.2, -0.2),
        (-0.2, 0.2),
        (0.2, 0.2),
        (0.0, 0.1),
    ] {
        v.observe(Vector3::new(x, y, z));
    }
    map
}

#[test]
fn a_shift_carries_the_surface_plane_with_the_points() {
    let mut map = flat_patch(-0.1);
    map.shift_in_place(&[[0.0, 0.0, 0.3]], 1.0);
    let plane = map
        .voxels
        .surface(&(0, 0, 0))
        .expect("refit after the shift");
    assert!((plane.normal.z - 1.0).abs() < 1e-4);
    assert!((plane.offset - 0.2).abs() < 1e-4, "{plane:?}");
}

#[test]
fn a_move_to_another_key_refits_the_plane_about_the_new_center() {
    let mut map = flat_patch(0.4);
    // The points sit 0.4 above center; lifted 0.2 and filed one key up, they sit 0.4 below its center.
    map.move_voxels(
        &[VoxelMove {
            from: (0, 0, 0),
            to: (0, 0, 1),
            offset: Vector3::new(0.0, 0.0, -0.8),
        }],
        1.0,
    );
    assert!(map.voxels.surface(&(0, 0, 0)).is_none());
    let plane = map
        .voxels
        .surface(&(0, 0, 1))
        .expect("refit at the new key");
    assert!((plane.offset + 0.4).abs() < 1e-4, "{plane:?}");
}
