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
    map.move_voxels(&moves);
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
fn a_rebuild_lands_the_same_map_as_a_repair() {
    let mut repaired = floor();
    let mut rebuilt = floor();
    // Half the floor moves over, into the other half.
    let moves: Vec<VoxelMove> = shift_all(&repaired, (2, 0, 0))
        .into_iter()
        .filter(|m| m.from.0 < 2)
        .collect();
    repaired.move_voxels(&moves);
    rebuilt.rebuild_moved(&moves);
    assert_eq!(snapshot(&repaired), snapshot(&rebuilt));
    let supports: Vec<(VoxelKey, u8)> = snapshot(&rebuilt)
        .iter()
        .map(|(k, _, s)| (*k, *s))
        .collect();
    assert_eq!(supports, recounted(&rebuilt));
    // Each landed voxel holds its own point and the one it moved onto.
    assert!(rebuilt.voxels.iter().all(|(_, v)| v.num_pts == 2));
    assert_eq!(repaired.healthy_count(), 8);
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
    map.move_voxels(&[VoxelMove {
        from: (0, 0, 0),
        to: (1, 0, 0),
        offset: Vector3::new(-0.5, 0.0, 0.0),
    }]);
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
    map.move_voxels(&[VoxelMove {
        from: (1, 1, 0),
        to: (1, 1, 0),
        offset: Vector3::new(0.2, 0.0, 0.0),
    }]);
    assert_eq!(snapshot(&map), before);
    assert!((map.voxels.get(&(1, 1, 0)).unwrap().sum.x - 0.3).abs() < 1e-6);
}

#[test]
fn moves_land_by_the_warped_center() {
    let keys = [(0, 0, 0), (-1, 2, 3)];
    let centers = voxel_centers(&keys, 0.5);
    assert_eq!(centers[1], [-0.25, 1.25, 1.75]);
    let warped = [[0.6, 0.2, 0.2], [-0.25, 1.25, 1.75]];
    let moves = voxel_moves(&keys, &centers, &warped, 0.5);
    assert_eq!(moves[0].to, (1, 0, 0));
    assert!((moves[0].offset - Vector3::new(-0.15, -0.05, -0.05)).norm() < 1e-6);
    assert_eq!((moves[1].from, moves[1].to), ((-1, 2, 3), (-1, 2, 3)));
}

/// Rebuild cost on a real map: RT_WARP_BENCH=<tracer dump, f32 xyz centers>
/// cargo test --release --lib rebuild_bench -- --ignored --nocapture
#[test]
#[ignore = "needs RT_WARP_BENCH, a reform_ray_tracing dump"]
fn rebuild_bench() {
    let path = std::env::var("RT_WARP_BENCH").expect("RT_WARP_BENCH");
    let raw = std::fs::read(path).unwrap();
    let vs = 0.08_f32;
    let load = || {
        let mut map = VoxelMap::with_support_min(4);
        for p in raw.as_chunks::<12>().0 {
            let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            map.set_health(world_to_voxel(f(0), f(4), f(8), 1.0 / vs), 3);
        }
        map
    };
    let started = std::time::Instant::now();
    let mut map = load();
    let loaded = started.elapsed();
    println!("voxels {}", map.voxels.len());
    // Every voxel moves one key over, with a sub-voxel offset, like a closure that shifts the map.
    let moves: Vec<VoxelMove> = map
        .voxels
        .iter()
        .map(|(k, _)| VoxelMove {
            from: k,
            to: (k.0 + 1, k.1, k.2),
            offset: Vector3::new(0.01, 0.0, 0.0),
        })
        .collect();
    let mut rebuilt = load();
    let started = std::time::Instant::now();
    rebuilt.rebuild_moved(&moves);
    println!("load {loaded:?}, rebuild_moved {:?}", started.elapsed());
    let started = std::time::Instant::now();
    map.move_voxels(&moves);
    println!("move_voxels {:?}", started.elapsed());
}
