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

fn basic_config() -> Config {
    Config {
        voxel_size: 1.0,
        fine_divisor: 0,
        max_range: 100.0,
        ray_subsample: 1,
        shadow_depth: 2.0,
        grace_depth: 0.0,
        min_health: 0,
        max_health: 1,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    }
}

#[test]
fn update_map_drops_invalid_and_out_of_range_points() {
    let cfg = Config {
        max_range: 5.0,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    let origin = (0.5, 0.5, 0.5);
    let points = [
        (f32::NAN, 0.5, 0.5),
        (0.5, f32::INFINITY, 0.5),
        (100.0, 0.5, 0.5),
        (0.5, 0.5, 0.5),
        (2.5, 0.5, 0.5),
    ];
    update_map(&mut map, origin, &points, &cfg);
    let keys: Vec<VoxelKey> = map.voxels.keys().copied().collect();
    assert_eq!(keys, vec![(2, 0, 0)], "only the valid in-range point lands");
}

#[test]
fn find_misses_along_ray_hits_correct_voxels() {
    let voxel_size = 1.0;
    let shadow_depth = 2.0;
    let origin = (0.5, 0.5, 0.5);
    let end = (5.5, 0.5, 0.5);
    let inv = 1.0 / voxel_size;
    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

    let expected: AHashSet<VoxelKey> = [
        (1, 0, 0),
        (2, 0, 0),
        (3, 0, 0),
        (4, 0, 0),
        (6, 0, 0),
        (7, 0, 0),
    ]
    .into_iter()
    .collect();
    let mut map_voxels: AHashMap<VoxelKey, Voxel> = AHashMap::new();
    for v in &expected {
        map_voxels.insert(*v, Voxel::with_health(1));
    }

    let mut misses: AHashSet<VoxelKey> = AHashSet::new();
    find_misses_along_ray(
        &mut misses,
        &map_voxels,
        origin,
        end,
        voxel_size,
        shadow_depth,
        0.0,
        0.5,
        None,
        origin_voxel,
        endpoint,
    );

    assert_eq!(misses, expected);
}

#[test]
fn batch_bounds_ignore_far_outlier() {
    let origins = [(1.0, 1.0, 0.5), (3.0, 1.0, 0.5)];
    let mut points: Vec<(f32, f32, f32)> = (0..99)
        .map(|i| {
            let a = i as f32 / 99.0 * std::f32::consts::TAU;
            (2.0 + a.cos(), 1.0 + a.sin(), (i % 10) as f32 * 0.1)
        })
        .collect();
    points.push((60.0, 1.0, 30.0));
    let c = batch_local_bounds(&points, &origins, 95.0, 0.3);
    assert_eq!(c.cx, 2.0);
    assert_eq!(c.cy, 1.0);
    assert!(c.radius < 2.0, "outlier inflated radius to {}", c.radius);
    assert!(c.z_max < 2.0, "outlier inflated z_max to {}", c.z_max);
    assert!(
        (-0.5..=0.0).contains(&c.z_min),
        "z_min out of range: {}",
        c.z_min
    );
}

#[test]
fn batch_bounds_empty_points_zero_radius() {
    let origins = [(1.0, 2.0, 3.0)];
    let c = batch_local_bounds(&[], &origins, 95.0, 0.3);
    assert_eq!((c.cx, c.cy, c.radius), (1.0, 2.0, 0.0));
    assert_eq!(c.z_min, 3.0);
    assert_eq!(c.z_max, 3.0);
}

/// clear() must empty the healthy-chunk index too. With support_min 0,
/// emit_points reads only the index, so a stale entry would resurface here.
#[test]
fn clear_empties_healthy_chunk_index() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    let no_live = AHashSet::new();
    assert!(!emit_points(&map, 1.0, None, 0, &no_live).is_empty());

    map.clear();
    assert!(map.voxels.is_empty());
    assert!(
        emit_points(&map, 1.0, None, 0, &no_live).is_empty(),
        "cleared map must not emit from a stale chunk index"
    );
}

#[test]
fn hits_insert_voxels() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    update_map(
        &mut map,
        (0.0, 0.0, 0.0),
        &[(5.5, 0.5, 0.5), (0.5, 5.5, 0.5)],
        &cfg,
    );
    assert_eq!(map.health((5, 0, 0)), Some(1));
    assert_eq!(map.health((0, 5, 0)), Some(1));
    assert_eq!(map.voxels.len(), 2);
}

#[test]
fn voxels_on_ray_are_removed() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    map.set_health((3, 0, 0), 1);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    // The voxel on the ray should be cleared.
    assert!(!map.voxels.contains_key(&(3, 0, 0)));
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

#[test]
fn voxels_not_on_ray_survive() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    map.set_health((3, 5, 0), 1);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((3, 5, 0)), Some(1));
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

#[test]
fn voxels_within_shadow_region_are_removed() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    map.set_health((6, 0, 0), 1);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    // The voxel inside the shadow region should be cleared.
    assert!(!map.voxels.contains_key(&(6, 0, 0)));
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

#[test]
fn voxels_beyond_shadow_region_survive() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    map.set_health((8, 0, 0), 1);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((8, 0, 0)), Some(1));
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

#[test]
fn hit_caught_by_other_ray_is_not_removed() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    update_map(
        &mut map,
        (0.0, 0.0, 0.0),
        &[(3.5, 0.5, 0.5), (5.5, 0.5, 0.5)],
        &cfg,
    );
    assert_eq!(map.health((3, 0, 0)), Some(1));
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

#[test]
fn point_beyond_max_range_does_not_clear() {
    let cfg = Config {
        max_range: 3.0,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    map.set_health((3, 0, 0), 1);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((3, 0, 0)), Some(1));
}

#[test]
fn fine_gate_spares_ray_through_unobserved_cells() {
    let cfg = Config {
        fine_divisor: 2,
        shadow_depth: 0.0,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.5, 0.75, 0.25), &[(5.5, 0.75, 0.25)], &cfg);
    assert_eq!(map.health((5, 0, 0)), Some(1));

    // The clearing ray crosses only the empty top cells of the voxel.
    update_map(&mut map, (0.5, 0.75, 0.75), &[(9.5, 0.75, 0.75)], &cfg);
    assert_eq!(
        map.health((5, 0, 0)),
        Some(1),
        "a ray through unobserved cells must not decrement"
    );
}

#[test]
fn fine_gate_clears_when_ray_crosses_observed_cells() {
    let cfg = Config {
        fine_divisor: 2,
        shadow_depth: 0.0,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.5, 0.75, 0.25), &[(5.5, 0.75, 0.25)], &cfg);
    assert_eq!(map.health((5, 0, 0)), Some(1));

    update_map(&mut map, (0.5, 0.75, 0.25), &[(9.5, 0.75, 0.25)], &cfg);
    assert_eq!(
        map.health((5, 0, 0)),
        None,
        "a ray through the observed cell still clears"
    );
}

#[test]
fn two_hits_needed_when_min_health_is_negative() {
    let cfg = Config {
        min_health: -1,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((5, 0, 0)), Some(0));

    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((5, 0, 0)), Some(1));
}

/// A grazing ray along a floor must not clip floor voxels near its hit.
#[test]
fn ground_clipping_single_ray() {
    let voxel_size = 0.1_f32;
    let lidar_height = 1.0_f32;
    let cfg = Config {
        voxel_size,
        fine_divisor: 0,
        max_range: 50.0,
        ray_subsample: 1,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        min_health: 0,
        max_health: 1,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    };
    // Build the floor over a y band so it is a 2d plane, not a wire.
    let max_x = 25.0_f32;
    let y_half = 0.3_f32;
    let ds = voxel_size / 3.0;
    let nx = (max_x / ds).ceil() as i32;
    let ny = (2.0 * y_half / ds).ceil() as i32;
    let floor_z = voxel_size * 0.5;
    let floor_points: Vec<(f32, f32, f32)> = (0..=nx)
        .flat_map(|i| (0..=ny).map(move |j| (i as f32 * ds, -y_half + j as f32 * ds, floor_z)))
        .collect();

    let ranges: Vec<f32> = (1..=20).map(|i| i as f32).collect();
    let mut table = format!(
        "voxel_size={voxel_size} lidar_height={lidar_height} grace={} shadow={}\n\
         range_m  ground_voxels_in_row  clipped  clipped_pct\n",
        cfg.grace_depth, cfg.shadow_depth
    );
    let mut total_clipped = 0usize;
    for &range in &ranges {
        let (mut map, _) = build_surface(&floor_points, voxel_size, cfg.max_health);
        // The ray walks the y=0, z=0 row, so only that row is ever at risk.
        let center_row: Vec<VoxelKey> = map
            .voxels
            .keys()
            .copied()
            .filter(|k| k.1 == 0 && k.2 == 0)
            .collect();
        let n_before = center_row.len();

        let origin = (0.0_f32, 0.0_f32, lidar_height);
        let points = vec![(range, 0.0_f32, 0.0_f32)];
        update_map(&mut map, origin, &points, &cfg);

        let n_after_ground = center_row
            .iter()
            .filter(|k| map.voxels.contains_key(k))
            .count();
        let clipped = n_before - n_after_ground;
        let pct = 100.0 * clipped as f32 / n_before as f32;
        table.push_str(&format!(
            "{range:>6.1}  {n_before:>20}  {clipped:>7}  {pct:>10.1}\n"
        ));
        total_clipped += clipped;
    }
    assert!(
        total_clipped == 0,
        "planar grace regressed, ground voxels clipped:\n{table}"
    );
}

/// Sample axis-aligned segments across a y band so each patch is a 2d surface.
fn sample_segments(segments: &[(bool, f32, f32, f32)], voxel_size: f32) -> Vec<(f32, f32, f32)> {
    let ds = voxel_size / 6.0;
    // Sample the full step width so treads keep two in-plane directions.
    let width = 3.0 * voxel_size;
    let ny = 19;
    let mut pts = Vec::new();
    for &(vertical, fixed, lo, hi) in segments {
        let n = ((hi - lo) / ds).round().max(1.0) as i32;
        for i in 0..=n {
            let t = lo + (hi - lo) * (i as f32 / n as f32);
            for j in 0..ny {
                let yy = width * (j as f32 / (ny - 1) as f32);
                pts.push(if vertical {
                    (fixed, yy, t)
                } else {
                    (t, yy, fixed)
                });
            }
        }
    }
    pts
}

/// Build a map by accumulating sampled returns and marking each touched
/// voxel occupied. Returns the map and the sorted unique voxel keys.
fn build_surface(
    lidar: &[(f32, f32, f32)],
    voxel_size: f32,
    health: VoxelHealth,
) -> (VoxelMap, Vec<VoxelKey>) {
    let inv = 1.0 / voxel_size;
    let mut map = VoxelMap::default();
    for &p in lidar {
        map.accumulate(p, voxel_size, None);
    }
    let mut keys: Vec<VoxelKey> = lidar
        .iter()
        .map(|&(x, y, z)| world_to_voxel(x, y, z, inv))
        .collect();
    keys.sort();
    keys.dedup();
    for &k in &keys {
        map.set_health(k, health);
    }
    map.recompute_all_normals(voxel_size);
    (map, keys)
}

/// Nearest forward intersection (t > 0) of a ray with the segments, as an
/// x-z point.
fn nearest_hit(
    origin: (f32, f32, f32),
    d: (f32, f32),
    segments: &[(bool, f32, f32, f32)],
) -> Option<(f32, f32)> {
    let mut best: Option<(f32, (f32, f32))> = None;
    for &(vertical, fixed, lo, hi) in segments {
        let hit = if vertical {
            if d.0.abs() < 1e-9 {
                continue;
            }
            let t = (fixed - origin.0) / d.0;
            let z = origin.2 + t * d.1;
            (t > 1e-4 && z >= lo && z <= hi).then_some((t, (fixed, z)))
        } else {
            if d.1.abs() < 1e-9 {
                continue;
            }
            let t = (fixed - origin.2) / d.1;
            let x = origin.0 + t * d.0;
            (t > 1e-4 && x >= lo && x <= hi).then_some((t, (x, fixed)))
        };
        if let Some(cand) = hit {
            if best.is_none_or(|b| cand.0 < b.0) {
                best = Some(cand);
            }
        }
    }
    best.map(|(_, p)| p)
}

/// A ray fan from the foot of a staircase grazes lower steps en route to
/// upper ones. The grazing gate must leave every planar surface voxel intact.
#[test]
fn stair_clipping_ray_fan() {
    let voxel_size = 0.1_f32;
    let half = voxel_size * 0.5;
    let cfg = Config {
        voxel_size,
        fine_divisor: 0,
        max_range: 50.0,
        ray_subsample: 1,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        min_health: 0,
        max_health: 1,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    };

    // Staircase
    const N: i32 = 5;
    let run = 3.0 * voxel_size;
    let rise = 2.0 * voxel_size;
    let first_riser_x = 3.0 * voxel_size + half;
    let base_z = half;
    let mut segments: Vec<(bool, f32, f32, f32)> = Vec::new();
    for k in 1..=N {
        let rx = first_riser_x + (k - 1) as f32 * run;
        let zb = base_z + (k - 1) as f32 * rise;
        let zt = base_z + k as f32 * rise;
        segments.push((true, rx, zb, zt));
        segments.push((false, zt, rx, rx + run));
    }

    let lidar = sample_segments(&segments, voxel_size);
    let (mut map, all_stairs) = build_surface(&lidar, voxel_size, cfg.max_health);

    // Voxels with a normal must be spared. Only edge voxels with no plane may clear.
    let planar: Vec<VoxelKey> = all_stairs
        .iter()
        .copied()
        .filter(|k| map.voxels.get(k).and_then(Voxel::planar_normal).is_some())
        .collect();

    let origin = (half, half, base_z + 0.23);

    // A ray fan sweeping up the staircase.
    const N_RAYS: usize = 6;
    let (lo_deg, hi_deg) = (0.0_f32, 27.0_f32);
    let mut hits: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..N_RAYS {
        let frac = i as f32 / (N_RAYS - 1) as f32;
        let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
        if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
            hits.push((hx, half, hz));
        }
    }

    update_map(&mut map, origin, &hits, &cfg);

    let cleared_planar: Vec<VoxelKey> = planar
        .iter()
        .copied()
        .filter(|v| !map.voxels.contains_key(v))
        .collect();
    assert!(
        cleared_planar.is_empty(),
        "grazing rays eroded {} planar surface voxel(s): {cleared_planar:?}",
        cleared_planar.len()
    );
}

/// A flat landing floor with a far wall, scanned by a downward ray fan. The
/// grazing gate must not erode the floor.
#[test]
fn landing_floor_ray_fan() {
    let voxel_size = 0.1_f32;
    let half = voxel_size * 0.5;
    let cfg = Config {
        voxel_size,
        fine_divisor: 0,
        max_range: 50.0,
        ray_subsample: 1,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        min_health: 0,
        max_health: 1,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    };

    // Flat floor from the sensor out to a vertical wall.
    let floor_z = half;
    let x_wall = 25.0 * voxel_size + half;
    let segments = vec![
        (false, floor_z, half, x_wall),         // floor
        (true, x_wall, floor_z, floor_z + 1.0), // wall
    ];

    let lidar = sample_segments(&segments, voxel_size);
    let (mut map, all_surf) = build_surface(&lidar, voxel_size, cfg.max_health);

    // Sensor above the floor, so grazing rays skim it on the way to the wall.
    const SENSOR_HEIGHT: f32 = 0.3;
    let origin = (half, half, floor_z + SENSOR_HEIGHT);

    let floor: Vec<VoxelKey> = all_surf.iter().copied().filter(|k| k.2 == 0).collect();

    const N_RAYS: usize = 16;
    let (lo_deg, hi_deg) = (-35.0_f32, 18.0_f32);
    let mut hits: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..N_RAYS {
        let frac = i as f32 / (N_RAYS - 1) as f32;
        let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
        if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
            hits.push((hx, half, hz));
        }
    }

    update_map(&mut map, origin, &hits, &cfg);

    let cleared: Vec<VoxelKey> = floor
        .iter()
        .copied()
        .filter(|v| !map.voxels.contains_key(v))
        .collect();
    assert!(
        cleared.is_empty(),
        "ray fan cleared {} floor voxel(s): {cleared:?}",
        cleared.len()
    );
}

/// A landing seen edge-on from just below must survive the grazing rays.
#[test]
fn landing_grazed_from_below() {
    let voxel_size = 0.1_f32;
    let half = voxel_size * 0.5;
    let cfg = |graze_cos| Config {
        voxel_size,
        fine_divisor: 0,
        max_range: 50.0,
        ray_subsample: 1,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        min_health: 0,
        max_health: 1,
        graze_cos,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    };

    // Staircase topped by a flat landing and a back wall.
    const N: i32 = 5;
    let run = 3.0 * voxel_size;
    let rise = 2.0 * voxel_size;
    let first_riser_x = 3.0 * voxel_size + half;
    let base_z = half;
    let mut segments: Vec<(bool, f32, f32, f32)> = Vec::new();
    for k in 1..=N {
        let rx = first_riser_x + (k - 1) as f32 * run;
        let zb = base_z + (k - 1) as f32 * rise;
        let zt = base_z + k as f32 * rise;
        segments.push((true, rx, zb, zt));
        if k < N {
            segments.push((false, zt, rx, rx + run));
        }
    }
    let z_top = base_z + N as f32 * rise;
    let landing_x0 = first_riser_x + (N - 1) as f32 * run;
    segments.push((false, z_top, landing_x0, landing_x0 + 1.0));
    segments.push((true, landing_x0 + 1.0, z_top, z_top + 1.0));

    let lidar = sample_segments(&segments, voxel_size);
    let landing_row = (z_top / voxel_size).floor() as i32;

    let step_below_x = first_riser_x + (N - 2) as f32 * run + run * 0.5;
    let origin = (step_below_x, half, z_top - rise + 0.3);
    const N_RAYS: usize = 16;
    let (lo_deg, hi_deg) = (-38.0_f32, -2.0_f32);
    let mut hits: Vec<(f32, f32, f32)> = Vec::new();
    for i in 0..N_RAYS {
        let frac = i as f32 / (N_RAYS - 1) as f32;
        let theta = (lo_deg + (hi_deg - lo_deg) * frac).to_radians();
        if let Some((hx, hz)) = nearest_hit(origin, (theta.cos(), theta.sin()), &segments) {
            hits.push((hx, half, hz));
        }
    }

    let (mut map, surf) = build_surface(&lidar, voxel_size, 1);
    update_map(&mut map, origin, &hits, &cfg(0.7));

    let cleared: Vec<VoxelKey> = surf
        .iter()
        .copied()
        .filter(|k| k.2 == landing_row && !map.voxels.contains_key(k))
        .collect();
    assert!(
        cleared.is_empty(),
        "landing must survive when the robot can see over it, cleared {cleared:?}"
    );
}

#[test]
fn two_misses_needed_when_max_health_is_two() {
    let cfg = Config {
        max_health: 2,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
    update_map(&mut map, (0.0, 0.0, 0.0), &[(3.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((3, 0, 0)), Some(2));

    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert_eq!(map.health((3, 0, 0)), Some(1));

    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert!(!map.voxels.contains_key(&(3, 0, 0)));
}

#[test]
fn planar_patch_yields_vertical_normal() {
    let mut v = Voxel::default();
    for i in 0..8 {
        for j in 0..8 {
            let x = 0.09 * (i as f32 / 7.0 - 0.5);
            let y = 0.09 * (j as f32 / 7.0 - 0.5);
            v.observe(Vector3::new(x, y, 0.0));
        }
    }
    let (n, _) = v
        .self_normal()
        .expect("a flat 2d patch must yield a normal");
    assert!(n[2].abs() > 0.99, "expected ~vertical normal, got {n:?}");
}

#[test]
fn line_like_patch_has_no_normal() {
    // A scan-line is not planar, so it gets no normal.
    let mut v = Voxel::default();
    for j in 0..20 {
        let y = 0.08 * (j as f32 / 19.0 - 0.5);
        let z = 0.003 * ((j % 3) - 1) as f32;
        v.observe(Vector3::new(0.0, y, z));
    }
    assert!(
        v.self_normal().is_none(),
        "a scan-line has no trustworthy normal"
    );
}

/// A grazing ray spares a planar floor, with no dependence on how recently it
/// was hit: the normal alone earns the spare.
#[test]
fn grazing_ray_spares_planar_floor() {
    let voxel_size = 0.1_f32;
    let y_half = 0.3_f32;
    let ds = voxel_size / 3.0;
    let nx = (20.0 / ds).ceil() as i32;
    let ny = (2.0 * y_half / ds).ceil() as i32;
    let floor_z = voxel_size * 0.5;
    let floor: Vec<(f32, f32, f32)> = (0..=nx)
        .flat_map(|i| (0..=ny).map(move |j| (i as f32 * ds, -y_half + j as f32 * ds, floor_z)))
        .collect();
    let origin = (0.0_f32, 0.0_f32, 0.35_f32);
    let ray = vec![(8.0_f32, 0.0, 0.0)];

    let cfg = Config {
        voxel_size,
        fine_divisor: 0,
        max_range: 50.0,
        ray_subsample: 1,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        min_health: 0,
        max_health: 1,
        graze_cos: 0.5,
        support_min: 0,
        emit_every: 1,
        global_emit_every: 1,
        region_percentile: 95.0,
        world_frame: "world".to_string(),
        tf_match_tolerance_s: 0.1,
        worker_threads: 4,
    };
    let (mut map, _) = build_surface(&floor, voxel_size, cfg.max_health);
    let row: Vec<VoxelKey> = map
        .voxels
        .keys()
        .copied()
        .filter(|k| k.1 == 0 && k.2 == 0)
        .collect();
    update_map(&mut map, origin, &ray, &cfg);
    let clipped = row.iter().filter(|k| !map.voxels.contains_key(k)).count();
    assert_eq!(clipped, 0, "a planar floor keeps its grazing spare");
}

#[test]
fn support_gate_drops_isolated_voxels() {
    let voxel_size = 1.0;
    let mut map = VoxelMap::default();
    // A 3x3 surface patch, plus one isolated voxel far from anything.
    for x in 0..3 {
        for y in 0..3 {
            map.set_health((x, y, 0), 1);
        }
    }
    map.set_health((20, 20, 0), 1);
    let bounds = LocalBounds {
        origin_x: 0.0,
        origin_y: 0.0,
        r_xy_max_sq: 1e6,
        z_min: -10.0,
        z_max: 10.0,
    };

    let no_live = AHashSet::new();
    // support_min 0 emits every surface voxel.
    assert_eq!(
        tuples(emit_points(&map, voxel_size, Some(&bounds), 0, &no_live)).len(),
        10
    );

    // Every patch cell has at least 3 surface neighbors (the corners exactly
    // 3), so support_min 3 keeps the patch and drops only the isolated voxel.
    let gated = tuples(emit_points(&map, voxel_size, Some(&bounds), 3, &no_live));
    assert_eq!(gated.len(), 9);
    let half = voxel_size * 0.5;
    let isolated = (20.0 + half, 20.0 + half, half);
    assert!(
        !gated.contains(&isolated),
        "isolated voxel must be gated out"
    );
}

/// The whole-map scan `emit_points` used before the chunk index, kept only as a
/// reference to differentially test the indexed implementation against.
fn emit_points_naive(
    map: &VoxelMap,
    voxel_size: f32,
    bounds: Option<&LocalBounds>,
    support_min: i32,
    live: &AHashSet<VoxelKey>,
) -> Vec<(f32, f32, f32)> {
    let in_bounds = |x, y, z| bounds.is_none_or(|b| b.contains(x, y, z));
    let mut out = Vec::with_capacity(map.voxels.len() + live.len());
    for (&key, c) in map.voxels.iter() {
        if c.health <= 0 {
            continue;
        }
        let (x, y, z) = voxel_center(key, voxel_size);
        if !in_bounds(x, y, z) {
            continue;
        }
        if support_min > 0 && c.support < support_min as u32 {
            continue;
        }
        out.push((x, y, z));
    }
    for &key in live.iter() {
        if matches!(map.voxels.get(&key), Some(c) if c.health > 0) {
            continue;
        }
        let (x, y, z) = voxel_center(key, voxel_size);
        if !in_bounds(x, y, z) {
            continue;
        }
        out.push((x, y, z));
    }
    out
}

fn sort_points(mut pts: Vec<(f32, f32, f32)>) -> Vec<(f32, f32, f32)> {
    pts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    pts
}

/// Flat emitter output as (x, y, z) tuples for assertions.
fn tuples(flat: Vec<f32>) -> Vec<(f32, f32, f32)> {
    flat.as_chunks::<3>()
        .0
        .iter()
        .map(|p| (p[0], p[1], p[2]))
        .collect()
}

/// The chunk-indexed `emit_points` must return exactly what the old whole-map
/// scan did, across randomized maps that straddle chunk boundaries, sparse and
/// dense regions, and both bounded and unbounded queries.
#[test]
fn emit_points_matches_naive_scan_on_random_maps() {
    let mut state = 88172645463325252_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };

    let voxel_size = 0.5;
    let bounds = LocalBounds {
        origin_x: 0.0,
        origin_y: 0.0,
        r_xy_max_sq: 25.0,
        z_min: -2.0,
        z_max: 2.0,
    };
    let live: AHashSet<VoxelKey> = AHashSet::new();

    for trial in 0..20 {
        let mut map = VoxelMap::default();
        for _ in 0..400 {
            let key = (
                (next_u64() % 40) as i32 - 20,
                (next_u64() % 40) as i32 - 20,
                (next_u64() % 10) as i32 - 5,
            );
            // Mostly unhealthy so healthy voxels are sparse relative to entries.
            let health = (next_u64() % 7) as i32 - 5;
            map.set_health(key, health);
        }

        for (support_min, use_bounds) in [(0, false), (0, true), (2, false), (2, true), (4, true)] {
            let b = use_bounds.then_some(&bounds);
            let got = sort_points(tuples(emit_points(&map, voxel_size, b, support_min, &live)));
            let want = sort_points(emit_points_naive(&map, voxel_size, b, support_min, &live));
            assert_eq!(
                got, want,
                "trial {trial} support_min={support_min} bounds={use_bounds}"
            );
        }
    }
}

/// The healthy-chunk index must stay lean regardless of how many unhealthy
/// entries pile up in `voxels`. Indexing on health transitions instead of
/// scanning every entry at emit time is its whole point.
#[test]
fn healthy_chunk_index_excludes_dead_entries_regardless_of_count() {
    let mut map = VoxelMap::default();
    for i in 0..50_000_i32 {
        map.set_health((i % 500, (i / 500) % 500, 0), 0); // health=0, never healthy
    }
    for x in 0..3 {
        for y in 0..3 {
            map.set_health((x, y, 100), 1);
        }
    }

    let live = AHashSet::new();
    let points = tuples(emit_points(&map, 1.0, None, 0, &live));
    assert_eq!(points.len(), 9, "only the healthy patch is emitted");

    let indexed: usize = map.healthy_chunks.values().map(|s| s.len()).sum();
    assert_eq!(
        indexed, 9,
        "dead entries never enter the healthy-chunk index"
    );
}

/// `update_map` drives both `record_hit` (new voxel becomes healthy) and
/// `record_miss` (healthy voxel cleared). The index must track both.
#[test]
fn healthy_chunk_index_tracks_health_transitions_through_update_map() {
    let cfg = basic_config(); // min_health=0, max_health=1
    let mut map = VoxelMap::default();
    map.set_health((3, 0, 0), 1);
    assert_eq!(
        map.healthy_chunks.values().map(|s| s.len()).sum::<usize>(),
        1,
        "set_health() indexes the healthy voxel"
    );

    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);

    assert!(
        !map.voxels.contains_key(&(3, 0, 0)),
        "voxel on the ray is cleared"
    );
    let indexed: Vec<VoxelKey> = map
        .healthy_chunks
        .values()
        .flat_map(|s| s.iter().copied())
        .collect();
    assert_eq!(
        indexed,
        vec![(5, 0, 0)],
        "index drops the cleared voxel and gains the new hit"
    );
}

/// Every voxel's incremental `support` field must equal a from-scratch
/// 26-neighbor scan after any sequence of hits, misses, and direct
/// `set_health` calls. Covers transitions that happen after a voxel's
/// neighbors already exist, which seeding only at creation would miss.
#[test]
fn support_field_matches_neighbor_scan_after_random_transitions() {
    let mut state = 5573589319906701683_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };

    let cfg = Config {
        min_health: -1,
        max_health: 2,
        ..basic_config()
    };

    for trial in 0..10 {
        let mut map = VoxelMap::default();
        for step in 0..300 {
            let key = (
                (next_u64() % 8) as i32 - 4,
                (next_u64() % 8) as i32 - 4,
                (next_u64() % 4) as i32 - 2,
            );
            match next_u64() % 3 {
                0 => {
                    map.record_hit(key, cfg.min_health, cfg.max_health);
                }
                1 => {
                    map.record_miss(key, cfg.min_health);
                }
                _ => {
                    let health = (next_u64() % 5) as i32 - 2;
                    map.set_health(key, health);
                }
            }

            let keys: Vec<VoxelKey> = map.voxels.keys().copied().collect();
            for k in keys {
                let want = map.count_healthy_neighbors(k);
                let got = map.voxels[&k].support;
                assert_eq!(
                    got, want,
                    "trial {trial} step {step}: support({k:?}) = {got}, want {want}"
                );
            }
        }
    }
}

/// A voxel created into a neighborhood that already has healthy neighbors
/// must pick up their count immediately, not start at zero.
#[test]
fn new_voxel_seeds_support_from_existing_healthy_neighbors() {
    let mut map = VoxelMap::default();
    map.set_health((1, 0, 0), 1);
    map.set_health((-1, 0, 0), 1);
    map.set_health((0, 1, 0), 1);

    map.set_health((0, 0, 0), 1);

    assert_eq!(
        map.voxels[&(0, 0, 0)].support,
        3,
        "new voxel must count its 3 pre-existing healthy neighbors"
    );
}

fn fine_config(divisor: u32) -> Config {
    Config {
        fine_divisor: divisor,
        ..basic_config()
    }
}

#[test]
fn fine_key_split_join_round_trip() {
    for divisor in [2_i32, 3, 4] {
        for fine_key in [(0, 0, 0), (5, -7, 2), (-1, -2, -3), (-9, 8, -27)] {
            let (coarse, index) = split_fine_key(fine_key, divisor);
            assert!(index < (divisor * divisor * divisor) as usize);
            assert_eq!(
                join_fine_key(coarse, index, divisor),
                fine_key,
                "divisor={divisor} fine_key={fine_key:?}"
            );
        }
    }
}

#[test]
fn config_rejects_bad_fine_settings() {
    use validator::Validate;
    let mut cfg = basic_config();
    assert!(cfg.validate().is_ok());
    cfg.fine_divisor = 1;
    assert!(cfg.validate().is_err(), "divisor 1 duplicates the map");
    cfg.fine_divisor = 5;
    assert!(cfg.validate().is_err(), "divisor^3 bits must fit in a u64");
    cfg.fine_divisor = 2;
    assert!(cfg.validate().is_ok());
}

#[test]
fn fine_layer_off_stores_no_counts() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    let hits = update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    assert!(hits.fine.is_empty());
    assert!(map.voxels.values().all(|v| v.fine == 0));
}

#[test]
fn fine_emission_nests_inside_healthy_voxels() {
    let cfg = fine_config(2);
    let mut map = VoxelMap::default();
    update_map(
        &mut map,
        (0.0, 0.0, 0.0),
        &[(5.1, 0.1, 0.1), (5.6, 0.6, 0.6)],
        &cfg,
    );
    let no_live = AHashSet::new();
    let got = sort_points(tuples(emit_points_fine(&map, 1.0, 2, None, 0, &no_live)));
    assert_eq!(got, vec![(5.25, 0.25, 0.25), (5.75, 0.75, 0.75)]);
}

#[test]
fn cleared_voxel_drops_fine_detail() {
    let cfg = fine_config(2);
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.1, 0.1, 0.1)], &cfg);

    // A ray through voxel (5, 0, 0) clears it, taking its fine counts along.
    update_map(&mut map, (0.0, 0.0, 0.0), &[(8.5, 0.5, 0.5)], &cfg);
    assert!(!map.voxels.contains_key(&(5, 0, 0)));

    // Re-observing the voxel starts fresh instead of resurrecting old detail.
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.6, 0.6, 0.6)], &cfg);
    let no_live = AHashSet::new();
    let got = tuples(emit_points_fine(&map, 1.0, 2, None, 0, &no_live));
    assert!(got.contains(&(5.75, 0.75, 0.75)));
    assert!(
        !got.contains(&(5.25, 0.25, 0.25)),
        "cleared fine detail must not resurface"
    );
}

#[test]
fn live_fine_cells_emit_before_confirmation() {
    let cfg = Config {
        min_health: -1,
        ..fine_config(2)
    };
    let mut map = VoxelMap::default();
    let hits = update_map(&mut map, (0.0, 0.0, 0.0), &[(5.1, 0.1, 0.1)], &cfg);
    assert_eq!(map.health((5, 0, 0)), Some(0), "not yet healthy");

    let no_live = AHashSet::new();
    assert!(emit_points_fine(&map, 1.0, 2, None, 0, &no_live).is_empty());
    assert_eq!(
        tuples(emit_points_fine(&map, 1.0, 2, None, 0, &hits.fine)),
        vec![(5.25, 0.25, 0.25)],
        "this frame's returns show through the live merge"
    );

    // Once the voxel confirms healthy the gated path covers the live cell.
    let hits = update_map(&mut map, (0.0, 0.0, 0.0), &[(5.1, 0.1, 0.1)], &cfg);
    assert_eq!(
        tuples(emit_points_fine(&map, 1.0, 2, None, 0, &hits.fine)),
        vec![(5.25, 0.25, 0.25)],
        "a covered live cell is not emitted twice"
    );
}

#[test]
fn fine_emission_applies_support_min() {
    let mut map = VoxelMap::default();
    for x in 0..3 {
        for y in 0..3 {
            map.set_health((x, y, 0), 1);
            map.accumulate((x as f32 + 0.5, y as f32 + 0.5, 0.5), 1.0, Some(2));
        }
    }
    map.set_health((20, 20, 0), 1);
    map.accumulate((20.5, 20.5, 0.5), 1.0, Some(2));

    let no_live = AHashSet::new();
    assert_eq!(
        tuples(emit_points_fine(&map, 1.0, 2, None, 0, &no_live)).len(),
        10
    );
    let gated = tuples(emit_points_fine(&map, 1.0, 2, None, 3, &no_live));
    assert_eq!(gated.len(), 9, "isolated voxel's fine cell is gated out");
}

/// Milestone-gated refits must track continuously-refit normals: after
/// streaming a jittered floor over many frames, every cached normal matches
/// a from-scratch pooled fit almost exactly.
#[test]
fn milestone_gated_normals_match_full_refit() {
    let mut state = 7043284794951226509_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let jitter = move || ((next_u64() % 1000) as f32 / 1000.0 - 0.5) * 0.01;

    let voxel_size = 0.1_f32;
    let cfg = Config {
        voxel_size,
        max_range: 50.0,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        graze_cos: 0.5,
        ..basic_config()
    };
    let ds = voxel_size / 3.0;
    let n = (2.0 / ds).ceil() as i32;
    let origin = (1.0, 1.0, 1.0);
    let mut map = VoxelMap::default();
    for _ in 0..12 {
        let floor: Vec<(f32, f32, f32)> = (0..=n)
            .flat_map(|i| {
                let mut j2 = jitter;
                (0..=n).map(move |j| (i as f32 * ds, j as f32 * ds, voxel_size * 0.5 + j2()))
            })
            .collect();
        update_map(&mut map, origin, &floor, &cfg);
    }

    let mut checked = 0;
    for (&key, v) in map.voxels.iter() {
        if v.num_pts < 10 {
            continue;
        }
        let Some((want, _)) = pooled_normal(&map.voxels, key, voxel_size) else {
            continue;
        };
        let got = v.normal.expect("streamed voxel must carry a normal");
        assert!(
            got.dot(&want).abs() > 0.99,
            "stale normal at {key:?}: cached {got:?}, fresh {want:?}"
        );
        checked += 1;
    }
    assert!(checked > 100, "expected a real floor, checked {checked}");
}

/// A voxel created below its first milestone gets a pooled fit from converged
/// neighbors on its creation frame, keeping the grazing spare available.
#[test]
fn new_voxel_gets_pooled_normal_on_creation() {
    let voxel_size = 0.1_f32;
    let cfg = Config {
        voxel_size,
        max_range: 50.0,
        shadow_depth: 0.2,
        grace_depth: 0.2,
        graze_cos: 0.5,
        ..basic_config()
    };
    let ds = voxel_size / 3.0;
    let n = (2.0 / ds).ceil() as i32;
    let origin = (1.0, 1.0, 1.0);
    let gap = (10, 10, 0);
    let mut map = VoxelMap::default();
    for _ in 0..12 {
        let floor: Vec<(f32, f32, f32)> = (0..=n)
            .flat_map(|i| (0..=n).map(move |j| (i as f32 * ds, j as f32 * ds, voxel_size * 0.5)))
            .filter(|&(x, y, z)| world_to_voxel(x, y, z, 1.0 / voxel_size) != gap)
            .collect();
        update_map(&mut map, origin, &floor, &cfg);
    }
    assert!(
        !map.voxels.contains_key(&gap),
        "gap voxel must start absent"
    );

    update_map(&mut map, origin, &[(1.05, 1.05, 0.05)], &cfg);
    let v = &map.voxels[&gap];
    assert_eq!(v.num_pts, 1);
    assert!(
        v.normal.is_some(),
        "creation-frame voxel must carry a pooled normal"
    );
}

/// Whole-map fine scan kept as the reference for the parallel, interior-hoisted
/// `emit_points_fine`.
fn emit_points_fine_naive(
    map: &VoxelMap,
    voxel_size: f32,
    divisor: i32,
    bounds: Option<&LocalBounds>,
    support_min: i32,
) -> Vec<(f32, f32, f32)> {
    let fine_size = voxel_size / divisor as f32;
    let mut out = Vec::new();
    for (&key, v) in map.voxels.iter() {
        if v.health <= 0 {
            continue;
        }
        if support_min > 0 && v.support < support_min as u32 {
            continue;
        }
        let mut bits = v.fine;
        while bits != 0 {
            let i = bits.trailing_zeros() as usize;
            bits &= bits - 1;
            let (x, y, z) = voxel_center(join_fine_key(key, i, divisor), fine_size);
            if bounds.is_none_or(|b| b.contains(x, y, z)) {
                out.push((x, y, z));
            }
        }
    }
    out
}

/// The parallel fine emission with chunk and voxel interior hoisting must
/// return exactly what a whole-map scan does, across randomized maps and both
/// bounded and unbounded queries.
#[test]
fn emit_points_fine_matches_naive_scan_on_random_maps() {
    let mut state = 3141592653589793238_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut next_coord = move || (next_u64() % 2400) as f32 / 100.0 - 12.0;

    let cfg = fine_config(3);
    let bounds = LocalBounds {
        origin_x: 0.5,
        origin_y: -0.5,
        r_xy_max_sq: 36.0,
        z_min: -3.0,
        z_max: 3.0,
    };
    let no_live = AHashSet::new();

    for trial in 0..10 {
        let mut map = VoxelMap::default();
        for _ in 0..4 {
            let points: Vec<(f32, f32, f32)> = (0..200)
                .map(|_| (next_coord(), next_coord(), next_coord()))
                .collect();
            update_map(&mut map, (20.0, 20.0, 20.0), &points, &cfg);
        }

        for (support_min, use_bounds) in [(0, false), (0, true), (2, true), (4, true)] {
            let b = use_bounds.then_some(&bounds);
            let got = sort_points(tuples(emit_points_fine(
                &map,
                1.0,
                3,
                b,
                support_min,
                &no_live,
            )));
            let want = sort_points(emit_points_fine_naive(&map, 1.0, 3, b, support_min));
            assert_eq!(
                got, want,
                "trial {trial} support_min={support_min} bounds={use_bounds}"
            );
        }
    }
}

/// Binning fine-emitted points by the divisor recovers exactly the healthy
/// voxel set: every fine child nests inside its parent and every healthy
/// voxel that saw returns emits at least one child.
#[test]
fn fine_children_bin_exactly_to_their_parents() {
    let mut state = 2891336453748303025_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut next_coord = move || (next_u64() % 1600) as f32 / 100.0 - 8.0;

    let cfg = fine_config(3);
    let mut map = VoxelMap::default();
    let points: Vec<(f32, f32, f32)> = (0..200)
        .map(|_| (next_coord(), next_coord(), next_coord()))
        .collect();
    update_map(&mut map, (20.0, 20.0, 20.0), &points, &cfg);

    let no_live = AHashSet::new();
    let fine = tuples(emit_points_fine(&map, 1.0, 3, None, 0, &no_live));
    let binned: AHashSet<VoxelKey> = fine
        .iter()
        .map(|&(x, y, z)| world_to_voxel(x, y, z, 1.0))
        .collect();
    let healthy: AHashSet<VoxelKey> = tuples(emit_points(&map, 1.0, None, 0, &no_live))
        .iter()
        .map(|&(x, y, z)| world_to_voxel(x, y, z, 1.0))
        .collect();
    assert_eq!(binned, healthy);
}

#[test]
fn metric_voxel_keys_quantize_by_map_resolution() {
    let keys: Vec<VoxelKey> =
        metric_voxel_keys([(0.049, 0.05, -0.001), (0.1, -0.1, 0.0)], 0.05).collect();

    assert_eq!(keys, vec![(0, 1, -1), (2, -2, 0)]);
}

/// A naive `voxels.remove` leaves the healthy-chunk index pointing at a voxel
/// that is gone. With support_min 0, emit_points reads only the index, so the
/// deleted voxel would come straight back out.
#[test]
fn clear_voxels_removes_from_the_healthy_chunk_index() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
    let no_live = AHashSet::new();
    assert!(!emit_points(&map, 1.0, None, 0, &no_live).is_empty());

    assert_eq!(map.clear_voxels([(5, 0, 0)]), 1);

    assert_eq!(map.health((5, 0, 0)), None);
    assert!(
        emit_points(&map, 1.0, None, 0, &no_live).is_empty(),
        "cleared voxel must not emit from a stale chunk index"
    );
}

/// Every one of the 26 neighbors counts a healthy voxel in its `support`.
/// Deleting it without decrementing them leaves counts that never come back
/// down, and support_min then keeps phantom surfaces in the local map.
#[test]
fn clear_voxels_decrements_neighbor_support() {
    let mut map = VoxelMap::default();
    map.set_health((0, 0, 0), 1);
    map.set_health((1, 0, 0), 1);
    map.set_health((0, 1, 0), 1);
    assert_eq!(map.voxels[&(1, 0, 0)].support, 2);
    assert_eq!(map.voxels[&(0, 1, 0)].support, 2);

    assert_eq!(map.clear_voxels([(0, 0, 0)]), 1);

    assert_eq!(map.voxels[&(1, 0, 0)].support, 1);
    assert_eq!(map.voxels[&(0, 1, 0)].support, 1);
}

/// An unhealthy voxel was never counted in its neighbors' support, so removing
/// it must not decrement them or the counts go stale downward.
#[test]
fn clear_voxels_leaves_support_alone_for_an_unhealthy_voxel() {
    let mut map = VoxelMap::default();
    map.set_health((1, 0, 0), 1);
    map.set_health((0, 0, 0), 0);
    assert_eq!(map.voxels[&(1, 0, 0)].support, 0);

    assert_eq!(map.clear_voxels([(0, 0, 0)]), 1);

    assert_eq!(map.voxels[&(1, 0, 0)].support, 0);
}

#[test]
fn clear_voxels_skips_keys_the_map_does_not_hold() {
    let mut map = VoxelMap::default();
    map.set_health((0, 0, 0), 1);

    assert_eq!(map.clear_voxels([(0, 0, 0), (9, 9, 9), (0, 0, 0)]), 1);

    assert!(map.voxels.is_empty());
}

/// The fine-cell bitmask lives inside the voxel, so clearing the coarse voxel
/// has to take its fine children with it.
#[test]
fn clear_voxels_takes_the_fine_layer_with_it() {
    let cfg = Config {
        fine_divisor: 2,
        ..basic_config()
    };
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.1, 0.1, 0.1)], &cfg);
    let no_live = AHashSet::new();
    assert!(!emit_points_fine(&map, 1.0, 2, None, 0, &no_live).is_empty());

    assert_eq!(map.clear_voxels([(5, 0, 0)]), 1);

    assert!(emit_points_fine(&map, 1.0, 2, None, 0, &no_live).is_empty());
}

/// Seeding a cloud into an empty map must build exactly what the maintained
/// incremental path builds: same voxels, health, moments, support counts,
/// chunk index, and normals.
#[test]
fn seed_matches_from_scratch_build() {
    let mut state = 6364136223846793005_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut next_coord = move || (next_u64() % 1200) as f32 / 100.0 - 6.0;
    let cfg = basic_config();
    let points: Vec<(f32, f32, f32)> = (0..600)
        .map(|_| (next_coord(), next_coord(), next_coord()))
        .collect();

    let mut seeded = VoxelMap::default();
    let created = seed_points(&mut seeded, &points, &cfg);
    let (reference, keys) = build_surface(&points, cfg.voxel_size, SEED_HEALTH);

    let mut created_sorted: Vec<VoxelKey> = created.iter().copied().collect();
    created_sorted.sort();
    assert_eq!(created_sorted, keys);
    assert_eq!(seeded.voxels.len(), reference.voxels.len());
    for (&key, want) in reference.voxels.iter() {
        let got = &seeded.voxels[&key];
        assert_eq!(got.health, want.health, "health at {key:?}");
        assert_eq!(got.num_pts, want.num_pts, "num_pts at {key:?}");
        assert_eq!(got.sum, want.sum, "sum at {key:?}");
        assert_eq!(got.support, want.support, "support at {key:?}");
        match (got.normal, want.normal) {
            (Some(a), Some(b)) => {
                assert!(a.dot(&b).abs() > 0.999, "normal at {key:?}: {a:?} vs {b:?}")
            }
            (a, b) => assert_eq!(a.is_some(), b.is_some(), "normal presence at {key:?}"),
        }
    }
    assert_eq!(seeded.healthy_chunks, reference.healthy_chunks);

    let no_live = AHashSet::new();
    for support_min in [0, 3] {
        let got = sort_points(tuples(emit_points(
            &seeded,
            1.0,
            None,
            support_min,
            &no_live,
        )));
        let want = sort_points(tuples(emit_points(
            &reference,
            1.0,
            None,
            support_min,
            &no_live,
        )));
        assert_eq!(got, want, "emit at support_min={support_min}");
    }
}

/// A seed never touches a voxel live data already observed: health and
/// moments stay exactly as they were. Only the support bookkeeping sees the
/// new healthy neighbor.
#[test]
fn seed_leaves_live_voxels_untouched() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    update_map(&mut map, (0.0, 0.0, 0.0), &[(5.2, 0.2, 0.2)], &cfg);
    let before = map.voxels[&(5, 0, 0)].clone();

    let created = seed_points(&mut map, &[(5.7, 0.7, 0.7), (6.5, 0.5, 0.5)], &cfg);

    assert_eq!(created.len(), 1);
    assert!(created.contains(&(6, 0, 0)));
    let after = &map.voxels[&(5, 0, 0)];
    assert_eq!(after.health, before.health);
    assert_eq!(
        after.num_pts, before.num_pts,
        "premap points must not pollute live moments"
    );
    assert_eq!(after.sum, before.sum);
    assert_eq!(after.support, 1, "the seeded neighbor still counts");
    assert_eq!(map.voxels[&(6, 0, 0)].support, 1);
    assert_eq!(map.health((6, 0, 0)), Some(SEED_HEALTH));
}

/// Seeding the same cloud twice creates nothing and adds no moments.
#[test]
fn second_seed_is_a_no_op() {
    let cfg = basic_config();
    let mut map = VoxelMap::default();
    let points = [(1.5, 1.5, 0.5), (2.5, 1.5, 0.5)];
    assert_eq!(seed_points(&mut map, &points, &cfg).len(), 2);
    let num_pts_before = map.voxels[&(1, 1, 0)].num_pts;
    let support_before = map.voxels[&(1, 1, 0)].support;

    let again = seed_points(&mut map, &points, &cfg);

    assert!(again.is_empty());
    assert_eq!(map.voxels.len(), 2);
    assert_eq!(map.voxels[&(1, 1, 0)].num_pts, num_pts_before);
    assert_eq!(map.voxels[&(1, 1, 0)].support, support_before);
}

/// Seeded wall voxels get pooled normal fits, so a grazing ray spares them
/// where a moment-less wall of the same shape is carved.
#[test]
fn seeded_wall_normals_spare_grazing_rays() {
    let cfg = basic_config();
    let mut wall: Vec<(f32, f32, f32)> = Vec::new();
    for iy in 0..16 {
        for iz in 0..16 {
            wall.push((5.5, 0.25 + iy as f32 * 0.5, 0.25 + iz as f32 * 0.5));
        }
    }

    let mut seeded = VoxelMap::default();
    seed_points(&mut seeded, &wall, &cfg);
    assert!(
        seeded.voxels[&(5, 4, 4)].planar_normal().is_some(),
        "seeded wall voxel must carry a normal"
    );

    // The same wall occupied without moments, so nothing fits a plane.
    let mut bare = VoxelMap::default();
    for key in metric_voxel_keys(wall.iter().copied(), cfg.voxel_size) {
        bare.set_health(key, SEED_HEALTH);
    }

    // A ray running along the wall plane inside one wall voxel column.
    let origin = (5.5, -2.5, 4.5);
    let hit = [(5.5, 10.5, 4.5)];
    update_map(&mut seeded, origin, &hit, &cfg);
    update_map(&mut bare, origin, &hit, &cfg);

    assert_eq!(
        seeded.health((5, 4, 4)),
        Some(SEED_HEALTH),
        "the normal earns the grazing spare"
    );
    assert_eq!(
        bare.health((5, 4, 4)),
        None,
        "without a normal the graze carves"
    );
}

/// With a fine layer on, seeding marks the observed fine cells.
#[test]
fn seed_sets_fine_bits_when_layer_is_on() {
    let cfg = fine_config(2);
    let mut map = VoxelMap::default();
    seed_points(&mut map, &[(5.1, 0.1, 0.1)], &cfg);

    let no_live = AHashSet::new();
    assert_eq!(
        tuples(emit_points_fine(&map, 1.0, 2, None, 0, &no_live)),
        vec![(5.25, 0.25, 0.25)]
    );
}

/// Seeding into a live map must leave every pre-existing voxel's health
/// alone while keeping the support counts and the healthy-chunk index
/// exactly what a from-scratch scan would produce.
#[test]
fn seed_into_live_map_keeps_indexes_consistent() {
    let mut state = 1442695040888963407_u64;
    let mut next_u64 = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut next_coord = move || (next_u64() % 800) as f32 / 100.0 - 4.0;
    let cfg = Config {
        min_health: -1,
        max_health: 3,
        ..basic_config()
    };

    for trial in 0..5 {
        let mut map = VoxelMap::default();
        for _ in 0..3 {
            let frame: Vec<(f32, f32, f32)> = (0..150)
                .map(|_| (next_coord(), next_coord(), next_coord()))
                .collect();
            update_map(&mut map, (0.0, 0.0, 10.0), &frame, &cfg);
        }
        let cloud: Vec<(f32, f32, f32)> = (0..400)
            .map(|_| (next_coord(), next_coord(), next_coord()))
            .collect();
        let healths_before: AHashMap<VoxelKey, VoxelHealth> =
            map.voxels.iter().map(|(&k, v)| (k, v.health)).collect();

        let created = seed_points(&mut map, &cloud, &cfg);

        for (key, health) in healths_before {
            assert_eq!(
                map.voxels[&key].health, health,
                "trial {trial}: pre-existing {key:?} changed"
            );
        }
        for &key in created.iter() {
            assert_eq!(map.voxels[&key].health, SEED_HEALTH, "trial {trial}");
        }
        let keys: Vec<VoxelKey> = map.voxels.keys().copied().collect();
        for k in keys {
            assert_eq!(
                map.voxels[&k].support,
                map.count_healthy_neighbors(k),
                "trial {trial}: support({k:?})"
            );
        }
        let indexed: AHashSet<VoxelKey> = map
            .healthy_chunks
            .values()
            .flat_map(|s| s.iter().copied())
            .collect();
        let healthy: AHashSet<VoxelKey> = map
            .voxels
            .iter()
            .filter(|(_, v)| v.health > 0)
            .map(|(&k, _)| k)
            .collect();
        assert_eq!(indexed, healthy, "trial {trial}: chunk index");
    }
}
