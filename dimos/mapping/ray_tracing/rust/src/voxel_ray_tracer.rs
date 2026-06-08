// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use ahash::{AHashMap, AHashSet};
use serde::Deserialize;
use validator::{Validate, ValidationError};

pub type VoxelKey = (i32, i32, i32);
pub type VoxelHealth = i32;

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
#[validate(schema(function = "validate_health_range"))]
pub struct Config {
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(min = 0.0))]
    pub max_range: f32,
    #[validate(range(min = 1))]
    pub ray_subsample: u32,
    #[validate(range(min = 0.0))]
    pub shadow_depth: f32,
    #[validate(range(min = 0.0))]
    pub grace_depth: f32,
    pub min_health: i32,
    #[validate(range(min = 1))]
    pub max_health: i32,
}

fn validate_health_range(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    Ok(())
}

#[derive(Default)]
pub struct VoxelMap {
    pub voxels: AHashMap<VoxelKey, VoxelHealth>,
}

impl VoxelMap {
    pub fn healthy_count(&self) -> usize {
        self.voxels.values().filter(|h| **h > 0).count()
    }
}

pub struct LocalBounds {
    pub origin_x: f32,
    pub origin_y: f32,
    pub r_xy_max_sq: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl LocalBounds {
    pub fn contains(&self, x: f32, y: f32, z: f32) -> bool {
        if z < self.z_min || z > self.z_max {
            return false;
        }
        let dx = x - self.origin_x;
        let dy = y - self.origin_y;
        dx * dx + dy * dy <= self.r_xy_max_sq
    }
}

pub fn iter_global_points(
    map: &VoxelMap,
    voxel_size: f32,
) -> impl Iterator<Item = (f32, f32, f32)> + '_ {
    let half = voxel_size * 0.5;
    map.voxels
        .iter()
        .filter(|(_, &h)| h > 0)
        .map(move |(&(kx, ky, kz), _)| {
            (
                kx as f32 * voxel_size + half,
                ky as f32 * voxel_size + half,
                kz as f32 * voxel_size + half,
            )
        })
}

fn live_voxels(points: &[(f32, f32, f32)], voxel_size: f32) -> AHashSet<VoxelKey> {
    let inv = 1.0_f32 / voxel_size;
    let mut out: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
    for &(x, y, z) in points {
        out.insert(world_to_voxel(x, y, z, inv));
    }
    out
}

pub fn update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> AHashSet<VoxelKey> {
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    let hits = live_voxels(points, cfg.voxel_size);

    let mut misses: AHashSet<VoxelKey> = AHashSet::new();
    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let step = cfg.ray_subsample as usize;
    for (i, &p) in points.iter().enumerate() {
        if i % step != 0 {
            continue;
        }
        let dx = p.0 - origin.0;
        let dy = p.1 - origin.1;
        let dz = p.2 - origin.2;
        if dx * dx + dy * dy + dz * dz > max_range_sq {
            continue;
        }
        let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
        find_misses_along_ray(
            &mut misses,
            &map.voxels,
            origin,
            p,
            cfg.voxel_size,
            cfg.shadow_depth,
            cfg.grace_depth,
            origin_voxel,
            endpoint,
        );
    }

    // add new hits
    for v in &hits {
        let h = map.voxels.entry(*v).or_insert(cfg.min_health);
        *h = (*h + 1).min(cfg.max_health);
    }

    // each miss is only checked once
    for v in misses.difference(&hits) {
        if let Some(h) = map.voxels.get_mut(v) {
            *h -= 1;
            if *h <= cfg.min_health {
                map.voxels.remove(v);
            }
        }
    }

    hits
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Amanatides & Woo 3d DDA. Records voxels on ray in between the end of the shadow region
/// and origin if it is in the map. Voxels within grace region of the endpoint are spared from being marked as misses.
#[allow(clippy::too_many_arguments)]
fn find_misses_along_ray(
    misses: &mut AHashSet<VoxelKey>,
    map_voxels: &AHashMap<VoxelKey, VoxelHealth>,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
    shadow_depth: f32,
    grace_depth: f32,
    origin_voxel: VoxelKey,
    endpoint: VoxelKey,
) {
    if origin_voxel == endpoint {
        return;
    }

    let (ox, oy, oz) = origin;
    let dx = end.0 - ox;
    let dy = end.1 - oy;
    let dz = end.2 - oz;

    let (mut x, mut y, mut z) = origin_voxel;

    let step_x = dx.signum() as i32;
    let step_y = dy.signum() as i32;
    let step_z = dz.signum() as i32;

    let t_max_init = |p: f32, d: f32, vox: i32, step: i32| -> f32 {
        if step == 0 {
            return f32::INFINITY;
        }
        let next_boundary = if step > 0 {
            (vox + 1) as f32 * voxel_size
        } else {
            vox as f32 * voxel_size
        };
        (next_boundary - p) / d
    };

    let mut tx = t_max_init(ox, dx, x, step_x);
    let mut ty = t_max_init(oy, dy, y, step_y);
    let mut tz = t_max_init(oz, dz, z, step_z);

    let dt_x = if step_x == 0 {
        f32::INFINITY
    } else {
        voxel_size / dx.abs()
    };
    let dt_y = if step_y == 0 {
        f32::INFINITY
    } else {
        voxel_size / dy.abs()
    };
    let dt_z = if step_z == 0 {
        f32::INFINITY
    } else {
        voxel_size / dz.abs()
    };

    let half = voxel_size * 0.5;
    let endpoint_center = (
        endpoint.0 as f32 * voxel_size + half,
        endpoint.1 as f32 * voxel_size + half,
        endpoint.2 as f32 * voxel_size + half,
    );
    let shadow_sq = shadow_depth.powi(2);
    let grace_sq = grace_depth.powi(2);

    let ray_len = (dx * dx + dy * dy + dz * dz).sqrt();
    let t_max = 1.0 + shadow_depth / ray_len.max(f32::EPSILON);

    let mut past_endpoint = false;
    loop {
        let t_enter = tx.min(ty).min(tz);
        if t_enter > t_max {
            return;
        }
        if t_enter >= 1.0 {
            past_endpoint = true;
        }

        if tx < ty {
            if tx < tz {
                x += step_x;
                tx += dt_x;
            } else {
                z += step_z;
                tz += dt_z;
            }
        } else if ty < tz {
            y += step_y;
            ty += dt_y;
        } else {
            z += step_z;
            tz += dt_z;
        }

        if (x, y, z) == endpoint {
            past_endpoint = true;
            continue;
        }

        // don't remove points in the same xy plane as the hit, unless the plane only walks that plane
        // we do this to preserve floors, which is more important than some missed points
        if origin_voxel.2 != endpoint.2 && z == endpoint.2 {
            continue;
        }

        let cx = x as f32 * voxel_size + half;
        let cy = y as f32 * voxel_size + half;
        let cz = z as f32 * voxel_size + half;
        let ddx = cx - endpoint_center.0;
        let ddy = cy - endpoint_center.1;
        let ddz = cz - endpoint_center.2;
        let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;

        if past_endpoint {
            // continue past the endpoint and in to the shadow realm
            if dist_sq > shadow_sq {
                return;
            }
        } else if dist_sq < grace_sq {
            // too close to the endpoint to safely mark as miss because we might be clipping other voxel's rays
            continue;
        }

        if map_voxels.contains_key(&(x, y, z)) {
            misses.insert((x, y, z));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basic_config() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            ray_subsample: 1,
            shadow_depth: 2.0,
            grace_depth: 0.0,
            min_health: 0,
            max_health: 1,
        }
    }

    #[test]
    fn find_misses_along_ray_hits_correct_voxels_1() {
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
        let mut map_voxels: AHashMap<VoxelKey, VoxelHealth> = AHashMap::new();
        for v in &expected {
            map_voxels.insert(*v, 1);
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
            origin_voxel,
            endpoint,
        );

        assert_eq!(misses, expected);
    }

    #[test]
    fn find_misses_along_ray_hits_correct_voxels_2() {
        let voxel_size = 1.0;
        let shadow_depth = 2.0;
        let origin = (0.5, 0.5, 0.5);
        let end = (3.5, 2.5, 1.5);
        let inv = 1.0 / voxel_size;
        let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
        let endpoint = world_to_voxel(end.0, end.1, end.2, inv);

        let walked: AHashSet<VoxelKey> = [
            (1, 0, 0),
            (1, 1, 0),
            (1, 1, 1),
            (2, 1, 1),
            (2, 2, 1),
            (4, 2, 1),
            (4, 3, 1),
            (4, 3, 2),
        ]
        .into_iter()
        .collect();
        let mut map_voxels: AHashMap<VoxelKey, VoxelHealth> = AHashMap::new();
        for v in &walked {
            map_voxels.insert(*v, 1);
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
            origin_voxel,
            endpoint,
        );

        // z-slab protection skips voxels in the endpoint's z-slab when the
        // ray crosses z-slabs. Endpoint is at z=1 here.
        let expected: AHashSet<VoxelKey> = walked
            .iter()
            .filter(|v| v.2 != endpoint.2)
            .copied()
            .collect();
        assert_eq!(misses, expected);
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
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(0, 5, 0)), Some(&1));
        assert_eq!(map.voxels.len(), 2);
    }

    #[test]
    fn voxels_on_ray_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // make sure the initial point got cleared by the new update
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_not_on_ray_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 5, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 5, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_within_shadow_region_are_removed() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((6, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        // point within the shadow is no longer included, new point is included
        assert!(!map.voxels.contains_key(&(6, 0, 0)));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn voxels_beyond_shadow_region_survive() {
        let cfg = basic_config();
        let mut map = VoxelMap::default();
        map.voxels.insert((8, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(8, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
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
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    #[test]
    fn point_beyond_max_range_does_not_clear() {
        let cfg = Config {
            max_range: 3.0,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        map.voxels.insert((3, 0, 0), 1);
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));
    }

    #[test]
    fn two_hits_needed_when_min_health_is_negative() {
        let cfg = Config {
            min_health: -1,
            ..basic_config()
        };
        let mut map = VoxelMap::default();
        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&0));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(5, 0, 0)), Some(&1));
    }

    /// Test how bad the planar ray clipping is.
    /// For example, points on floors can be counted as misses because they are close to the same ray as the hit.
    #[test]
    fn ground_clipping_single_ray() {
        let voxel_size = 0.1_f32;
        let lidar_height = 1.0_f32;
        let cfg = Config {
            voxel_size,
            max_range: 50.0,
            ray_subsample: 1,
            shadow_depth: 0.2,
            grace_depth: 0.2,
            min_health: 0,
            max_health: 1,
        };
        let inv = 1.0 / voxel_size;

        // Cover the full range we will probe, plus a little for shadow.
        let max_x = 25.0_f32;
        let n_ground = (max_x / voxel_size).ceil() as i32;

        let ranges: Vec<f32> = (1..=20).map(|i| i as f32).collect();
        let mut table = format!(
            "voxel_size={voxel_size} lidar_height={lidar_height} grace={} shadow={}\n\
             range_m  ground_voxels_in_row  clipped  clipped_pct\n",
            cfg.grace_depth, cfg.shadow_depth
        );
        let mut total_clipped = 0usize;
        for &range in &ranges {
            let mut map = VoxelMap::default();
            for i in 0..n_ground {
                let x = (i as f32) * voxel_size + voxel_size * 0.5;
                let key = world_to_voxel(x, 0.0, 0.0, inv);
                map.voxels.insert(key, cfg.max_health);
            }
            let n_before = map.voxels.len();

            let origin = (0.0_f32, 0.0_f32, lidar_height);
            let points = vec![(range, 0.0_f32, 0.0_f32)];
            update_map(&mut map, origin, &points, &cfg);

            let n_after_ground: usize = (0..n_ground)
                .filter(|i| {
                    let x = (*i as f32) * voxel_size + voxel_size * 0.5;
                    let key = world_to_voxel(x, 0.0, 0.0, inv);
                    map.voxels.contains_key(&key)
                })
                .count();
            let clipped = n_before - n_after_ground;
            let pct = 100.0 * clipped as f32 / n_before as f32;
            table.push_str(&format!(
                "{range:>6.1}  {n_before:>20}  {clipped:>7}  {pct:>10.1}\n"
            ));
            total_clipped += clipped;
        }
        eprint!("{table}");
        assert!(
            total_clipped == 0,
            "planar grace regressed, ground voxels clipped:\n{table}"
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
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&2));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert_eq!(map.voxels.get(&(3, 0, 0)), Some(&1));

        update_map(&mut map, (0.0, 0.0, 0.0), &[(5.5, 0.5, 0.5)], &cfg);
        assert!(!map.voxels.contains_key(&(3, 0, 0)));
    }

    #[test]
    fn validate_rejects_zero_voxel_size() {
        let cfg = Config {
            voxel_size: 0.0,
            ..basic_config()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_min_health_geq_max_health() {
        let cfg = Config {
            min_health: 5,
            max_health: 1,
            ..basic_config()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_accepts_basic_config() {
        assert!(basic_config().validate().is_ok());
    }
}
