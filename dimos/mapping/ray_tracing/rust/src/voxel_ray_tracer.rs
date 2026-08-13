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

use ahash::{AHashMap, AHashSet};
use dimos_module::native_config;
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;
use validator::ValidationError;

mod normals;
#[cfg(test)]
mod tests;

#[cfg(test)]
use normals::{fit_normal, pooled_normal};
use normals::{refresh_voxels, should_spare, NORMAL_MIN_POINTS};

pub type VoxelKey = (i32, i32, i32);
pub type VoxelHealth = i32;
type ChunkKey = (i32, i32, i32);

/// Voxels per chunk edge for the healthy-voxel spatial index `emit_points` scans.
const CHUNK_SIZE: i32 = 16;

#[inline]
fn chunk_of(key: VoxelKey) -> ChunkKey {
    (
        key.0.div_euclid(CHUNK_SIZE),
        key.1.div_euclid(CHUNK_SIZE),
        key.2.div_euclid(CHUNK_SIZE),
    )
}

#[inline]
fn voxel_center(key: VoxelKey, voxel_size: f32) -> (f32, f32, f32) {
    let half = voxel_size * 0.5;
    (
        key.0 as f32 * voxel_size + half,
        key.1 as f32 * voxel_size + half,
        key.2 as f32 * voxel_size + half,
    )
}

#[native_config]
#[validate(schema(function = "validate_config"))]
#[derive(Clone)]
pub struct Config {
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    /// Fine cells per voxel edge for the fine emission layer: fine cell size =
    /// `voxel_size / fine_divisor`. 2 to 4 (divisor^3 occupancy bits per voxel
    /// must fit in a u64). Zero disables the layer.
    #[validate(range(min = 0, max = 4))]
    pub fine_divisor: u32,
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
    /// Spare a miss when abs of ray dot normal is below this. Higher clears only
    /// on direct hits, lower clears on slight grazes too.
    #[validate(range(min = 0.0, max = 1.0))]
    pub graze_cos: f32,
    /// Occupied neighbors a surface voxel needs to appear in the local map. Zero
    /// emits all. Higher drops isolated returns. The global map is unfiltered.
    #[validate(range(min = 0))]
    pub support_min: i32,
    /// Publish the accumulated local map and region bounds every Nth frame. Zero disables them.
    #[validate(range(min = 0))]
    pub emit_every: u32,
    /// Publish the global map every Nth frame. Zero disables it.
    #[validate(range(min = 0))]
    pub global_emit_every: u32,
    /// Publish the fine local map every Nth frame. Zero disables it.
    #[validate(range(min = 0))]
    pub fine_emit_every: u32,
    /// Size the local region to this percentile of batch point distances, so a
    /// stray far hit cannot inflate it.
    #[validate(range(min = 0.0, max = 100.0))]
    pub region_percentile: f32,
}

fn validate_config(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    if cfg.fine_divisor == 1 {
        return Err(ValidationError::new("fine_divisor_min_2"));
    }
    if cfg.fine_emit_every > 0 && cfg.fine_divisor == 0 {
        return Err(ValidationError::new("fine_emit_requires_fine_divisor"));
    }
    Ok(())
}

impl Config {
    /// The enabled fine layer as (divisor, fine cell size), or None when off.
    pub fn fine_layer(&self) -> Option<(u32, f32)> {
        (self.fine_divisor >= 2).then(|| {
            (
                self.fine_divisor,
                self.voxel_size / self.fine_divisor as f32,
            )
        })
    }
}

/// Split a fine key into its voxel key and flat child index.
#[inline]
fn split_fine_key(fine_key: VoxelKey, divisor: i32) -> (VoxelKey, usize) {
    let coarse = (
        fine_key.0.div_euclid(divisor),
        fine_key.1.div_euclid(divisor),
        fine_key.2.div_euclid(divisor),
    );
    let lx = fine_key.0.rem_euclid(divisor);
    let ly = fine_key.1.rem_euclid(divisor);
    let lz = fine_key.2.rem_euclid(divisor);
    (coarse, ((lx * divisor + ly) * divisor + lz) as usize)
}

/// Rebuild a fine key from its voxel key and flat child index.
#[inline]
fn join_fine_key(coarse: VoxelKey, index: usize, divisor: i32) -> VoxelKey {
    let i = index as i32;
    (
        coarse.0 * divisor + i / (divisor * divisor),
        coarse.1 * divisor + (i / divisor) % divisor,
        coarse.2 * divisor + i % divisor,
    )
}

#[derive(Default)]
pub struct VoxelMap {
    pub voxels: AHashMap<VoxelKey, Voxel>,
    /// Healthy (health > 0) voxel keys grouped by chunk, kept in sync with `voxels`
    /// on every health transition. `emit_points` scans this instead of the whole map.
    healthy_chunks: AHashMap<ChunkKey, AHashSet<VoxelKey>>,
}

impl VoxelMap {
    pub fn healthy_count(&self) -> usize {
        self.voxels.values().filter(|c| c.health > 0).count()
    }

    /// Add a return to its voxel's accumulated moments. Returns the key when
    /// the return crossed the voxel's normal-refit milestone.
    fn accumulate(&mut self, point: (f32, f32, f32), voxel_size: f32) -> Option<VoxelKey> {
        let key = world_to_voxel(point.0, point.1, point.2, 1.0 / voxel_size);
        let center = Vector3::new(
            (key.0 as f32 + 0.5) * voxel_size,
            (key.1 as f32 + 0.5) * voxel_size,
            (key.2 as f32 + 0.5) * voxel_size,
        );
        self.voxels
            .entry(key)
            .or_default()
            .observe(Vector3::new(point.0, point.1, point.2) - center)
            .then_some(key)
    }

    /// Mark a return's fine cell occupied. The voxel key uses the same coarse
    /// quantization as hits and `accumulate` (a fine-grid float path can
    /// disagree at cell boundaries and would plant phantom entries that break
    /// `record_hit`'s new-voxel semantics). The fine offset is computed within
    /// that cell and clamped to it.
    fn observe_fine(&mut self, point: (f32, f32, f32), divisor: i32, voxel_size: f32) {
        let coarse = world_to_voxel(point.0, point.1, point.2, 1.0 / voxel_size);
        let inv_fine = divisor as f32 / voxel_size;
        let local = |p: f32, k: i32| {
            (((p - k as f32 * voxel_size) * inv_fine).floor() as i32).clamp(0, divisor - 1)
        };
        let lx = local(point.0, coarse.0);
        let ly = local(point.1, coarse.1);
        let lz = local(point.2, coarse.2);
        let index = (lx * divisor + ly) * divisor + lz;
        self.voxels.entry(coarse).or_default().fine |= 1 << index;
    }

    /// Move a voxel in or out of the healthy-chunk index on a health-sign crossing.
    fn update_health_index(&mut self, key: VoxelKey, was_healthy: bool, now_healthy: bool) {
        if now_healthy == was_healthy {
            return;
        }
        let chunk = chunk_of(key);
        if now_healthy {
            self.healthy_chunks.entry(chunk).or_default().insert(key);
        } else if let Some(set) = self.healthy_chunks.get_mut(&chunk) {
            set.remove(&key);
            if set.is_empty() {
                self.healthy_chunks.remove(&chunk);
            }
        }
    }

    /// Count of a key's 26 neighbors that currently exist and are healthy. O(26)
    /// map lookups — called once per voxel, when it's first created, to seed its
    /// incremental `support` field. (Also the differential-test reference for
    /// that field.)
    fn count_healthy_neighbors(&self, key: VoxelKey) -> u32 {
        let mut n = 0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if (dx, dy, dz) == (0, 0, 0) {
                        continue;
                    }
                    let nk = (key.0 + dx, key.1 + dy, key.2 + dz);
                    if self.voxels.get(&nk).is_some_and(|c| c.health > 0) {
                        n += 1;
                    }
                }
            }
        }
        n
    }

    /// Adjust every existing neighbor's `support` count by `delta` (+1 or -1),
    /// following `key`'s own health crossing the healthy/unhealthy boundary.
    /// Neighbors that don't exist yet are skipped — they pick up the right count
    /// from `count_healthy_neighbors` when they're created.
    fn propagate_neighbor_support(&mut self, key: VoxelKey, delta: i32) {
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if (dx, dy, dz) == (0, 0, 0) {
                        continue;
                    }
                    let nk = (key.0 + dx, key.1 + dy, key.2 + dz);
                    if let Some(c) = self.voxels.get_mut(&nk) {
                        let updated = c.support as i32 + delta;
                        debug_assert!(
                            (0..=26).contains(&updated),
                            "support count out of range: {updated}"
                        );
                        c.support = updated as u32;
                    }
                }
            }
        }
    }

    /// Register a ray hit: create the voxel at `min_health` if new, then bump its
    /// health. Keeps the healthy-chunk index and every neighbor's `support` count
    /// in sync.
    fn record_hit(&mut self, key: VoxelKey, min_health: VoxelHealth, max_health: VoxelHealth) {
        let (was_healthy, now_healthy) = if let Some(c) = self.voxels.get_mut(&key) {
            let was_healthy = c.health > 0;
            c.health = (c.health + 1).min(max_health);
            (was_healthy, c.health > 0)
        } else {
            let support = self.count_healthy_neighbors(key);
            let health = (min_health + 1).min(max_health);
            self.voxels.insert(
                key,
                Voxel {
                    health,
                    support,
                    ..Default::default()
                },
            );
            (false, health > 0)
        };
        self.update_health_index(key, was_healthy, now_healthy);
        if was_healthy != now_healthy {
            self.propagate_neighbor_support(key, if now_healthy { 1 } else { -1 });
        }
    }

    /// Apply a clearing miss: drop the voxel's health by one, removing it once it
    /// reaches `min_health`. Keeps the healthy-chunk index and every neighbor's
    /// `support` count in sync. Returns whether the voxel was removed.
    fn record_miss(&mut self, key: VoxelKey, min_health: VoxelHealth) -> bool {
        let Some(c) = self.voxels.get_mut(&key) else {
            return false;
        };
        let was_healthy = c.health > 0;
        c.health -= 1;
        let removed = c.health <= min_health;
        let now_healthy = !removed && c.health > 0;
        if removed {
            self.voxels.remove(&key);
        }
        self.update_health_index(key, was_healthy, now_healthy);
        if was_healthy != now_healthy {
            self.propagate_neighbor_support(key, if now_healthy { 1 } else { -1 });
        }
        removed
    }

    /// Set a voxel's health directly, creating it if absent and leaving any
    /// existing accumulated moments untouched. Bypasses hit/miss health
    /// accounting — for tests and map construction outside `update_map`'s normal
    /// flow. Keeps the healthy-chunk index and every neighbor's `support` count
    /// in sync.
    pub fn set_health(&mut self, key: VoxelKey, health: VoxelHealth) {
        let was_healthy = if let Some(c) = self.voxels.get_mut(&key) {
            let was_healthy = c.health > 0;
            c.health = health;
            was_healthy
        } else {
            let support = self.count_healthy_neighbors(key);
            self.voxels.insert(
                key,
                Voxel {
                    health,
                    support,
                    ..Default::default()
                },
            );
            false
        };
        let now_healthy = health > 0;
        self.update_health_index(key, was_healthy, now_healthy);
        if was_healthy != now_healthy {
            self.propagate_neighbor_support(key, if now_healthy { 1 } else { -1 });
        }
    }

    /// Reset to empty, including the healthy-chunk index.
    pub fn clear(&mut self) {
        self.voxels.clear();
        self.healthy_chunks.clear();
    }

    #[cfg(test)]
    fn health(&self, key: VoxelKey) -> Option<VoxelHealth> {
        self.voxels.get(&key).map(|c| c.health)
    }

    /// Fit every occupied voxel's normal from its pooled neighborhood.
    #[cfg(test)]
    fn recompute_all_normals(&mut self, voxel_size: f32) {
        let updates: Vec<(VoxelKey, Option<Vector3<f32>>)> = self
            .voxels
            .keys()
            .copied()
            .map(|k| (k, pooled_normal(&self.voxels, k, voxel_size)))
            .collect();
        for (k, n) in updates {
            self.voxels.get_mut(&k).unwrap().normal = n;
        }
    }
}

/// Occupancy health, accumulated point moments about the voxel center, and the
/// normal fit from the voxel's neighborhood.
#[derive(Clone)]
pub struct Voxel {
    pub health: VoxelHealth,
    /// Count of this voxel's 26 neighbors that currently exist and are healthy,
    /// maintained incrementally by `VoxelMap` instead of rescanned per query.
    support: u32,
    /// Fine voxel occupancy within a macro voxel is stored with this bitmask.
    fine: u64,
    num_pts: u32,
    /// Point count at which the next normal refit fires, advancing ~1.5x per
    /// milestone. A pooled normal only moves when a contributor's evidence
    /// grows materially, so converged voxels stop paying for refits.
    next_fit_pts: u32,
    sum: Vector3<f32>,
    m2: Matrix3<f32>,
    normal: Option<Vector3<f32>>,
}

impl Default for Voxel {
    fn default() -> Self {
        Self {
            health: 0,
            support: 0,
            fine: 0,
            num_pts: 0,
            next_fit_pts: NORMAL_MIN_POINTS,
            sum: Vector3::zeros(),
            m2: Matrix3::zeros(),
            normal: None,
        }
    }
}

impl Voxel {
    pub fn with_health(health: VoxelHealth) -> Self {
        Self {
            health,
            ..Default::default()
        }
    }

    /// Fold a centered point into the running moments. True when the count
    /// crosses the refit milestone, which then advances geometrically.
    fn observe(&mut self, q: Vector3<f32>) -> bool {
        self.num_pts += 1;
        self.sum += q;
        self.m2 += q * q.transpose();
        if self.num_pts >= self.next_fit_pts {
            self.next_fit_pts = self.num_pts + (self.num_pts / 2).max(1);
            true
        } else {
            false
        }
    }

    #[cfg(test)]
    fn planar_normal(&self) -> Option<Vector3<f32>> {
        self.normal
    }

    /// Fit a normal from this voxel's own points alone, ignoring neighbors.
    #[cfg(test)]
    fn self_normal(&self) -> Option<Vector3<f32>> {
        if self.num_pts < NORMAL_MIN_POINTS {
            return None;
        }
        let n = self.num_pts as f32;
        let mean = self.sum / n;
        fit_normal(self.m2 / n - mean * mean.transpose())
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

/// A cylinder (cx, cy, radius, z_min, z_max) on the mean origin, sized to a
/// percentile of the point distances so a stray far hit cannot inflate it.
/// Points must be finite. An empty batch yields a zero-radius region.
pub fn batch_local_bounds(
    points: &[(f32, f32, f32)],
    origins: &[(f32, f32, f32)],
    percentile_pct: f32,
    margin: f32,
) -> (f32, f32, f32, f32, f32) {
    let n = origins.len().max(1) as f64;
    let cx = (origins.iter().map(|o| o.0 as f64).sum::<f64>() / n) as f32;
    let cy = (origins.iter().map(|o| o.1 as f64).sum::<f64>() / n) as f32;
    if points.is_empty() {
        let cz = (origins.iter().map(|o| o.2 as f64).sum::<f64>() / n) as f32;
        return (cx, cy, 0.0, cz, cz);
    }

    let mut dist: Vec<f32> = points.iter().map(|p| (p.0 - cx).hypot(p.1 - cy)).collect();
    let mut zs: Vec<f32> = points.iter().map(|p| p.2).collect();
    let radius = percentile(&mut dist, percentile_pct) + margin;
    let z_min = percentile(&mut zs, 100.0 - percentile_pct) - margin;
    let z_max = percentile(&mut zs, percentile_pct) + margin;
    (cx, cy, radius, z_min, z_max)
}

fn percentile(values: &mut [f32], p: f32) -> f32 {
    let n = values.len();
    if n == 1 {
        return values[0];
    }
    let rank = (p as f64 / 100.0).clamp(0.0, 1.0) * (n - 1) as f64;
    let lo = rank.floor() as usize;
    let frac = (rank - lo as f64) as f32;
    let (_, &mut v_lo, rest) = values.select_nth_unstable_by(lo, |a, b| a.total_cmp(b));
    if frac == 0.0 || rest.is_empty() {
        return v_lo;
    }
    let v_hi = rest.iter().copied().fold(f32::INFINITY, f32::min);
    v_lo + frac * (v_hi - v_lo)
}

/// Healthy voxel centers paired with their surface normal, the zero vector where
/// there is no plane.
pub fn iter_global_normals(
    map: &VoxelMap,
    voxel_size: f32,
) -> impl Iterator<Item = ((f32, f32, f32), [f32; 3])> + '_ {
    let half = voxel_size * 0.5;
    map.voxels
        .iter()
        .filter(|(_, c)| c.health > 0)
        .map(move |(&(kx, ky, kz), c)| {
            let pos = (
                kx as f32 * voxel_size + half,
                ky as f32 * voxel_size + half,
                kz as f32 * voxel_size + half,
            );
            let normal = c.normal.map_or([0.0; 3], |n| [n[0], n[1], n[2]]);
            (pos, normal)
        })
}

/// Chunk range (inclusive) covering the axis-aligned box a cylinder bounds fits in.
fn chunk_range_for_bounds(bounds: &LocalBounds, voxel_size: f32) -> (ChunkKey, ChunkKey) {
    let inv = 1.0 / voxel_size;
    let radius = bounds.r_xy_max_sq.sqrt();
    let lo = world_to_voxel(
        bounds.origin_x - radius,
        bounds.origin_y - radius,
        bounds.z_min,
        inv,
    );
    let hi = world_to_voxel(
        bounds.origin_x + radius,
        bounds.origin_y + radius,
        bounds.z_max,
        inv,
    );
    (chunk_of(lo), chunk_of(hi))
}

/// Whether the axis-aligned box is entirely inside the cylinder bounds.
fn box_inside(b: &LocalBounds, min: (f32, f32, f32), max: (f32, f32, f32)) -> bool {
    if min.2 < b.z_min || max.2 > b.z_max {
        return false;
    }
    let dx = (b.origin_x - min.0).abs().max((b.origin_x - max.0).abs());
    let dy = (b.origin_y - min.1).abs().max((b.origin_y - max.1).abs());
    dx * dx + dy * dy <= b.r_xy_max_sq
}

/// The world-space box of the grid cell at `key`, for cells of `edge` meters.
fn cell_box(key: (i32, i32, i32), edge: f32) -> ((f32, f32, f32), (f32, f32, f32)) {
    let min = (
        key.0 as f32 * edge,
        key.1 as f32 * edge,
        key.2 as f32 * edge,
    );
    (min, (min.0 + edge, min.1 + edge, min.2 + edge))
}

/// Healthy-voxel key sets of every chunk overlapping `bounds`.
fn chunks_in_bounds<'a>(
    map: &'a VoxelMap,
    bounds: &LocalBounds,
    voxel_size: f32,
) -> Vec<(ChunkKey, &'a AHashSet<VoxelKey>)> {
    let (lo, hi) = chunk_range_for_bounds(bounds, voxel_size);
    let mut out = Vec::new();
    for cx in lo.0..=hi.0 {
        for cy in lo.1..=hi.1 {
            for cz in lo.2..=hi.2 {
                if let Some(keys) = map.healthy_chunks.get(&(cx, cy, cz)) {
                    out.push(((cx, cy, cz), keys));
                }
            }
        }
    }
    out
}

fn flatten_with_capacity(parts: Vec<Vec<(f32, f32, f32)>>, extra: usize) -> Vec<(f32, f32, f32)> {
    let total: usize = parts.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(total + extra);
    for part in parts {
        out.extend(part);
    }
    out
}

fn is_supported(map: &VoxelMap, key: VoxelKey, support_min: i32) -> bool {
    support_min <= 0
        || map
            .voxels
            .get(&key)
            .is_some_and(|c| c.support >= support_min as u32)
}

/// Points for an emitted cloud: healthy surface voxels within `bounds` (all
/// when `None`) with at least `support_min` occupied neighbors, plus this
/// frame's not-yet-healthy `live` voxels within `bounds`. Scans chunks in
/// parallel; chunks entirely inside the cylinder skip per-voxel bounds checks.
pub fn emit_points(
    map: &VoxelMap,
    voxel_size: f32,
    bounds: Option<&LocalBounds>,
    support_min: i32,
    live: &AHashSet<VoxelKey>,
) -> Vec<(f32, f32, f32)> {
    let chunk_edge = CHUNK_SIZE as f32 * voxel_size;
    let parts: Vec<Vec<(f32, f32, f32)>> = match bounds {
        Some(b) => chunks_in_bounds(map, b, voxel_size)
            .par_iter()
            .map(|&(chunk, keys)| {
                let (min, max) = cell_box(chunk, chunk_edge);
                let chunk_inside = box_inside(b, min, max);
                let mut part = Vec::with_capacity(keys.len());
                for &key in keys {
                    if !is_supported(map, key, support_min) {
                        continue;
                    }
                    let (x, y, z) = voxel_center(key, voxel_size);
                    if chunk_inside || b.contains(x, y, z) {
                        part.push((x, y, z));
                    }
                }
                part
            })
            .collect(),
        None => map
            .healthy_chunks
            .values()
            .collect::<Vec<_>>()
            .par_iter()
            .map(|keys| {
                let mut part = Vec::with_capacity(keys.len());
                for &key in keys.iter() {
                    if is_supported(map, key, support_min) {
                        part.push(voxel_center(key, voxel_size));
                    }
                }
                part
            })
            .collect(),
    };
    let mut out = flatten_with_capacity(parts, live.len());

    for &key in live.iter() {
        if matches!(map.voxels.get(&key), Some(c) if c.health > 0) {
            continue;
        }
        let (x, y, z) = voxel_center(key, voxel_size);
        if !bounds.is_none_or(|b| b.contains(x, y, z)) {
            continue;
        }
        out.push((x, y, z));
    }
    out
}

/// Points for a fine emitted cloud: observed fine cells inside healthy voxels
/// clearing `support_min`, within `bounds` (all when `None`), plus this frame's
/// `live_fine` cells whose voxel is not yet healthy, within `bounds`. Mirrors
/// `emit_points`: once a voxel is healthy, the support gate alone decides its
/// visibility and live cells never override it. Scans chunks in parallel;
/// chunks and voxels entirely inside the cylinder skip per-cell bounds checks.
pub fn emit_points_fine(
    map: &VoxelMap,
    voxel_size: f32,
    fine_divisor: u32,
    bounds: Option<&LocalBounds>,
    support_min: i32,
    live_fine: &AHashSet<VoxelKey>,
) -> Vec<(f32, f32, f32)> {
    let divisor = fine_divisor as i32;
    let fine_size = voxel_size / fine_divisor as f32;
    let chunk_edge = CHUNK_SIZE as f32 * voxel_size;

    let emit_children =
        |key: VoxelKey, inside: bool, b: Option<&LocalBounds>, part: &mut Vec<(f32, f32, f32)>| {
            let Some(v) = map.voxels.get(&key) else {
                return;
            };
            if support_min > 0 && v.support < support_min as u32 {
                return;
            }
            let mut bits = v.fine;
            while bits != 0 {
                let i = bits.trailing_zeros() as usize;
                bits &= bits - 1;
                let (x, y, z) = voxel_center(join_fine_key(key, i, divisor), fine_size);
                if inside || b.is_none_or(|b| b.contains(x, y, z)) {
                    part.push((x, y, z));
                }
            }
        };

    let parts: Vec<Vec<(f32, f32, f32)>> = match bounds {
        Some(b) => chunks_in_bounds(map, b, voxel_size)
            .par_iter()
            .map(|&(chunk, keys)| {
                let (min, max) = cell_box(chunk, chunk_edge);
                let chunk_inside = box_inside(b, min, max);
                let mut part = Vec::with_capacity(keys.len() * 4);
                for &key in keys {
                    // Boundary chunks test each voxel's box; boundary voxels
                    // fall back to per-cell checks.
                    let inside = chunk_inside || {
                        let (min, max) = cell_box(key, voxel_size);
                        box_inside(b, min, max)
                    };
                    emit_children(key, inside, Some(b), &mut part);
                }
                part
            })
            .collect(),
        None => map
            .healthy_chunks
            .values()
            .collect::<Vec<_>>()
            .par_iter()
            .map(|keys| {
                let mut part = Vec::with_capacity(keys.len() * 4);
                for &key in keys.iter() {
                    emit_children(key, true, None, &mut part);
                }
                part
            })
            .collect(),
    };
    let mut out = flatten_with_capacity(parts, live_fine.len());

    for &fine_key in live_fine.iter() {
        let (coarse, _) = split_fine_key(fine_key, divisor);
        if matches!(map.voxels.get(&coarse), Some(v) if v.health > 0) {
            continue;
        }
        let (x, y, z) = voxel_center(fine_key, fine_size);
        if !bounds.is_none_or(|b| b.contains(x, y, z)) {
            continue;
        }
        out.push((x, y, z));
    }
    out
}

fn live_voxels(points: &[(f32, f32, f32)], voxel_size: f32) -> AHashSet<VoxelKey> {
    let inv = 1.0_f32 / voxel_size;
    let mut out: AHashSet<VoxelKey> = AHashSet::with_capacity(points.len());
    for &(x, y, z) in points {
        out.insert(world_to_voxel(x, y, z, inv));
    }
    out
}

/// Cumulative wall time (s) of `update_map`'s internal phases, for benchmarks.
#[derive(Default, Clone, Copy)]
pub struct UpdatePhases {
    pub filter: f64,
    pub raycast: f64,
    pub hits: f64,
    pub accumulate: f64,
    pub misses: f64,
    pub normals: f64,
    pub fine_live: f64,
}

/// One frame's hit sets from `update_map`.
#[derive(Default)]
pub struct FrameHits {
    /// Hit voxels at map resolution, the live merge for `emit_points`.
    pub coarse: AHashSet<VoxelKey>,
    /// Hit fine cells, the live merge for `emit_points_fine`. Empty when the
    /// fine layer is off.
    pub fine: AHashSet<VoxelKey>,
}

pub fn update_map(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> FrameHits {
    update_map_timed(map, origin, points, cfg, &mut UpdatePhases::default())
}

/// `update_map` with per-phase wall time accumulated into `phases`.
pub fn update_map_timed(
    map: &mut VoxelMap,
    origin: (f32, f32, f32),
    points: &[(f32, f32, f32)],
    cfg: &Config,
    phases: &mut UpdatePhases,
) -> FrameHits {
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    let t = std::time::Instant::now();
    // Drop invalid returns and out-of-range points before they enter the map.
    let mut filtered: Vec<(f32, f32, f32)> = Vec::with_capacity(points.len());
    filtered.extend(points.iter().copied().filter(|&(x, y, z)| {
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            return false;
        }
        let dx = x - origin.0;
        let dy = y - origin.1;
        let dz = z - origin.2;
        let d2 = dx * dx + dy * dy + dz * dz;
        d2 > 0.0 && d2 <= max_range_sq
    }));
    let points = &filtered[..];

    let hits = live_voxels(points, cfg.voxel_size);
    phases.filter += t.elapsed().as_secs_f64();

    let origin_voxel = world_to_voxel(origin.0, origin.1, origin.2, inv);
    let step = cfg.ray_subsample as usize;
    let voxels = &map.voxels;
    let t = std::time::Instant::now();
    let misses: AHashSet<VoxelKey> = points
        .par_iter()
        .enumerate()
        .fold(AHashSet::new, |mut misses, (i, &p)| {
            if i % step != 0 {
                return misses;
            }
            let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
            find_misses_along_ray(
                &mut misses,
                voxels,
                origin,
                p,
                cfg.voxel_size,
                cfg.shadow_depth,
                cfg.grace_depth,
                cfg.graze_cos,
                origin_voxel,
                endpoint,
            );
            misses
        })
        .reduce(AHashSet::new, |mut a, mut b| {
            if a.len() < b.len() {
                std::mem::swap(&mut a, &mut b);
            }
            a.extend(b);
            a
        });
    phases.raycast += t.elapsed().as_secs_f64();

    let t = std::time::Instant::now();
    for &v in &hits {
        map.record_hit(v, cfg.min_health, cfg.max_health);
    }
    phases.hits += t.elapsed().as_secs_f64();

    let t = std::time::Instant::now();
    let fine = cfg.fine_layer().map(|(d, size)| (d as i32, size));
    let mut changed: AHashSet<VoxelKey> = AHashSet::new();
    for &p in points {
        if let Some(key) = map.accumulate(p, cfg.voxel_size) {
            changed.insert(key);
        }
        if let Some((divisor, _)) = fine {
            map.observe_fine(p, divisor, cfg.voxel_size);
        }
    }
    phases.accumulate += t.elapsed().as_secs_f64();

    let t = std::time::Instant::now();
    let mut removed: Vec<VoxelKey> = Vec::new();
    for &v in misses.difference(&hits) {
        if map.record_miss(v, cfg.min_health) {
            removed.push(v);
        }
    }
    phases.misses += t.elapsed().as_secs_f64();

    let t = std::time::Instant::now();
    refresh_voxels(map, &changed, &removed, cfg.voxel_size);
    phases.normals += t.elapsed().as_secs_f64();

    let t = std::time::Instant::now();
    let fine_live = fine.map_or_else(AHashSet::new, |(_, fine_size)| {
        live_voxels(points, fine_size)
    });
    phases.fine_live += t.elapsed().as_secs_f64();

    FrameHits {
        coarse: hits,
        fine: fine_live,
    }
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Amanatides and Woo 3d DDA. Records in-map voxels along the ray between the
/// origin and the end of the shadow region. Voxels within the grace region of
/// the endpoint are spared from being marked as misses.
#[allow(clippy::too_many_arguments)]
fn find_misses_along_ray(
    misses: &mut AHashSet<VoxelKey>,
    map_voxels: &AHashMap<VoxelKey, Voxel>,
    origin: (f32, f32, f32),
    end: (f32, f32, f32),
    voxel_size: f32,
    shadow_depth: f32,
    grace_depth: f32,
    graze_cos: f32,
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
    let ray_unit = Vector3::new(dx, dy, dz) / ray_len.max(f32::EPSILON);

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

        let cx = x as f32 * voxel_size + half;
        let cy = y as f32 * voxel_size + half;
        let cz = z as f32 * voxel_size + half;
        let ddx = cx - endpoint_center.0;
        let ddy = cy - endpoint_center.1;
        let ddz = cz - endpoint_center.2;
        let dist_sq = ddx * ddx + ddy * ddy + ddz * ddz;

        if past_endpoint {
            // Past the endpoint, keep going until we leave the shadow region.
            if dist_sq > shadow_sq {
                return;
            }
        } else if dist_sq < grace_sq {
            // Too close to the endpoint to safely mark a miss, we might be clipping another voxel's ray.
            continue;
        }

        if let Some(c) = map_voxels.get(&(x, y, z)) {
            if !should_spare(c, ray_unit, graze_cos) {
                misses.insert((x, y, z));
            }
        }
    }
}
