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

mod chunk_map;
mod normals;
#[cfg(test)]
mod tests;

use chunk_map::{chunk_of, ChunkCursor, ChunkRef, HealthyVoxel};
pub use chunk_map::{ChunkKey, ChunkMap, CHUNK_EDGE};

#[cfg(test)]
use normals::fit_normal;
use normals::{
    mark_stale, pooled_normal, resolve_deferred, should_spare, NormalFit, NORMAL_MIN_POINTS,
};

pub type VoxelKey = (i32, i32, i32);
pub type VoxelHealth = i32;

/// A cell of the region grid seeds load by and the map viz publishes by.
pub type Cell = (i32, i32);

/// The region grid cell a chunk's center falls in. Cells never run smaller
/// than a chunk, so a chunk lies in exactly one.
pub fn region_of(chunk: ChunkKey, voxel_size: f32, region_m: f32) -> Cell {
    let edge = CHUNK_EDGE as f32 * voxel_size;
    let cell_m = region_m.max(edge);
    (
        (((chunk.0 as f32) + 0.5) * edge / cell_m).floor() as i32,
        (((chunk.1 as f32) + 0.5) * edge / cell_m).floor() as i32,
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
    /// Factor of the fine grain voxels against the coarse grain voxels.
    #[validate(range(min = 0, max = 4))]
    pub fine_divisor: u32,
    /// Publish the local fine map alongside the local map. Needs a fine divisor.
    pub emit_fine: bool,
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
    /// Size the local region to this percentile of batch point distances, so a
    /// stray far hit cannot inflate it.
    #[validate(range(min = 0.0, max = 100.0))]
    pub region_percentile: f32,
    /// Fixed frame clouds are registered and published in.
    #[validate(length(min = 1))]
    pub world_frame: String,
    /// Max stamp gap between a cloud and the transform used to register it (s).
    #[validate(range(exclusive_min = 0.0))]
    pub tf_match_tolerance_s: f64,
    /// Clouds this much (s) older than the newest transform for their frame are skipped, so a map that fell behind catches up
    /// instead of working through a stale queue; 0 keeps every cloud.
    #[validate(range(min = 0.0))]
    pub max_cloud_age_s: f64,
    /// How long to wait for a late transform before dropping a cloud (s).
    #[validate(range(min = 0.0))]
    pub tf_wait_timeout_s: f64,
    /// Worker threads for parallel map work.
    #[validate(range(min = 1))]
    pub worker_threads: u32,
    /// Edge of the square regions a seed load is handed on in and the map
    /// viz publishes by.
    #[validate(range(exclusive_min = 0.0))]
    pub region_m: f32,
    /// Publish the regions whose chunks changed every Nth frame. Zero disables it.
    pub viz_emit_every: u32,
    /// Unchanged regions republished per viz tick, round robin. Zero turns the sweep off.
    pub viz_sweep_regions: u32,
}

fn validate_config(cfg: &Config) -> Result<(), ValidationError> {
    if cfg.min_health >= cfg.max_health {
        return Err(ValidationError::new("min_health_lt_max_health"));
    }
    if cfg.fine_divisor == 1 {
        return Err(ValidationError::new("fine_divisor_min_2"));
    }
    if cfg.emit_fine && cfg.fine_divisor == 0 {
        return Err(ValidationError::new("emit_fine_requires_fine_divisor"));
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

/// The voxel a fine key belongs to.
#[inline]
pub fn coarse_of_fine(fine_key: VoxelKey, divisor: i32) -> VoxelKey {
    split_fine_key(fine_key, divisor).0
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
    pub voxels: ChunkMap,
    /// Chunks whose emitted points changed since the viz last took them.
    changed_chunks: AHashSet<ChunkKey>,
    /// Occupied neighbors a healthy voxel needs to be emitted, zero for none.
    support_min: i32,
}

impl VoxelMap {
    pub fn with_support_min(support_min: i32) -> Self {
        Self {
            support_min,
            ..Default::default()
        }
    }

    pub fn healthy_count(&self) -> usize {
        self.voxels.healthy_len()
    }

    /// Add a return to its existing voxel's accumulated moments, marking its
    /// fine cell when a divisor is given. The fine child index derives from the
    /// coarse key so cell-boundary float error cannot plant phantom voxels.
    fn accumulate(
        &mut self,
        point: (f32, f32, f32),
        voxel_size: f32,
        fine_divisor: Option<i32>,
    ) -> (VoxelKey, bool, Option<VoxelKey>) {
        let key = world_to_voxel(point.0, point.1, point.2, 1.0 / voxel_size);
        let center = Vector3::new(
            (key.0 as f32 + 0.5) * voxel_size,
            (key.1 as f32 + 0.5) * voxel_size,
            (key.2 as f32 + 0.5) * voxel_size,
        );
        let v = self
            .voxels
            .get_mut(&key)
            .expect("a return's voxel is recorded before its points are accumulated");
        let milestone = v.observe(Vector3::new(point.0, point.1, point.2) - center);
        let fine = fine_divisor.map(|divisor| {
            let inv_fine = divisor as f32 / voxel_size;
            let local = |p: f32, k: i32| {
                (((p - k as f32 * voxel_size) * inv_fine).floor() as i32).clamp(0, divisor - 1)
            };
            let lx = local(point.0, key.0);
            let ly = local(point.1, key.1);
            let lz = local(point.2, key.2);
            let index = (lx * divisor + ly) * divisor + lz;
            v.fine |= 1 << index;
            join_fine_key(key, index as usize, divisor)
        });
        (key, milestone, fine)
    }

    /// Settle `key`'s health crossing zero: its neighbors' support counts and
    /// the viz.
    fn health_crossed(&mut self, key: VoxelKey, now_healthy: bool) {
        self.changed_chunks.insert(chunk_of(key));
        self.propagate_neighbor_support(key, if now_healthy { 1 } else { -1 });
    }

    /// The chunks whose emitted points changed since the last take.
    pub fn take_changed_chunks(&mut self) -> AHashSet<ChunkKey> {
        std::mem::take(&mut self.changed_chunks)
    }

    /// Chunks holding at least one healthy voxel.
    pub fn healthy_chunk_keys(&self) -> impl Iterator<Item = ChunkKey> + '_ {
        self.voxels
            .chunks()
            .filter(|c| c.healthy_len() > 0)
            .map(|c| c.key)
    }

    #[cfg(test)]
    pub(crate) fn set_support_min(&mut self, support_min: i32) {
        self.support_min = support_min;
    }

    /// Count of a key's 26 neighbors that currently exist and are healthy.
    /// Called once per voxel, at creation, to seed its support count.
    fn count_healthy_neighbors(&self, key: VoxelKey) -> u8 {
        let near = self.voxels.neighborhood(key, 1);
        let mut n = 0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if (dx, dy, dz) == (0, 0, 0) {
                        continue;
                    }
                    if near.is_healthy((key.0 + dx, key.1 + dy, key.2 + dz)) {
                        n += 1;
                    }
                }
            }
        }
        n
    }

    /// Adjust every existing neighbor's `support` count by `delta` after
    /// `key`'s health crossed the healthy boundary. Absent neighbors pick up
    /// the right count from `count_healthy_neighbors` at creation.
    fn propagate_neighbor_support(&mut self, key: VoxelKey, delta: i32) {
        let support_min = self.support_min;
        let changed = &mut self.changed_chunks;
        self.voxels.for_each_near_mut(key, 1, |nk, v, support| {
            if nk == key {
                return;
            }
            let updated = *support as i32 + delta;
            debug_assert!(
                (0..=26).contains(&updated),
                "support count out of range: {updated}"
            );
            let was_emitted = v.health > 0 && voxel_supported(*support, support_min);
            *support = updated as u8;
            if was_emitted != (v.health > 0 && voxel_supported(*support, support_min)) {
                changed.insert(chunk_of(nk));
            }
        });
    }

    /// Make room for `additional` chunks ahead of a bulk load.
    pub fn reserve_chunks(&mut self, additional: usize) {
        self.voxels.reserve_chunks(additional);
    }

    /// Create a healthy seed voxel if absent. Returns whether it was created.
    fn insert_seed(&mut self, key: VoxelKey) -> bool {
        if self.voxels.contains_key(&key) {
            return false;
        }
        let support = self.count_healthy_neighbors(key);
        self.voxels
            .insert(key, Voxel::with_health(SEED_HEALTH), support);
        self.health_crossed(key, true);
        true
    }

    /// Register a ray hit: create the voxel at `min_health` if new, then bump its
    /// health. Keeps every neighbor's `support` count in sync. Returns whether the
    /// voxel was created.
    fn record_hit(
        &mut self,
        key: VoxelKey,
        min_health: VoxelHealth,
        max_health: VoxelHealth,
    ) -> bool {
        let bumped = self.voxels.update_health(key, |h| (h + 1).min(max_health));
        let (created, was_healthy, now_healthy) = match bumped {
            Some((was, now)) => (false, was > 0, now > 0),
            None => {
                let support = self.count_healthy_neighbors(key);
                let health = (min_health + 1).min(max_health);
                self.voxels.insert(key, Voxel::with_health(health), support);
                (true, false, health > 0)
            }
        };
        if was_healthy != now_healthy {
            self.health_crossed(key, now_healthy);
        }
        created
    }

    /// Apply a clearing miss: drop the voxel's health by one, removing it once it
    /// reaches `min_health`. Keeps every neighbor's `support` count in sync.
    /// Returns whether the voxel was removed.
    fn record_miss(&mut self, key: VoxelKey, min_health: VoxelHealth) -> bool {
        let Some((was, now)) = self.voxels.update_health(key, |h| h - 1) else {
            return false;
        };
        let removed = now <= min_health;
        let now_healthy = !removed && now > 0;
        if removed {
            self.voxels.remove(&key);
        }
        if (was > 0) != now_healthy {
            self.health_crossed(key, now_healthy);
        }
        removed
    }

    /// Delete voxels outright, whatever their health, keeping every neighbor's
    /// `support` count in sync. Unknown keys are skipped. Returns how many voxels
    /// were removed.
    ///
    /// This is the escape hatch for space a sensor cannot ray-trace clear: a
    /// wrist camera's own arm occludes the volume behind it, so no ray ever
    /// fires a miss there and the arm's own returns would sit in the map
    /// forever. A caller that knows those keys are free names them here.
    pub fn clear_voxels(&mut self, keys: impl IntoIterator<Item = VoxelKey>) -> usize {
        let mut removed = 0;
        for key in keys {
            let Some(voxel) = self.voxels.remove(&key) else {
                continue;
            };
            // The fine-cell bitmask rides inside the removed voxel, so the fine
            // layer needs no separate cleanup.
            if voxel.health > 0 {
                self.health_crossed(key, false);
            }
            removed += 1;
        }
        removed
    }

    /// Set a voxel's health directly, creating it if absent. Bypasses hit and
    /// miss accounting but keeps support counts in sync.
    #[cfg(test)]
    pub fn set_health(&mut self, key: VoxelKey, health: VoxelHealth) {
        let was_healthy = match self.voxels.update_health(key, |_| health) {
            Some((was, _)) => was > 0,
            None => {
                let support = self.count_healthy_neighbors(key);
                self.voxels.insert(key, Voxel::with_health(health), support);
                false
            }
        };
        if was_healthy != (health > 0) {
            self.health_crossed(key, health > 0);
        }
    }

    pub fn clear(&mut self) {
        self.voxels.clear();
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
            .map(|k| {
                (
                    k,
                    pooled_normal(&self.voxels, k, voxel_size).map(|(n, _)| n),
                )
            })
            .collect();
        for (k, n) in updates {
            self.voxels.get_mut(&k).unwrap().normal = NormalFit::Fitted(n);
        }
    }
}

/// Occupancy health, accumulated point moments about the voxel center, and the
/// normal fit from the voxel's neighborhood. Health is written by the
/// `ChunkMap` holding the voxel, which mirrors it in a healthy mask.
#[derive(Clone)]
pub struct Voxel {
    health: VoxelHealth,
    /// Occupancy bitmask of this voxel's fine cells.
    fine: u64,
    num_pts: u32,
    /// Point count at which the next normal refit fires, advancing ~1.5x per
    /// milestone so converged voxels stop paying for refits.
    next_fit_pts: u32,
    sum: Vector3<f32>,
    m2: Matrix3<f32>,
    normal: NormalFit,
}

impl Default for Voxel {
    fn default() -> Self {
        Self {
            health: 0,
            fine: 0,
            num_pts: 0,
            next_fit_pts: NORMAL_MIN_POINTS,
            sum: Vector3::zeros(),
            m2: Matrix3::zeros(),
            normal: NormalFit::Stale,
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
        match self.normal {
            NormalFit::Fitted(n) => n,
            NormalFit::Stale => panic!("normal read while stale"),
        }
    }

    /// Fit a normal from this voxel's own points alone, ignoring neighbors.
    #[cfg(test)]
    fn self_normal(&self) -> Option<(Vector3<f32>, f32)> {
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

/// A local region cylinder: center, radius, and z band.
#[derive(Clone, Copy)]
pub struct Cylinder {
    pub cx: f32,
    pub cy: f32,
    pub radius: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl Cylinder {
    pub fn bounds(&self) -> LocalBounds {
        LocalBounds {
            origin_x: self.cx,
            origin_y: self.cy,
            r_xy_max_sq: self.radius * self.radius,
            z_min: self.z_min,
            z_max: self.z_max,
        }
    }
}

/// The cylinder on the mean origin sized to a percentile of the point
/// distances, so a stray far hit cannot inflate it. Points must be finite.
/// An empty batch yields a zero-radius region.
pub fn batch_local_bounds(
    points: &[(f32, f32, f32)],
    origins: &[(f32, f32, f32)],
    percentile_pct: f32,
    margin: f32,
) -> Cylinder {
    let n = origins.len().max(1) as f64;
    let cx = (origins.iter().map(|o| o.0 as f64).sum::<f64>() / n) as f32;
    let cy = (origins.iter().map(|o| o.1 as f64).sum::<f64>() / n) as f32;
    if points.is_empty() {
        let cz = (origins.iter().map(|o| o.2 as f64).sum::<f64>() / n) as f32;
        return Cylinder {
            cx,
            cy,
            radius: 0.0,
            z_min: cz,
            z_max: cz,
        };
    }

    let mut dist: Vec<f32> = points.iter().map(|p| (p.0 - cx).hypot(p.1 - cy)).collect();
    let mut zs: Vec<f32> = points.iter().map(|p| p.2).collect();
    Cylinder {
        cx,
        cy,
        radius: percentile(&mut dist, percentile_pct) + margin,
        z_min: percentile(&mut zs, 100.0 - percentile_pct) - margin,
        z_max: percentile(&mut zs, percentile_pct) + margin,
    }
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

/// Apply `f` to every healthy voxel in parallel, keeping map order.
fn par_map_healthy<T, F>(map: &VoxelMap, f: F) -> Vec<T>
where
    T: Send,
    F: Fn(VoxelKey, &Voxel) -> T + Sync,
{
    let healthy: Vec<HealthyVoxel> = map.voxels.chunks().flat_map(|c| c.healthy()).collect();
    healthy.par_iter().map(|hv| f(hv.key, hv.voxel)).collect()
}

fn flat3(v: Option<Vector3<f32>>) -> [f32; 3] {
    v.map_or([0.0; 3], |n| [n[0], n[1], n[2]])
}

/// Healthy voxel centers and their surface normals, flat, the zero vector
/// where there is no plane. Stale normals are fit for the output only.
/// Whole-map cost. A visualization helper, not for control paths.
pub fn global_normals(map: &VoxelMap, voxel_size: f32) -> (Vec<f32>, Vec<f32>) {
    let fits = par_map_healthy(map, |key, v| {
        let normal = match v.normal {
            NormalFit::Fitted(n) => n,
            NormalFit::Stale => pooled_normal(&map.voxels, key, voxel_size).map(|(n, _)| n),
        };
        (voxel_center(key, voxel_size), flat3(normal))
    });
    let mut positions: Vec<f32> = Vec::with_capacity(fits.len() * 3);
    let mut normals: Vec<f32> = Vec::with_capacity(fits.len() * 3);
    for ((x, y, z), n) in fits {
        positions.extend_from_slice(&[x, y, z]);
        normals.extend_from_slice(&n);
    }
    (positions, normals)
}

/// `global_normals` with every fit recomputed, plus each fit's smallest
/// eigenvalue, zero where there is no plane.
pub fn global_normal_fits(map: &VoxelMap, voxel_size: f32) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
    let fits = par_map_healthy(map, |key, _| {
        let (normal, min_eig) =
            pooled_normal(&map.voxels, key, voxel_size).map_or((None, 0.0), |(n, e)| (Some(n), e));
        (voxel_center(key, voxel_size), flat3(normal), min_eig)
    });
    let mut positions: Vec<f32> = Vec::with_capacity(fits.len() * 3);
    let mut normals: Vec<f32> = Vec::with_capacity(fits.len() * 3);
    let mut eigs: Vec<f32> = Vec::with_capacity(fits.len());
    for ((x, y, z), n, e) in fits {
        positions.extend_from_slice(&[x, y, z]);
        normals.extend_from_slice(&n);
        eigs.push(e);
    }
    (positions, normals, eigs)
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

/// Whether the axis-aligned box is entirely outside the cylinder bounds,
/// by its nearest corner.
fn box_outside(b: &LocalBounds, min: (f32, f32, f32), max: (f32, f32, f32)) -> bool {
    if max.2 < b.z_min || min.2 > b.z_max {
        return true;
    }
    let dx = (min.0 - b.origin_x).max(b.origin_x - max.0).max(0.0);
    let dy = (min.1 - b.origin_y).max(b.origin_y - max.1).max(0.0);
    dx * dx + dy * dy > b.r_xy_max_sq
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

/// Every chunk overlapping `bounds`.
fn chunks_in_bounds<'a>(
    map: &'a VoxelMap,
    bounds: &LocalBounds,
    voxel_size: f32,
) -> Vec<ChunkRef<'a>> {
    let (lo, hi) = chunk_range_for_bounds(bounds, voxel_size);
    // Work must scale with map contents, not the requested box. A huge query
    // box would walk its full chunk range even where the map is empty, so
    // filter the chunks instead when there are fewer of them than the range.
    let range_count = (hi.0 as i64 - lo.0 as i64 + 1) as u128
        * (hi.1 as i64 - lo.1 as i64 + 1) as u128
        * (hi.2 as i64 - lo.2 as i64 + 1) as u128;
    if range_count > map.voxels.chunk_count() as u128 {
        return map
            .voxels
            .chunks()
            .filter(|c| {
                (lo.0..=hi.0).contains(&c.key.0)
                    && (lo.1..=hi.1).contains(&c.key.1)
                    && (lo.2..=hi.2).contains(&c.key.2)
            })
            .collect();
    }
    let mut out = Vec::new();
    for cx in lo.0..=hi.0 {
        for cy in lo.1..=hi.1 {
            for cz in lo.2..=hi.2 {
                out.extend(map.voxels.chunk(&(cx, cy, cz)));
            }
        }
    }
    out
}

/// Scan `chunks` in parallel, one output buffer per rayon split, and flatten
/// the buffers in chunk order. `scan` appends a chunk's points to the buffer.
fn par_scan_chunks<F>(chunks: &[ChunkRef], extra_points: usize, scan: F) -> Vec<f32>
where
    F: Fn(ChunkRef, &mut Vec<f32>) + Sync,
{
    let parts: Vec<Vec<f32>> = chunks
        .par_iter()
        .fold(Vec::new, |mut part: Vec<f32>, &chunk| {
            scan(chunk, &mut part);
            part
        })
        .collect();
    let total: usize = parts.iter().map(Vec::len).sum();
    let mut out = Vec::with_capacity(total + 3 * extra_points);
    for part in parts {
        out.extend(part);
    }
    out
}

fn voxel_supported(support: u8, support_min: i32) -> bool {
    support_min <= 0 || support as i32 >= support_min
}

/// Scan the healthy voxels of every chunk overlapping `bounds` (all chunks
/// when `None`) in parallel. `emit` gets each voxel and whether its whole
/// chunk is inside the cylinder.
fn scan_chunks<F>(
    map: &VoxelMap,
    voxel_size: f32,
    bounds: Option<&LocalBounds>,
    points_per_voxel: usize,
    extra_points: usize,
    emit: F,
) -> Vec<f32>
where
    F: Fn(HealthyVoxel, bool, &mut Vec<f32>) + Sync,
{
    let chunk_edge = CHUNK_EDGE as f32 * voxel_size;
    let chunks: Vec<ChunkRef> = match bounds {
        Some(b) => chunks_in_bounds(map, b, voxel_size),
        None => map.voxels.chunks().collect(),
    };
    par_scan_chunks(&chunks, extra_points, |chunk, part| {
        let chunk_inside = match bounds {
            Some(b) => {
                let (min, max) = cell_box(chunk.key, chunk_edge);
                if box_outside(b, min, max) {
                    return;
                }
                box_inside(b, min, max)
            }
            None => true,
        };
        part.reserve(3 * points_per_voxel * chunk.healthy_len());
        for hv in chunk.healthy() {
            emit(hv, chunk_inside, part);
        }
    })
}

/// Points of the given chunks, flat (x, y, z) triples: their healthy voxels
/// that clear the map's support gate.
pub fn chunk_points(map: &VoxelMap, voxel_size: f32, chunks: &[ChunkKey]) -> Vec<f32> {
    let chunks: Vec<ChunkRef> = chunks
        .iter()
        .filter_map(|ck| map.voxels.chunk(ck))
        .collect();
    par_scan_chunks(&chunks, 0, |chunk, part| {
        part.reserve(3 * chunk.healthy_len());
        for hv in chunk.healthy() {
            if voxel_supported(hv.support, map.support_min) {
                let (x, y, z) = voxel_center(hv.key, voxel_size);
                part.extend_from_slice(&[x, y, z]);
            }
        }
    })
}

/// Points for an emitted cloud, flat (x, y, z) triples: healthy surface voxels
/// within `bounds` that clear the map's support gate, plus this frame's
/// not-yet-healthy `live` voxels within `bounds`. No bounds means the whole map.
pub fn emit_points(
    map: &VoxelMap,
    voxel_size: f32,
    bounds: Option<&LocalBounds>,
    live: &AHashSet<VoxelKey>,
) -> Vec<f32> {
    emit_points_impl(map, voxel_size, bounds, live, true)
}

/// Every healthy voxel plus this frame's live voxels, with no support gate.
pub fn emit_points_ungated(map: &VoxelMap, voxel_size: f32, live: &AHashSet<VoxelKey>) -> Vec<f32> {
    emit_points_impl(map, voxel_size, None, live, false)
}

fn emit_points_impl(
    map: &VoxelMap,
    voxel_size: f32,
    bounds: Option<&LocalBounds>,
    live: &AHashSet<VoxelKey>,
    gated: bool,
) -> Vec<f32> {
    let mut out = scan_chunks(
        map,
        voxel_size,
        bounds,
        1,
        live.len(),
        |hv, chunk_inside, part| {
            if gated && !voxel_supported(hv.support, map.support_min) {
                return;
            }
            let (x, y, z) = voxel_center(hv.key, voxel_size);
            if chunk_inside || bounds.is_none_or(|b| b.contains(x, y, z)) {
                part.extend_from_slice(&[x, y, z]);
            }
        },
    );

    for &key in live.iter() {
        if map.voxels.is_healthy(&key) {
            continue;
        }
        let (x, y, z) = voxel_center(key, voxel_size);
        if !bounds.is_none_or(|b| b.contains(x, y, z)) {
            continue;
        }
        out.extend_from_slice(&[x, y, z]);
    }
    out
}

/// Points for a fine emitted cloud, flat (x, y, z) triples: observed fine
/// cells inside healthy voxels clearing the map's support gate, within `bounds`,
/// plus this frame's `live_fine` cells whose voxel is not yet healthy. No bounds
/// means the whole map.
pub fn emit_points_fine(
    map: &VoxelMap,
    voxel_size: f32,
    fine_divisor: u32,
    bounds: Option<&LocalBounds>,
    live_fine: &AHashSet<VoxelKey>,
) -> Vec<f32> {
    let divisor = fine_divisor as i32;
    let fine_size = voxel_size / fine_divisor as f32;

    let mut out = scan_chunks(
        map,
        voxel_size,
        bounds,
        4,
        live_fine.len(),
        |hv, chunk_inside, part| {
            if !voxel_supported(hv.support, map.support_min) {
                return;
            }
            // Boundary chunks test each voxel's box. Boundary voxels fall back
            // to per-cell checks.
            let inside = chunk_inside || {
                let (min, max) = cell_box(hv.key, voxel_size);
                if bounds.is_some_and(|b| box_outside(b, min, max)) {
                    return;
                }
                bounds.is_some_and(|b| box_inside(b, min, max))
            };
            let mut bits = hv.voxel.fine;
            while bits != 0 {
                let i = bits.trailing_zeros() as usize;
                bits &= bits - 1;
                let (x, y, z) = voxel_center(join_fine_key(hv.key, i, divisor), fine_size);
                if inside || bounds.is_none_or(|b| b.contains(x, y, z)) {
                    part.extend_from_slice(&[x, y, z]);
                }
            }
        },
    );

    for &fine_key in live_fine.iter() {
        let (coarse, _) = split_fine_key(fine_key, divisor);
        if map.voxels.is_healthy(&coarse) {
            continue;
        }
        let (x, y, z) = voxel_center(fine_key, fine_size);
        if !bounds.is_none_or(|b| b.contains(x, y, z)) {
            continue;
        }
        out.extend_from_slice(&[x, y, z]);
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
    let inv = 1.0_f32 / cfg.voxel_size;
    let max_range_sq = if cfg.max_range > 0.0 {
        cfg.max_range * cfg.max_range
    } else {
        f32::INFINITY
    };

    // Drop invalid, unaddressable and out-of-range returns before they enter
    // the map.
    let mut filtered: Vec<(f32, f32, f32)> = Vec::with_capacity(points.len());
    filtered.extend(points.iter().copied().filter(|&(x, y, z)| {
        if !in_key_range(x, y, z, inv) {
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
    let fine = cfg.fine_layer().map(|(d, _)| d as i32);

    let frame = RayFrame {
        voxels: &map.voxels,
        hits: &hits,
        origin,
        origin_voxel: world_to_voxel(origin.0, origin.1, origin.2, inv),
        voxel_size: cfg.voxel_size,
        shadow_depth: cfg.shadow_depth,
        grace_depth: cfg.grace_depth,
        graze_cos: cfg.graze_cos,
        fine_divisor: fine,
    };
    let step = cfg.ray_subsample as usize;
    let walk = points
        .par_iter()
        .enumerate()
        .fold(RayWalk::default, |mut walk, (i, &p)| {
            if i % step != 0 {
                return walk;
            }
            let endpoint = world_to_voxel(p.0, p.1, p.2, inv);
            find_misses_along_ray(&mut walk, &frame, p, endpoint);
            walk
        })
        .reduce(RayWalk::default, RayWalk::merge);
    let RayWalk {
        mut misses,
        deferred,
    } = walk;
    resolve_deferred(map, &mut misses, &deferred, cfg.voxel_size, cfg.graze_cos);

    // New voxels join the stale set so a sparse voxel among converged
    // neighbors is pooled from them by the first ray that needs it, not only
    // at its own first milestone.
    let mut changed: AHashSet<VoxelKey> = AHashSet::new();
    for &v in &hits {
        if map.record_hit(v, cfg.min_health, cfg.max_health) {
            changed.insert(v);
        }
    }

    let mut fine_live: AHashSet<VoxelKey> =
        AHashSet::with_capacity(if fine.is_some() { points.len() } else { 0 });
    for &p in points {
        let (key, milestone, fine_key) = map.accumulate(p, cfg.voxel_size, fine);
        if milestone {
            changed.insert(key);
        }
        if let Some(fk) = fine_key {
            fine_live.insert(fk);
        }
    }

    let mut removed: Vec<VoxelKey> = Vec::new();
    for &v in &misses {
        if map.record_miss(v, cfg.min_health) {
            removed.push(v);
        }
    }

    mark_stale(map, &changed, &removed);

    FrameHits {
        coarse: hits,
        fine: fine_live,
    }
}

/// Health seeded voxels start at: healthy, but easily carved by live misses.
const SEED_HEALTH: VoxelHealth = 1;

/// One tile of a seed load: the world-frame points falling in one chunk.
pub type SeedTile = Vec<(f32, f32, f32)>;

/// The chunk tiles under one square of the seed region grid, with the
/// cylinder that covers them.
pub struct SeedRegion {
    pub cylinder: Cylinder,
    pub tiles: Vec<SeedTile>,
}

/// A cloud split for seeding, with the count of distinct voxels it covers.
pub struct SeedPartition {
    pub regions: Vec<SeedRegion>,
    pub voxels: usize,
}

impl SeedPartition {
    pub fn tiles(&self) -> impl Iterator<Item = &SeedTile> {
        self.regions.iter().flat_map(|r| r.tiles.iter())
    }

    pub fn tile_count(&self) -> usize {
        self.regions.iter().map(|r| r.tiles.len()).sum()
    }
}

/// Split a world-frame cloud into `region_m` squares of chunk tiles, nearest
/// `origin` first at both levels.
pub fn partition_seed(
    points: &[(f32, f32, f32)],
    voxel_size: f32,
    origin: (f32, f32, f32),
    region_m: f32,
) -> SeedPartition {
    let (tiles, voxels) = seed_tiles(points, voxel_size, origin);
    SeedPartition {
        regions: seed_regions(tiles, voxel_size, region_m),
        voxels,
    }
}

/// Bucket a cloud into chunk tiles, nearest `origin` first, with the count of
/// distinct voxels it covers.
fn seed_tiles(
    points: &[(f32, f32, f32)],
    voxel_size: f32,
    origin: (f32, f32, f32),
) -> (Vec<(ChunkKey, SeedTile)>, usize) {
    let inv = 1.0 / voxel_size;
    let mut tiles: AHashMap<ChunkKey, SeedTile> = AHashMap::new();
    let mut keys: AHashSet<VoxelKey> = AHashSet::new();
    for &(x, y, z) in points {
        if !in_key_range(x, y, z, inv) {
            continue;
        }
        let key = world_to_voxel(x, y, z, inv);
        keys.insert(key);
        tiles.entry(chunk_of(key)).or_default().push((x, y, z));
    }
    let edge = CHUNK_EDGE as f32 * voxel_size;
    let mut ordered: Vec<(f32, ChunkKey, SeedTile)> = tiles
        .into_iter()
        .map(|(chunk, tile)| {
            let dx = (chunk.0 as f32 + 0.5) * edge - origin.0;
            let dy = (chunk.1 as f32 + 0.5) * edge - origin.1;
            let dz = (chunk.2 as f32 + 0.5) * edge - origin.2;
            (dx * dx + dy * dy + dz * dz, chunk, tile)
        })
        .collect();
    ordered.sort_by(|a, b| a.0.total_cmp(&b.0));
    let ordered = ordered
        .into_iter()
        .map(|(_, chunk, tile)| (chunk, tile))
        .collect();
    (ordered, keys.len())
}

/// Group tiles into region grid cells in the order the tiles arrive, each
/// region's cylinder sized to the chunk boxes it holds.
fn seed_regions(
    tiles: Vec<(ChunkKey, SeedTile)>,
    voxel_size: f32,
    region_m: f32,
) -> Vec<SeedRegion> {
    let edge = CHUNK_EDGE as f32 * voxel_size;
    let mut regions: AHashMap<Cell, (Aabb, Vec<SeedTile>)> = AHashMap::new();
    let mut order: Vec<Cell> = Vec::new();
    for (chunk, tile) in tiles {
        let lo = (
            chunk.0 as f32 * edge,
            chunk.1 as f32 * edge,
            chunk.2 as f32 * edge,
        );
        let hi = (lo.0 + edge, lo.1 + edge, lo.2 + edge);
        let cell = region_of(chunk, voxel_size, region_m);
        let (aabb, tiles) = regions.entry(cell).or_insert_with(|| {
            order.push(cell);
            (Aabb { lo, hi }, Vec::new())
        });
        aabb.lo = (
            aabb.lo.0.min(lo.0),
            aabb.lo.1.min(lo.1),
            aabb.lo.2.min(lo.2),
        );
        aabb.hi = (
            aabb.hi.0.max(hi.0),
            aabb.hi.1.max(hi.1),
            aabb.hi.2.max(hi.2),
        );
        tiles.push(tile);
    }
    order
        .into_iter()
        .map(|cell| {
            let (aabb, tiles) = regions.remove(&cell).expect("region recorded in order");
            SeedRegion {
                cylinder: aabb.covering_cylinder(voxel_size),
                tiles,
            }
        })
        .collect()
}

/// An axis-aligned box of world-frame chunk extents.
struct Aabb {
    lo: (f32, f32, f32),
    hi: (f32, f32, f32),
}

impl Aabb {
    /// The cylinder on the box's xy center that reaches its corners with a
    /// voxel of margin all round.
    fn covering_cylinder(&self, margin: f32) -> Cylinder {
        let hx = (self.hi.0 - self.lo.0) * 0.5;
        let hy = (self.hi.1 - self.lo.1) * 0.5;
        Cylinder {
            cx: self.lo.0 + hx,
            cy: self.lo.1 + hy,
            radius: hx.hypot(hy) + margin,
            z_min: self.lo.2 - margin,
            z_max: self.hi.2 + margin,
        }
    }
}

/// Seed one tile without ray tracing, creating only absent voxels. Returns
/// the created keys.
pub fn seed_tile(
    map: &mut VoxelMap,
    points: &[(f32, f32, f32)],
    cfg: &Config,
) -> AHashSet<VoxelKey> {
    let inv = 1.0 / cfg.voxel_size;
    let fine = cfg.fine_layer().map(|(d, _)| d as i32);

    let mut created: AHashSet<VoxelKey> = AHashSet::new();
    for &(x, y, z) in points {
        if !in_key_range(x, y, z, inv) {
            continue;
        }
        let key = world_to_voxel(x, y, z, inv);
        if map.insert_seed(key) {
            created.insert(key);
        } else if !created.contains(&key) {
            continue;
        }
        map.accumulate((x, y, z), cfg.voxel_size, fine);
    }
    mark_stale(map, &created, &[]);
    created
}

/// Bulk-load a whole world-frame cloud in one call, tile by tile. Returns
/// how many voxels were created.
pub fn seed_points(map: &mut VoxelMap, points: &[(f32, f32, f32)], cfg: &Config) -> usize {
    let region_m = CHUNK_EDGE as f32 * cfg.voxel_size;
    let part = partition_seed(points, cfg.voxel_size, (0.0, 0.0, 0.0), region_m);
    map.reserve_chunks(part.tile_count());
    part.tiles()
        .map(|tile| seed_tile(map, tile, cfg).len())
        .sum()
}

#[inline]
fn world_to_voxel(x: f32, y: f32, z: f32, inv: f32) -> VoxelKey {
    (
        (x * inv).floor() as i32,
        (y * inv).floor() as i32,
        (z * inv).floor() as i32,
    )
}

/// Keys stay this far inside i32 so the neighborhood and chunk arithmetic
/// around any stored voxel cannot overflow.
const KEY_LIMIT: f32 = (1 << 30) as f32;

/// Whether a point quantizes to a key the map can hold. Non-finite
/// coordinates fail too.
#[inline]
fn in_key_range(x: f32, y: f32, z: f32, inv: f32) -> bool {
    (x * inv).abs() < KEY_LIMIT && (y * inv).abs() < KEY_LIMIT && (z * inv).abs() < KEY_LIMIT
}

/// Quantize world-frame metric points to voxel keys the same way returns are
/// quantized, so a caller naming voxels by position lands on the ones the map
/// actually holds.
pub fn metric_voxel_keys(
    points: impl IntoIterator<Item = (f32, f32, f32)>,
    voxel_size: f32,
) -> impl Iterator<Item = VoxelKey> {
    let inv = 1.0 / voxel_size;
    points
        .into_iter()
        .map(move |(x, y, z)| world_to_voxel(x, y, z, inv))
}

/// Fine cells of `key` crossed by the ray segment between `t0` and `t1`,
/// as a bitmask of flat child indices.
fn crossed_fine_cells(
    origin: (f32, f32, f32),
    delta: (f32, f32, f32),
    t0: f32,
    t1: f32,
    key: VoxelKey,
    divisor: i32,
    voxel_size: f32,
) -> u64 {
    let fine_size = voxel_size / divisor as f32;
    // The entry point sits on a voxel face, so clamp float error into range.
    let local = |o: f32, d: f32, k: i32| {
        ((((o + t0 * d) / fine_size).floor() as i32) - k * divisor).clamp(0, divisor - 1)
    };
    let mut lx = local(origin.0, delta.0, key.0);
    let mut ly = local(origin.1, delta.1, key.1);
    let mut lz = local(origin.2, delta.2, key.2);

    let axis = |o: f32, d: f32, k: i32, l: i32| -> (i32, f32, f32) {
        if d == 0.0 {
            return (0, f32::INFINITY, f32::INFINITY);
        }
        let step = if d > 0.0 { 1 } else { -1 };
        let boundary = (k * divisor + l + (step > 0) as i32) as f32 * fine_size;
        (step, (boundary - o) / d, fine_size / d.abs())
    };
    let (step_x, mut tx, dt_x) = axis(origin.0, delta.0, key.0, lx);
    let (step_y, mut ty, dt_y) = axis(origin.1, delta.1, key.1, ly);
    let (step_z, mut tz, dt_z) = axis(origin.2, delta.2, key.2, lz);

    let mut mask = 0u64;
    loop {
        mask |= 1 << ((lx * divisor + ly) * divisor + lz);
        if tx.min(ty).min(tz) >= t1 {
            return mask;
        }
        if tx <= ty && tx <= tz {
            lx += step_x;
            tx += dt_x;
            if !(0..divisor).contains(&lx) {
                return mask;
            }
        } else if ty <= tz {
            ly += step_y;
            ty += dt_y;
            if !(0..divisor).contains(&ly) {
                return mask;
            }
        } else {
            lz += step_z;
            tz += dt_z;
            if !(0..divisor).contains(&lz) {
                return mask;
            }
        }
    }
}

/// One frame's clearing decisions. Deferred crossings hit a voxel whose normal
/// was stale and wait on its refit.
#[derive(Default)]
struct RayWalk {
    misses: AHashSet<VoxelKey>,
    deferred: Vec<(VoxelKey, Vector3<f32>)>,
}

impl RayWalk {
    fn merge(mut self, mut other: Self) -> Self {
        if self.misses.len() < other.misses.len() {
            std::mem::swap(&mut self.misses, &mut other.misses);
        }
        self.misses.extend(other.misses);
        self.deferred.append(&mut other.deferred);
        self
    }
}

/// The inputs one frame's ray walks share.
#[derive(Clone, Copy)]
struct RayFrame<'a> {
    voxels: &'a ChunkMap,
    hits: &'a AHashSet<VoxelKey>,
    origin: (f32, f32, f32),
    origin_voxel: VoxelKey,
    voxel_size: f32,
    shadow_depth: f32,
    grace_depth: f32,
    graze_cos: f32,
    fine_divisor: Option<i32>,
}

/// Amanatides and Woo 3d DDA. Records in-map voxels along the ray between the
/// origin and the end of the shadow region. Voxels within the grace region of
/// the endpoint are spared from being marked as misses, as are voxels hit this
/// frame. With a fine divisor, a voxel is also spared when the ray misses all of
/// its observed fine cells.
fn find_misses_along_ray(
    walk: &mut RayWalk,
    frame: &RayFrame,
    end: (f32, f32, f32),
    endpoint: VoxelKey,
) {
    let RayFrame {
        voxels,
        hits,
        origin,
        origin_voxel,
        voxel_size,
        shadow_depth,
        grace_depth,
        graze_cos,
        fine_divisor,
    } = *frame;
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

    let mut cursor = ChunkCursor::new(voxels);
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

        if let Some(c) = cursor.get((x, y, z)) {
            if hits.contains(&(x, y, z)) {
                continue;
            }
            // A ray through unobserved fine cells contradicts nothing.
            if let Some(divisor) = fine_divisor {
                if c.fine != 0 {
                    let crossed = crossed_fine_cells(
                        origin,
                        (dx, dy, dz),
                        t_enter,
                        tx.min(ty).min(tz),
                        (x, y, z),
                        divisor,
                        voxel_size,
                    );
                    if crossed & c.fine == 0 {
                        continue;
                    }
                }
            }
            match c.normal {
                NormalFit::Fitted(n) => {
                    if !should_spare(n, ray_unit, graze_cos) {
                        walk.misses.insert((x, y, z));
                    }
                }
                NormalFit::Stale => walk.deferred.push(((x, y, z), ray_unit)),
            }
        }
    }
}
