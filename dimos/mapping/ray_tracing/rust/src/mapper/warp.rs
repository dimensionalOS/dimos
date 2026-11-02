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

//! Carry the map through a loop closure's warp in place: every voxel the warp
//! reaches moves to where fast-reform puts its center, at the time it landed.

use std::time::Instant;

use ahash::AHashMap;
use rayon::prelude::*;

use super::Mapper;
use crate::reform::{Tile, Warp};
use crate::voxel_ray_tracer::{voxel_centers, voxel_moves, VoxelKey};

/// A point's place before and after a warp.
pub type Displacement = ([f32; 3], [f32; 3]);

/// What one warp did and what it cost.
#[derive(Clone, Debug, Default)]
pub struct WarpReport {
    /// Voxels in tiles the warp reaches.
    pub voxels_warped: usize,
    /// Of those, the ones that landed in another voxel.
    pub voxels_moved: usize,
    /// Whether so much moved that the map was rebuilt instead of repaired.
    pub rebuilt: bool,
    pub warp_ms: f64,
    pub voxel_ms: f64,
    /// Voxels the warp sent somewhere non-finite or implausibly far, left in place.
    pub implausible: usize,
    /// The worst of those: where it was, where the warp sent it.
    pub worst_implausible: Option<Displacement>,
}

/// Leave in place every warped point that is non-finite or moved farther than
/// `max_move_m`, counting them and keeping the worst for the log.
fn hold_implausible(
    queries: &[[f32; 3]],
    warped: &mut [[f32; 3]],
    max_move_m: f64,
) -> (usize, Option<Displacement>) {
    let mut count = 0;
    let mut worst: Option<(f64, [f32; 3], [f32; 3])> = None;
    for (from, to) in queries.iter().zip(warped.iter_mut()) {
        let moved = ((to[0] - from[0]) as f64)
            .hypot((to[1] - from[1]) as f64)
            .hypot((to[2] - from[2]) as f64);
        let finite = to.iter().all(|v| v.is_finite());
        if finite && moved <= max_move_m {
            continue;
        }
        count += 1;
        let score = if finite { moved } else { f64::INFINITY };
        if worst.is_none_or(|(w, _, _)| score > w) {
            worst = Some((score, *from, *to));
        }
        *to = *from;
    }
    (count, worst.map(|(_, from, to)| (from, to)))
}

/// Seconds since `epoch` at which each voxel landed, and the clock new ones
/// are stamped with.
#[derive(Default)]
pub(super) struct VoxelTimes {
    epoch: Option<f64>,
    now: f32,
    by_key: AHashMap<VoxelKey, f32>,
}

impl VoxelTimes {
    fn stamp(&mut self, t: f64) {
        let epoch = *self.epoch.get_or_insert(t);
        self.now = (t - epoch) as f32;
    }

    /// Stamp the voxels this frame hit that have no time yet.
    pub(super) fn stamp_hits<'a>(&mut self, hits: impl IntoIterator<Item = &'a VoxelKey>) {
        for &k in hits {
            self.by_key.entry(k).or_insert(self.now);
        }
    }

    /// Absolute time of a voxel, NaN when unknown.
    fn get(&self, k: &VoxelKey) -> f64 {
        match (self.by_key.get(k), self.epoch) {
            (Some(&t), Some(epoch)) => epoch + t as f64,
            _ => f64::NAN,
        }
    }

    pub(super) fn clear(&mut self) {
        self.by_key.clear();
    }
}

fn ms_since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

impl Mapper {
    /// Voxels, chunks and voxel times held, for the periodic memory log.
    pub fn sizes(&self) -> (usize, usize, usize) {
        (
            self.map.voxels.len(),
            self.map.voxels.chunk_count(),
            self.voxel_times.as_ref().map_or(0, |t| t.by_key.len()),
        )
    }

    /// Record when each voxel lands, for warps. Costs a map entry per voxel.
    pub fn track_voxel_times(&mut self) {
        self.voxel_times.get_or_insert_with(VoxelTimes::default);
    }

    /// The time, in seconds, the next frame's new voxels are stamped with.
    pub fn set_clock(&mut self, t: f64) {
        if let Some(times) = &mut self.voxel_times {
            times.stamp(t);
        }
    }

    /// Absolute time a voxel landed, NaN when untracked or unknown.
    pub fn voxel_time(&self, k: &VoxelKey) -> f64 {
        self.voxel_times.as_ref().map_or(f64::NAN, |t| t.get(k))
    }

    /// Carry the map through a warp. When more than the warp's rebuild
    /// fraction of the voxels land in another voxel, the map is rebuilt from
    /// the warped voxels instead of repaired around each.
    pub fn apply_warp(&mut self, warp: &Warp) -> WarpReport {
        let pool = std::sync::Arc::clone(&self.pool);
        pool.install(|| self.apply_warp_in_pool(warp))
    }

    fn apply_warp_in_pool(&mut self, warp: &Warp) -> WarpReport {
        let vs = self.config.voxel_size;
        let mut report = WarpReport::default();
        let stage = Instant::now();

        // Which tiles the warp reaches, decided once per tile.
        let chunks: Vec<_> = self.map.voxels.chunks().collect();
        let mut tiles: Vec<Tile> = chunks
            .par_iter()
            .flat_map_iter(|c| c.iter().map(|(k, _)| tile_at(warp, k, vs)))
            .collect();
        tiles.par_sort_unstable();
        tiles.dedup();
        let reached: ahash::AHashSet<Tile> = tiles
            .into_iter()
            .filter(|&t| warp.affects_tile(t))
            .collect();
        let keys: Vec<VoxelKey> = chunks
            .par_iter()
            .flat_map_iter(|c| {
                c.iter()
                    .map(|(k, _)| k)
                    .filter(|&k| reached.contains(&tile_at(warp, k, vs)))
                    .collect::<Vec<_>>()
            })
            .collect();
        drop(chunks);
        report.voxels_warped = keys.len();
        if keys.is_empty() {
            return report;
        }
        let times: Vec<f64> = keys.iter().map(|k| self.voxel_time(k)).collect();
        let centers = voxel_centers(&keys, vs);
        let mut warped = warp.apply(&centers, &times);
        (report.implausible, report.worst_implausible) =
            hold_implausible(&centers, &mut warped, warp.max_plausible_move_m());
        report.warp_ms = ms_since(stage);

        let stage = Instant::now();
        let moves = voxel_moves(&keys, &warped, vs);
        report.voxels_moved = moves.par_iter().filter(|m| m.from != m.to).count();
        report.rebuilt =
            report.voxels_moved as f64 > warp.rebuild_fraction() * self.map.voxels.len() as f64;
        if report.rebuilt {
            self.map.rebuild_moved(&moves);
        } else {
            self.map.move_voxels(&moves);
        }
        if let Some(t) = &mut self.voxel_times {
            let landed: Vec<(VoxelKey, Option<f32>)> = moves
                .iter()
                .filter(|m| m.from != m.to)
                .map(|m| (m.to, t.by_key.remove(&m.from)))
                .collect();
            for (to, time) in landed {
                if let Some(time) = time {
                    t.by_key
                        .entry(to)
                        .and_modify(|e| *e = e.min(time))
                        .or_insert(time);
                }
            }
            // Drop times of voxels that are gone, so a key reused later starts fresh.
            let map = &self.map.voxels;
            t.by_key.retain(|k, _| map.contains_key(k));
        }
        report.voxel_ms = ms_since(stage);
        report
    }
}

fn tile_at(warp: &Warp, k: VoxelKey, vs: f32) -> Tile {
    warp.tile((k.0 as f32 + 0.5) * vs, (k.1 as f32 + 0.5) * vs)
}

#[cfg(test)]
mod tests;
