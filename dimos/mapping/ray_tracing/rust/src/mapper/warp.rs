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

use rayon::prelude::*;

use super::Mapper;
use super::Point;
use crate::reform::Warp;
use crate::voxel_ray_tracer::{voxel_moves, ChunkMap, VoxelKey};

/// A point's place before and after a warp.
pub type Displacement = ([f32; 3], [f32; 3]);

/// What one warp did and what it cost.
#[derive(Clone, Debug, Default)]
pub struct WarpReport {
    /// Voxels the warp moved in place.
    pub voxels_warped: usize,
    /// Voxels re-keyed by their moved points, those in the local area.
    pub voxels_rebinned: usize,
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
}

impl VoxelTimes {
    fn stamp(&mut self, t: f64) {
        let epoch = *self.epoch.get_or_insert(t);
        self.now = (t - epoch) as f32;
    }

    /// Stamp the voxels this frame hit that have no time yet.
    pub(super) fn stamp_hits<'a>(
        &self,
        map: &mut ChunkMap,
        hits: impl IntoIterator<Item = &'a VoxelKey>,
    ) {
        for k in hits {
            if let Some(v) = map.get_mut(k) {
                if v.time.is_nan() {
                    v.time = self.now;
                }
            }
        }
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
            self.voxel_times.as_ref().map_or(0, |_| {
                self.map
                    .voxels
                    .iter()
                    .filter(|(_, v)| !v.time.is_nan())
                    .count()
            }),
        )
    }

    /// Every healthy voxel's center, flat xyz triples.
    pub fn healthy_centers(&self) -> Vec<f32> {
        let live = ahash::AHashSet::new();
        self.pool.install(|| {
            crate::voxel_ray_tracer::emit_points_ungated(&self.map, self.config.voxel_size, &live)
        })
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
        match (
            self.voxel_times.as_ref().and_then(|t| t.epoch),
            self.map.voxels.get(k),
        ) {
            (Some(epoch), Some(v)) => epoch + v.time as f64,
            _ => f64::NAN,
        }
    }

    /// Carry the map through a warp: every voxel's points move in place by
    /// fast-reform, keeping its key; only the voxels in the local area are
    /// re-keyed by their moved points, the rest when the local area reaches them.
    pub fn apply_warp(&mut self, warp: &Warp) -> WarpReport {
        let pool = std::sync::Arc::clone(&self.pool);
        pool.install(|| self.apply_warp_in_pool(warp))
    }

    fn apply_warp_in_pool(&mut self, warp: &Warp) -> WarpReport {
        let vs = self.config.voxel_size;
        let mut report = WarpReport::default();
        let stage = Instant::now();
        let keys: Vec<VoxelKey> = self.map.voxels.iter().map(|(k, _)| k).collect();
        report.voxels_warped = keys.len();
        if keys.is_empty() {
            return report;
        }
        let times: Vec<f64> = keys.par_iter().map(|k| self.voxel_time(k)).collect();
        let means = self.map.voxel_means(&keys, vs);
        let mut warped = warp.apply(&means, &times);
        (report.implausible, report.worst_implausible) =
            hold_implausible(&means, &mut warped, warp.max_plausible_move_m());
        report.warp_ms = ms_since(stage);

        let stage = Instant::now();
        let offsets: Vec<[f32; 3]> = means
            .par_iter()
            .zip(warped.par_iter())
            .map(|(p, w)| [w[0] - p[0], w[1] - p[1], w[2] - p[2]])
            .collect();
        self.map.shift_in_place(&offsets);
        report.voxels_rebinned = self.rebin_local(self.last_origin);
        report.voxel_ms = ms_since(stage);
        report
    }

    /// Re-key the voxels within ray reach of `origin` whose points a warp moved
    /// out of their key, so rays meet them where they are.
    pub(super) fn rebin_local(&mut self, origin: Point) -> usize {
        let vs = self.config.voxel_size;
        let near = self.map.displaced_near(origin, self.config.max_range, vs);
        if near.is_empty() {
            return 0;
        }
        let map = &self.map;
        let keys = crate::reform::with_chained(near, |k| map.displaced(k, vs));
        let means = self.map.voxel_means(&keys, vs);
        let moving: ahash::AHashSet<VoxelKey> = keys.iter().copied().collect();
        let voxels = &self.map.voxels;
        let moves = voxel_moves(&keys, &means, &means, vs, |k| {
            voxels.contains_key(k) && !moving.contains(k)
        });
        self.map.move_voxels(&moves);
        keys.len()
    }
}

#[cfg(test)]
mod tests;
