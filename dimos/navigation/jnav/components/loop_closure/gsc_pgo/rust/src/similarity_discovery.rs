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

//! Stage 2, similarity discovery: which earlier keyframes this one might be, by Scan Context or by
//! position. Scan Context is a polar-binned lidar place-recognition descriptor
//! (inspired by Kim & Kim 2018 and the irapkaist/scancontext reference, MIT).
//!
//! Each scan becomes an (n_rings x n_sectors) matrix where cell [i, j] holds
//! the max z among points falling in that (range, azimuth) bin. The "ring
//! key" — the per-row mean — is the coarse feature used for fast candidate
//! retrieval; the full matrix is then column-shifted against the candidate to
//! measure rotation-invariant cosine distance.
//!
//! Numeric discipline: the descriptor is stored as float and the cosine /
//! mean arithmetic is done in float, but ranges/azimuths are binned in double
//! and structure statistics accumulate in double.

use std::f64::consts::PI;

use crate::mat3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    /// Radial bins.
    pub n_rings: usize,
    /// Azimuth bins.
    pub n_sectors: usize,
    /// Ignore points beyond this.
    pub max_range_m: f64,
    /// Ring-key neighbours to score with the full descriptor.
    pub candidate_top_k: usize,
    /// Accepted cosine distance (0..2).
    pub match_threshold: f64,
    /// Shifts body-frame z so all cells are positive before cosine distance,
    /// matching irapkaist/scancontext's LIDAR_HEIGHT convention. Ground
    /// points sit near -lidar_height_m in the body frame; without this shift,
    /// negative cells make cosine similarity meaningless for revisits.
    pub lidar_height_m: f64,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            n_rings: 20,
            n_sectors: 60,
            max_range_m: 80.0,
            candidate_top_k: 10,
            match_threshold: 0.4,
            lidar_height_m: 2.0,
        }
    }
}

/// (n_rings x n_sectors) max-z matrix, row-major. Empty cells are 0.
#[derive(Debug, Clone, PartialEq)]
pub struct Descriptor {
    pub n_rings: usize,
    pub n_sectors: usize,
    /// Row-major: `data[ring * n_sectors + sector]`.
    pub data: Vec<f32>,
}

impl Descriptor {
    pub fn zeros(n_rings: usize, n_sectors: usize) -> Descriptor {
        Descriptor {
            n_rings,
            n_sectors,
            data: vec![0.0; n_rings * n_sectors],
        }
    }

    /// An empty (0x0) descriptor — used for a node without a cloud.
    pub fn empty() -> Descriptor {
        Descriptor {
            n_rings: 0,
            n_sectors: 0,
            data: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    #[inline]
    pub fn at(&self, ring: usize, sector: usize) -> f32 {
        self.data[ring * self.n_sectors + sector]
    }

    #[inline]
    fn at_mut(&mut self, ring: usize, sector: usize) -> &mut f32 {
        &mut self.data[ring * self.n_sectors + sector]
    }
}

/// Mean per row — the coarse feature used for candidate retrieval.
pub type RingKey = Vec<f32>;
/// Mean per column — only used for the optional sector-key alignment.

/// Build the polar-max-z descriptor for a body-frame scan (points as
/// `[x, y, z]`, f32 like PCL). Points outside `max_range_m` or at the sensor
/// origin are ignored.
pub fn make_descriptor(points: &[[f32; 3]], config: &Config) -> Descriptor {
    // Empty cells stay at 0; we shift z by lidar_height so real points are
    // strictly positive and "no point here" is distinguishable from ground
    // level. Matches irapkaist/scancontext's NO_POINT convention closely
    // enough that the column-wise cosine distance behaves.
    let mut descriptor = Descriptor::zeros(config.n_rings, config.n_sectors);
    if config.n_rings == 0 || config.n_sectors == 0 || config.max_range_m <= 0.0 {
        return descriptor;
    }

    let ring_step = config.max_range_m / config.n_rings as f64;
    let sector_step = 2.0 * PI / config.n_sectors as f64;
    let height_offset = config.lidar_height_m as f32;

    for point in points {
        let x = point[0] as f64;
        let y = point[1] as f64;
        let z = point[2];

        let range = (x * x + y * y).sqrt();
        if range >= config.max_range_m || range <= 1e-6 {
            continue;
        }

        let ring = (range / ring_step).floor() as i64;
        if ring < 0 || ring >= config.n_rings as i64 {
            continue;
        }

        let mut azimuth = y.atan2(x);
        if azimuth < 0.0 {
            azimuth += 2.0 * PI;
        }
        let mut sector = (azimuth / sector_step).floor() as i64;
        if sector < 0 {
            sector = 0;
        }
        if sector >= config.n_sectors as i64 {
            sector = config.n_sectors as i64 - 1;
        }

        let shifted_z = z + height_offset;
        // Clip to >= 0 — points slightly below the sensor frame (rare in
        // properly-mounted lidars) shouldn't pull the cell negative.
        let cell_value = if shifted_z > 0.0 { shifted_z } else { 0.0 };
        let cell = descriptor.at_mut(ring as usize, sector as usize);
        if cell_value > *cell {
            *cell = cell_value;
        }
    }
    descriptor
}

/// Auto-select the ring range from a scan's spatial extent, for when no
/// explicit `max_range_m` is configured (e.g. the Go2's short-range L1 lidar,
/// where the 80 m default collapses every point into the innermost ring). Uses
/// a high percentile of the points' planar (xy) range so a few stray far
/// returns don't blow the binning out; returns 0.0 for an empty cloud so the
/// caller can leave the range unresolved. Resolve ONCE and reuse it: Scan
/// Context descriptors only compare across scans when every scan shares the
/// same binning.
pub fn auto_max_range(points: &[[f32; 3]]) -> f64 {
    const RANGE_PERCENTILE: f64 = 0.95;
    let mut ranges: Vec<f64> = points
        .iter()
        .map(|point| ((point[0] as f64).powi(2) + (point[1] as f64).powi(2)).sqrt())
        .filter(|range| *range > 1e-6)
        .collect();
    if ranges.is_empty() {
        return 0.0;
    }
    ranges.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let index = ((ranges.len() - 1) as f64 * RANGE_PERCENTILE).round() as usize;
    ranges[index]
}

/// Mean per row (over ALL columns, zeros included) — the retrieval key.
pub fn make_ring_key(descriptor: &Descriptor) -> RingKey {
    let mut key = vec![0.0f32; descriptor.n_rings];
    if descriptor.n_sectors == 0 {
        return key;
    }
    for (i, slot) in key.iter_mut().enumerate() {
        let mut sum = 0.0f32;
        for j in 0..descriptor.n_sectors {
            sum += descriptor.at(i, j);
        }
        *slot = sum / descriptor.n_sectors as f32;
    }
    key
}

/// Cosine distance between two descriptors after column-shifting `candidate`
/// by `shift` columns. 0 = identical, 2 = opposite. Columns where either side
/// is (near-)empty are skipped; if none are valid, returns 2.
pub fn column_cosine_distance(query: &Descriptor, candidate: &Descriptor, shift: i32) -> f32 {
    if query.n_rings != candidate.n_rings || query.n_sectors != candidate.n_sectors {
        return 2.0;
    }
    let cols = query.n_sectors as i32;
    if cols == 0 {
        return 2.0;
    }

    let mut total = 0.0f32;
    let mut valid_cols = 0usize;
    for j in 0..cols {
        let shifted_j = (((j + shift) % cols + cols) % cols) as usize;
        let j = j as usize;
        let mut dot = 0.0f32;
        let mut query_sq = 0.0f32;
        let mut candidate_sq = 0.0f32;
        for i in 0..query.n_rings {
            let a = query.at(i, j);
            let b = candidate.at(i, shifted_j);
            dot += a * b;
            query_sq += a * a;
            candidate_sq += b * b;
        }
        let query_norm = query_sq.sqrt();
        let candidate_norm = candidate_sq.sqrt();
        if query_norm <= 1e-6 || candidate_norm <= 1e-6 {
            continue;
        }
        let cos_sim = dot / (query_norm * candidate_norm);
        total += 1.0 - cos_sim;
        valid_cols += 1;
    }
    if valid_cols == 0 {
        return 2.0;
    }
    total / valid_cols as f32
}

/// Best (min-distance, best-shift) pair across all column shifts. To recover
/// yaw rotation from the shift: `yaw_from_shift(shift, n_sectors)`.
pub fn best_distance(query: &Descriptor, candidate: &Descriptor) -> (f32, i32) {
    let cols = query.n_sectors as i32;
    let mut min_distance = 2.0f32;
    let mut best_shift = 0i32;
    for shift in 0..cols {
        let distance = column_cosine_distance(query, candidate, shift);
        if distance < min_distance {
            min_distance = distance;
            best_shift = shift;
        }
    }
    (min_distance, best_shift)
}

/// Count of occupied (non-zero) cells: how much of the polar FOV returned structure out to range.
pub fn descriptor_occupancy(descriptor: &Descriptor) -> usize {
    descriptor.data.iter().filter(|&&value| value > 0.0).count()
}

/// Convert sector shift to yaw rotation (radians). `shift` comes from
/// `best_distance`, which scans [0, n_sectors-1], so the raw yaw lies in
/// (-2pi, 0]; wrap into [-pi, pi].
pub fn yaw_from_shift(shift: i32, n_sectors: usize) -> f64 {
    let mut yaw = -2.0 * PI * shift as f64 / n_sectors as f64;
    if yaw < -PI {
        yaw += 2.0 * PI;
    }
    yaw
}

/// Rank candidate ring keys by L2 distance to `query_key` and return the
/// top-k `(distance, index)` pairs, ascending — the coarse retrieval stage of
/// `SimplePGO::searchByScanContext` (a linear scan + partial sort; small
/// enough at keyframe counts that a kd-tree buys nothing yet). Entries whose
/// ring key length differs from the query (e.g. cloud-less nodes) are
/// skipped.
///
/// TODO: switch to a kd-tree later for O(log n) instead of O(n) — this linear
/// scan is O(N) per query (O(N^2) over a run) and won't scale to massive maps.
pub fn ring_key_top_k(query_key: &RingKey, candidates: &[RingKey], k: usize) -> Vec<(f32, usize)> {
    let mut ranked: Vec<(f32, usize)> = Vec::with_capacity(candidates.len());
    for (idx, key) in candidates.iter().enumerate() {
        if key.len() != query_key.len() || key.is_empty() {
            continue;
        }
        let mut sq = 0.0f32;
        for (a, b) in key.iter().zip(query_key.iter()) {
            let d = a - b;
            sq += d * d;
        }
        ranked.push((sq.sqrt(), idx));
    }
    ranked.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    ranked.truncate(k);
    ranked
}

impl crate::gsc_pgo::GscPgo {
    /// The newest keyframe a closure may close onto: one whose submap leaves out the source's own scan (a match
    /// against itself only checks odometry against itself). Extended ICP keeps its stack clear of the target on its own.
    pub(crate) fn newest_closure_target(&self) -> usize {
        (self.key_poses.len() - 1)
            .saturating_sub(self.config.closure_submap_half_range.max(0) as usize + 1)
    }
    /// Radius search on past key-pose global positions (candidates ascending
    /// by distance), returning the nearest that is far enough back.
    pub(crate) fn search_by_position(&self) -> i64 {
        let here = self.key_poses.last().unwrap().translation_global;
        self.key_poses[..self.newest_closure_target()]
            .iter()
            .map(|k| mat3::norm(&mat3::sub(&k.translation_global, &here)))
            .enumerate()
            .filter(|&(_, d)| d <= self.config.closure_search_radius)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map_or(-1, |(i, _)| i as i64)
    }
    /// Scan-context candidate search. Returns (target_idx or -1,
    /// sector_shift, best, second): the accepted candidate under the match
    /// threshold, plus the closest / 2nd-closest cosine distances across the
    /// top-K (the Lowe ratio distinctiveness signal).
    pub(crate) fn search_by_scan_context(&self) -> (i64, i32, f32, f32) {
        let mut out_best = 2.0f32;
        let mut out_second = 2.0f32;
        if self.scan_context_descriptors.is_empty()
            || self.scan_context_descriptors.last().unwrap().is_empty()
        {
            return (-1, 0, out_best, out_second);
        }
        let query = self.scan_context_descriptors.last().unwrap();
        // two-stage retrieval: rank by ring-key distance, then score the top k on the full descriptor
        let ranked = ring_key_top_k(
            self.scan_context_ring_keys.last().unwrap(),
            &self.scan_context_ring_keys[..self.newest_closure_target()],
            self.scan_context_config.candidate_top_k,
        );
        let mut best_dist = f32::MAX;
        let mut second_dist = f32::MAX;
        let mut best_dist_filtered = self.scan_context_config.match_threshold as f32;
        let mut best_idx: i64 = -1;
        let mut best_shift = 0i32;
        for &(_, idx) in &ranked {
            let (distance, shift) = best_distance(query, &self.scan_context_descriptors[idx]);
            if distance < best_dist {
                second_dist = best_dist; // demote previous best
                best_dist = distance;
            } else if distance < second_dist {
                second_dist = distance;
            }
            if distance < best_dist_filtered {
                best_dist_filtered = distance;
                best_idx = idx as i64;
                best_shift = shift;
            }
        }

        if best_dist < 2.0 {
            out_best = best_dist;
        }
        if second_dist < 2.0 {
            out_second = second_dist;
        }
        (best_idx, best_shift, out_best, out_second)
    }
}
