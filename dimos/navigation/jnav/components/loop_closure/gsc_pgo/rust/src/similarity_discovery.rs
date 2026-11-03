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

//! Stage 2, similarity discovery: which earlier keyframes this one might be, by position or by Scan Context
//! (Kim & Kim 2018): a polar max-z grid, ring-key retrieval then column-shifted cosine distance.

use std::f64::consts::PI;

use std::sync::Arc;

use crate::mat3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Config {
    pub n_rings: usize,
    pub n_sectors: usize,
    /// Ignore points beyond this planar range.
    pub max_range_m: f64,
    /// Ring-key neighbours to score with the full descriptor.
    pub candidate_top_k: usize,
    /// Accepted cosine distance (0..2).
    pub match_threshold: f64,
    /// Added to body-frame z so ground cells are positive (negative cells break cosine distance).
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

    /// A 0x0 descriptor, for a keyframe without a cloud.
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

/// Mean per row, the coarse retrieval feature.
pub type RingKey = Vec<f32>;

/// The polar max-z descriptor of a body-frame scan; points beyond `max_range_m` or at the origin are ignored.
pub fn make_descriptor(points: &[[f32; 3]], config: &Config) -> Descriptor {
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
        // clipped to >= 0 so 0 stays "no point"
        let cell_value = if shifted_z > 0.0 { shifted_z } else { 0.0 };
        let cell = descriptor.at_mut(ring as usize, sector as usize);
        if cell_value > *cell {
            *cell = cell_value;
        }
    }
    descriptor
}

/// 95th-percentile planar range of a scan (0.0 if empty); resolve once, since descriptors only compare at equal binning.
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
    let index = ((ranges.len() - 1) as f64 * RANGE_PERCENTILE).round() as usize;
    *ranges.select_nth_unstable_by(index, f64::total_cmp).1
}

/// Mean per row, over all columns including empty ones.
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

/// Mean column cosine distance with `candidate` shifted `shift` columns: 0 = identical, 2 = opposite or no shared columns.
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

/// (min distance, its shift) over all column shifts; `yaw_from_shift` turns the shift into yaw.
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

/// Sector shift (0..n_sectors) -> yaw in radians, wrapped into [-pi, pi].
pub fn yaw_from_shift(shift: i32, n_sectors: usize) -> f64 {
    let mut yaw = -2.0 * PI * shift as f64 / n_sectors as f64;
    if yaw < -PI {
        yaw += 2.0 * PI;
    }
    yaw
}

/// `(ring-key L2 distance, index)` to every candidate, unordered; skips keys of a different length (cloud-less keyframes).
pub fn ring_key_distances(query_key: &RingKey, candidates: &[RingKey]) -> Vec<(f32, usize)> {
    let mut distances: Vec<(f32, usize)> = Vec::with_capacity(candidates.len());
    for (idx, key) in candidates.iter().enumerate() {
        if key.len() != query_key.len() || key.is_empty() {
            continue;
        }
        let mut sq = 0.0f32;
        for (a, b) in key.iter().zip(query_key.iter()) {
            let d = a - b;
            sq += d * d;
        }
        distances.push((sq.sqrt(), idx));
    }
    distances
}

/// Make `ranking[from..from + count]` the closest `count` of `ranking[from..]`, ascending; the rest stays unordered.
pub fn order_closest(ranking: &mut [(f32, usize)], from: usize, count: usize) {
    let start = from.min(ranking.len());
    let rest = &mut ranking[start..];
    let count = count.min(rest.len());
    if count == 0 {
        return;
    }
    let by_distance = |a: &(f32, usize), b: &(f32, usize)| a.0.total_cmp(&b.0);
    if count < rest.len() {
        rest.select_nth_unstable_by(count - 1, by_distance);
    }
    rest[..count].sort_unstable_by(by_distance);
}

/// Lazily counts the separate places a scan matches, scoring candidates in ring-key order, at most `cap` of them.
pub struct LookAlikes {
    ranking: Vec<(f32, usize)>,
    ordered: usize,
    scored: usize,
    places: Vec<usize>,
    cap: usize,
}

impl LookAlikes {
    pub fn new(ranking: Vec<(f32, usize)>, cap: usize) -> LookAlikes {
        LookAlikes {
            ranking,
            ordered: 0,
            scored: 0,
            places: Vec::new(),
            cap,
        }
    }
    /// Places counted so far (at least one: the scan's own match).
    pub fn places(&self) -> usize {
        self.places.len().max(1)
    }
    /// Count until more than `limit` places are known or the candidates or `cap` run out; returns the places counted.
    pub fn count_past(
        &mut self,
        limit: f64,
        query: &Descriptor,
        descriptors: &crate::store::Appended<Arc<Descriptor>>,
        match_threshold: f64,
        same_place: impl Fn(usize, usize) -> bool,
    ) -> usize {
        let end = self.cap.min(self.ranking.len());
        while self.places.len() as f64 <= limit && self.scored < end {
            if self.scored == self.ordered {
                // order the next batch, as large as everything scored so far
                let batch = self.ordered.max(1);
                order_closest(&mut self.ranking, self.ordered, batch);
                self.ordered += batch;
            }
            let idx = self.ranking[self.scored].1;
            self.scored += 1;
            if (best_distance(query, &descriptors[idx]).0 as f64) < match_threshold
                && self.places.iter().all(|&place| !same_place(place, idx))
            {
                self.places.push(idx);
            }
        }
        self.places()
    }
}

impl crate::gsc_pgo::GscPgo {
    /// The newest keyframe a closure may close onto: its submap must leave out the source's own scan.
    pub(crate) fn newest_closure_target(&self) -> usize {
        (self.key_poses.len() - 1)
            .saturating_sub(self.config.target_half_range(1) + 1)
    }
    /// The nearest earlier closure target within `closure_search_radius`.
    pub(crate) fn search_by_position(&self) -> Option<usize> {
        let here = self.key_poses.last().unwrap().translation_global;
        (0..self.newest_closure_target())
            .map(|k| mat3::norm(&mat3::sub(&self.key_poses.position(k), &here)))
            .enumerate()
            .filter(|&(_, d)| d <= self.config.closure_search_radius)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }
    /// Ring-key rank earlier keyframes, score the closest `candidate_top_k` on the full descriptor.
    pub(crate) fn search_by_scan_context(&self) -> ScanContextMatch {
        let (mut best_dist, mut second_dist) = (2.0f32, 2.0f32);
        let Some(query) = self
            .scan_context_descriptors
            .last()
            .filter(|d| !d.is_empty())
        else {
            return (None, 0, best_dist, second_dist, Vec::new(), Vec::new());
        };
        let mut ranked = ring_key_distances(
            self.scan_context_ring_keys.last().unwrap(),
            &self.scan_context_ring_keys[..self.newest_closure_target()],
        );
        let top_k = self.scan_context_config.candidate_top_k;
        order_closest(&mut ranked, 0, top_k);
        let mut best_dist_filtered = self.scan_context_config.match_threshold as f32;
        let mut best: Option<usize> = None;
        let mut best_shift = 0i32;
        let mut matches = Vec::new();
        for &(_, idx) in ranked.iter().take(top_k) {
            let (distance, shift) = best_distance(query, &self.scan_context_descriptors[idx]);
            if (distance as f64) < self.scan_context_config.match_threshold {
                matches.push((distance, idx, shift));
            }
            if distance < best_dist {
                second_dist = best_dist; // demote previous best
                best_dist = distance;
            } else if distance < second_dist {
                second_dist = distance;
            }
            if distance < best_dist_filtered {
                best_dist_filtered = distance;
                best = Some(idx);
                best_shift = shift;
            }
        }
        matches.retain(|&(_, idx, _)| Some(idx) != best);
        matches.sort_by(|a, b| a.0.total_cmp(&b.0));
        (best, best_shift, best_dist, second_dist, ranked, matches)
    }
}

/// (accepted target, shift, best/2nd cosine distance (2 = none), ring-key ranking, other matches (dist, idx, shift)).
pub(crate) type ScanContextMatch = (Option<usize>, i32, f32, f32, Vec<(f32, usize)>, Vec<(f32, usize, i32)>);
