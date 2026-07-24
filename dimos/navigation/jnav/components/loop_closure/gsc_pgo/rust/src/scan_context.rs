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

//! Scan Context — polar-binned lidar place-recognition descriptor.
//!
//! Faithful port of gsc_pgo's scan_context.{h,cpp} (itself a self-contained
//! reimplementation inspired by Kim & Kim 2018 and the irapkaist/scancontext
//! reference, MIT). Each scan becomes an (n_rings x n_sectors) matrix where
//! cell [i, j] holds the max z among points falling in that (range, azimuth)
//! bin. The "ring key" — the per-row mean — is the coarse feature used for
//! fast candidate retrieval; the full matrix is then column-shifted against
//! the candidate to measure rotation-invariant cosine distance.
//!
//! Numeric discipline: the C++ stores the descriptor as float and does the
//! cosine / mean arithmetic in float, but bins ranges/azimuths in double and
//! accumulates structure statistics in double. This port keeps the same
//! f32/f64 split so distances match the C++ bit-for-bit-ish.

use std::f64::consts::PI;

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

    /// An empty (0x0) descriptor — what the C++ stores for a node without a
    /// cloud (`Descriptor()` default-constructed).
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
pub type SectorKey = Vec<f32>;

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

/// Mean per column (over ALL rows, zeros included).
pub fn make_sector_key(descriptor: &Descriptor) -> SectorKey {
    let mut key = vec![0.0f32; descriptor.n_sectors];
    if descriptor.n_rings == 0 {
        return key;
    }
    for (j, slot) in key.iter_mut().enumerate() {
        let mut sum = 0.0f32;
        for i in 0..descriptor.n_rings {
            sum += descriptor.at(i, j);
        }
        *slot = sum / descriptor.n_rings as f32;
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

/// "Vertical-structure" score: the standard deviation of the occupied
/// (non-zero) cells. Cells hold max-z + lidar_height, so a flat, feature-poor
/// scene (open grass: every bin a near-ground return) has nearly uniform cell
/// values -> low std, while walls/buildings/poles spread the values -> high
/// std. A degeneracy proxy for "this scan can't reliably place itself".
/// Returns 0 for an empty/degenerate descriptor.
pub fn descriptor_structure(descriptor: &Descriptor) -> f32 {
    // Std of occupied (non-zero) cells. Empty cells are excluded so the
    // metric reflects the spread of actual returns' heights, not how full
    // the FOV is.
    let mut sum = 0.0f64;
    let mut sum_sq = 0.0f64;
    let mut count = 0i64;
    for &value in &descriptor.data {
        if value <= 0.0 {
            continue;
        }
        sum += value as f64;
        sum_sq += value as f64 * value as f64;
        count += 1;
    }
    if count < 2 {
        return 0.0;
    }
    let mean = sum / count as f64;
    let variance = sum_sq / count as f64 - mean * mean;
    if variance > 0.0 {
        variance.sqrt() as f32
    } else {
        0.0
    }
}

/// Count of occupied (non-zero) cells — how much of the polar FOV returned
/// structure out to range. A cheap "how much of the scene has structure"
/// term, complementary to `descriptor_structure` (which only looks at the
/// spread of occupied cells).
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
