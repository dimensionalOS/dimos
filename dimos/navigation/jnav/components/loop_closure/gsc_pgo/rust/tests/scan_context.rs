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

//! Scan Context invariants the loop-closure search relies on: sector-shift
//! equivariance under yaw rotation, ring-key rotation invariance, and
//! near-zero self-match distance at the correct shift.

use std::f32::consts::PI as PI32;
use std::f64::consts::PI;

use dimos_gsc_pgo::scan_context::{
    self, best_distance, column_cosine_distance, descriptor_occupancy, descriptor_structure,
    make_descriptor, make_ring_key, ring_key_top_k, yaw_from_shift, Config,
};

/// A synthetic structured scene: one point per chosen (ring, sector) cell,
/// placed at the bin center with a height that varies per cell so the
/// descriptor has real structure (not a flat field).
fn synthetic_cloud(config: &Config) -> Vec<[f32; 3]> {
    let ring_step = config.max_range_m / config.n_rings as f64;
    let sector_step = 2.0 * PI / config.n_sectors as f64;
    let mut points = Vec::new();
    for ring in (0..config.n_rings).step_by(2) {
        for sector in (0..config.n_sectors).step_by(3) {
            let range = (ring as f64 + 0.5) * ring_step;
            let azimuth = (sector as f64 + 0.5) * sector_step;
            // Distinct per-cell height, well above -lidar_height.
            let z = 0.5 + 0.13 * ring as f64 + 0.07 * sector as f64;
            points.push([
                (range * azimuth.cos()) as f32,
                (range * azimuth.sin()) as f32,
                z as f32,
            ]);
        }
    }
    points
}

/// Rotate a cloud about +z by `yaw` radians.
fn rotate_cloud(points: &[[f32; 3]], yaw: f32) -> Vec<[f32; 3]> {
    let (sin, cos) = yaw.sin_cos();
    points
        .iter()
        .map(|p| [p[0] * cos - p[1] * sin, p[0] * sin + p[1] * cos, p[2]])
        .collect()
}

#[test]
fn rotation_by_k_sectors_shifts_the_descriptor_columns() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let descriptor = make_descriptor(&cloud, &config);

    let k = 7usize;
    let sector_step = 2.0 * PI32 / config.n_sectors as f32;
    let rotated = make_descriptor(&rotate_cloud(&cloud, k as f32 * sector_step), &config);

    // A point at azimuth a moves to a + k*step, so original cell [i, j]
    // lands at [i, (j + k) % n_sectors]. z is untouched by a yaw rotation,
    // so occupied cells match exactly (bin centers keep points off edges).
    for ring in 0..config.n_rings {
        for sector in 0..config.n_sectors {
            let expected = descriptor.at(ring, sector);
            let got = rotated.at(ring, (sector + k) % config.n_sectors);
            assert!(
                (expected - got).abs() < 1e-5,
                "cell ({ring}, {sector}) should shift by {k} columns: {expected} vs {got}"
            );
        }
    }
}

#[test]
fn ring_key_is_invariant_under_rotation() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let key = make_ring_key(&make_descriptor(&cloud, &config));

    let sector_step = 2.0 * PI32 / config.n_sectors as f32;
    let rotated_key = make_ring_key(&make_descriptor(
        &rotate_cloud(&cloud, 13.0 * sector_step),
        &config,
    ));

    assert_eq!(key.len(), config.n_rings);
    for (ring, (a, b)) in key.iter().zip(rotated_key.iter()).enumerate() {
        assert!(
            (a - b).abs() < 1e-5,
            "ring {ring} mean changed under rotation: {a} vs {b}"
        );
    }
    // The key carries actual signal (occupied rings have positive means).
    assert!(key.iter().any(|&v| v > 0.0));
}

#[test]
fn self_match_distance_is_zero_at_the_right_shift() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let descriptor = make_descriptor(&cloud, &config);

    // Identity: distance 0 at shift 0.
    let (distance, shift) = best_distance(&descriptor, &descriptor);
    assert!(distance < 1e-6, "self distance: {distance}");
    assert_eq!(shift, 0);

    // Rotated by k sectors: candidate.col(j + k) == query.col(j), so
    // column_cosine_distance(query, candidate, k) compares identical
    // columns -> best shift is exactly k, distance ~0, and yaw_from_shift
    // recovers the (negated, wrapped) rotation.
    let k = 11i32;
    let sector_step = 2.0 * PI32 / config.n_sectors as f32;
    let rotated = make_descriptor(&rotate_cloud(&cloud, k as f32 * sector_step), &config);
    let (distance, shift) = best_distance(&descriptor, &rotated);
    assert!(distance < 1e-4, "rotated self-match distance: {distance}");
    assert_eq!(shift, k);
    let yaw = yaw_from_shift(shift, config.n_sectors);
    let expected_yaw = -(k as f64) * 2.0 * PI / config.n_sectors as f64;
    assert!(
        (yaw - expected_yaw).abs() < 1e-9,
        "yaw {yaw} vs {expected_yaw}"
    );

    // And a wrong shift is clearly worse than the right one.
    let wrong = column_cosine_distance(&descriptor, &rotated, k + 17);
    assert!(
        wrong > distance + 0.05,
        "wrong shift should score worse: {wrong} vs {distance}"
    );
}

#[test]
fn structure_occupancy_and_edge_cases() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let descriptor = make_descriptor(&cloud, &config);

    // One point per chosen cell -> occupancy equals the number of cells hit.
    let expected_cells = config.n_rings.div_ceil(2) * config.n_sectors.div_ceil(3);
    assert_eq!(descriptor_occupancy(&descriptor), expected_cells);
    assert!(
        descriptor_structure(&descriptor) > 0.0,
        "varied heights -> positive std"
    );

    // Out-of-range and at-origin points are ignored.
    let ignored = make_descriptor(
        &[[100.0, 0.0, 1.0], [0.0, 0.0, 1.0], [80.0, 0.0, 1.0]],
        &config,
    );
    assert_eq!(descriptor_occupancy(&ignored), 0);

    // Empty descriptor: no valid columns -> distance 2, structure 0.
    let empty = make_descriptor(&[], &config);
    assert_eq!(descriptor_structure(&empty), 0.0);
    let (distance, shift) = best_distance(&empty, &empty);
    assert_eq!(distance, 2.0);
    assert_eq!(shift, 0);

    // Points below the sensor clip to 0 (stay "unoccupied"), matching the
    // C++ shifted-z clamp.
    let below = make_descriptor(&[[10.0, 0.0, -5.0]], &config);
    assert_eq!(descriptor_occupancy(&below), 0);

    // A 0x0 descriptor (cloud-less node) never matches anything.
    let none = scan_context::Descriptor::empty();
    assert!(none.is_empty());
    assert_eq!(column_cosine_distance(&descriptor, &none, 0), 2.0);
}

#[test]
fn ring_key_top_k_ranks_by_l2_distance() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let query = make_ring_key(&make_descriptor(&cloud, &config));

    let sector_step = 2.0 * PI32 / config.n_sectors as f32;
    // Candidate 0: same place rotated (ring key ~identical). Candidate 1: a
    // different scene (taller, sparser). Candidate 2: cloud-less (skipped).
    let rotated = make_ring_key(&make_descriptor(
        &rotate_cloud(&cloud, 5.0 * sector_step),
        &config,
    ));
    let other_cloud: Vec<[f32; 3]> = (0..40)
        .map(|i| [1.0 + i as f32, 0.5, 3.0 + 0.1 * i as f32])
        .collect();
    let other = make_ring_key(&make_descriptor(&other_cloud, &config));
    let candidates = vec![rotated, other, Vec::new()];

    let ranked = ring_key_top_k(&query, &candidates, 10);
    assert_eq!(ranked.len(), 2, "empty ring keys are skipped");
    assert_eq!(ranked[0].1, 0, "the rotated revisit ranks first");
    assert!(
        ranked[0].0 < 1e-4,
        "revisit ring-key distance ~0: {}",
        ranked[0].0
    );
    assert!(ranked[1].0 > ranked[0].0);

    let top_1 = ring_key_top_k(&query, &candidates, 1);
    assert_eq!(top_1.len(), 1);
    assert_eq!(top_1[0].1, 0);
}
