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

//! Scan Context invariants: yaw shifts descriptor columns, ring keys ignore yaw, self-match is ~0 at the right shift.

use std::f32::consts::PI as PI32;
use std::f64::consts::PI;

use dimos_gsc_pgo::similarity_discovery::{
    self, auto_max_range, best_distance, column_cosine_distance, descriptor_occupancy,
    make_descriptor, make_ring_key, yaw_from_shift, Config,
};

/// One point at the center of every 2nd ring / 3rd sector cell, with a per-cell height.
fn synthetic_cloud(config: &Config) -> Vec<[f32; 3]> {
    let ring_step = config.max_range_m / config.n_rings as f64;
    let sector_step = 2.0 * PI / config.n_sectors as f64;
    let mut points = Vec::new();
    for ring in (0..config.n_rings).step_by(2) {
        for sector in (0..config.n_sectors).step_by(3) {
            let range = (ring as f64 + 0.5) * ring_step;
            let azimuth = (sector as f64 + 0.5) * sector_step;
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

    // cell [i, j] lands at [i, (j + k) % n_sectors]; bin-centered points keep it exact
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
    // the key is not all zeros
    assert!(key.iter().any(|&v| v > 0.0));
}

#[test]
fn self_match_distance_is_zero_at_the_right_shift() {
    let config = Config::default();
    let cloud = synthetic_cloud(&config);
    let descriptor = make_descriptor(&cloud, &config);

    let (distance, shift) = best_distance(&descriptor, &descriptor);
    assert!(distance < 1e-6, "self distance: {distance}");
    assert_eq!(shift, 0);

    // rotated by k sectors: best shift is k, distance ~0, and yaw_from_shift gives the negated rotation
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

    let expected_cells = config.n_rings.div_ceil(2) * config.n_sectors.div_ceil(3);
    assert_eq!(descriptor_occupancy(&descriptor), expected_cells);

    // out-of-range and at-origin points are ignored
    let ignored = make_descriptor(
        &[[100.0, 0.0, 1.0], [0.0, 0.0, 1.0], [80.0, 0.0, 1.0]],
        &config,
    );
    assert_eq!(descriptor_occupancy(&ignored), 0);

    // empty descriptor: no valid columns -> distance 2
    let empty = make_descriptor(&[], &config);
    let (distance, shift) = best_distance(&empty, &empty);
    assert_eq!(distance, 2.0);
    assert_eq!(shift, 0);

    // points below the sensor clip to 0 (unoccupied)
    let below = make_descriptor(&[[10.0, 0.0, -5.0]], &config);
    assert_eq!(descriptor_occupancy(&below), 0);

    // a 0x0 descriptor (cloud-less node) never matches anything
    let none = similarity_discovery::Descriptor::empty();
    assert!(none.is_empty());
    assert_eq!(column_cosine_distance(&descriptor, &none, 0), 2.0);
}

#[test]
fn auto_max_range_tracks_the_cloud_extent_and_ignores_outliers() {
    // a ~1-2 m cloud plus one 500 m outlier: the 95th-percentile range follows the bulk
    let mut cloud: Vec<[f32; 3]> = (0..100)
        .map(|i| {
            let range = 1.0 + 0.01 * i as f32; // 1.0 .. ~2.0 m
            [range, 0.0, 0.2]
        })
        .collect();
    cloud.push([500.0, 0.0, 0.2]);

    let auto = auto_max_range(&cloud);
    assert!(
        (1.5..=2.5).contains(&auto),
        "auto range follows the ~2 m bulk, not the 500 m outlier: {auto}"
    );

    // empty / origin-only clouds -> 0
    assert_eq!(auto_max_range(&[]), 0.0);
    assert_eq!(auto_max_range(&[[0.0, 0.0, 1.0]]), 0.0);
}
