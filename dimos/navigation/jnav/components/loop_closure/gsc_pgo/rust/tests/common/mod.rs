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

//! Shared synthetic trajectories and deterministic noise for the integration tests.
#![allow(dead_code)]

use dimos_gsc_pgo::big_picture_fit::Edge;
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

pub const N: usize = 160;

/// True (yaw, position) of each keyframe on a 40 m square, 1 m apart.
pub fn truth() -> Vec<(f64, Vec3)> {
    (0..N)
        .map(|k| {
            let (side, along) = (k / 40, (k % 40) as f64);
            let yaw = side as f64 * std::f64::consts::FRAC_PI_2;
            let c = [[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]][side];
            (
                yaw,
                [c[0] + along * yaw.cos(), c[1] + along * yaw.sin(), 0.0],
            )
        })
        .collect()
}

/// Deterministic noise uniform on [-1, 1).
pub fn noise(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453).rem_euclid(1.0) * 2.0 - 1.0
}

/// Odometry along `truth` with errors matching its stated variances, and its dead-reckoned poses.
pub fn odometry(truth: &[(f64, Vec3)]) -> (Vec<Edge>, Vec<Mat3>, Vec<Vec3>) {
    let (heading_sigma, step_sigma) = (0.02, 0.03);
    // uniform on [-sqrt3 sigma, sqrt3 sigma] has standard deviation sigma
    let u = |k: usize, salt: f64, sigma: f64| noise(k, salt) * 3f64.sqrt() * sigma;
    let edges: Vec<Edge> = (1..N)
        .map(|k| {
            let ((yaw_a, a), (yaw_b, b)) = (truth[k - 1], truth[k]);
            let step = mat3::mat_vec(&mat3::transpose(&mat3::rot_z(yaw_a)), &mat3::sub(&b, &a));
            Edge {
                from: k - 1,
                to: k,
                rotation: mat3::rot_z(yaw_b - yaw_a + u(k, 1.0, heading_sigma)),
                translation: [
                    step[0] + u(k, 2.0, step_sigma),
                    step[1] + u(k, 3.0, step_sigma),
                    step[2],
                ],
                variances: [
                    1e-6,
                    1e-6,
                    heading_sigma.powi(2),
                    step_sigma.powi(2),
                    step_sigma.powi(2),
                    1e-6,
                ],
            }
        })
        .collect();
    let (mut rotations, mut positions) = (vec![mat3::identity()], vec![[0.0; 3]]);
    for e in &edges {
        let (r, p) = (rotations[e.from], positions[e.from]);
        rotations.push(mat3::mat_mul(&r, &e.rotation));
        positions.push(mat3::add(&p, &mat3::mat_vec(&r, &e.translation)));
    }
    (edges, rotations, positions)
}

/// Deterministic uniform on (0, 1).
pub fn uniform(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453)
        .rem_euclid(1.0)
        .max(1e-12)
}

/// Deterministic standard normal (Box-Muller).
pub fn normal(k: usize, salt: f64) -> f64 {
    (-2.0 * uniform(k, salt).ln()).sqrt() * (std::f64::consts::TAU * uniform(k, salt + 0.5)).cos()
}
