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

//! The odometry noise scale from closure innovations (`odometry_noise_ln_scale_ml`).

use dimos_gsc_pgo::gsc_pgo::odometry_noise_ln_scale_ml;
use dimos_gsc_pgo::mat3::{Mat3, Vec3};

/// Deterministic uniform on (0, 1).
fn uniform(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453)
        .rem_euclid(1.0)
        .max(1e-12)
}

/// Deterministic standard normal (Box-Muller).
fn normal(k: usize, salt: f64) -> f64 {
    (-2.0 * uniform(k, salt).ln()).sqrt() * (std::f64::consts::TAU * uniform(k, salt + 0.5)).cos()
}

/// Closures over loops of different lengths: the graph's covariance P grows with the loop, the closure's own variance
/// is small, and the innovations are drawn with the odometry `true_scale` times noisier than reported.
fn innovations(true_scale: f64, count: usize) -> Vec<(Vec3, Mat3, f64)> {
    (0..count)
        .map(|k| {
            let length = 10.0 + 10.0 * k as f64;
            // horizontal drift grows with the loop, vertical much less
            let p = [
                [1e-3 * length, 0.0, 0.0],
                [0.0, 1e-3 * length, 0.0],
                [0.0, 0.0, 1e-4 * length],
            ];
            let c = 1e-3;
            let v =
                std::array::from_fn(|i| (true_scale * p[i][i] + c).sqrt() * normal(k * 3 + i, 1.0));
            (v, p, c)
        })
        .collect()
}

#[test]
fn the_scale_that_explains_the_closures_is_found() {
    for true_scale in [0.25, 1.0, 4.0, 30.0] {
        let found = odometry_noise_ln_scale_ml(&innovations(true_scale, 60))
            .expect("innovations given")
            .exp();
        // 60 three-dimensional innovations: the ln scale's standard error is sqrt(2 / 180) ~ 0.11
        eprintln!("true scale {true_scale}: found {found:.3}");
        assert!(
            (found / true_scale).ln().abs() < 0.35,
            "found {found} for {true_scale}"
        );
    }
}

#[test]
fn no_closures_no_scale() {
    assert!(odometry_noise_ln_scale_ml(&[]).is_none());
}
