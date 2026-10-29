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

//! The odometry noise from a solved graph's residuals (`Solution::odometry_noise_evidence`): odometry whose translation
//! noise is reported 25x too high and heading noise 7x too low, closed by exact loops; the evidence gives each back.

use dimos_gsc_pgo::big_picture_fit::{solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

fn uniform(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453)
        .rem_euclid(1.0)
        .max(1e-12)
}

fn normal(k: usize, salt: f64) -> f64 {
    (-2.0 * uniform(k, salt).ln()).sqrt() * (std::f64::consts::TAU * uniform(k, salt + 0.5)).cos()
}

#[test]
fn residuals_measure_translation_and_heading_noise_apart() {
    // laps of a 20 m square, a keyframe every metre, closed by an exact loop at every corner after the first lap
    let (true_xy, true_yaw) = (4e-6_f64, 2e-5_f64);
    let (reported_xy, reported_yaw) = (true_xy * 25.0, true_yaw / 7.0);
    let n = 400;
    let truth: Vec<(f64, Vec3)> = {
        let mut out = vec![(0.0, [0.0; 3])];
        for k in 1..n {
            let (yaw, p) = out[k - 1];
            let turn = if k % 20 == 0 {
                std::f64::consts::FRAC_PI_2
            } else {
                0.0
            };
            out.push((yaw + turn, mat3::add(&p, &[yaw.cos(), yaw.sin(), 0.0])));
        }
        out
    };
    let odometry: Vec<Edge> = (1..n)
        .map(|k| {
            let ((yaw_a, a), (yaw_b, b)) = (truth[k - 1], truth[k]);
            let step = mat3::mat_vec(&mat3::transpose(&mat3::rot_z(yaw_a)), &mat3::sub(&b, &a));
            Edge {
                from: k - 1,
                to: k,
                rotation: mat3::rot_z(yaw_b - yaw_a + true_yaw.sqrt() * normal(k, 1.0)),
                translation: [
                    step[0] + true_xy.sqrt() * normal(k, 2.0),
                    step[1] + true_xy.sqrt() * normal(k, 3.0),
                    0.0,
                ],
                variances: [1e-8, 1e-8, reported_yaw, reported_xy, reported_xy, 1e-8],
            }
        })
        .collect();
    let (mut rotations, mut positions): (Vec<Mat3>, Vec<Vec3>) =
        (vec![mat3::identity()], vec![[0.0; 3]]);
    for e in &odometry {
        let (r, p) = (rotations[e.from], positions[e.from]);
        rotations.push(mat3::mat_mul(&r, &e.rotation));
        positions.push(mat3::add(&p, &mat3::mat_vec(&r, &e.translation)));
    }
    let closures: Vec<Edge> = (80..n)
        .step_by(20)
        .map(|k| {
            let target = k % 80;
            let ((yaw_t, pt), (yaw_s, ps)) = (truth[target], truth[k]);
            Edge {
                from: target,
                to: k,
                rotation: mat3::rot_z(yaw_s - yaw_t),
                translation: mat3::mat_vec(
                    &mat3::transpose(&mat3::rot_z(yaw_t)),
                    &mat3::sub(&ps, &pt),
                ),
                variances: [1e-8; 6],
            }
        })
        .collect();
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        pcm_odometry_translation_variance: 0.0,
        gnc_overtrust_check: false,
        overtrust_noise_headroom: [1.0; 4],
        gravity: Vec::new(),
    };
    // Helmert's iteration: rescale each source by its ratio and solve again until the ratios settle at 1
    let mut odometry = odometry;
    let mut scale = [1.0_f64; 4];
    let mut evidence = [(0.0, 0.0); 6];
    for _ in 0..20 {
        let solution = solve(&rotations, &positions, &free, &odometry, &closures, &config);
        evidence = solution.odometry_noise_evidence.expect("factored");
        for (source, ks) in [(0, [3, 4].as_slice()), (2, [2].as_slice())] {
            let ratio = evidence[source].0 / evidence[source].1;
            scale[source] *= ratio;
            for e in odometry.iter_mut() {
                for &k in ks {
                    e.variances[k] *= ratio;
                }
            }
        }
    }
    let ratio = |c: usize| scale[c];
    eprintln!(
        "xy: x{:.3} of reported (true 1/25 = 0.04, redundancy {:.1}); yaw: x{:.2} (true 7, redundancy {:.1})",
        ratio(0),
        evidence[0].1,
        ratio(2),
        evidence[2].1
    );
    assert!(
        evidence[0].1 > 1.0 && evidence[2].1 > 1.0,
        "the loops give redundancy"
    );
    assert!((ratio(0) / 0.04).ln().abs() < 0.7, "xy ratio {}", ratio(0));
    assert!((ratio(2) / 7.0).ln().abs() < 0.5, "yaw ratio {}", ratio(2));
}
