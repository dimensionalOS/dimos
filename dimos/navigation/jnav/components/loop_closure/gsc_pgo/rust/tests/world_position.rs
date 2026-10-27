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

//! Absolute position priors (`solve_with_priors`): a drifting square path with no loop closures, and noisy
//! world-frame positions of some keyframes in a frame turned and shifted from the map, one of them wildly wrong.

use dimos_gsc_pgo::big_picture_fit::{solve_with_priors, Edge, PositionPrior, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

#[test]
fn world_positions_pull_a_drifting_path_onto_the_world() {
    // truth: a 40 m square, a keyframe every metre
    let n = 160;
    let truth: Vec<(f64, Vec3)> = (0..n)
        .map(|k| {
            let (side, along) = (k / 40, (k % 40) as f64);
            let yaw = side as f64 * std::f64::consts::FRAC_PI_2;
            let corners = [[0.0, 0.0], [40.0, 0.0], [40.0, 40.0], [0.0, 40.0]];
            let c = corners[side];
            (
                yaw,
                [c[0] + along * yaw.cos(), c[1] + along * yaw.sin(), 0.0],
            )
        })
        .collect();
    // odometry: the true steps with zero-mean errors as large as it reports (deterministic pseudo-random: heading
    // noise ~1.1 deg, 3 cm/step position noise), which random-walk into metres of drift
    let noise = |k: usize, salt: f64| {
        ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453).rem_euclid(1.0) * 2.0 - 1.0
    };
    let (heading_sigma, step_sigma) = (0.02, 0.03);
    let odometry: Vec<Edge> = (1..n)
        .map(|k| {
            let (yaw_a, a) = truth[k - 1];
            let (yaw_b, b) = truth[k];
            let step = mat3::mat_vec(&mat3::transpose(&mat3::rot_z(yaw_a)), &mat3::sub(&b, &a));
            // uniform on [-sqrt3 sigma, sqrt3 sigma] has standard deviation sigma
            let u = |salt: f64, sigma: f64| noise(k, salt) * 3f64.sqrt() * sigma;
            Edge {
                from: k - 1,
                to: k,
                rotation: mat3::rot_z(yaw_b - yaw_a + u(1.0, heading_sigma)),
                translation: [
                    step[0] + u(2.0, step_sigma),
                    step[1] + u(3.0, step_sigma),
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
    // the dead-reckoned start
    let (mut rotations, mut positions): (Vec<Mat3>, Vec<Vec3>) =
        (vec![mat3::identity()], vec![[0.0; 3]]);
    for e in &odometry {
        let (r, p) = (rotations[e.from], positions[e.from]);
        rotations.push(mat3::mat_mul(&r, &e.rotation));
        positions.push(mat3::add(&p, &mat3::mat_vec(&r, &e.translation)));
    }
    let drifted_end = mat3::norm(&mat3::sub(&positions[n - 1], &truth[n - 1].1));
    // the world frame: the map turned 0.7 rad and shifted; a fix every 10 keyframes, 0.5 m noise (deterministic), one 30 m off
    let (world_yaw, world_shift) = (0.7, [100.0, 50.0, 3.0]);
    let to_world = |p: &Vec3| mat3::add(&mat3::mat_vec(&mat3::rot_z(world_yaw), p), &world_shift);
    let sigma = 0.5;
    let priors: Vec<PositionPrior> = (0..n)
        .step_by(10)
        .enumerate()
        .map(|(i, k)| {
            let wobble = [
                sigma * ((i * 7) as f64).sin(),
                sigma * ((i * 13) as f64).cos(),
                0.0,
            ];
            let outlier = if i == 5 { [30.0, -20.0, 0.0] } else { [0.0; 3] };
            PositionPrior {
                keyframe: k,
                world: mat3::add(&mat3::add(&to_world(&truth[k].1), &wobble), &outlier),
                covariance: [
                    [sigma * sigma, 0.0, 0.0],
                    [0.0, sigma * sigma, 0.0],
                    [0.0, 0.0, sigma * sigma],
                ],
            }
        })
        .collect();
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        pcm_odometry_translation_variance: 0.0,
    };
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
        &odometry,
        &[],
        &priors,
        &config,
    );

    // the map -> world alignment is read at keyframe 0, whose own heading the biased odometry leaves uncertain, so it is
    // not compared with the truth; where every keyframe lands in the world is
    let (yaw, shift) = solution
        .world_alignment
        .expect("the priors pin the heading");
    assert_eq!(
        solution.prior_weights[5], 0.0,
        "the 30 m outlier is voted out"
    );
    assert_eq!(
        solution.prior_weights.iter().filter(|&&w| w == 1.0).count(),
        priors.len() - 1
    );
    let worst = (0..n)
        .map(|k| {
            mat3::norm(&mat3::sub(
                &to_world(&truth[k].1),
                &mat3::add(
                    &mat3::mat_vec(&mat3::rot_z(yaw), &solution.positions[k]),
                    &shift,
                ),
            ))
        })
        .fold(0.0, f64::max);
    eprintln!("dead-reckoned end off {drifted_end:.1} m; with world positions worst keyframe off {worst:.2} m");
    assert!(drifted_end > 3.0, "the odometry drifts ({drifted_end} m)");
    assert!(
        worst < drifted_end / 2.0,
        "every keyframe much closer to the truth than the odometry (worst {worst} m)"
    );
}
