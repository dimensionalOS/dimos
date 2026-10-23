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

//! The pose graph on a drifted square: a true loop closes it, a false one is voted out.

use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};
use dimos_gsc_pgo::pose_graph::{solve, Edge, SolveConfig};

const SIDE: usize = 40;

/// A square walk of 4 * SIDE keyframes, 1 m apart, back to the start: the true poses, and the
/// odometry's, which over-turns every keyframe by `yaw_bias`.
fn square(yaw_bias: f64) -> (Vec<(Mat3, Vec3)>, Vec<(Mat3, Vec3)>) {
    let (mut truth, mut odometry) = (Vec::new(), Vec::new());
    let (mut true_yaw, mut drift_yaw) = (0.0f64, 0.0f64);
    let (mut true_xy, mut drift_xy) = ([0.0; 3], [0.0; 3]);
    for k in 0..4 * SIDE + 1 {
        truth.push((mat3::rot_z(true_yaw), true_xy));
        odometry.push((mat3::rot_z(drift_yaw), drift_xy));
        let turn = if (k + 1) % SIDE == 0 {
            std::f64::consts::FRAC_PI_2
        } else {
            0.0
        };
        true_xy = mat3::add(
            &true_xy,
            &mat3::mat_vec(&mat3::rot_z(true_yaw), &[1.0, 0.0, 0.0]),
        );
        drift_xy = mat3::add(
            &drift_xy,
            &mat3::mat_vec(&mat3::rot_z(drift_yaw), &[1.0, 0.0, 0.0]),
        );
        true_yaw += turn;
        drift_yaw += turn + yaw_bias;
    }
    (truth, odometry)
}

fn between(poses: &[(Mat3, Vec3)], from: usize, to: usize, variances: [f64; 6]) -> Edge {
    let back = mat3::transpose(&poses[from].0);
    Edge {
        from,
        to,
        rotation: mat3::mat_mul(&back, &poses[to].0),
        translation: mat3::mat_vec(&back, &mat3::sub(&poses[to].1, &poses[from].1)),
        variances,
    }
}

#[test]
fn a_true_loop_closes_the_drift_and_a_false_one_is_rejected() {
    let (truth, odometry) = square(0.0005);
    let last = truth.len() - 1;
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, [1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
        .collect();
    // the true loop: the walk ends where it started; the false one claims a far corner is the start
    let true_loop = between(&truth, 0, last, [1e-3; 6]);
    let mut false_loop = between(&truth, 0, 2 * SIDE, [1e-3; 6]);
    false_loop.translation = [0.0, 0.5, 0.0];
    let drifted_end = mat3::norm(&odometry[last].1);
    let solution = solve(
        &odometry.iter().map(|p| p.0).collect::<Vec<_>>(),
        &odometry.iter().map(|p| p.1).collect::<Vec<_>>(),
        &odometry_edges,
        &[true_loop, false_loop],
        &SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
        },
    );
    assert_eq!(solution.loop_weights, vec![1.0, 0.0]);
    let closed_end = mat3::norm(&solution.positions[last]);
    assert!(
        drifted_end > 1.0,
        "the odometry should drift: {drifted_end}"
    );
    assert!(
        closed_end < 0.05,
        "the loop should close the end onto the start: {closed_end}"
    );
}
