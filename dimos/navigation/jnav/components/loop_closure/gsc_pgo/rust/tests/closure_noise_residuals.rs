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

//! With closures stated 30x too loose, learning odometry noise alone underestimates it; learning closures too fixes both.

use dimos_gsc_pgo::big_picture_fit::{solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

mod common;
use common::normal;

/// Laps of a 20 m square (1 m keyframes), closures back one lap; returns odometry, dead reckoning and closures.
fn graph(closure_reported_over_true: f64) -> (Vec<Edge>, Vec<Mat3>, Vec<Vec3>, Vec<Edge>) {
    let (xy, yaw, closure) = (1e-4_f64, 1e-5_f64, 1e-4_f64);
    let n = 400;
    let truth: Vec<(f64, Vec3)> = {
        let mut out = vec![(0.0, [0.0; 3])];
        for k in 1..n {
            let (heading, p) = out[k - 1];
            let turn = if k % 20 == 0 {
                std::f64::consts::FRAC_PI_2
            } else {
                0.0
            };
            out.push((
                heading + turn,
                mat3::add(&p, &[heading.cos(), heading.sin(), 0.0]),
            ));
        }
        out
    };
    let relative = |a: usize, b: usize| {
        let ((ya, pa), (yb, pb)) = (truth[a], truth[b]);
        (
            yb - ya,
            mat3::mat_vec(&mat3::transpose(&mat3::rot_z(ya)), &mat3::sub(&pb, &pa)),
        )
    };
    let odometry: Vec<Edge> = (1..n)
        .map(|k| {
            let (turn, step) = relative(k - 1, k);
            Edge {
                from: k - 1,
                to: k,
                rotation: mat3::rot_z(turn + yaw.sqrt() * normal(k, 1.0)),
                translation: [
                    step[0] + xy.sqrt() * normal(k, 2.0),
                    step[1] + xy.sqrt() * normal(k, 3.0),
                    0.0,
                ],
                variances: [1e-8, 1e-8, yaw, xy, xy, 1e-8],
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
    // closure noise on all six axes, one shared variance per axis kind
    let closures: Vec<Edge> = (80..n)
        .step_by(2)
        .map(|k| {
            let (turn, offset) = relative(k - 80, k);
            let tilt = |stream: f64| (closure * 1e-2).sqrt() * normal(k, stream);
            let tilt_rotation = mat3::mat_from_quat(&{
                let (a, b) = (tilt(7.0) / 2.0, tilt(8.0) / 2.0);
                let n = (1.0 + a * a + b * b).sqrt();
                [1.0 / n, a / n, b / n, 0.0]
            });
            Edge {
                from: k - 80,
                to: k,
                rotation: mat3::mat_mul(&mat3::rot_z(turn + tilt(4.0)), &tilt_rotation),
                translation: [
                    offset[0] + closure.sqrt() * normal(k, 5.0),
                    offset[1] + closure.sqrt() * normal(k, 6.0),
                    offset[2] + closure.sqrt() * normal(k, 9.0),
                ],
                // rotation stated 10x looser than translation
                variances: [10e-2, 10e-2, 10e-2, 1.0, 1.0, 1.0]
                    .map(|f| f * closure * closure_reported_over_true),
            }
        })
        .collect();
    (odometry, rotations, positions, closures)
}

/// Helmert variance-component iteration; returns the learned (odometry xy, odometry yaw, closure rot, closure trans) scales.
fn helmert(with_closures: bool) -> (f64, f64, f64, f64) {
    let (mut odometry, rotations, positions, mut closures) = graph(30.0);
    let free: Vec<bool> = (0..rotations.len()).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: false,
        landmark_warm_start_xy_variance_scale: 0.0,
        landmark_warm_start_xy_variance_ladder: 0.0,
    };
    let mut scale = [1.0_f64; 4];
    for iteration in 0..15 {
        let solution = solve(&rotations, &positions, &free, &odometry, &closures, &config);
        let Some(evidence) = solution.odometry_error_estimate_evidence else {
            panic!("not factored at iteration {iteration}, scales {scale:?}");
        };
        for (group, (source, rows)) in [(0, [3, 4].as_slice()), (2, [2].as_slice())]
            .into_iter()
            .enumerate()
        {
            let ratio = evidence[source].0 / evidence[source].1;
            scale[group] *= ratio;
            for e in odometry.iter_mut() {
                for &k in rows {
                    e.variances[k] *= ratio;
                }
            }
        }
        if with_closures {
            for group in 0..2 {
                let ratio = evidence[4 + group].0 / evidence[4 + group].1;
                scale[2 + group] *= ratio;
                for e in closures.iter_mut() {
                    for k in 3 * group..3 * group + 3 {
                        e.variances[k] *= ratio;
                    }
                }
            }
        }
    }
    (scale[0], scale[1], scale[2], scale[3])
}

#[test]
fn learning_the_closures_too_keeps_the_odometry_honest() {
    let (xy_alone, yaw_alone, ..) = helmert(false);
    let (xy, yaw, rotation, closure) = helmert(true);
    eprintln!(
        "odometry alone: xy x{xy_alone:.3} yaw x{yaw_alone:.3} (true 1); jointly: xy x{xy:.3} yaw x{yaw:.3}, closure \
         rotation x{rotation:.4} (true 1/300), translation x{closure:.4} (true 1/30)"
    );
    assert!(
        xy_alone < 0.5,
        "alone, the odometry looks better than it is (xy x{xy_alone})"
    );
    assert!(
        xy.ln().abs() < 0.5 && yaw.ln().abs() < 0.7,
        "jointly the odometry is right (xy x{xy}, yaw x{yaw})"
    );
    assert!(
        (closure * 30.0).ln().abs() < 0.7,
        "jointly the closures are right (x{closure})"
    );
}
