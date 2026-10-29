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

//! The pose graph on a drifted square: a true closure closes it, a false one is voted out.

use dimos_gsc_pgo::big_picture_fit::{solve, Edge, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const SIDE: usize = 40;

type Poses = Vec<(Mat3, Vec3)>;

/// A square walk of 4 * SIDE keyframes, 1 m apart, back to the start: the true poses, and the
/// odometry's, which over-turns every keyframe by `yaw_bias`.
fn square(yaw_bias: f64) -> (Poses, Poses) {
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
fn a_true_closure_closes_the_drift_and_a_false_one_is_rejected() {
    let (truth, odometry) = square(0.0005);
    let last = truth.len() - 1;
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, [1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
        .collect();
    // the true closure: the walk ends where it started; the false one claims a far corner is the start
    let true_closure = between(&truth, 0, last, [1e-3; 6]);
    let mut false_closure = between(&truth, 0, 2 * SIDE, [1e-3; 6]);
    false_closure.translation = [0.0, 0.5, 0.0];
    let drifted_end = mat3::norm(&odometry[last].1);
    let solution = solve(
        &odometry.iter().map(|p| p.0).collect::<Vec<_>>(),
        &odometry.iter().map(|p| p.1).collect::<Vec<_>>(),
        &(0..odometry.len()).map(|k| k > 0).collect::<Vec<_>>(),
        &odometry_edges,
        &[true_closure, false_closure],
        &SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
            pcm_odometry_translation_variance: 0.0,
            gnc_overtrust_check: false,
            overtrust_noise_headroom: [1.0; 4],
            gravity: Vec::new(),
            committed: Vec::new(),
        },
    );
    assert_eq!(solution.closure_weights, vec![1.0, 0.0]);
    let closed_end = mat3::norm(&solution.positions[last]);
    assert!(
        drifted_end > 1.0,
        "the odometry should drift: {drifted_end}"
    );
    assert!(
        closed_end < 0.05,
        "the closure should close the end onto the start: {closed_end}"
    );
}

/// On an odometry chain (no closures) a keyframe's position uncertainty relative to the newest one is the chain's noise
/// between them: each step's translation noise, plus each step's rotation noise swinging the newest keyframe by its lever
/// arm. The solver's Takahashi readout must give the same.
#[test]
fn relative_covariance_of_a_chain_is_its_odometry_noise() {
    let (translation_variance, rotation_variances) = (1e-4, [1e-6, 1e-6, 3e-6]);
    let n = 30;
    let (mut rotations, mut positions) = (Vec::new(), Vec::new());
    let (mut yaw, mut position) = (0.0f64, [0.0; 3]);
    for _ in 0..n {
        rotations.push(mat3::rot_z(yaw));
        positions.push(position);
        position = mat3::add(
            &position,
            &mat3::mat_vec(&mat3::rot_z(yaw), &[1.0, 0.0, 0.1]),
        );
        yaw += 0.1;
    }
    let odometry: Vec<Edge> = (0..n - 1)
        .map(|i| {
            let back = mat3::transpose(&rotations[i]);
            Edge {
                from: i,
                to: i + 1,
                rotation: mat3::mat_mul(&back, &rotations[i + 1]),
                translation: mat3::mat_vec(&back, &mat3::sub(&positions[i + 1], &positions[i])),
                variances: [
                    rotation_variances[0],
                    rotation_variances[1],
                    rotation_variances[2],
                    translation_variance,
                    translation_variance,
                    translation_variance,
                ],
            }
        })
        .collect();
    let free: Vec<bool> = (0..n).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 10.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        pcm_odometry_translation_variance: 0.0,
        gnc_overtrust_check: false,
        overtrust_noise_headroom: [1.0; 4],
        gravity: Vec::new(),
        committed: Vec::new(),
    };
    let solution = solve(&rotations, &positions, &free, &odometry, &[], &config);
    assert_eq!(solution.relative_position_covariance.len(), n);
    let newest = positions[n - 1];
    let sandwich = |a: &Mat3, d: [f64; 3]| -> Mat3 {
        std::array::from_fn(|r| {
            std::array::from_fn(|c| (0..3).map(|k| a[r][k] * d[k] * a[c][k]).sum())
        })
    };
    for j in 0..n {
        let mut expected = [[0.0; 3]; 3];
        for i in j..n - 1 {
            let translation = sandwich(&rotations[i], [translation_variance; 3]);
            // the step's rotation noise, in the turned-to keyframe's axes, swings the newest one about it
            let lever = mat3::skew(&mat3::sub(&newest, &positions[i + 1]));
            let rotation = mat3::mat_mul(&lever, &rotations[i + 1]);
            let swing = sandwich(&rotation, rotation_variances);
            for r in 0..3 {
                for c in 0..3 {
                    expected[r][c] += translation[r][c] + swing[r][c];
                }
            }
        }
        let got = solution.relative_position_covariance[j];
        for r in 0..3 {
            for c in 0..3 {
                let scale = expected[r][r].abs().max(expected[c][c].abs()).max(1e-12);
                assert!(
                    (got[r][c] - expected[r][c]).abs() <= 1e-3 * scale,
                    "keyframe {j} [{r}][{c}]: solver {} vs chain {}",
                    got[r][c],
                    expected[r][c]
                );
            }
        }
    }
}

/// A square walk of 4 * SIDE keyframes plus `revisit` more along its first side again, whose odometry's yaw takes a
/// random walk of `yaw_sd` rad per keyframe (a fixed seed): the true poses and the odometry's.
fn revisited_square(yaw_sd: f64, revisit: usize) -> (Poses, Poses) {
    let mut seed = 0x2545F4914F6CDD1Du64;
    let mut gauss = || {
        let mut uniform = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (a, b) = (uniform().max(1e-12), uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
    };
    let (mut truth, mut odometry) = (Vec::new(), Vec::new());
    let (mut true_yaw, mut drift_yaw) = (0.0f64, 0.0f64);
    let (mut true_xy, mut drift_xy) = ([0.0; 3], [0.0; 3]);
    for k in 0..4 * SIDE + revisit + 1 {
        truth.push((mat3::rot_z(true_yaw), true_xy));
        odometry.push((mat3::rot_z(drift_yaw), drift_xy));
        let turn = if (k + 1) % SIDE == 0 && k < 4 * SIDE {
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
        drift_yaw += turn + yaw_sd * gauss();
    }
    (truth, odometry)
}

/// The revisited square with overtrusted odometry: its stated yaw variance is 100x smaller than the random walk it
/// really takes, so the five true closures (the revisit seeing the first side again, all agreeing) are tens of sigmas
/// from where the dead-reckoned graph puts them, and a false closure sits among them. GNC-TLS starts near mu = 0, where
/// it weighs every closure at about cutoff x sqrt(mu) / residual against the always-in odometry, so the odometry wins
/// and every closure is voted out. With the overtrust check, GNC-GM (started at least squares, where the agreeing
/// closures together outweigh the odometry) keeps them and not the false one, its odometry residuals show the stated
/// noise far too small, so its verdict is taken and the revisit lands on the first side.
fn overtrusted_square(gnc_overtrust_check: bool) -> (Vec<f64>, f64, f64) {
    // 3 mrad of yaw per keyframe, five revisited keyframes, closures good to 1 cm, the yaw variance stated 100x too small
    let (yaw_sd, revisit, closure_var, overtrust) = (0.003, 5, 1e-4, 100.0);
    let (truth, odometry) = revisited_square(yaw_sd, revisit);
    let last = truth.len() - 1;
    let overtrusted = [1e-8, 1e-8, yaw_sd * yaw_sd / overtrust, 1e-6, 1e-6, 1e-6];
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, overtrusted))
        .collect();
    let mut closures: Vec<Edge> = (0..revisit)
        .map(|j| between(&truth, j, 4 * SIDE + j, [closure_var; 6]))
        .collect();
    let mut false_closure = between(&truth, SIDE, 3 * SIDE, [closure_var; 6]);
    false_closure.translation = [0.0, 0.5, 0.0];
    closures.push(false_closure);
    let solution = solve(
        &odometry.iter().map(|p| p.0).collect::<Vec<_>>(),
        &odometry.iter().map(|p| p.1).collect::<Vec<_>>(),
        &(0..odometry.len()).map(|k| k > 0).collect::<Vec<_>>(),
        &odometry_edges,
        &closures,
        &SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
            pcm_odometry_translation_variance: 0.0,
            gnc_overtrust_check,
            overtrust_noise_headroom: [1.0; 4],
            gravity: Vec::new(),
            committed: Vec::new(),
        },
    );
    let gap = |positions: &[Vec3]| mat3::norm(&mat3::sub(&positions[4 * SIDE], &positions[0]));
    (
        solution.closure_weights,
        gap(&odometry.iter().map(|p| p.1).collect::<Vec<_>>()),
        gap(&solution.positions),
    )
}

#[test]
fn overtrusted_odometry_votes_out_every_true_closure_under_gnc_tls_alone() {
    let (weights, drifted, solved) = overtrusted_square(false);
    assert!(
        drifted > 0.2,
        "the odometry should drift well past the closures' 1 cm: {drifted}"
    );
    assert_eq!(
        weights,
        vec![0.0; 6],
        "TLS should lose the true closures to the overtrusted odometry"
    );
    assert!(
        solved > 0.9 * drifted,
        "the revisit should stay off: {solved} of {drifted}"
    );
}

#[test]
fn overtrusted_odometry_keeps_agreeing_true_closures_under_gnc_overtrust_check() {
    let (weights, drifted, solved) = overtrusted_square(true);
    assert_eq!(weights, [vec![1.0; 5], vec![0.0]].concat());
    assert!(
        solved < 0.25 * drifted,
        "the revisit should land on the first side: {solved} of {drifted}"
    );
}

/// The overtrust check leaves an honest odometry's verdict to TLS: the drifted square's true closure is kept and the
/// false one is not, with the check on as without it.
#[test]
fn the_overtrust_check_keeps_tls_verdict_on_an_honest_odometry() {
    let (truth, odometry) = square(0.0005);
    let last = truth.len() - 1;
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, [1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
        .collect();
    let true_closure = between(&truth, 0, last, [1e-3; 6]);
    let mut false_closure = between(&truth, 0, 2 * SIDE, [1e-3; 6]);
    false_closure.translation = [0.0, 0.5, 0.0];
    let solve_with = |gnc_overtrust_check: bool| {
        solve(
            &odometry.iter().map(|p| p.0).collect::<Vec<_>>(),
            &odometry.iter().map(|p| p.1).collect::<Vec<_>>(),
            &(0..odometry.len()).map(|k| k > 0).collect::<Vec<_>>(),
            &odometry_edges,
            &[true_closure, false_closure],
            &SolveConfig {
                gnc_variance_scale: 10.0,
                gnc_inlier_probability: 0.95,
                huber_k: 1.345,
                pcm_odometry_translation_variance: 0.0,
                gnc_overtrust_check,
                overtrust_noise_headroom: [1.0; 4],
                gravity: Vec::new(),
                committed: Vec::new(),
            },
        )
    };
    let (without, with) = (solve_with(false), solve_with(true));
    assert_eq!(with.closure_weights, vec![1.0, 0.0]);
    assert_eq!(with.closure_weights, without.closure_weights);
    assert!(mat3::norm(&with.positions[last]) < 0.05);
}

/// The revisited square with its five true closures committed (the previous solve kept them), then nine look-alikes
/// arrive that agree with each other and with the dead-reckoned graph (they claim the revisit is where the drifted
/// odometry puts it, as a burst of wrong matches dragged along by the drift does). A cold solve, starting from the
/// odometry, keeps the nine and drops the true five (`cold`); held as commitments, the five stay kept, the look-alikes,
/// which contradict them, are voted out and the revisit lands on the first side (`closure_commitments`).
fn committed_square(committed: bool) -> (Vec<f64>, f64, f64) {
    let (yaw_sd, revisit, closure_var) = (0.003, 5, 1e-4);
    let (truth, odometry) = revisited_square(yaw_sd, revisit);
    let last = truth.len() - 1;
    let honest = [1e-8, 1e-8, yaw_sd * yaw_sd, 1e-6, 1e-6, 1e-6];
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, honest))
        .collect();
    let mut closures: Vec<Edge> = (0..revisit)
        .map(|j| between(&truth, j, 4 * SIDE + j, [closure_var; 6]))
        .collect();
    closures
        .extend((0..9).map(|j| between(&odometry, j, 4 * SIDE + j % revisit, [closure_var; 6])));
    let solution = solve(
        &odometry.iter().map(|p| p.0).collect::<Vec<_>>(),
        &odometry.iter().map(|p| p.1).collect::<Vec<_>>(),
        &(0..odometry.len()).map(|k| k > 0).collect::<Vec<_>>(),
        &odometry_edges,
        &closures,
        &SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
            pcm_odometry_translation_variance: 0.0,
            gnc_overtrust_check: true,
            overtrust_noise_headroom: [1.0; 4],
            gravity: Vec::new(),
            committed: if committed {
                [vec![true; revisit], vec![false; 9]].concat()
            } else {
                Vec::new()
            },
        },
    );
    let gap = |positions: &[Vec3]| mat3::norm(&mat3::sub(&positions[4 * SIDE], &positions[0]));
    (
        solution.closure_weights,
        gap(&odometry.iter().map(|p| p.1).collect::<Vec<_>>()),
        gap(&solution.positions),
    )
}

#[test]
fn a_cold_solve_follows_a_larger_group_of_look_alikes_that_agree_with_the_drift() {
    let (weights, drifted, solved) = committed_square(false);
    eprintln!("cold {weights:?} drifted {drifted} solved {solved}");
    assert_eq!(weights, [vec![0.0; 5], vec![1.0; 9]].concat());
    assert!(
        solved > 0.9 * drifted,
        "the revisit should stay drifted: {solved} of {drifted}"
    );
}

#[test]
fn committed_closures_hold_against_a_larger_group_of_agreeing_look_alikes() {
    let (weights, drifted, solved) = committed_square(true);
    eprintln!("committed {weights:?} drifted {drifted} solved {solved}");
    assert_eq!(weights, [vec![1.0; 5], vec![0.0; 9]].concat());
    assert!(
        solved < 0.25 * drifted,
        "the revisit should land on the first side: {solved} of {drifted}"
    );
}
