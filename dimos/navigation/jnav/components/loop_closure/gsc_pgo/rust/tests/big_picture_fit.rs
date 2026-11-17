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

use dimos_gsc_pgo::big_picture_fit::{consistent_closures, solve, Edge, Solution, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const SIDE: usize = 40;

type Poses = Vec<(Mat3, Vec3)>;

/// A closed square walk of 4 * SIDE 1 m steps: true poses, and odometry over-turning each keyframe by `yaw_bias`.
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
            evidence_core: false,
            learned_evidence: true,
            seed_closure_weights: Vec::new(),
            incumbent_closure_weights: Vec::new(),
            seed_start: Vec::new(),
            planar: false,
            landmark_warm_start_xy_variance_scale: 0.0,
            landmark_warm_start_xy_variance_ladder: 0.0,
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

/// On a closure-free chain, the solver's relative position covariance must equal the summed per-step odometry noise.
#[test]
fn relative_covariance_of_a_chain_is_its_odometry_error_estimate() {
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
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        incumbent_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: false,
        landmark_warm_start_xy_variance_scale: 0.0,
        landmark_warm_start_xy_variance_ladder: 0.0,
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

/// The square plus `revisit` steps back along its first side, odometry yaw random-walking `yaw_sd` rad per keyframe.
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

/// Solves the revisited square with 5 true closures plus 9 look-alikes that agree with the drifted odometry.
fn look_alike_square() -> (Vec<f64>, f64, f64) {
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
            evidence_core: false,
            learned_evidence: true,
            seed_closure_weights: Vec::new(),
            incumbent_closure_weights: Vec::new(),
            seed_start: Vec::new(),
            planar: false,
            landmark_warm_start_xy_variance_scale: 0.0,
            landmark_warm_start_xy_variance_ladder: 0.0,
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
    let (weights, drifted, solved) = look_alike_square();
    eprintln!("{weights:?} drifted {drifted} solved {solved}");
    assert_eq!(weights, [vec![0.0; 5], vec![1.0; 9]].concat());
    assert!(
        solved > 0.9 * drifted,
        "the revisit should stay drifted: {solved} of {drifted}"
    );
}

/// Two laps of a circle with +-1 deg/step yaw drift on lap two: two true closures agree, a misplaced one does not.
#[test]
fn closures_agree_around_a_drifting_cycle() {
    let (steps_per_lap, radius) = (200, 20.0);
    let turn = std::f64::consts::TAU / steps_per_lap as f64;
    let chord = 2.0 * radius * (turn / 2.0).sin();
    let drift = 1.0_f64.to_radians();
    let (yaw_variance, xy_variance) = (drift * drift, 1e-6);
    let truth = |k: usize| -> (Mat3, Vec3) {
        let yaw = k as f64 * turn;
        (
            mat3::rot_z(yaw),
            [radius * yaw.sin(), radius * (1.0 - yaw.cos()), 0.0],
        )
    };
    let odometry: Vec<Edge> = (0..2 * steps_per_lap)
        .map(|k| Edge {
            from: k,
            to: k + 1,
            // +-drift from a fixed hash, on the second lap only
            rotation: mat3::rot_z(
                turn + if k >= steps_per_lap {
                    drift
                        * ((k as f64 * 12.9898).sin() * 43758.5453)
                            .rem_euclid(1.0)
                            .mul_add(2.0, -1.0)
                            .signum()
                } else {
                    0.0
                },
            ),
            translation: mat3::mat_vec(&mat3::rot_z(turn / 2.0), &[chord, 0.0, 0.0]),
            variances: [
                1e-8,
                1e-8,
                yaw_variance,
                xy_variance,
                xy_variance,
                xy_variance,
            ],
        })
        .collect();
    let closure = |from: usize, to: usize, offset: Vec3| {
        let (a, b) = (truth(from), truth(to));
        let back = mat3::transpose(&a.0);
        Edge {
            from,
            to,
            rotation: mat3::mat_mul(&back, &b.0),
            translation: mat3::add(&mat3::mat_vec(&back, &mat3::sub(&b.1, &a.1)), &offset),
            variances: [1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4],
        }
    };
    let closures = [
        closure(50, 50 + steps_per_lap, [0.0; 3]),
        closure(120, 120 + steps_per_lap, [0.0; 3]),
        closure(150, 150 + steps_per_lap, [8.0, 0.0, 0.0]),
    ];
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.99,
        huber_k: 0.0,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        incumbent_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: false,
        landmark_warm_start_xy_variance_scale: 0.0,
        landmark_warm_start_xy_variance_ladder: 0.0,
    };
    assert_eq!(
        consistent_closures(&closures, &odometry, &config),
        vec![true, true, false]
    );
    // at a hundredth of the yaw variance the drift is out of reach, so the true pair disagrees too
    let tight: Vec<Edge> = odometry
        .iter()
        .map(|e| Edge {
            variances: [
                1e-8,
                1e-8,
                yaw_variance / 100.0,
                xy_variance,
                xy_variance,
                xy_variance,
            ],
            ..*e
        })
        .collect();
    assert_eq!(
        consistent_closures(&closures, &tight, &config),
        vec![false, false, false]
    );
}

/// A square over-turning `yaw_bias` per keyframe, closed by `loop_closures` agreeing closures (keyframe i to its mirror
/// near the end), plus `false_count` false closures between random keyframes, each off by up to 2.5 m: (odometry poses,
/// odometry edges, closures, true poses).
fn collapsing_loop(yaw_bias: f64, loop_closures: usize, false_count: usize) -> (Poses, Vec<Edge>, Vec<Edge>, Poses) {
    let (truth, odometry) = square(yaw_bias);
    let last = truth.len() - 1;
    let odometry_edges: Vec<Edge> = (1..=last)
        .map(|k| between(&odometry, k - 1, k, [1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
        .collect();
    let mut closures: Vec<Edge> = (0..loop_closures).map(|i| between(&truth, i, last - i, [1e-3; 6])).collect();
    let mut state: u64 = 12345;
    let mut uniform = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    while closures.len() < loop_closures + false_count {
        let (a, b) = ((uniform() * last as f64) as usize, (uniform() * last as f64) as usize);
        let mut false_closure = between(&truth, a.min(b), a.max(b), [1e-3; 6]);
        false_closure.translation = [5.0 * (uniform() - 0.5), 5.0 * (uniform() - 0.5), 0.0];
        if a.abs_diff(b) >= 10 {
            closures.push(false_closure);
        }
    }
    (odometry, odometry_edges, closures, truth)
}

fn collapse_config(incumbent: Vec<f64>, seed_start: Poses) -> SolveConfig {
    SolveConfig {
        gnc_variance_scale: 10.0,
        gnc_inlier_probability: 0.95,
        huber_k: 1.345,
        evidence_core: false,
        learned_evidence: false,
        seed_closure_weights: Vec::new(),
        incumbent_closure_weights: incumbent,
        seed_start,
        planar: false,
        landmark_warm_start_xy_variance_scale: 0.0,
        landmark_warm_start_xy_variance_ladder: 0.0,
    }
}

/// Solve from the odometry with the accepted graph = truth and the incumbent keeping the first `kept` closures.
fn solve_collapsing(odometry: &Poses, odometry_edges: &[Edge], closures: &[Edge], truth: &Poses, kept: usize) -> Solution {
    let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = odometry.iter().copied().unzip();
    let free: Vec<bool> = (0..odometry.len()).map(|k| k > 0).collect();
    let mut incumbent = vec![1.0; kept];
    if kept > 0 {
        incumbent.resize(closures.len(), f64::NAN);
    }
    solve(&rotations, &positions, &free, odometry_edges, closures, &collapse_config(incumbent, truth.clone()))
}

#[test]
fn gnc_from_the_odometry_collapses_a_loop_the_incumbent_keeps() {
    // 300 false closures (none fits the accepted graph, so GNC starts from the odometry) make GNC drop the loop too
    let (odometry, odometry_edges, closures, truth) = collapsing_loop(0.002, 6, 300);
    let alone = solve_collapsing(&odometry, &odometry_edges, &closures, &truth, 0);
    assert!(alone.closure_weights.iter().all(|&w| w == 0.0), "GNC alone should collapse the loop");
    // the incumbent kept the 6 on the accepted graph: its re-solve has the lower truncated cost and wins
    let challenged = solve_collapsing(&odometry, &odometry_edges, &closures, &truth, 6);
    assert_eq!(challenged.closure_weights[..6], [1.0; 6]);
    assert!(challenged.closure_weights[6..].iter().all(|&w| w == 0.0));
    let closed_end = mat3::norm(&challenged.positions[odometry.len() - 1]);
    assert!(closed_end < 0.05, "the kept loop should close the end onto the start: {closed_end}");
}

#[test]
fn an_incumbent_loop_the_odometry_cannot_bend_to_loses_to_gnc() {
    // a drift the stated odometry error estimate calls too unlikely to close: rejecting the loop is GNC's right answer
    let (odometry, odometry_edges, closures, truth) = collapsing_loop(0.004, 4, 1);
    let challenged = solve_collapsing(&odometry, &odometry_edges, &closures, &truth, 4);
    assert!(challenged.closure_weights.iter().all(|&w| w == 0.0));
}

#[test]
fn an_incumbent_false_closure_loses_to_gnc() {
    let (odometry, odometry_edges, closures, _) = collapsing_loop(0.0005, 1, 0);
    let (rotations, positions): (Vec<Mat3>, Vec<Vec3>) = odometry.iter().copied().unzip();
    let free: Vec<bool> = (0..odometry.len()).map(|k| k > 0).collect();
    // a false closure the incumbent kept: a far corner claimed to be the start, on a graph bent to fit it
    let mut false_closure = between(&odometry, 0, 2 * SIDE, [1e-3; 6]);
    false_closure.translation = [0.0, 0.5, 0.0];
    let edges = vec![false_closure, closures[0]];
    let bent = solve(&rotations, &positions, &free, &odometry_edges, &edges[..1], &collapse_config(Vec::new(), Vec::new()));
    let bent_start: Poses = bent.rotations.iter().copied().zip(bent.positions.iter().copied()).collect();
    let challenged = solve(&rotations, &positions, &free, &odometry_edges, &edges, &collapse_config(vec![1.0, f64::NAN], bent_start));
    assert_eq!(challenged.closure_weights, vec![0.0, 1.0], "the true closure in, the false one out");
}
