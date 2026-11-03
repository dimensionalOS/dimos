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

//! `solve_with_priors` world-position priors on a drifting closure-free square, in world frames rotated/shifted from the map.

use dimos_gsc_pgo::big_picture_fit::{solve_with_priors, PositionPrior, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Vec3};

mod common;
use common::{odometry, truth, N};

/// Fixes of `keyframes` in frame `frame` (the map turned by `yaw` and shifted by `shift`) with `sigma` noise.
fn fixes(
    truth: &[(f64, Vec3)],
    keyframes: impl Iterator<Item = usize>,
    frame: u32,
    yaw: f64,
    shift: Vec3,
    sigma: f64,
) -> Vec<PositionPrior> {
    keyframes
        .enumerate()
        .map(|(i, k)| {
            let salt = (i * 7 + frame as usize * 3) as f64;
            let wobble = [sigma * salt.sin(), sigma * (salt * 1.7).cos(), 0.0];
            PositionPrior {
                keyframe: k,
                frame,
                lever: [0.0; 3],
                world: mat3::add(
                    &mat3::add(&mat3::mat_vec(&mat3::rot_z(yaw), &truth[k].1), &shift),
                    &wobble,
                ),
                covariance: [
                    [sigma * sigma, 0.0, 0.0],
                    [0.0, sigma * sigma, 0.0],
                    [0.0, 0.0, sigma * sigma],
                ],
            }
        })
        .collect()
}

fn config() -> SolveConfig {
    SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        landmark_warm_start_xy_variance_scale: 0.0,
    }
}

#[test]
fn world_positions_pull_a_drifting_path_onto_the_world() {
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    let drifted_end = mat3::norm(&mat3::sub(&positions[N - 1], &truth[N - 1].1));
    // GPS-like: a fix every 10 keyframes, 0.5 m noise, one 30 m off
    let (world_yaw, world_shift) = (0.7, [100.0, 50.0, 3.0]);
    let mut priors = fixes(&truth, (0..N).step_by(10), 0, world_yaw, world_shift, 0.5);
    priors[5].world = mat3::add(&priors[5].world, &[30.0, -20.0, 0.0]);
    let free: Vec<bool> = (0..N).map(|k| k > 0).collect();
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
        &edges,
        &[],
        &priors,
        &[],
        &config(),
    );

    // the alignment itself is not compared with the truth (keyframe 0's heading is uncertain); keyframe placement is
    let &(frame, yaw, shift) = solution
        .world_alignment
        .first()
        .expect("the priors pin the heading");
    assert_eq!(frame, 0);
    assert_eq!(
        solution.prior_weights[5], 0.0,
        "the 30 m outlier is voted out"
    );
    assert_eq!(
        solution.prior_weights.iter().filter(|&&w| w == 1.0).count(),
        priors.len() - 1
    );
    let to_world = |p: &Vec3| mat3::add(&mat3::mat_vec(&mat3::rot_z(world_yaw), p), &world_shift);
    let worst = (0..N)
        .map(|k| {
            let placed = mat3::add(
                &mat3::mat_vec(&mat3::rot_z(yaw), &solution.positions[k]),
                &shift,
            );
            mat3::norm(&mat3::sub(&to_world(&truth[k].1), &placed))
        })
        .fold(0.0, f64::max);
    eprintln!("dead-reckoned end off {drifted_end:.1} m; with world positions worst keyframe off {worst:.2} m");
    assert!(
        drifted_end > 3.0,
        "the odometry error estimates ({drifted_end} m)"
    );
    assert!(
        worst < drifted_end / 2.0,
        "every keyframe much closer to the truth than the odometry (worst {worst} m)"
    );
}

#[test]
fn two_world_frames_each_get_their_own_alignment() {
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    // GPS-like fixes over the whole loop (frame 0), precise mocap over the first side only (frame 1)
    let (gps_yaw, gps_shift) = (0.7, [100.0, 50.0, 3.0]);
    let (mocap_yaw, mocap_shift) = (-1.2, [-20.0, 5.0, 1.0]);
    let mut priors = fixes(&truth, (0..N).step_by(10), 0, gps_yaw, gps_shift, 0.5);
    priors.extend(fixes(
        &truth,
        (0..40).step_by(4),
        1,
        mocap_yaw,
        mocap_shift,
        0.05,
    ));
    let free: Vec<bool> = (0..N).map(|k| k > 0).collect();
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
        &edges,
        &[],
        &priors,
        &[],
        &config(),
    );

    let find = |f: u32| {
        *solution
            .world_alignment
            .iter()
            .find(|a| a.0 == f)
            .unwrap_or_else(|| panic!("frame {f} aligned"))
    };
    let ((_, gps_fit, gps_t), (_, mocap_fit, mocap_t)) = (find(0), find(1));
    // the two frames' relative pose is fully determined by the data: GPS -> mocap
    let wrap = |a: f64| {
        (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    };
    let relative_yaw = mocap_fit - gps_fit;
    let yaw_error = wrap(relative_yaw - (mocap_yaw - gps_yaw));
    // judge translation inside the mocap volume, since at the far-off GPS origin any yaw error is magnified
    let fit_relative_t = mat3::sub(&mocap_t, &mat3::mat_vec(&mat3::rot_z(relative_yaw), &gps_t));
    let inside = truth[20].1;
    let in_gps = mat3::add(&mat3::mat_vec(&mat3::rot_z(gps_yaw), &inside), &gps_shift);
    let carried = mat3::add(
        &mat3::mat_vec(&mat3::rot_z(relative_yaw), &in_gps),
        &fit_relative_t,
    );
    let in_mocap = mat3::add(
        &mat3::mat_vec(&mat3::rot_z(mocap_yaw), &inside),
        &mocap_shift,
    );
    let overlap_error = mat3::norm(&mat3::sub(&carried, &in_mocap));
    eprintln!("GPS -> mocap: yaw error {yaw_error:.4} rad, error inside the mocap volume {overlap_error:.2} m");
    assert!(
        yaw_error.abs() < 0.03,
        "GPS -> mocap yaw off {yaw_error} rad"
    );
    assert!(
        overlap_error < 1.0,
        "GPS -> mocap off {overlap_error} m inside the mocap volume"
    );
    assert!(
        solution.prior_weights.iter().all(|&w| w == 1.0),
        "no fix is an outlier here"
    );
}

#[test]
fn an_off_center_antenna_is_placed_through_its_lever_arm() {
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    let drifted_end = mat3::norm(&mat3::sub(&positions[N - 1], &truth[N - 1].1));
    // the antenna sits 0.6 m forward and 0.3 m up: each fix is where the antenna was, not the body origin
    let lever = [0.6, 0.0, 0.3];
    let (world_yaw, world_shift) = (0.7, [100.0, 50.0, 3.0]);
    let mut priors = fixes(&truth, (0..N).step_by(5), 0, world_yaw, world_shift, 0.1);
    for p in priors.iter_mut() {
        let body_yaw = truth[p.keyframe].0;
        let offset = mat3::mat_vec(&mat3::rot_z(world_yaw + body_yaw), &lever);
        p.world = mat3::add(&p.world, &offset);
        p.lever = lever;
    }
    let free: Vec<bool> = (0..N).map(|k| k > 0).collect();
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
        &edges,
        &[],
        &priors,
        &[],
        &config(),
    );
    let &(_, yaw, shift) = solution
        .world_alignment
        .first()
        .expect("the priors pin the heading");
    assert!(
        solution.prior_weights.iter().all(|&w| w == 1.0),
        "no fix is an outlier"
    );
    let to_world = |p: &Vec3| mat3::add(&mat3::mat_vec(&mat3::rot_z(world_yaw), p), &world_shift);
    let worst = (0..N)
        .map(|k| {
            let placed = mat3::add(
                &mat3::mat_vec(&mat3::rot_z(yaw), &solution.positions[k]),
                &shift,
            );
            mat3::norm(&mat3::sub(&to_world(&truth[k].1), &placed))
        })
        .fold(0.0, f64::max);
    eprintln!("antenna lever arm: dead-reckoned end off {drifted_end:.1} m, worst body origin off {worst:.2} m");
    assert!(
        worst < drifted_end / 2.0,
        "body origins much closer to the truth than the odometry (worst {worst} m)"
    );
}
