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

//! `solve_with_priors` landmark sightings correct a drifting closure-free square: rough-range bearings, or one tag pose.

use dimos_gsc_pgo::big_picture_fit::{
    solve_with_priors, Edge, LandmarkSighting, SolveConfig, Start,
};
use dimos_gsc_pgo::landmarks::known_inverse;
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

mod common;
use common::{noise, odometry, truth, N};

/// A bearing + range as a point and covariance: `range_variance` along the ray, `range² · bearing_variance` across it.
fn bearing_and_range(
    direction: &Vec3,
    range: f64,
    bearing_variance: f64,
    range_variance: f64,
) -> (Vec3, Mat3) {
    let b = mat3::scale(direction, 1.0 / mat3::norm(direction));
    let across = range * range * bearing_variance;
    let covariance = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            let along = b[i] * b[j];
            range_variance * along + across * ((i == j) as u8 as f64 - along)
        })
    });
    (mat3::scale(&b, range), covariance)
}

#[test]
fn bearings_with_rough_ranges_pull_a_drifting_path_back() {
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    let drifted = (0..N)
        .map(|k| mat3::norm(&mat3::sub(&positions[k], &truth[k].1)))
        .fold(0.0, f64::max);
    // three posts seen from every keyframe within 35 m: bearing to 0.01 rad, range only to within half of itself
    let posts: [Vec3; 3] = [[20.0, -10.0, 2.0], [55.0, 25.0, 2.0], [-12.0, 30.0, 2.0]];
    let bearing_sigma: f64 = 0.01;
    let mut sightings = Vec::new();
    for (k, &(yaw, at)) in truth.iter().enumerate() {
        for (landmark, post) in posts.iter().enumerate() {
            let body = mat3::mat_vec(&mat3::transpose(&mat3::rot_z(yaw)), &mat3::sub(post, &at));
            let range = mat3::norm(&body);
            if range > 35.0 {
                continue;
            }
            // the bearing off by up to its sigma about the vertical, the range by up to its sigma
            let off = noise(k * 3 + landmark, 4.0) * bearing_sigma;
            let direction = mat3::mat_vec(&mat3::rot_z(off), &mat3::scale(&body, 1.0 / range));
            let range_sigma = 0.5 * range;
            let guessed = range + noise(k * 3 + landmark, 5.0) * range_sigma;
            let (position, covariance) = bearing_and_range(
                &direction,
                guessed,
                bearing_sigma.powi(2),
                range_sigma.powi(2),
            );
            sightings.push(LandmarkSighting::point(k, landmark, position, covariance));
        }
    }
    // a wrong data association: a sighting of post 0 labelled post 1
    let wrong = sightings
        .iter()
        .position(|s| s.landmark == 0 && s.keyframe > 100)
        .expect("post 0 seen late");
    sightings[wrong].landmark = 1;
    let free: Vec<bool> = (0..N).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: false,
        landmark_warm_start_xy_variance_scale: dimos_gsc_pgo::gsc_pgo::Config::default()
            .landmark_warm_start_xy_variance_scale,
        landmark_warm_start_xy_variance_ladder: dimos_gsc_pgo::gsc_pgo::Config::default()
            .landmark_warm_start_xy_variance_ladder,
    };
    let solution = solve_with_priors(
        Start {
            rotations: &rotations,
            positions: &positions,
            free: &free,
        },
        &edges,
        &[],
        &[],
        &sightings,
        &config,
    );
    let worst = (0..N)
        .map(|k| mat3::norm(&mat3::sub(&solution.positions[k], &truth[k].1)))
        .fold(0.0, f64::max);
    let post_error = posts
        .iter()
        .zip(&solution.landmarks)
        .map(|(p, l)| mat3::norm(&mat3::sub(p, l)))
        .fold(0.0, f64::max);
    eprintln!(
        "{} sightings; dead-reckoned worst {drifted:.2} m, with sightings {worst:.2} m; posts off at most {post_error:.2} m",
        sightings.len()
    );
    assert_eq!(
        solution.sighting_weights[wrong], 0.0,
        "the mislabelled sighting is voted out"
    );
    assert!(
        solution
            .sighting_weights
            .iter()
            .filter(|&&w| w == 1.0)
            .count()
            >= sightings.len() - 2,
        "nearly every sighting kept"
    );
    assert!(drifted > 3.0, "the odometry error estimates ({drifted} m)");
    assert!(
        worst < drifted / 3.0,
        "every keyframe much closer to the truth than the odometry (worst {worst} m)"
    );
    assert!(
        post_error < 1.0,
        "the posts triangulated (off {post_error} m)"
    );
}

#[test]
fn one_tag_seen_at_both_ends_closes_heading_and_position() {
    // a full-pose tag near the start, seen only from the first and last 3 keyframes, pins heading as a point cannot
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    let heading = |r: &Mat3| r[1][0].atan2(r[0][0]);
    let wrap = |a: f64| {
        (a + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
    };
    let last = N - 1;
    let drifted_heading = wrap(heading(&rotations[last]) - truth[last].0).abs();
    let drifted = mat3::norm(&mat3::sub(&positions[last], &truth[last].1));
    let (tag_yaw, tag): (f64, Vec3) = (2.0, [2.0, -3.0, 1.0]);
    let (position_sigma, rotation_sigma) = (0.02_f64, 0.01_f64);
    let mut covariance = [[0.0; 6]; 6];
    for i in 0..3 {
        covariance[i][i] = rotation_sigma.powi(2);
        covariance[3 + i][3 + i] = position_sigma.powi(2);
    }
    let sightings: Vec<LandmarkSighting> = (0..3)
        .chain(last - 2..=last)
        .map(|k| {
            let (yaw, at) = truth[k];
            let back = mat3::transpose(&mat3::rot_z(yaw));
            LandmarkSighting {
                keyframe: k,
                landmark: 0,
                rotation: mat3::rot_z(tag_yaw - yaw + noise(k, 7.0) * rotation_sigma),
                position: mat3::add(
                    &mat3::mat_vec(&back, &mat3::sub(&tag, &at)),
                    &[
                        noise(k, 8.0) * position_sigma,
                        noise(k, 9.0) * position_sigma,
                        0.0,
                    ],
                ),
                information: known_inverse(&covariance),
            }
        })
        .collect();
    let free: Vec<bool> = (0..N).map(|k| k > 0).collect();
    let config = SolveConfig {
        gnc_variance_scale: 1.0,
        gnc_inlier_probability: 0.95,
        huber_k: 0.0,
        evidence_core: false,
        learned_evidence: true,
        seed_closure_weights: Vec::new(),
        seed_start: Vec::new(),
        planar: false,
        landmark_warm_start_xy_variance_scale: dimos_gsc_pgo::gsc_pgo::Config::default()
            .landmark_warm_start_xy_variance_scale,
        landmark_warm_start_xy_variance_ladder: dimos_gsc_pgo::gsc_pgo::Config::default()
            .landmark_warm_start_xy_variance_ladder,
    };
    let solution = solve_with_priors(
        Start {
            rotations: &rotations,
            positions: &positions,
            free: &free,
        },
        &edges,
        &[],
        &[],
        &sightings,
        &config,
    );
    let solved_heading = wrap(heading(&solution.rotations[last]) - truth[last].0).abs();
    let solved = mat3::norm(&mat3::sub(&solution.positions[last], &truth[last].1));
    let tag_heading = wrap(heading(&solution.landmark_rotations[0]) - tag_yaw).abs();
    eprintln!(
        "last keyframe: heading off {drifted_heading:.3} -> {solved_heading:.3} rad, position {drifted:.2} -> {solved:.2} m; \
         tag heading off {tag_heading:.3} rad"
    );
    assert!(
        solution.sighting_weights.iter().all(|&w| w == 1.0),
        "every sighting kept"
    );
    assert!(
        drifted_heading > 0.05 && drifted > 1.0,
        "the odometry error estimates"
    );
    assert!(
        solved_heading < 0.03,
        "heading closed ({solved_heading} rad)"
    );
    assert!(solved < 0.2, "position closed ({solved} m)");
    assert!(
        tag_heading < 0.03,
        "the tag's rotation solved ({tag_heading} rad)"
    );
}

#[test]
fn unknown_dimensions_carry_no_information() {
    // ROS's -1 and an infinite variance are both unknown; the rest inverts on its own
    let mut covariance = [[0.0; 6]; 6];
    covariance[0][0] = -1.0;
    covariance[1][1] = f64::INFINITY;
    covariance[2][2] = 4.0;
    for (i, row) in covariance.iter_mut().enumerate().skip(3) {
        row[i] = 0.25;
    }
    covariance[3][4] = 0.1;
    covariance[4][3] = 0.1;
    let information = known_inverse(&covariance);
    assert!(information[0]
        .iter()
        .chain(&information[1])
        .all(|&v| v == 0.0));
    assert!((information[2][2] - 0.25).abs() < 1e-12);
    let det = 0.25 * 0.25 - 0.1 * 0.1;
    assert!((information[3][3] - 0.25 / det).abs() < 1e-9);
    assert!((information[3][4] + 0.1 / det).abs() < 1e-9);
}

#[test]
fn factors_before_the_first_keyframe_wait_for_it() {
    // a fix or sighting can reach the module before its first scan; there is nothing to solve yet
    let mut pgo = dimos_gsc_pgo::gsc_pgo::GscPgo::new(dimos_gsc_pgo::gsc_pgo::Config::default());
    let point = LandmarkSighting::point(
        0,
        0,
        [1.0, 0.0, 0.0],
        [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]],
    );
    pgo.add_landmark_factor(0.0, 7, point.rotation, point.position, point.information);
    pgo.add_coordinate_factor(
        0.0,
        0,
        [0.0; 3],
        [0.0; 3],
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    );
    assert!(!pgo.update());
    assert!(!pgo.finish());
}

/// A 200 m square walked 1 m a step whose heading drifts steadily, far past what the odometry states, and a point tag by the start seen on the first steps and again on the last, with every sighting's world spread: (with, without) warm start.
fn drifted_loop_tag_spread() -> (f64, f64) {
    let steps = 800;
    let truth: Vec<(f64, Vec3)> = (0..steps)
        .map(|k| {
            let (side, along) = (k / 200, (k % 200) as f64);
            let yaw = side as f64 * std::f64::consts::FRAC_PI_2;
            let c = [[0.0, 0.0], [200.0, 0.0], [200.0, 200.0], [0.0, 200.0]][side];
            (
                yaw,
                [c[0] + along * yaw.cos(), c[1] + along * yaw.sin(), 0.0],
            )
        })
        .collect();
    let (heading_sigma, step_sigma): (f64, f64) = (1e-3, 0.005);
    // the heading's steady drift over the loop: 16x the sd the odometry states for it
    let heading_bias = 16.0 * heading_sigma / (steps as f64).sqrt();
    let edges: Vec<Edge> = (1..steps)
        .map(|k| {
            let ((yaw_a, a), (yaw_b, b)) = (truth[k - 1], truth[k]);
            Edge {
                from: k - 1,
                to: k,
                rotation: mat3::rot_z(yaw_b - yaw_a + heading_bias + noise(k, 1.0) * heading_sigma),
                translation: mat3::mat_vec(
                    &mat3::transpose(&mat3::rot_z(yaw_a)),
                    &mat3::sub(&b, &a),
                ),
                variances: [
                    1e-8,
                    1e-8,
                    heading_sigma.powi(2),
                    step_sigma.powi(2),
                    step_sigma.powi(2),
                    1e-8,
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
    let tag: Vec3 = [3.0, -2.0, 0.5];
    let sigma: f64 = 0.05;
    let seen: Vec<usize> = (0..3).chain(steps - 3..steps).collect();
    let sightings: Vec<LandmarkSighting> = seen
        .iter()
        .map(|&k| {
            let (yaw, at) = truth[k];
            let body = mat3::mat_vec(&mat3::transpose(&mat3::rot_z(yaw)), &mat3::sub(&tag, &at));
            LandmarkSighting::point(
                k,
                0,
                body,
                std::array::from_fn(|i| {
                    std::array::from_fn(|j| (i == j) as u8 as f64 * sigma * sigma)
                }),
            )
        })
        .collect();
    let spread = |scale: f64| {
        let config = SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
            evidence_core: false,
            learned_evidence: false,
            seed_closure_weights: Vec::new(),
            seed_start: Vec::new(),
            planar: false,
            landmark_warm_start_xy_variance_scale: scale,
            landmark_warm_start_xy_variance_ladder: dimos_gsc_pgo::gsc_pgo::Config::default()
                .landmark_warm_start_xy_variance_ladder,
        };
        let free: Vec<bool> = (0..steps).map(|k| k > 0).collect();
        let solution = solve_with_priors(
            Start {
                rotations: &rotations,
                positions: &positions,
                free: &free,
            },
            &edges,
            &[],
            &[],
            &sightings,
            &config,
        );
        let world: Vec<Vec3> = sightings
            .iter()
            .map(|s| {
                mat3::add(
                    &solution.positions[s.keyframe],
                    &mat3::mat_vec(&solution.rotations[s.keyframe], &s.position),
                )
            })
            .collect();
        world
            .iter()
            .flat_map(|a| world.iter().map(move |b| mat3::norm(&mat3::sub(a, b))))
            .fold(0.0, f64::max)
    };
    let default_scale =
        dimos_gsc_pgo::gsc_pgo::Config::default().landmark_warm_start_xy_variance_scale;
    (spread(default_scale), spread(0.0))
}

#[test]
fn a_tag_re_seen_across_a_drifted_loop_closes_it() {
    let (warm, cold) = drifted_loop_tag_spread();
    assert!(
        warm < 0.5,
        "the tag's sightings meet once the loop closes ({warm} m apart)"
    );
    // without the warm start the re-sightings are out of the solve's reach and GNC drops them
    assert!(
        cold > 5.0,
        "a solve from the drifted odometry leaves them {cold} m apart"
    );
}
