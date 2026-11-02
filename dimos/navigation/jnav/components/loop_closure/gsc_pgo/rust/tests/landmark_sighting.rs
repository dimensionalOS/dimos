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

//! Landmark sightings (`solve_with_priors`): a drifting square path with no loop closures and no world positions,
//! seeing a few landmarks by bearing with only a rough range (its uncertainty half the range).

use dimos_gsc_pgo::big_picture_fit::{
    bearing_and_range, solve_with_priors, Edge, LandmarkSighting, SolveConfig,
};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const N: usize = 160;

/// A 40 m square, a keyframe every metre: the true (yaw, position) of each.
fn truth() -> Vec<(f64, Vec3)> {
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
fn noise(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453).rem_euclid(1.0) * 2.0 - 1.0
}

/// Odometry along `truth` with errors as large as it reports, and its dead-reckoned poses.
fn odometry(truth: &[(f64, Vec3)]) -> (Vec<Edge>, Vec<Mat3>, Vec<Vec3>) {
    let (heading_sigma, step_sigma) = (0.02, 0.03);
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

#[test]
fn bearings_with_rough_ranges_pull_a_drifting_path_back() {
    let truth = truth();
    let (edges, rotations, positions) = odometry(&truth);
    let drifted = (0..N)
        .map(|k| mat3::norm(&mat3::sub(&positions[k], &truth[k].1)))
        .fold(0.0, f64::max);
    // three posts outside the square, each seen from every keyframe within 35 m: a bearing good to 0.01 rad, a range
    // guessed (from its apparent size, say) to within half of itself
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
            sightings.push(LandmarkSighting {
                keyframe: k,
                landmark,
                position,
                covariance,
            });
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
        pcm_odometry_translation_variance: 0.0,
    };
    let solution = solve_with_priors(
        &rotations,
        &positions,
        &free,
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
    assert!(drifted > 3.0, "the odometry drifts ({drifted} m)");
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
fn bearing_and_range_is_tight_across_the_ray_and_loose_along_it() {
    let (point, covariance) = bearing_and_range(&[0.0, 2.0, 0.0], 100.0, 1e-4, 2500.0);
    assert!(mat3::norm(&mat3::sub(&point, &[0.0, 100.0, 0.0])) < 1e-9);
    // along the ray: the range's variance; across it: (range x bearing sigma)^2 = 1 m^2
    assert!((covariance[1][1] - 2500.0).abs() < 1e-9);
    assert!((covariance[0][0] - 1.0).abs() < 1e-9 && (covariance[2][2] - 1.0).abs() < 1e-9);
    assert!(covariance[0][1].abs() < 1e-9);
}
