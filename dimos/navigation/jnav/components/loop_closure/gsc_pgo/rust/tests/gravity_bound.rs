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

//! A look-alike floor: a walk that climbs one storey around a square and ends right above its start, with a closure
//! claiming the end is the start. The odometry's per-step tilt noise lets a chain of small tilts sink the end a whole
//! storey, so the solve keeps the look-alike; held to the gravity-referenced tilt (`GravityPrior`) it cannot.

use dimos_gsc_pgo::big_picture_fit::{solve, Edge, GravityPrior, SolveConfig};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const SIDE: usize = 10;
const STOREY: f64 = 3.0;

/// 4 * SIDE level keyframes 1 m apart around a square, climbing STOREY along the second side.
fn spiral() -> Vec<(Mat3, Vec3)> {
    let (mut poses, mut yaw, mut at) = (Vec::new(), 0.0f64, [0.0; 3]);
    for k in 0..4 * SIDE + 1 {
        poses.push((mat3::rot_z(yaw), at));
        let climb = if k / SIDE == 1 {
            STOREY / SIDE as f64
        } else {
            0.0
        };
        at = mat3::add(&at, &mat3::mat_vec(&mat3::rot_z(yaw), &[1.0, 0.0, climb]));
        if (k + 1) % SIDE == 0 {
            yaw += std::f64::consts::FRAC_PI_2;
        }
    }
    poses
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

/// The closure's verdict (1 kept, 0 rejected), with or without the gravity factors at `gravity_variance`.
fn verdict(closure: Edge, gravity_variance: Option<f64>) -> f64 {
    let poses = spiral();
    // a Mid-360 Point-LIO's per-keyframe variances: tilt and yaw loose, z tight
    let odometry: Vec<Edge> = (1..poses.len())
        .map(|k| {
            between(
                &poses,
                k - 1,
                k,
                [5e-4, 5e-4, 1.3e-3, 1.4e-4, 1.4e-4, 2.5e-6],
            )
        })
        .collect();
    let gravity = gravity_variance.map_or_else(Vec::new, |variance| {
        (1..poses.len())
            .map(|k| GravityPrior {
                keyframe: k,
                up: [0.0, 0.0, 1.0],
                body_up: mat3::mat_vec(&mat3::transpose(&poses[k].0), &[0.0, 0.0, 1.0]),
                variance,
            })
            .collect()
    });
    let solution = solve(
        &poses.iter().map(|p| p.0).collect::<Vec<_>>(),
        &poses.iter().map(|p| p.1).collect::<Vec<_>>(),
        &(0..poses.len()).map(|k| k > 0).collect::<Vec<_>>(),
        &odometry,
        &[closure],
        &SolveConfig {
            gnc_variance_scale: 10.0,
            gnc_inlier_probability: 0.95,
            huber_k: 1.345,
            pcm_odometry_translation_variance: 0.0,
            gnc_overtrust_check: false,
            overtrust_noise_headroom: [1.0; 4],
            gravity,
        },
    );
    solution.closure_weights[0]
}

#[test]
fn a_look_alike_floor_is_kept_by_tilting_unless_tilt_is_held_to_gravity() {
    let last = 4 * SIDE;
    let mut look_alike = between(&spiral(), 0, last, [1e-4; 6]);
    look_alike.translation = [0.0, 0.0, 0.0];
    // about the bound small_mid's committed closures give (0.56 degrees)
    let bound = 0.5f64.to_radians().powi(2);
    assert_eq!(
        verdict(look_alike, None),
        1.0,
        "per-step tilt alone should absorb a storey"
    );
    assert_eq!(
        verdict(look_alike, Some(bound)),
        0.0,
        "held to gravity, the storey cannot be absorbed"
    );
    // the true closure (the end is a storey above the start) is kept either way
    let truth = between(&spiral(), 0, last, [1e-4; 6]);
    assert_eq!(verdict(truth, None), 1.0);
    assert_eq!(verdict(truth, Some(bound)), 1.0);
}
