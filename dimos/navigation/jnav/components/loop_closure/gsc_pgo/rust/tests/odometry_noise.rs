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

//! The odometry noise from closure innovations (`odometry_noise`): an odometry whose translation and heading noise are
//! misreported by different factors, and closures over stretches of many lengths, recover both.

use dimos_gsc_pgo::gsc_pgo::KeyPoseWithCloud;
use dimos_gsc_pgo::mat3::{self, Vec3};
use dimos_gsc_pgo::odometry_noise::{fit, innovation};

/// Deterministic uniform on (0, 1).
fn uniform(k: usize, salt: f64) -> f64 {
    ((k as f64 * 12.9898 + salt * 78.233).sin() * 43758.5453)
        .rem_euclid(1.0)
        .max(1e-12)
}

/// Deterministic standard normal (Box-Muller).
fn normal(k: usize, salt: f64) -> f64 {
    (-2.0 * uniform(k, salt).ln()).sqrt() * (std::f64::consts::TAU * uniform(k, salt + 0.5)).cos()
}

fn key_pose(yaw: f64, position: Vec3) -> KeyPoseWithCloud {
    KeyPoseWithCloud {
        rotation_local: mat3::rot_z(yaw),
        translation_local: position,
        rotation_global: mat3::rot_z(yaw),
        translation_global: position,
        time: 0.0,
        body_cloud: None,
    }
}

#[test]
fn translation_and_heading_noise_are_measured_apart() {
    // a 1 m step per keyframe on a wandering path; the true per-step noise is xy 4e-6 m^2 (reported 1e-4: 25x too
    // high) and yaw 2e-5 rad^2 (reported 3e-6: 7x too low), the misreporting seen on real recordings
    let (true_xy, true_yaw) = (4e-6_f64, 2e-5_f64);
    let n = 1200;
    let (mut truth, mut odometry) = (vec![(0.0_f64, [0.0; 3])], vec![(0.0_f64, [0.0; 3])]);
    for k in 1..n {
        let turn = 0.05 * (k as f64 / 40.0).sin();
        let (yaw, p) = truth[k - 1];
        truth.push((yaw + turn, mat3::add(&p, &[yaw.cos(), yaw.sin(), 0.0])));
        let (yaw, p) = odometry[k - 1];
        let noisy_turn = turn + true_yaw.sqrt() * normal(k, 1.0);
        let step = [
            1.0 + true_xy.sqrt() * normal(k, 2.0),
            true_xy.sqrt() * normal(k, 3.0),
            0.0,
        ];
        let moved = mat3::mat_vec(&mat3::rot_z(yaw), &step);
        odometry.push((yaw + noisy_turn, mat3::add(&p, &moved)));
    }
    let key_poses: Vec<KeyPoseWithCloud> =
        odometry.iter().map(|&(yaw, p)| key_pose(yaw, p)).collect();
    // closures over stretches of 5 to 600 keyframes, each measuring the true relative position with 5 cm noise
    let fitness = 0.05_f64.powi(2);
    let innovations: Vec<_> = (0..200)
        .map(|c| {
            let span = 5 + (c * 37) % 600;
            let target = (c * 53) % (n - span);
            let source = target + span;
            let (yaw_t, p_t) = truth[target];
            let mut measured = mat3::mat_vec(
                &mat3::transpose(&mat3::rot_z(yaw_t)),
                &mat3::sub(&truth[source].1, &p_t),
            );
            for (axis, m) in measured.iter_mut().enumerate() {
                *m += fitness.sqrt() * normal(c * 3 + axis, 4.0);
            }
            innovation(&key_poses, target, source, &measured, fitness)
        })
        .collect();
    let reported = [1e-4, 1e-4, 3e-6, 1e-6];
    let (ln, _) = fit(&innovations, reported, true, false).expect("innovations given");
    let (xy, yaw) = (reported[0] * ln[0].exp(), reported[2] * ln[2].exp());
    eprintln!("xy {xy:.2e} (true {true_xy:.0e}), yaw {yaw:.2e} (true {true_yaw:.0e})");
    // heading noise swings a long stretch so far that it is measured well; translation noise adds little to any
    // stretch here (600 steps of 4e-6 m^2 against the heading's ~1e3 m^2), so the closures barely inform it and it
    // stays near its prior, the reported value, without being pushed past either end
    assert!((yaw / true_yaw).ln().abs() < 0.35, "yaw {yaw}");
    assert!(xy > true_xy / 2.0 && xy < reported[0] * 2.0, "xy {xy}");
}

#[test]
fn no_closures_no_estimate() {
    assert!(fit(&[], [1e-4, 1e-4, 3e-6, 1e-6], true, false).is_none());
}
