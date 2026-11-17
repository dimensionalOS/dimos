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

//! The graph change heat cools with data time: a solve it holds is adopted while the robot stands still (no new keyframes).

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, Mode, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const KEYFRAMES: usize = 40;

/// Keyframe k's true pose on a 5 m circle, and its odometry pose (heading drifting `drift` deg a step).
fn poses(k: usize, drift: f64) -> ((Mat3, Vec3), (Mat3, Vec3)) {
    let step = std::f64::consts::TAU / KEYFRAMES as f64;
    let truth = |k: usize| {
        let a = step * k as f64;
        (mat3::rot_z(a + std::f64::consts::FRAC_PI_2), [5.0 * a.cos(), 5.0 * a.sin(), 0.0])
    };
    let (mut rotation, mut position) = truth(0);
    for j in 1..=k {
        let ((r0, p0), (r1, p1)) = (truth(j - 1), truth(j));
        let local = mat3::mat_vec(&mat3::transpose(&r0), &mat3::sub(&p1, &p0));
        let turn = mat3::mat_mul(&mat3::transpose(&r0), &r1);
        position = mat3::add(&position, &mat3::mat_vec(&rotation, &local));
        rotation = mat3::mat_mul(&rotation, &mat3::mat_mul(&turn, &mat3::rot_z(drift.to_radians())));
    }
    (truth(k), (rotation, position))
}

fn scan(pgo: &mut GscPgo, scan: usize, (rotation, translation): (Mat3, Vec3)) {
    let mut pose = PoseWithTime::new(rotation, translation);
    pose.set_time((scan / 10) as i32, (scan % 10) as u32 * 100_000_000);
    pgo.add_key_pose(&CloudWithPose { cloud: None, pose });
}

/// The true relative pose from keyframe `to` to keyframe `from`, as a closure.
fn commit_true_closure(pgo: &mut GscPgo, from: usize, to: usize) {
    let ((r0, p0), (r1, p1)) = (poses(to, 0.0).0, poses(from, 0.0).0);
    let back = mat3::transpose(&r0);
    pgo.commit_closure(from, to, mat3::mat_mul(&back, &r1), mat3::mat_vec(&back, &mat3::sub(&p1, &p0)), 0.01);
}

#[test]
fn a_solve_the_heat_holds_is_adopted_while_the_robot_stands_still() {
    let mut pgo = GscPgo::new(Config {
        mode: Mode::Deterministic,
        odometry_error_estimate_from_lidar_range: false,
        graph_change_heat_enabled: true,
        // every move costs heat and any heat holds the next solve, cooling 1000x in 10 s of data
        graph_change_heat_floor: 0.0,
        graph_change_heat_threshold: 1e-9,
        graph_change_heat_half_life_s: 1.0,
        auto_aggregate_cloud_size: 0,
        drop_reseen_points: false,
        ..Config::default()
    });
    pgo.set_wait_for_due_jobs(true);
    let drift = 0.1;
    for k in 0..KEYFRAMES {
        scan(&mut pgo, k, poses(k, drift).1);
        pgo.update();
    }
    let still = poses(KEYFRAMES - 1, drift).1;
    // the first loop's solve finds the heat cold, is adopted, and heats it
    commit_true_closure(&mut pgo, KEYFRAMES - 1, 0);
    let mut s = KEYFRAMES;
    while pgo.committed_closures().weight(0).is_nan() {
        scan(&mut pgo, s, still);
        pgo.wait_for_due_solve();
        s += 1;
        assert!(s < KEYFRAMES + 200, "the first solve was never adopted");
    }
    let first_adopted = s;
    // the second closure's solve finishes while the heat is up; standing still, no keyframe arrives to cool it by
    commit_true_closure(&mut pgo, KEYFRAMES - 2, 1);
    while pgo.committed_closures().weight(1).is_nan() {
        scan(&mut pgo, s, still);
        pgo.wait_for_due_solve();
        s += 1;
        assert!(s < first_adopted + 600, "the held solve was never adopted (heat never cooled standing still)");
    }
    assert_eq!(pgo.key_poses().len(), KEYFRAMES, "no keyframe arrived while standing still");
    assert!(s > first_adopted + 10, "the second solve was held for a while ({} scans)", s - first_adopted);
}
