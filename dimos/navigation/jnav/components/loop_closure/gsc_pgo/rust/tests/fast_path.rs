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

//! The solver fast path on a small drifting loop: its correction lands before GNC's and moves the loop toward GNC's answer,
//! and GNC's adopted solve supersedes it (the final graph matches a run without the fast path).

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

/// Mean distance of keyframes `range` from the truth.
fn error(pgo: &GscPgo, range: std::ops::Range<usize>) -> f64 {
    let count = range.len() as f64;
    range.map(|k| mat3::norm(&mat3::sub(&pgo.key_poses().position(k), &poses(k, 0.0).0 .1))).sum::<f64>() / count
}

/// Drive the loop, close it newest -> 0, then hold still; returns the graph each scan's `update` left.
fn run(fast_path: bool, drift: f64) -> (GscPgo, Vec<(bool, f64)>) {
    let mut config = Config {
        mode: Mode::Deterministic,
        solver_fast_path_enabled: fast_path,
        odometry_error_estimate_from_lidar_range: false,
        graph_change_heat_enabled: false,
        // keyframes without clouds
        auto_aggregate_cloud_size: 0,
        drop_reseen_points: false,
        ..Config::default()
    };
    // GNC held 30 scans, the fast path one
    config.deterministic_config.solve_scans_scale = 30.0 / (KEYFRAMES as f64).powf(config.deterministic_config.solve_keyframe_power) / 2f64.powf(config.deterministic_config.solve_closure_power);
    config.deterministic_config.fast_path_scans_fixed_overhead = 0.5;
    let mut pgo = GscPgo::new(config);
    pgo.set_wait_for_due_jobs(true);
    for k in 0..KEYFRAMES {
        scan(&mut pgo, k, poses(k, drift).1);
        pgo.update();
    }
    assert_eq!(pgo.key_poses().len(), KEYFRAMES);
    // the true pose of the newest keyframe in keyframe 0's frame
    let ((r0, p0), (r1, p1)) = (poses(0, drift).0, poses(KEYFRAMES - 1, drift).0);
    let back = mat3::transpose(&r0);
    pgo.commit_closure(KEYFRAMES - 1, 0, mat3::mat_mul(&back, &r1), mat3::mat_vec(&back, &mat3::sub(&p1, &p0)), 0.01);
    let mut trace = Vec::new();
    for s in KEYFRAMES..KEYFRAMES + 60 {
        scan(&mut pgo, s, poses(KEYFRAMES - 1, drift).1);
        pgo.wait_for_due_solve();
        trace.push((pgo.committed_closures().weight(0).is_nan(), error(&pgo, 0..KEYFRAMES)));
    }
    (pgo, trace)
}

/// The largest distance between two graphs' keyframes.
fn largest_difference(a: &GscPgo, b: &GscPgo) -> f64 {
    (0..KEYFRAMES)
        .map(|k| mat3::norm(&mat3::sub(&a.key_poses().position(k), &b.key_poses().position(k))))
        .fold(0.0, f64::max)
}

#[test]
fn the_fast_path_corrects_a_small_loop_before_gnc_and_gnc_supersedes_it() {
    let (without, slow) = run(false, 0.1);
    let (with, fast) = run(true, 0.1);
    let drifted = slow[0].1;
    assert!(drifted > 0.1, "the odometry drifted {drifted:.3} m");
    // GNC lands at the same scan either way; before it, only the fast path has corrected the loop
    let before_gnc = slow.iter().take_while(|(unjudged, _)| *unjudged).count();
    assert!(before_gnc >= 10, "GNC landed after {before_gnc} scans");
    assert_eq!(fast.iter().take_while(|(unjudged, _)| *unjudged).count(), before_gnc);
    assert_eq!(slow[before_gnc - 1].1, drifted, "no correction before GNC without the fast path");
    let corrected = fast[before_gnc - 1].1;
    assert!(corrected < drifted, "fast path: {corrected:.3} m from the truth, the odometry {drifted:.3} m");
    let latency = &with.closure_latency()[0];
    assert!(latency.fast_path.unwrap().0 + 10 <= latency.gnc.unwrap().0, "{latency:?}");
    assert_eq!(with.committed_closures().weight(0), 1.0, "GNC kept the closure");
    // the fast path's window lands nearly where GNC then puts it (the same loop, without GNC's iterations)
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "adopted");
    let mean = |a: &[Vec3], b: &[Vec3]| {
        a.iter().zip(b).map(|(a, b)| mat3::norm(&mat3::sub(a, b))).sum::<f64>() / a.len() as f64
    };
    let (before, after) = (mean(&record.before, &record.gnc), mean(&record.after, &record.gnc));
    assert!(after < 0.1 * before, "from GNC's answer: {before:.3} m before the fast path, {after:.3} m after");
    // superseded: the final graph is GNC's, as without the fast path
    let largest = largest_difference(&with, &without);
    assert!(largest < 1e-3, "final graphs differ by {largest:.2e} m");
}

#[test]
fn the_fast_path_leaves_a_closure_gnc_would_weigh_to_gnc() {
    // the drift is far past the odometry error estimate: the closure is over GNC's cutoff (GNC rejects it)
    let (without, slow) = run(false, 0.5);
    let (with, fast) = run(true, 0.5);
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "over_cutoff");
    assert!(record.worst_over_cutoff > 1.0);
    assert_eq!(with.committed_closures().weight(0), 0.0, "GNC rejected the closure");
    assert!(fast.iter().all(|&(_, error)| (error - slow[0].1).abs() < 1e-6), "the loop was never bent");
    assert!(largest_difference(&with, &without) < 1e-6);
}
