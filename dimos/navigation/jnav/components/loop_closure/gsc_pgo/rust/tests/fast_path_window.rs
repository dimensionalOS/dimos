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

//! The fast path on a loop 3000 keyframes long whose odometry drifted over its last `DRIFTING` steps: the closure moves
//! only the robot's own recent trail (as one block, bent in from where it last agreed with the map), so the loop's span
//! does not matter; it is published before GNC lands, the rest of the loop stays exactly where it was, keyframes made
//! after it ride it; a wrong closure is left to GNC (and dropped from a job it shares with a right one); GNC's solve
//! supersedes either way.

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, Mode, PoseWithTime};
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

const KEYFRAMES: usize = 3000;
/// Keyframe spacing (m), as the small-loop fast path test's.
const SPACING: f64 = 0.785;
/// The odometry drifts over the loop's last this many steps (inside the window's free half): the correction is local.
const DRIFTING: usize = 100;

/// Keyframe k's true pose on a circle KEYFRAMES * SPACING around.
fn truth(k: usize) -> (Mat3, Vec3) {
    let step = std::f64::consts::TAU / KEYFRAMES as f64;
    let a = step * k as f64;
    (mat3::rot_z(a + std::f64::consts::FRAC_PI_2), [SPACING / step * a.cos(), SPACING / step * a.sin(), 0.0])
}

fn scan(pgo: &mut GscPgo, scan: usize, (rotation, translation): (Mat3, Vec3)) {
    let mut pose = PoseWithTime::new(rotation, translation);
    pose.set_time((scan / 10) as i32, (scan % 10) as u32 * 100_000_000);
    pgo.add_key_pose(&CloudWithPose { cloud: None, pose });
}

/// Drive the loop, close it newest -> 0, then hold still; per scan, whether GNC has yet to judge the closure and the newest
/// keyframe's published distance from the truth. The closure's rotation is off by `closure_turn` rad (0: the truth).
fn run(window: bool, drift: f64, closure_turn: f64) -> (GscPgo, Vec<(bool, f64)>) {
    let (pgo, trace, _) = run_with(window, drift, closure_turn, false);
    (pgo, trace)
}

/// `run`; with `stop_at_publish`, it stops on the first scan the fast path published on, and also returns the published
/// positions on the scan before it.
fn run_with(window: bool, drift: f64, closure_turn: f64, stop_at_publish: bool) -> (GscPgo, Vec<(bool, f64)>, Vec<Vec3>) {
    run_with_wrong(window, drift, closure_turn, stop_at_publish, false)
}

/// `run_with`; with `wrong_too`, a second closure (keyframe KEYFRAMES - 2 -> 1, turned 150 degrees) is committed with it.
fn run_with_wrong(
    window: bool,
    drift: f64,
    closure_turn: f64,
    stop_at_publish: bool,
    wrong_too: bool,
) -> (GscPgo, Vec<(bool, f64)>, Vec<Vec3>) {
    let mut config = Config {
        mode: Mode::Deterministic,
        solver_fast_path_enabled: window,
        odometry_error_estimate_from_lidar_range: false,
        graph_change_heat_enabled: false,
        auto_aggregate_cloud_size: 0,
        drop_reseen_points: false,
        ..Config::default()
    };
    // GNC held 30 scans (with or without its iterations)
    let dc = &mut config.deterministic_config;
    dc.solve_gnc_scans_scale = 30.0 / (KEYFRAMES as f64).powf(dc.solve_gnc_keyframe_power);
    dc.solve_without_gnc_scans_scale = 30.0 / (KEYFRAMES as f64).powf(dc.solve_without_gnc_keyframe_power);
    let mut pgo = GscPgo::new(config);
    pgo.set_wait_for_due_jobs(true);
    // the true steps, the last DRIFTING each turned `drift` rad further
    let odometry: Vec<(Mat3, Vec3)> = {
        let ((r0, p0), (_, p1)) = (truth(0), truth(1));
        let local = mat3::mat_vec(&mat3::transpose(&r0), &mat3::sub(&p1, &p0));
        let step = std::f64::consts::TAU / KEYFRAMES as f64;
        let (mut rotation, mut position) = truth(0);
        let mut out = Vec::with_capacity(KEYFRAMES);
        for k in 0..KEYFRAMES {
            out.push((rotation, position));
            position = mat3::add(&position, &mat3::mat_vec(&rotation, &local));
            let extra = if k + 1 >= KEYFRAMES - DRIFTING { drift } else { 0.0 };
            rotation = mat3::mat_mul(&rotation, &mat3::rot_z(step + extra));
        }
        out
    };
    for (k, &pose) in odometry.iter().enumerate() {
        scan(&mut pgo, k, pose);
        pgo.update();
    }
    assert_eq!(pgo.key_poses().len(), KEYFRAMES);
    let ((r0, p0), (r1, p1)) = (truth(0), truth(KEYFRAMES - 1));
    let back = mat3::transpose(&r0);
    pgo.commit_closure(KEYFRAMES - 1, 0, mat3::mat_mul(&mat3::mat_mul(&back, &r1), &mat3::rot_z(closure_turn)), mat3::mat_vec(&back, &mat3::sub(&p1, &p0)), 0.01);
    // a second closure that agrees with the truth: the fast path needs two to agree
    let (rs, ps) = truth(KEYFRAMES - 3);
    pgo.commit_closure(KEYFRAMES - 3, 0, mat3::mat_mul(&back, &rs), mat3::mat_vec(&back, &mat3::sub(&ps, &p0)), 0.01);
    if wrong_too {
        let ((ra, pa), (rb, pb)) = (truth(1), truth(KEYFRAMES - 2));
        let back = mat3::transpose(&ra);
        let turn = mat3::rot_z(150f64.to_radians());
        pgo.commit_closure(KEYFRAMES - 2, 1, mat3::mat_mul(&mat3::mat_mul(&back, &rb), &turn), mat3::mat_vec(&back, &mat3::sub(&pb, &pa)), 0.01);
    }
    let mut trace = Vec::new();
    let mut before = Vec::new();
    for s in KEYFRAMES..KEYFRAMES + 60 {
        if stop_at_publish {
            before = (0..KEYFRAMES).map(|k| pgo.key_poses().position(k)).collect();
        }
        scan(&mut pgo, s, odometry[KEYFRAMES - 1]);
        pgo.wait_for_due_solve();
        let newest = mat3::norm(&mat3::sub(&pgo.key_poses().position(KEYFRAMES - 1), &truth(KEYFRAMES - 1).1));
        trace.push((pgo.committed_closures().weight(0).is_nan(), newest));
        if stop_at_publish && pgo.closure_latency()[0].fast_path.is_some() {
            break;
        }
    }
    (pgo, trace, before)
}

fn largest_difference(a: &GscPgo, b: &GscPgo) -> f64 {
    (0..KEYFRAMES)
        .map(|k| mat3::norm(&mat3::sub(&a.key_poses().position(k), &b.key_poses().position(k))))
        .fold(0.0, f64::max)
}

#[test]
fn the_window_publishes_a_small_change_on_a_3000_keyframe_loop_before_gnc() {
    // ~10 cm apart at the closure
    let (drift, turn) = (2.5e-5, 0.0);
    let (without, slow) = run(false, drift, turn);
    let (with, fast) = run(true, drift, turn);
    let drifted = slow[0].1;
    assert!(drifted > 0.05, "the odometry drifted {drifted:.3} m");
    let before_gnc = slow.iter().take_while(|(unjudged, _)| *unjudged).count();
    assert!(before_gnc >= 10, "GNC landed after {before_gnc} scans");
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "adopted", "{record:?}");
    assert_eq!(record.closures, vec![0, 1]);
    // the trail is the robot's own recent keyframes, not the loop's 3000-keyframe span
    assert!(record.free < KEYFRAMES / 4, "{} keyframes freed", record.free);
    let corrected = fast[before_gnc - 1].1;
    assert!(corrected < 0.5 * drifted, "window: {corrected:.3} m from the truth, the odometry {drifted:.3} m");
    let latency = &with.closure_latency()[0];
    assert!(latency.fast_path.unwrap().0 + 10 <= latency.gnc.unwrap().0, "{latency:?}");
    assert_eq!(with.committed_closures().weight(0), 1.0, "GNC kept the closure");
    // published only: GNC's graph is the one without the window, bit for bit
    assert_eq!(largest_difference(&with, &without), 0.0);
}

#[test]
fn the_correction_bends_the_trail_without_a_kink() {
    // ~1 m apart at the closure: the robot's own trail moves back onto the loop's start, without a kink where it meets
    // the held loop behind it
    let (drift, turn) = (2.5e-4, 0.0);
    let (_, slow) = run(false, drift, turn);
    let (with, fast) = run(true, drift, turn);
    let before_gnc = slow.iter().take_while(|(unjudged, _)| *unjudged).count();
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "adopted", "{record:?}");
    assert!(record.max_move_m > 0.3, "moved {:.3} m", record.max_move_m);
    assert!(fast[before_gnc - 1].1 < 0.5 * slow[0].1, "window: {:.3} m, odometry {:.3} m", fast[before_gnc - 1].1, slow[0].1);
    // published before GNC: every step between neighbours keeps the odometry's length (a kink would stretch one by the move)
    let (_, published) = run_until_published(drift);
    let worst = (1..KEYFRAMES)
        .map(|k| (mat3::norm(&mat3::sub(&published[k], &published[k - 1])) - SPACING).abs())
        .fold(0.0, f64::max);
    assert!(worst < 0.01 * record.max_move_m.max(0.1), "a step stretched by {worst:.4} m for a {:.3} m move", record.max_move_m);
}

#[test]
fn only_the_robots_trail_moves_and_newer_keyframes_ride_it() {
    let drift = 2.5e-4;
    let (_, slow) = run(false, drift, 0.0);
    let (mut with, _, before) = run_with(true, drift, 0.0, true);
    let published: Vec<Vec3> = (0..KEYFRAMES).map(|k| with.key_poses().position(k)).collect();
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "adopted", "{record:?}");
    // the trail: the newest `free` keyframes; nothing before it moves at all
    let first = KEYFRAMES - record.free;
    let moved = |k: usize| mat3::norm(&mat3::sub(&published[k], &before[k]));
    for k in 0..first {
        assert_eq!(moved(k), 0.0, "keyframe {k} off the trail moved");
    }
    assert!(moved(KEYFRAMES - 1) > 0.3, "the newest moved {:.3} m", moved(KEYFRAMES - 1));
    // the loop is closed: the newest keyframe nearer the truth than the odometry left it
    let newest = mat3::norm(&mat3::sub(&published[KEYFRAMES - 1], &truth(KEYFRAMES - 1).1));
    assert!(newest < 0.5 * slow[0].1, "window: {newest:.3} m from the truth, the odometry {:.3} m", slow[0].1);
    // keyframes made after the publish ride the shift: each lands one odometry step on from the shifted newest
    let shift = mat3::sub(&published[KEYFRAMES - 1], &before[KEYFRAMES - 1]);
    let step = mat3::norm(&mat3::sub(&before[KEYFRAMES - 1], &before[KEYFRAMES - 2]));
    let (rotation, position) = with.key_poses().global(KEYFRAMES - 1);
    let raw_newest = with.key_poses().fixed(KEYFRAMES - 1).translation_local;
    let raw_rotation = with.key_poses().fixed(KEYFRAMES - 1).rotation_local;
    let heading = mat3::mat_mul(&rotation, &mat3::transpose(&raw_rotation));
    for j in 1..=3 {
        let raw: Vec3 = mat3::add(&raw_newest, &mat3::mat_vec(&raw_rotation, &[step * j as f64, 0.0, 0.0]));
        scan(&mut with, KEYFRAMES + 100 + j, (raw_rotation, raw));
        with.wait_for_due_solve();
    }
    let n = with.key_poses().len();
    assert!(n > KEYFRAMES, "no new keyframe was made");
    for k in KEYFRAMES..n {
        let expected = mat3::add(&position, &mat3::mat_vec(&heading, &mat3::sub(&with.key_poses().fixed(k).translation_local, &raw_newest)));
        let off = mat3::norm(&mat3::sub(&with.key_poses().position(k), &expected));
        assert!(off < 1e-6, "keyframe {k} is {off:.4} m off the shifted trail (shift {:.3} m)", mat3::norm(&shift));
    }
}

/// `run` with the window, stopped once the first job landed: the published positions then, before GNC.
fn run_until_published(drift: f64) -> (GscPgo, Vec<Vec3>) {
    let (pgo, _, _) = run_with(true, drift, 0.0, true);
    let positions = (0..KEYFRAMES).map(|k| pgo.key_poses().position(k)).collect();
    (pgo, positions)
}

#[test]
fn the_window_leaves_a_wrong_closure_to_gnc() {
    // a closure turned 150 degrees: far past what the loop and its own variance allow, so GNC rejects it, and so does the
    // window's cutoff
    let (drift, turn) = (2.5e-5, 150f64.to_radians());
    let (without, slow) = run(false, drift, turn);
    let (with, fast) = run(true, drift, turn);
    assert_eq!(with.committed_closures().weight(0), 0.0, "GNC rejected the closure");
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "over_cutoff", "{record:?}");
    assert_eq!(record.refused, Some(0));
    assert!(with.closure_latency()[0].fast_path.is_none());
    assert_eq!(fast, slow, "the published map was never bent");
    assert_eq!(largest_difference(&with, &without), 0.0);
}

#[test]
fn the_window_drops_a_disagreeing_closure_and_publishes_the_agreeing_one() {
    // the true closure and a wrong one (150 degrees off) in one job: GNC's rule in the window drops the wrong one and
    // the right one is published at once, not held back with it
    let (drift, turn) = (2.5e-5, 0.0);
    let (with, _, _) = run_with_wrong(true, drift, turn, true, true);
    let record = &with.fast_path_records()[0];
    assert_eq!(record.outcome, "adopted", "{record:?}");
    assert_eq!(record.closures, vec![0, 1], "{record:?}");
    assert!(with.closure_latency()[0].fast_path.is_some());
    assert!(with.closure_latency()[2].fast_path.is_none(), "the wrong closure was published");
}
