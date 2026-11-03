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

//! `closure_sources: inputs_only`: no place recognition; a tag seen at both ends of a drifting square seeds the one ICP closure.

use std::sync::Arc;

use dimos_gsc_pgo::gsc_pgo::{ClosureSources, CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::landmarks::LandmarkSighting;
use dimos_gsc_pgo::mat3::{self, Vec3};

const TAG: Vec3 = [-3.0, -4.5, 0.5];

/// A 12 m room (floor, ceiling, 2 m walls) and four pillars, every 0.1 m.
fn room() -> Vec<Vec3> {
    let mut points = Vec::new();
    let steps = |from: f64, to: f64| {
        let n = ((to - from) / 0.1).round() as usize;
        (0..=n).map(move |i| from + i as f64 * 0.1)
    };
    // the floor and ceiling pin height and tilt, which walls alone leave free
    for x in steps(-6.0, 6.0) {
        for y in steps(-6.0, 6.0) {
            points.extend([[x, y, 0.0], [x, y, 2.0]]);
        }
    }
    for z in steps(0.0, 2.0) {
        for s in steps(-6.0, 6.0) {
            points.extend([[s, -6.0, z], [s, 6.0, z], [-6.0, s, z], [6.0, s, z]]);
        }
        for (cx, cy, half) in [
            (0.0, 0.0, 0.8),
            (-4.5, 1.5, 0.4),
            (4.0, -1.0, 0.5),
            (1.5, 4.5, 0.3),
        ] {
            for s in steps(-half, half) {
                points.extend([
                    [cx + s, cy - half, z],
                    [cx + s, cy + half, z],
                    [cx - half, cy + s, z],
                    [cx + half, cy + s, z],
                ]);
            }
        }
    }
    points
}

/// True (yaw, position) every 0.1 m round the 6 m square from (-3, -3), heading along it.
fn square() -> Vec<(f64, Vec3)> {
    let corners: [[f64; 2]; 5] = [
        [-3.0, -3.0],
        [3.0, -3.0],
        [3.0, 3.0],
        [-3.0, 3.0],
        [-3.0, -3.0],
    ];
    let mut path = Vec::new();
    for leg in corners.windows(2) {
        let (a, b) = (leg[0], leg[1]);
        let yaw = (b[1] - a[1]).atan2(b[0] - a[0]);
        for i in 0..60 {
            let f = i as f64 / 60.0;
            path.push((
                yaw,
                [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), 0.5],
            ));
        }
    }
    path.push((path.last().unwrap().0, [-3.0, -3.0, 0.5]));
    path
}

/// Replays the square with odometry whose heading drifts 0.3 deg per metre, the tag seen within 2 m; returns the pgo and odometry.
fn replay(config: Config) -> (GscPgo, Vec<Vec3>) {
    let world = room();
    let mut pgo = GscPgo::new(config);
    let (mut odometry_yaw, mut odometry_at, mut odometry) = (0.0, [-3.0, -3.0, 0.5], Vec::new());
    let path = square();
    for (scan, &(yaw, at)) in path.iter().enumerate() {
        if scan > 0 {
            let (previous_yaw, previous_at) = path[scan - 1];
            // the true step in the previous body frame, replayed on the drifted odometry heading
            let step = mat3::mat_vec(
                &mat3::transpose(&mat3::rot_z(previous_yaw)),
                &mat3::sub(&at, &previous_at),
            );
            odometry_at = mat3::add(
                &odometry_at,
                &mat3::mat_vec(&mat3::rot_z(odometry_yaw), &step),
            );
            odometry_yaw += yaw - previous_yaw + 0.3f64.to_radians() * mat3::norm(&step);
        }
        odometry.push(odometry_at);
        let to_body = mat3::transpose(&mat3::rot_z(yaw));
        let body = |p: &Vec3| mat3::mat_vec(&to_body, &mat3::sub(p, &at));
        let cloud: Vec<[f32; 3]> = world
            .iter()
            .filter(|p| mat3::norm(&mat3::sub(p, &at)) < 8.0)
            .map(|p| body(p).map(|v| v as f32))
            .collect();
        let time = scan as f64 * 0.1;
        if mat3::norm(&mat3::sub(&TAG, &at)) < 2.0 {
            let covariance = [[0.0025, 0.0, 0.0], [0.0, 0.0025, 0.0], [0.0, 0.0, 0.0025]];
            let tag = LandmarkSighting::point(0, 0, body(&TAG), covariance);
            pgo.add_landmark_factor(time, 7, tag.rotation, tag.position, tag.information);
        }
        let mut pose = PoseWithTime::new(mat3::rot_z(odometry_yaw), odometry_at);
        pose.set_time((time.floor()) as i32, ((time.fract()) * 1e9).round() as u32);
        let keyframe = pgo.add_key_pose(&CloudWithPose {
            cloud: Some(Arc::new(cloud)),
            pose,
        });
        if keyframe {
            pgo.search_for_closure_pairs();
            pgo.wait_for_due_solve();
        }
    }
    pgo.finish();
    (pgo, odometry)
}

fn inputs_only() -> Config {
    Config {
        closure_sources: ClosureSources::InputsOnly,
        drop_reseen_points: false,
        odometry_error_estimate_from_lidar_range: false,
        odometry_error_estimate_xy_per_sqrt_m: 0.05,
        odometry_error_estimate_yaw_per_sqrt_m: 0.01,
        ..Config::default()
    }
}

#[test]
fn a_tag_seen_again_seeds_an_icp_closure_onto_its_first_visit() {
    let (pgo, odometry) = replay(inputs_only());
    let diagnostics = pgo.closure_diagnostics();
    assert!(
        !diagnostics.is_empty(),
        "the tag's second visit proposes a closure"
    );
    assert!(
        diagnostics.iter().all(|d| !d.from_scan_context),
        "no place recognition runs"
    );
    let keyframes = pgo.key_poses().len();
    let closures = pgo.committed_closures();
    assert!(
        !closures.is_empty(),
        "outcomes: {:?}",
        diagnostics.iter().map(|d| d.outcome).collect::<Vec<_>>()
    );
    for i in 0..closures.len() {
        let closure = closures.fixed(i);
        assert!(
            closure.target_id < 6 && closure.source_id + 6 > keyframes,
            "start to end: {} -> {}",
            closure.source_id,
            closure.target_id
        );
    }
    // the loop closed: the last keyframe is back near the start, where the drifted odometry is not
    let end = pgo.key_poses().position(keyframes - 1);
    let start = pgo.key_poses().position(0);
    let odometry_gap = mat3::norm(&mat3::sub(odometry.last().unwrap(), &odometry[0]));
    let closed_gap = mat3::norm(&mat3::sub(&end, &start));
    assert!(
        odometry_gap > 0.5,
        "the odometry drifted {odometry_gap:.2} m"
    );
    assert!(
        closed_gap < 0.2,
        "closed to {closed_gap:.2} m (odometry {odometry_gap:.2} m)"
    );
}

#[test]
fn without_landmark_icp_the_tag_alone_moves_the_graph() {
    let (pgo, odometry) = replay(Config {
        landmark_closure_icp: false,
        ..inputs_only()
    });
    assert!(pgo.closure_diagnostics().is_empty());
    assert_eq!(pgo.committed_closures().len(), 0);
    let keyframes = pgo.key_poses();
    let closed_gap = mat3::norm(&mat3::sub(
        &keyframes.position(keyframes.len() - 1),
        &keyframes.position(0),
    ));
    let odometry_gap = mat3::norm(&mat3::sub(odometry.last().unwrap(), &odometry[0]));
    assert!(
        closed_gap < 0.5 * odometry_gap,
        "closed to {closed_gap:.2} m (odometry {odometry_gap:.2} m)"
    );
}
