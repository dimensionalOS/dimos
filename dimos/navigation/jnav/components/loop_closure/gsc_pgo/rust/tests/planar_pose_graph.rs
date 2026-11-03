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

//! `planar_pose_graph`: odometry that climbs and rocks still makes a level graph, every keyframe pose (raw and solved) at
//! z = 0 with no roll or pitch, through an ICP closure, landmark sightings and world positions.

use std::sync::Arc;

use dimos_gsc_pgo::gsc_pgo::{ClosureSources, CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::landmarks::LandmarkSighting;
use dimos_gsc_pgo::mat3::{self, Mat3, Vec3};

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

/// Roll about x then pitch about y (body axes), as R = Ry(pitch) Rx(roll).
fn tilt(roll: f64, pitch: f64) -> Mat3 {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]];
    let ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]];
    mat3::mat_mul(&ry, &rx)
}

/// Replays the square with the body rocking (up to 3 deg of roll and pitch, reported by the odometry), the odometry's z
/// climbing 5 cm per metre and its heading drifting 0.3 deg per metre; the tag is seen within 2 m, and with `fixes` a world
/// position 10 m up at every corner. Returns the pgo and odometry positions.
fn replay(config: Config, fixes: bool) -> (GscPgo, Vec<Vec3>) {
    let world = room();
    let mut pgo = GscPgo::new(config);
    let (mut odometry_yaw, mut odometry_at, mut odometry) = (0.0, [-3.0, -3.0, 0.5], Vec::new());
    let path = square();
    for (scan, &(yaw, at)) in path.iter().enumerate() {
        if scan > 0 {
            let (previous_yaw, previous_at) = path[scan - 1];
            let step = mat3::mat_vec(
                &mat3::transpose(&mat3::rot_z(previous_yaw)),
                &mat3::sub(&at, &previous_at),
            );
            odometry_at = mat3::add(
                &odometry_at,
                &mat3::mat_vec(&mat3::rot_z(odometry_yaw), &step),
            );
            odometry_at[2] += 0.05 * mat3::norm(&step);
            odometry_yaw += yaw - previous_yaw + 0.3f64.to_radians() * mat3::norm(&step);
        }
        odometry.push(odometry_at);
        let rocking = tilt(
            3f64.to_radians() * (scan as f64 * 0.7).sin(),
            3f64.to_radians() * (scan as f64 * 0.45).cos(),
        );
        // the true body (rocking) sees the room; the odometry reports the same rocking on its drifted pose
        let to_body = mat3::transpose(&mat3::mat_mul(&mat3::rot_z(yaw), &rocking));
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
        let corner = scan % 60 == 0;
        if fixes && corner {
            let covariance = [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]];
            pgo.add_coordinate_factor(time, 0, [0.0; 3], [at[0], at[1], 10.0], covariance);
        }
        let mut pose = PoseWithTime::new(
            mat3::mat_mul(&mat3::rot_z(odometry_yaw), &rocking),
            odometry_at,
        );
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

fn planar() -> Config {
    Config {
        planar_pose_graph: true,
        closure_sources: ClosureSources::InputsOnly,
        drop_reseen_points: false,
        odometry_error_estimate_from_lidar_range: false,
        odometry_error_estimate_xy_per_sqrt_m: 0.05,
        odometry_error_estimate_yaw_per_sqrt_m: 0.01,
        ..Config::default()
    }
}

/// Whether a pose is on the floor plane: z = 0 and its rotation a pure turn about z.
fn is_level(rotation: &Mat3, translation: &Vec3) -> bool {
    translation[2] == 0.0
        && rotation[2][2] == 1.0
        && [rotation[0][2], rotation[1][2], rotation[2][0], rotation[2][1]] == [0.0; 4]
}

/// Every keyframe's raw (odometry) and corrected (solved) pose is level; returns how many were solved.
fn assert_level(pgo: &GscPgo) -> usize {
    let keyframes = pgo.key_poses();
    for k in 0..keyframes.len() {
        let fixed = keyframes.fixed(k);
        assert!(
            is_level(&fixed.rotation_local, &fixed.translation_local),
            "keyframe {k} raw: {:?} {:?}",
            fixed.rotation_local,
            fixed.translation_local
        );
        let (rotation, translation) = keyframes.global(k);
        assert!(
            is_level(&rotation, &translation),
            "keyframe {k} corrected: {rotation:?} {translation:?}"
        );
    }
    keyframes.solved_len()
}

#[test]
fn a_rocking_climbing_odometry_closes_a_level_loop() {
    let (pgo, odometry) = replay(planar(), false);
    let closures = pgo.committed_closures();
    assert!(
        !closures.is_empty(),
        "outcomes: {:?}",
        pgo.closure_diagnostics().iter().map(|d| d.outcome).collect::<Vec<_>>()
    );
    for i in 0..closures.len() {
        let closure = closures.fixed(i);
        assert!(
            is_level(&closure.rotation_offset, &closure.translation_offset),
            "closure {i}: {:?} {:?}",
            closure.rotation_offset,
            closure.translation_offset
        );
    }
    assert!(odometry.last().unwrap()[2] - odometry[0][2] > 1.0, "the odometry climbed");
    let keyframes = pgo.key_poses().len();
    assert_eq!(assert_level(&pgo), keyframes, "the closure's solve placed every keyframe");
    let end = pgo.key_poses().position(keyframes - 1);
    let start = pgo.key_poses().position(0);
    let odometry_gap = mat3::norm(&mat3::sub(odometry.last().unwrap(), &odometry[0]));
    let closed_gap = mat3::norm(&mat3::sub(&end, &start));
    assert!(
        closed_gap < 0.2,
        "closed to {closed_gap:.2} m (odometry {odometry_gap:.2} m)"
    );
}

#[test]
fn world_positions_high_above_and_landmarks_alone_keep_it_level() {
    let (pgo, odometry) = replay(
        Config {
            landmark_closure_icp: false,
            ..planar()
        },
        true,
    );
    assert_eq!(pgo.committed_closures().len(), 0);
    let keyframes = pgo.key_poses().len();
    assert_eq!(assert_level(&pgo), keyframes, "the inputs' solve placed every keyframe");
    let closed_gap = mat3::norm(&mat3::sub(
        &pgo.key_poses().position(keyframes - 1),
        &pgo.key_poses().position(0),
    ));
    let mut horizontal = odometry.last().unwrap().to_owned();
    horizontal[2] = odometry[0][2];
    let odometry_gap = mat3::norm(&mat3::sub(&horizontal, &odometry[0]));
    assert!(
        closed_gap < 0.5 * odometry_gap,
        "closed to {closed_gap:.2} m (odometry {odometry_gap:.2} m across)"
    );
}
