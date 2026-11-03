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

//! `graph_simplify` on a small synthetic run: a drifting robot laps a square six times with closures from truth; the
//! simplified graph (far fewer nodes) puts every keyframe close to where the full graph does.

use dimos_gsc_pgo::gsc_pgo::{CloudWithPose, Config, GscPgo, PoseWithTime};
use dimos_gsc_pgo::mat3;

/// (keyframe poses, keyframe count, solve nodes) of a cloud-less run of six laps of a 12 m square.
fn run(simplify: bool) -> (Vec<[f64; 3]>, usize, usize, Vec<[f64; 3]>) {
    let config = Config {
        auto_aggregate_cloud_size: 0,
        odometry_error_estimate_from_lidar_range: false,
        graph_simplify: simplify,
        graph_simplify_frontier_keyframes: 15,
        graph_simplify_localized_keyframes: 40,
        ..Config::default()
    };
    let mut pgo = GscPgo::new(config);
    pgo.set_wait_for_due_jobs(true);
    // truth: 0.3 m steps around the square; odometry: the same with a small heading bias
    let mut truth: Vec<[f64; 3]> = Vec::new();
    let (mut x, mut y, mut yaw) = (0.0f64, 0.0f64, 0.0f64);
    let (mut ox, mut oy, mut oyaw) = (0.0f64, 0.0f64, 0.0f64);
    for step in 0..6 * 160 {
        let turn = if step % 40 == 39 { std::f64::consts::FRAC_PI_2 } else { 0.0 };
        x += 0.3 * yaw.cos();
        y += 0.3 * yaw.sin();
        yaw += turn;
        ox += 0.3 * oyaw.cos();
        oy += 0.3 * oyaw.sin();
        oyaw += turn * 1.01 + 0.0004;
        let mut pose = PoseWithTime::new(mat3::rot_z(oyaw), [ox, oy, 0.0]);
        pose.set_time(1000 + step, 0);
        if !pgo.add_key_pose(&CloudWithPose { cloud: None, pose }) {
            pgo.wait_for_due_solve();
            continue;
        }
        truth.push([x, y, yaw]);
        let n = truth.len();
        // every 5th keyframe past the first lap: a closure onto the same corner of the square from truth
        if n > 80 && n.is_multiple_of(5) {
            let here = truth[n - 1];
            let target = (0..n - 60)
                .filter(|&k| !pgo.key_poses().is_removed(k))
                .min_by(|&a, &b| {
                    let d = |k: usize| (truth[k][0] - here[0]).hypot(truth[k][1] - here[1]);
                    d(a).total_cmp(&d(b))
                })
                .unwrap();
            let t = truth[target];
            let (c, s) = (t[2].cos(), t[2].sin());
            let (dx, dy) = (here[0] - t[0], here[1] - t[1]);
            pgo.commit_closure(
                n - 1,
                target,
                mat3::rot_z(here[2] - t[2]),
                [c * dx + s * dy, -s * dx + c * dy, 0.0],
                0.001,
            );
        }
        pgo.wait_for_due_solve();
    }
    pgo.finish();
    let kept = pgo.committed_closures().iter().filter(|c| c.gnc_weight > 0.5).count();
    assert_eq!(kept, pgo.committed_closures().len(), "simplify {simplify}: a true closure was rejected");
    let poses = (0..pgo.key_poses().len()).map(|k| pgo.key_poses().position(k)).collect();
    (poses, pgo.key_poses().len(), pgo.simplify_stats().solve_nodes, truth)
}

#[test]
fn a_simplified_graph_solves_close_to_the_full_one() {
    let (full, keyframes, _, truth) = run(false);
    let (simplified, simplified_keyframes, nodes, _) = run(true);
    let worst = |poses: &[[f64; 3]]| {
        poses.iter().zip(&truth).map(|(p, t)| (p[0] - t[0]).hypot(p[1] - t[1])).fold(0.0, f64::max)
    };
    let (worst_full, worst_simplified) = (worst(&full), worst(&simplified));
    assert_eq!(keyframes, simplified_keyframes);
    assert!(nodes * 2 < keyframes, "{nodes} nodes for {keyframes} keyframes");
    let largest = full
        .iter()
        .zip(&simplified)
        .map(|(a, b)| mat3::norm(&mat3::sub(a, b)))
        .fold(0.0, f64::max);
    eprintln!("{nodes} nodes for {keyframes} keyframes; worst vs truth {worst_full:.3} m full, {worst_simplified:.3} m simplified; largest difference {largest:.3} m");
    assert!(largest < 0.25, "largest difference {largest} m");
    assert!(worst_simplified < 2.0 * worst_full, "worst vs truth {worst_simplified} m simplified, {worst_full} m full");
}
