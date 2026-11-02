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

use super::super::region_tests::{
    assert_chains_reach_their_nodes, assert_plans_equivalent, big_world, path_len, surface_set,
    test_config, voxel_set,
};
use super::*;
use crate::reform::{KeyframeRecord, MovedRecord, Pose, ReformParams, ReformState};
use fast_reform::{Quat, Vec3};

/// Keyframes on a 1 m grid over big_world, a second apart.
fn keyframes() -> Vec<KeyframeRecord> {
    (0..64)
        .map(|i| KeyframeRecord {
            id: i,
            ts: i as f64,
            pose: Pose {
                rotation: Quat::IDENTITY,
                translation: Vec3::new((i % 8) as f64 + 0.5, (i / 8) as f64 + 0.5, 0.3),
            },
        })
        .collect()
}

fn params() -> ReformParams {
    ReformParams {
        blend_sigma_m: 1.0,
        blend_time_sigma_s: 0.0,
        min_shift_m: 0.02,
        rebuild_fraction: 1.0,
    }
}

/// A warp that applies `correction` to the keyframes `pick` keeps.
fn warp_of(correction: Pose, pick: impl Fn(&KeyframeRecord) -> bool) -> Warp {
    warp_with(correction, pick, params())
}

fn warp_with(
    correction: Pose,
    pick: impl Fn(&KeyframeRecord) -> bool,
    params: ReformParams,
) -> Warp {
    let kfs = keyframes();
    let mut state = ReformState::default();
    state.add_keyframes(&kfs);
    let moved: Vec<MovedRecord> = kfs
        .iter()
        .filter(|k| pick(k))
        .map(|k| MovedRecord {
            keyframe: *k,
            delta: correction,
        })
        .collect();
    state.closure(&moved, &params).expect("keyframes moved")
}

fn shift(x: f64, y: f64) -> Pose {
    Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::new(x, y, 0.0),
    }
}

fn centers(p: &Planner, vs: f32) -> Vec<(f32, f32, f32)> {
    p.voxel_keys()
        .map(|(x, y, z)| {
            (
                (x as f32 + 0.5) * vs,
                (y as f32 + 0.5) * vs,
                (z as f32 + 0.5) * vs,
            )
        })
        .collect()
}

#[test]
fn a_rigid_shift_of_everything_matches_a_rebuild_of_the_shifted_map() {
    let cfg = test_config();
    let all = big_world();
    let mut warped = Planner::new(cfg.worker_threads);
    warped.update_global_map(&all, &cfg);
    let nodes_before = warped.graph().nodes.len();
    let voxels = warped.voxel_count();

    let warp = warp_of(shift(0.8, 0.3), |_| true);
    let report = warped.apply_warp(&warp, &mut [], &cfg);
    assert_eq!(report.voxels_moved, voxels);
    assert!(report.nodes_moved * 2 > nodes_before, "{report:?}");

    let moved: Vec<(f32, f32, f32)> = all.iter().map(|p| (p.0 + 0.8, p.1 + 0.3, p.2)).collect();
    let mut rebuilt = Planner::new(cfg.worker_threads);
    rebuilt.update_global_map(&moved, &cfg);
    assert_eq!(voxel_set(&warped), voxel_set(&rebuilt));
    assert_eq!(surface_set(&warped), surface_set(&rebuilt));
    assert_chains_reach_their_nodes(&warped);

    // Every pair moves with the map.
    let pairs = [
        ((1.3, 0.8, 0.05), (8.3, 7.8, 0.05)),
        ((1.3, 7.8, 0.05), (8.3, 0.8, 0.05)),
    ];
    for (s, g) in pairs {
        let a = warped.plan(s, g, &cfg).expect("warped plans");
        let b = rebuilt.plan(s, g, &cfg).expect("rebuilt plans");
        let (la, lb) = (path_len(&a), path_len(&b));
        assert!((la - lb).abs() <= 0.1 * lb + 0.3, "{la} vs {lb}");
    }
}

#[test]
fn a_partial_bend_matches_a_rebuild_of_the_same_voxels() {
    let cfg = test_config();
    let all = big_world();
    let mut warped = Planner::new(cfg.worker_threads);
    warped.update_global_map(&all, &cfg);

    // The far half turns 2 degrees about the gap and slides 13 cm.
    let yaw = 2f64.to_radians();
    let rotation = Quat::new(0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos());
    let about = Vec3::new(4.0, 4.0, 0.0);
    let turned = rotation.rotate(about);
    let bend = Pose {
        rotation,
        translation: Vec3::new(about.x - turned.x, about.y - turned.y + 0.13, 0.0),
    };
    let warp = warp_of(bend, |k| k.pose.translation.x > 4.0);
    let report = warped.apply_warp(&warp, &mut [], &cfg);
    assert!(
        report.voxels_moved > 0 && report.voxels_moved < warped.voxel_count(),
        "{report:?}"
    );

    let mut rebuilt = Planner::new(cfg.worker_threads);
    rebuilt.update_global_map(&centers(&warped, cfg.voxel_size), &cfg);
    assert_eq!(surface_set(&warped), surface_set(&rebuilt));
    assert_chains_reach_their_nodes(&warped);
    assert_plans_equivalent(&rebuilt, &warped, &cfg);
}

#[test]
fn the_goal_and_the_cached_path_move_with_the_map() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let (start, goal) = ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05));
    assert!(!p.plan_or_truncate(start, goal, &cfg).is_empty());
    let cached: Vec<VoxelKey> = p.last_path.as_ref().unwrap().1.clone();

    let warp = warp_of(shift(0.8, 0.3), |_| true);
    let mut held = [goal];
    p.apply_warp(&warp, &mut held, &cfg);
    let moved_goal = (goal.0 + 0.8, goal.1 + 0.3, goal.2);
    assert!((held[0].0 - moved_goal.0).abs() < 1e-4 && (held[0].1 - moved_goal.1).abs() < 1e-4);
    let (cached_goal, cells) = p.last_path.as_ref().unwrap();
    assert_eq!(
        *cached_goal, held[0],
        "the cached goal must still match the held one"
    );
    let want: Vec<VoxelKey> = cached.iter().map(|c| (c.0 + 8, c.1 + 3, c.2)).collect();
    assert_eq!(*cells, want);
}

#[test]
fn a_shift_under_half_a_voxel_leaves_the_graph_alone() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let surface = surface_set(&p);
    let nodes: Vec<_> = p.graph().nodes.iter().map(|n| n.pos).collect();

    let report = p.apply_warp(&warp_of(shift(0.03, 0.0), |_| true), &mut [], &cfg);
    assert_eq!(
        (report.voxels_moved, report.nodes_moved),
        (0, 0),
        "{report:?}"
    );
    assert_eq!(surface_set(&p), surface);
    let after: Vec<_> = p.graph().nodes.iter().map(|n| n.pos).collect();
    assert_eq!(after, nodes);
}

#[test]
fn voxel_times_follow_the_voxels_through_a_warp() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.track_voxel_times();
    p.set_clock(10.0);
    p.update_global_map(&all, &cfg);
    let warp = warp_of(shift(0.8, 0.3), |_| true);
    p.apply_warp(&warp, &mut [], &cfg);
    let times = p.voxel_times.as_ref().unwrap();
    assert_eq!(times.by_key.len(), p.voxel_count());
    assert!(p.voxel_keys().all(|k| times.get(&k) == 10.0));
}

#[test]
fn a_warp_that_moves_most_of_the_map_rebuilds_and_matches_a_rebuild() {
    let cfg = test_config();
    let all = big_world();
    let mut warped = Planner::new(cfg.worker_threads);
    warped.update_global_map(&all, &cfg);
    let (start, goal) = ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05));
    assert!(!warped.plan_or_truncate(start, goal, &cfg).is_empty());

    let rebuild = ReformParams {
        rebuild_fraction: 0.25,
        ..params()
    };
    let mut held = [goal];
    let report = warped.apply_warp(
        &warp_with(shift(0.8, 0.3), |_| true, rebuild),
        &mut held,
        &cfg,
    );
    assert!(report.rebuilt, "{report:?}");
    assert!((held[0].0 - 8.3).abs() < 1e-4 && (held[0].1 - 7.8).abs() < 1e-4);
    assert_eq!(warped.last_path.as_ref().unwrap().0, held[0]);

    let moved: Vec<(f32, f32, f32)> = all.iter().map(|p| (p.0 + 0.8, p.1 + 0.3, p.2)).collect();
    let mut rebuilt = Planner::new(cfg.worker_threads);
    rebuilt.update_global_map(&moved, &cfg);
    assert_eq!(voxel_set(&warped), voxel_set(&rebuilt));
    assert_eq!(surface_set(&warped), surface_set(&rebuilt));
    assert_eq!(node_count(&warped), node_count(&rebuilt));
}

fn node_count(p: &Planner) -> usize {
    p.graph().nodes.len()
}
