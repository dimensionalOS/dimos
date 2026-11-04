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
    assert_chains_reach_their_nodes, big_world, path_len, slice, surface_set, test_config,
    voxel_set,
};
use super::*;
use std::collections::BTreeSet;

use crate::reform::{KeyframeRecord, MovedRecord, Pose, ReformConfig, ReformState};
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

fn params() -> ReformConfig {
    ReformConfig {
        blend_sigma_m: 1.0,
        blend_time_sigma_s: 0.0,
        min_shift_m: 0.02,
        thin_cell_m: -1.0,
        reach_sigmas: 3.0,
        enabled: true,
    }
}

/// A warp that applies `correction` to the keyframes `pick` keeps.
fn warp_of(correction: Pose, pick: impl Fn(&KeyframeRecord) -> bool) -> Warp {
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
    state.closure(&moved, &params()).expect("keyframes moved")
}

fn shift(x: f64, y: f64) -> Pose {
    Pose {
        rotation: Quat::IDENTITY,
        translation: Vec3::new(x, y, 0.0),
    }
}

fn placed_voxels(p: &Planner, vs: f32) -> BTreeSet<VoxelKey> {
    p.placed_voxels(vs).collect()
}

fn placed_surface(p: &Planner, vs: f32) -> BTreeSet<VoxelKey> {
    p.placed_surface(vs).collect()
}

fn built(cfg: &Config, points: &[(f32, f32, f32)]) -> Planner {
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(points, cfg);
    p
}

fn shifted(points: &[(f32, f32, f32)], dx: f32, dy: f32) -> Vec<(f32, f32, f32)> {
    points.iter().map(|p| (p.0 + dx, p.1 + dy, p.2)).collect()
}

/// Every pair of key-adjacent surface cells within a step has a hop each way, as a full build gives them.
fn assert_adjacency_matches_keys(p: &Planner, cfg: &Config) {
    let g = p.graph();
    let step = cfg.step_cells();
    for u in g.cells.ids() {
        let k = g.cells.coord(u);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let Some(zs) = g.surface_lookup.get(&(k.0 + dx, k.1 + dy)) else {
                continue;
            };
            for &nz in zs {
                if (nz - k.2).abs() > step {
                    continue;
                }
                let v = g
                    .cells
                    .id((k.0 + dx, k.1 + dy, nz))
                    .expect("a lookup cell exists");
                assert!(
                    g.cells.neighbors(u).iter().any(|e| e.dest == v),
                    "{k:?} has no hop to its key neighbor {:?}",
                    (k.0 + dx, k.1 + dy, nz)
                );
            }
        }
    }
}

#[test]
fn a_rigid_shift_places_everything_where_a_rebuild_of_the_shifted_map_has_it() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let all = big_world();
    let mut warped = built(&cfg, &all);
    let (cells, edges, nodes) = (
        warped.graph().cells.live_len(),
        warped.graph().cells.edge_count(),
        warped.graph().nodes.len(),
    );
    let surface_keys = surface_set(&warped);

    let report = warped.apply_warp(&warp_of(shift(0.8, 0.3), |_| true), &mut [], &cfg);
    assert_eq!(report.voxels_warped, warped.voxel_count());
    let rebuilt = built(&cfg, &shifted(&all, 0.8, 0.3));
    assert_eq!(placed_voxels(&warped, vs), voxel_set(&rebuilt));
    assert_eq!(placed_surface(&warped, vs), surface_set(&rebuilt));
    // In place: the cells keep their keys, edges and nodes.
    assert_eq!(surface_set(&warped), surface_keys);
    assert_eq!(
        (
            warped.graph().cells.live_len(),
            warped.graph().cells.edge_count(),
            warped.graph().nodes.len()
        ),
        (cells, edges, nodes)
    );
    assert_chains_reach_their_nodes(&warped);

    // Every pair moves with the map.
    let pairs = [
        ((1.3, 0.8, 0.05), (8.3, 7.8, 0.05)),
        ((1.3, 7.8, 0.05), (8.3, 0.8, 0.05)),
    ];
    for (s, g) in pairs {
        let a = warped.plan_or_truncate(s, g, &cfg);
        assert!(!a.is_empty(), "warped plans");
        let b = rebuilt.plan(s, g, &cfg).expect("rebuilt plans");
        let (la, lb) = (path_len(&a), path_len(&b));
        assert!((la - lb).abs() <= 0.1 * lb + 0.3, "{la} vs {lb}");
        let end = a.last().unwrap();
        assert!((end.0 - g.0).hypot(end.1 - g.1) < 0.3, "{end:?} vs {g:?}");
    }
}

#[test]
fn a_region_update_rebins_only_what_it_reaches() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let all = big_world();
    let mut p = built(&cfg, &all);
    let keys = surface_set(&p);
    p.apply_warp(&warp_of(shift(0.8, 0.3), |_| true), &mut [], &cfg);
    let placed = placed_surface(&p, vs);
    p.rebin_near((2.0, 2.0), 1.5, &cfg);
    // Where it sits is unchanged; near (2, 2) the keys now say so, far off they do not.
    assert_eq!(placed_surface(&p, vs), placed);
    let key_of = |x: f32, y: f32| ((x / vs).floor() as i32, (y / vs).floor() as i32);
    let (nx, ny) = key_of(2.0, 2.0);
    let near_cell = placed
        .iter()
        .find(|k| (k.0, k.1) == (nx, ny))
        .copied()
        .unwrap();
    assert!(surface_set(&p).contains(&near_cell));
    let (fx, fy) = key_of(7.0, 7.0);
    let far_old = keys
        .iter()
        .find(|k| (k.0, k.1) == (fx, fy))
        .copied()
        .unwrap();
    assert!(surface_set(&p).contains(&far_old));
    assert_chains_reach_their_nodes(&p);
    assert_adjacency_matches_keys(&p, &cfg);
}

#[test]
fn region_updates_after_a_warp_keep_the_graph_whole() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.track_voxel_times();
    p.update_global_map(&all, &cfg);
    let (start, goal) = ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05));
    assert!(!p.plan_or_truncate(start, goal, &cfg).is_empty());
    p.apply_warp(&warp_of(shift(0.8, 0.3), |_| true), &mut [], &cfg);
    let moved = shifted(&all, 0.8, 0.3);
    // The robot drives across the moved map, re-observing it region by region.
    for i in 0..8 {
        let bounds = RegionBounds {
            origin_x: 1.3 + i as f32,
            origin_y: 0.8 + i as f32,
            radius: 1.5,
            z_min: -1.0,
            z_max: 1.0,
        };
        p.update_region(&slice(&moved, &bounds, vs), &bounds, &cfg);
        assert_chains_reach_their_nodes(&p);
        assert!(!p
            .plan_or_truncate((1.3, 0.8, 0.05), (8.3, 7.8, 0.05), &cfg)
            .is_empty());
    }
}

#[test]
fn the_goal_and_the_cached_path_move_with_the_map() {
    let cfg = test_config();
    let all = big_world();
    let mut p = built(&cfg, &all);
    let (start, goal) = ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05));
    assert!(!p.plan_or_truncate(start, goal, &cfg).is_empty());
    let cached: Vec<VoxelKey> = p.last_path.as_ref().unwrap().1.clone();

    let mut held = [goal];
    p.apply_warp(&warp_of(shift(0.8, 0.3), |_| true), &mut held, &cfg);
    let moved_goal = (goal.0 + 0.8, goal.1 + 0.3, goal.2);
    assert!((held[0].0 - moved_goal.0).abs() < 1e-4 && (held[0].1 - moved_goal.1).abs() < 1e-4);
    let (cached_goal, cells) = p.last_path.as_ref().unwrap();
    assert_eq!(
        *cached_goal, held[0],
        "the cached goal must still match the held one"
    );
    assert_eq!(*cells, cached, "the cached cells keep their keys");
}

#[test]
fn a_shift_under_half_a_voxel_keeps_every_key() {
    let cfg = test_config();
    let all = big_world();
    let mut p = built(&cfg, &all);
    let (surface, voxels) = (surface_set(&p), voxel_set(&p));
    let nodes: Vec<_> = p.graph().nodes.iter().map(|n| n.pos).collect();
    p.apply_warp(&warp_of(shift(0.03, 0.0), |_| true), &mut [], &cfg);
    assert_eq!(placed_surface(&p, cfg.voxel_size), surface);
    assert_eq!(placed_voxels(&p, cfg.voxel_size), voxels);
    let after: Vec<_> = p.graph().nodes.iter().map(|n| n.pos).collect();
    for (a, b) in after.iter().zip(&nodes) {
        assert!((a.0 - b.0 - 0.03).abs() < 1e-4, "{a:?} vs {b:?}");
    }
}

#[test]
fn voxel_times_stay_with_their_voxels_through_a_warp() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.track_voxel_times();
    p.set_clock(10.0);
    p.update_global_map(&all, &cfg);
    p.apply_warp(&warp_of(shift(0.8, 0.3), |_| true), &mut [], &cfg);
    p.rebin_near((4.0, 4.0), 100.0, &cfg);
    let times = p.voxel_times.as_ref().unwrap();
    assert_eq!(times.by_key.len(), p.voxel_count());
    assert!(p.voxel_keys().all(|k| times.get(&k) == 10.0));
}

#[test]
fn shifts_smaller_than_a_voxel_add_up_over_warps() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let all = big_world();
    let mut p = built(&cfg, &all);
    let before = voxel_set(&p);
    // A third of a voxel keeps every voxel where it is; the second third takes
    // them past the edge, one voxel over, as a rebuild of the shifted map has them.
    let third = vs as f64 / 3.0 + 1e-4;
    p.apply_warp(&warp_of(shift(third, 0.0), |_| true), &mut [], &cfg);
    assert_eq!(placed_voxels(&p, vs), before);
    p.apply_warp(&warp_of(shift(third, 0.0), |_| true), &mut [], &cfg);
    let rebuilt = built(&cfg, &shifted(&all, 2.0 * third as f32, 0.0));
    assert_eq!(placed_voxels(&p, vs), voxel_set(&rebuilt));
}

/// A closure that moves the map farther than the plan's re-key reach (a lap closure's
/// tens of meters): re-keying only discs around the start and goal leaves their cells
/// keys apart from the cells just outside, and drops movers that land on keys cells
/// still waiting to move hold, so the warped planner had no route that a rebuild of
/// the moved map has (huge_l1 at 8:02, a 19 m closure). It must plan like the rebuild.
#[test]
fn a_shift_beyond_the_rebin_reach_keeps_the_route() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.track_voxel_times();
    p.update_global_map(&all, &cfg);
    let (start, goal) = ((0.5, 0.5, 0.05), (0.5, 7.5, 0.05));
    assert!(!p.plan_or_truncate(start, goal, &cfg).is_empty());
    let mut held = [goal];
    // farther than the map is wide, so no stale key lies where the moved cells land
    p.apply_warp(&warp_of(shift(20.0, 0.0), |_| true), &mut held, &cfg);
    let (start, goal) = ((20.5, 0.5, 0.05), held[0]);
    let rebuilt = built(&cfg, &shifted(&all, 20.0, 0.0));
    let fresh = rebuilt
        .plan(start, goal, &cfg)
        .expect("the moved map has a route");
    let warped = p.plan_or_truncate(start, goal, &cfg);
    assert!(
        !warped.is_empty(),
        "the warped planner lost the route a rebuild has"
    );
    assert!(
        path_len(&warped) < path_len(&fresh) + 1.0,
        "warped route {} m vs rebuilt {} m",
        path_len(&warped),
        path_len(&fresh)
    );
}
