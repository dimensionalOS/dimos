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
        thin_cell_m: -1.0,
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

/// The stages of a full rebuild on a real map: MLS_REBUILD_BENCH=<planner .voxels dump>
/// cargo test --release --lib rebuild_stage_bench -- --ignored --nocapture
#[test]
#[ignore = "needs MLS_REBUILD_BENCH, a mls_reform_planner voxels dump"]
fn rebuild_stage_bench() {
    use crate::adjacency::{build_surface_cells, build_surface_lookup};
    use crate::edges::build_node_edges;
    use crate::nodes::place_nodes;
    use crate::surfaces::extract_surfaces;
    let raw = std::fs::read(std::env::var("MLS_REBUILD_BENCH").unwrap()).unwrap();
    // nav_3d_config's mls_planner_config.
    let cfg = Config {
        voxel_size: 0.08,
        robot_height: 0.45,
        start_z_offset_m: 0.287,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.75,
        wall_buffer_weight: 100.0,
        step_threshold_m: 0.16,
        step_penalty_weight: 4.0,
        ..test_config()
    };
    let points: Vec<(f32, f32, f32)> = raw
        .as_chunks::<12>()
        .0
        .iter()
        .map(|p| {
            let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            (f(0), f(4), f(8))
        })
        .collect();
    let mut p = Planner::new(4);
    let pool = Arc::clone(&p.pool);
    pool.install(|| {
        for &q in &points {
            p.voxel_map.insert(voxelize(q, cfg.voxel_size));
        }
        let t = Instant::now();
        let mut surface = Vec::new();
        extract_surfaces(
            &p.voxel_map,
            cfg.headroom_cells(),
            cfg.closing_passes(),
            &mut p.by_col,
            &mut surface,
        );
        let surfaces = ms_since(t);
        let t = Instant::now();
        build_surface_lookup(&surface, &mut p.graph.surface_lookup);
        let lookup = ms_since(t);
        let t = Instant::now();
        build_surface_cells(&mut p.graph.cells, &p.graph.surface_lookup, cfg.voxel_size, cfg.step_cells());
        let cells = ms_since(t);
        let t = Instant::now();
        place_nodes(
            &mut p.graph.cells,
            &p.by_col,
            &cfg.placement_params(),
            &mut p.graph.wall_state,
            &mut p.graph.nodes,
        );
        let nodes = ms_since(t);
        let t = Instant::now();
        build_node_edges(&p.graph.cells, &p.graph.nodes, &mut p.graph.cell_state, &mut p.graph.node_edges);
        let edges = ms_since(t);
        println!(
            "voxels {} surface cells {}: surfaces {surfaces:.0} lookup {lookup:.0} cells {cells:.0} nodes {nodes:.0} node_edges {edges:.0} ms",
            p.voxel_map.len(),
            surface.len()
        );
    });
}

/// RSS across repeated full rebuilds: MLS_REBUILD_BENCH=<planner .voxels dump>
/// cargo test --release --lib rebuild_rss_bench -- --ignored --nocapture
#[test]
#[ignore = "needs MLS_REBUILD_BENCH, a mls_reform_planner voxels dump"]
fn rebuild_rss_bench() {
    let raw = std::fs::read(std::env::var("MLS_REBUILD_BENCH").unwrap()).unwrap();
    let cfg = Config {
        voxel_size: 0.08,
        robot_height: 0.45,
        start_z_offset_m: 0.287,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.75,
        wall_buffer_weight: 100.0,
        step_threshold_m: 0.16,
        step_penalty_weight: 4.0,
        ..test_config()
    };
    let points: Vec<(f32, f32, f32)> = raw
        .as_chunks::<12>()
        .0
        .iter()
        .map(|p| {
            let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            (f(0), f(4), f(8))
        })
        .collect();
    let threads = std::env::var("MLS_BENCH_THREADS").map_or(4, |t| t.parse().unwrap());
    let mut p = Planner::new(threads);
    p.track_voxel_times();
    let rounds = std::env::var("MLS_BENCH_ROUNDS").map_or(12, |t| t.parse().unwrap());
    for i in 0..rounds {
        let shifted: Vec<(f32, f32, f32)> = points
            .iter()
            .map(|q| (q.0 + 0.03 * i as f32, q.1, q.2))
            .collect();
        let t = Instant::now();
        p.update_global_map(&shifted, &cfg);
        println!(
            "rebuild {i}: {:.0} ms, rss {:.0} MB, {:?}",
            ms_since(t),
            crate::rss::rss_mb().unwrap_or(f64::NAN),
            p.sizes()
        );
    }
}

/// A whole closure on a real map: MLS_REBUILD_BENCH=<planner .voxels dump>
/// MLS_KEYFRAMES=<gsc keyframes dump> cargo test --release --lib warp_bench -- --ignored --nocapture
/// The closure moves the keyframes past MLS_BENCH_FROM (a fraction, default 0.5) from their
/// corrected pose back to their raw one.
#[test]
#[ignore = "needs MLS_REBUILD_BENCH and MLS_KEYFRAMES dumps"]
fn warp_bench() {
    let raw = std::fs::read(std::env::var("MLS_REBUILD_BENCH").unwrap()).unwrap();
    let kf_raw = std::fs::read(std::env::var("MLS_KEYFRAMES").unwrap()).unwrap();
    let cfg = Config {
        voxel_size: 0.08,
        robot_height: 0.45,
        start_z_offset_m: 0.287,
        wall_clearance_m: 0.1,
        wall_buffer_m: 0.75,
        wall_buffer_weight: 100.0,
        step_threshold_m: 0.16,
        step_penalty_weight: 4.0,
        ..test_config()
    };
    let points: Vec<(f32, f32, f32)> = raw
        .as_chunks::<12>()
        .0
        .iter()
        .map(|p| {
            let f = |i: usize| f32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            (f(0), f(4), f(8))
        })
        .collect();
    let f64s: Vec<f64> = kf_raw
        .as_chunks::<8>()
        .0
        .iter()
        .map(|b| f64::from_le_bytes(*b))
        .collect();
    let pose = |v: &[f64]| Pose {
        translation: Vec3::new(v[0], v[1], v[2]),
        rotation: Quat::new(v[3], v[4], v[5], v[6]).normalized_or_identity(),
    };
    let keyframes: Vec<(f64, Pose, Pose)> = f64s
        .as_chunks::<15>()
        .0
        .iter()
        .map(|r| (r[0], pose(&r[1..8]), pose(&r[8..15])))
        .collect();
    let threads = std::env::var("MLS_BENCH_THREADS").map_or(8, |t| t.parse().unwrap());
    let mut p = Planner::new(threads);
    p.track_voxel_times();
    p.set_clock(keyframes[0].0);
    p.update_global_map(&points, &cfg);
    // Each voxel stamped as live stamps it: about when the drive passed it, here the
    // time of its nearest keyframe (1 m grid lookup, widened until one is found).
    let mut grid: AHashMap<(i32, i32), Vec<usize>> = AHashMap::new();
    for (k, (_, pose, _)) in keyframes.iter().enumerate() {
        let t = pose.translation;
        grid.entry((t.x.floor() as i32, t.y.floor() as i32))
            .or_default()
            .push(k);
    }
    let nearest_time = |(x, y, z): (f32, f32, f32)| {
        let (cx, cy) = (x.floor() as i32, y.floor() as i32);
        for r in 0..200 {
            let mut best: Option<(f64, f64)> = None;
            for dx in -r..=r {
                for dy in -r..=r {
                    for &k in grid.get(&(cx + dx, cy + dy)).into_iter().flatten() {
                        let t = keyframes[k].1.translation;
                        let d = (t.x - x as f64).powi(2)
                            + (t.y - y as f64).powi(2)
                            + (t.z - z as f64).powi(2);
                        if best.is_none_or(|(bd, _)| d < bd) {
                            best = Some((d, keyframes[k].0));
                        }
                    }
                }
            }
            if let Some((_, ts)) = best {
                return ts;
            }
        }
        keyframes[0].0
    };
    let times = p.voxel_times.as_mut().unwrap();
    let epoch = times.epoch.unwrap();
    let stamped: Vec<(VoxelKey, f32)> = times
        .by_key
        .keys()
        .map(|&k| {
            let c = (
                (k.0 as f32 + 0.5) * 0.08,
                (k.1 as f32 + 0.5) * 0.08,
                (k.2 as f32 + 0.5) * 0.08,
            );
            (k, (nearest_time(c) - epoch) as f32)
        })
        .collect();
    times.by_key.extend(stamped);
    let records: Vec<KeyframeRecord> = keyframes
        .iter()
        .enumerate()
        .map(|(i, (ts, corrected, _))| KeyframeRecord {
            id: i as u32,
            ts: *ts,
            pose: *corrected,
        })
        .collect();
    let mut state = ReformState::default();
    state.add_keyframes(&records);
    let from: f64 = std::env::var("MLS_BENCH_FROM").map_or(0.5, |t| t.parse().unwrap());
    let half = (records.len() as f64 * from) as usize;
    // The later half drifts back by the gap between raw and corrected, like a big closure.
    let moved: Vec<MovedRecord> = records[half..]
        .iter()
        .zip(&keyframes[half..])
        .map(|(k, (_, corrected, raw))| MovedRecord {
            keyframe: *k,
            delta: raw.compose(&corrected.inverse()),
        })
        .collect();
    let params = ReformParams {
        blend_sigma_m: 0.0,
        blend_time_sigma_s: 5.0,
        min_shift_m: 0.04,
        thin_cell_m: -1.0,
    };
    let warp = state
        .closure(&moved, &params)
        .expect("the closure moves keyframes");
    let report = p.apply_warp(&warp, &mut [], &cfg);
    println!("{report:?}");
}
