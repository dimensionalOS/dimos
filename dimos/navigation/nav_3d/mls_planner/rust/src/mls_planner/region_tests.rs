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

use super::*;
use std::collections::{BTreeMap, BTreeSet};

fn tiles_for(
    p: &Planner,
    points: &[(f32, f32, f32)],
    center: (f32, f32),
    cfg: &Config,
) -> Vec<MapTile> {
    let part = partition_cloud(points, cfg.full_map_tile_m, cfg.voxel_size);
    p.finish_partition(part, center, cfg)
}

fn load_full_map(p: &mut Planner, points: &[(f32, f32, f32)], center: (f32, f32), cfg: &Config) {
    for tile in tiles_for(p, points, center, cfg) {
        p.update_region(&tile.points, &tile.bounds, cfg);
    }
}

fn queue_load(
    p: &mut Planner,
    points: &[(f32, f32, f32)],
    center: (f32, f32),
    cfg: &Config,
) -> usize {
    let part = partition_cloud(points, cfg.full_map_tile_m, cfg.voxel_size);
    p.start_load(part, center, cfg)
}

fn finish_load(p: &mut Planner, cfg: &Config) {
    while !matches!(p.apply_next_tile(cfg), LoadStep::Idle) {}
}

/// Slack for comparing regional and full-rebuild path lengths. Node
/// placement differs between the two, so paths are equivalent, not equal.
const PATH_LEN_RATIO: f32 = 1.6;
const PATH_LEN_SLACK_M: f32 = 0.5;

fn test_config() -> Config {
    Config {
        world_frame: String::new(),
        base_frame: String::new(),
        voxel_size: 0.1,
        robot_height: 0.5,
        start_z_offset_m: 0.0,
        max_overhead_m: 2.0,
        surface_closing_radius: 0.3,
        node_spacing_m: 1.0,
        wall_clearance_m: 0.0,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 1.0,
        step_threshold_m: 0.25,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        full_map_tile_m: 2.0,
        viz_publish_hz: 2.0,
        worker_threads: 4,
    }
}

/// Floor slab with a wall down the middle, as world-frame point centers.
fn world_points() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..40 {
        for iy in 0..40 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    // a wall column from z=0 up, to create wall-adjacency for nodes
    for iy in 0..40 {
        for iz in 0..15 {
            pts.push((
                20.0 * vs + half,
                iy as f32 * vs + half,
                iz as f32 * vs + half,
            ));
        }
    }
    pts
}

fn surface_set(p: &Planner) -> BTreeSet<VoxelKey> {
    p.surface().collect()
}

fn voxel_set(p: &Planner) -> BTreeSet<VoxelKey> {
    p.voxel_map.iter().copied().collect()
}

/// Cell adjacency keyed by coordinate, independent of CellId.
fn cell_edges(p: &Planner) -> BTreeMap<VoxelKey, BTreeSet<(VoxelKey, u32)>> {
    let cells = &p.graph.cells;
    let mut out: BTreeMap<VoxelKey, BTreeSet<(VoxelKey, u32)>> = BTreeMap::new();
    for (id, edges) in cells.iter() {
        let src = cells.coord(id);
        let set = out.entry(src).or_default();
        for e in edges {
            set.insert((cells.coord(e.dest), e.cost.to_bits()));
        }
    }
    out
}

fn node_coords(p: &Planner) -> BTreeSet<VoxelKey> {
    p.graph
        .nodes
        .iter()
        .map(|n| p.graph.cells.coord(n.cell_id))
        .collect()
}

fn node_edge_pairs(p: &Planner) -> BTreeSet<(VoxelKey, VoxelKey, u32)> {
    let cells = &p.graph.cells;
    p.graph
        .node_edges
        .iter()
        .map(|e| {
            let a = cells.coord(e.a);
            let b = cells.coord(e.b);
            let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
            (lo, hi, e.cost.to_bits())
        })
        .collect()
}

#[test]
fn region_update_removes_stale_voxels() {
    let cfg = test_config();
    let bounds = RegionBounds {
        origin_x: 2.0,
        origin_y: 2.0,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let all = world_points();

    let mut full = Planner::new(cfg.worker_threads);
    full.update_global_map(&all, &cfg);

    let inside: Vec<_> = all
        .iter()
        .copied()
        .filter(|&p| bounds.contains_voxel(voxelize(p, cfg.voxel_size), cfg.voxel_size))
        .collect();
    let outside: Vec<_> = all
        .iter()
        .copied()
        .filter(|&p| !bounds.contains_voxel(voxelize(p, cfg.voxel_size), cfg.voxel_size))
        .collect();

    // Seed the cylinder with a stack of junk voxels not present in the
    // world, so update_region must clear them and the surface they induce.
    let mut seeded = outside.clone();
    for iz in 3..8 {
        seeded.push((2.05, 2.05, iz as f32 * cfg.voxel_size + 0.05));
    }
    let mut region = Planner::new(cfg.worker_threads);
    region.update_global_map(&seeded, &cfg);
    region.update_region(&inside, &bounds, &cfg);

    assert_eq!(voxel_set(&region), voxel_set(&full), "voxel mismatch");
    assert_eq!(surface_set(&region), surface_set(&full), "surface mismatch");
    assert_eq!(
        cell_edges(&region),
        cell_edges(&full),
        "cell edges mismatch"
    );
    // Nodes are sticky, not re-derived, so their positions may differ
    // from a fresh build. Equivalent planning is the contract.
    let s = (0.5, 0.5, 0.1);
    let g = (1.5, 3.5, 0.1);
    let pf = full.plan(s, g, &cfg).expect("full build plans");
    let pr = region.plan(s, g, &cfg).expect("region build plans");
    let (lf, lr) = (path_len(&pf), path_len(&pr));
    assert!(
        lr <= lf * PATH_LEN_RATIO + PATH_LEN_SLACK_M,
        "region path too long: {lr} vs {lf}"
    );
    assert!(
        lf <= lr * PATH_LEN_RATIO + PATH_LEN_SLACK_M,
        "full path too long: {lf} vs {lr}"
    );
}

/// A local change must not move nodes beyond its reach. Distant nodes are
/// sticky by contract, which keeps refined paths stable frame to frame.
/// A floor patch raised by one voxel must relocate its nodes upward in
/// place. Every node's position must match the cell it claims, which a
/// stale-id relocation breaks by leaving nodes meters from their cell.
#[test]
fn raised_floor_relocates_nodes_in_place() {
    use crate::voxel::surface_point_xyz;

    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let in_patch = |pos: (f32, f32, f32)| {
        let d = (pos.0 - 1.5, pos.1 - 1.5);
        (d.0 * d.0 + d.1 * d.1).sqrt() < 1.2
    };
    let doomed_xy: Vec<(f32, f32)> = p
        .graph
        .nodes
        .iter()
        .filter(|n| in_patch(n.pos))
        .map(|n| (n.pos.0, n.pos.1))
        .collect();
    assert!(!doomed_xy.is_empty(), "test needs a node inside the patch");

    let b = RegionBounds {
        origin_x: 1.5,
        origin_y: 1.5,
        radius: 1.2,
        z_min: -1.0,
        z_max: 2.0,
    };
    let pts: Vec<(f32, f32, f32)> = slice(&all, &b, vs)
        .iter()
        .map(|&(x, y, _)| (x, y, 0.15))
        .collect();
    p.update_region(&pts, &b, &cfg);

    for n in p.graph.nodes.iter() {
        let k = p.graph.cells.coord(n.cell_id);
        assert_eq!(
            n.pos,
            surface_point_xyz(k.0, k.1, k.2, vs),
            "node pos must match its cell {k:?}"
        );
    }
    // Relocation preserves each node's xy exactly. A drop-and-re-derive
    // pass would place fresh NMS nodes at different cells.
    for &(x, y) in &doomed_xy {
        assert!(
            p.graph
                .nodes
                .iter()
                .any(|n| n.pos.0 == x && n.pos.1 == y && (n.pos.2 - 0.2).abs() < 1e-6),
            "node at ({x}, {y}) must relocate up in place"
        );
    }
}

/// A junk voxel near a node shifts the local wall-distance field. A
/// fresh re-derivation would move the node to the new local maximum, but
/// sticky retention must keep it exactly where it was.
#[test]
fn sticky_node_keeps_its_place_when_the_field_shifts() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);

    // The node nearest the center of the open left half.
    let target = p
        .graph
        .nodes
        .iter()
        .map(|n| n.pos)
        .min_by(|a, b| {
            let da = (a.0 - 2.0).powi(2) + (a.1 - 2.0).powi(2);
            let db = (b.0 - 2.0).powi(2) + (b.1 - 2.0).powi(2);
            da.total_cmp(&db)
        })
        .unwrap();

    // Junk appears 0.25 m from the node, shrinking its wall distance.
    let b = RegionBounds {
        origin_x: target.0,
        origin_y: target.1,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let mut pts = slice(&all, &b, vs);
    pts.push((target.0 + 0.25, target.1, 0.45));
    p.update_region(&pts, &b, &cfg);

    assert!(
        p.graph.nodes.iter().any(|n| n.pos == target),
        "sticky node moved on a nearby junk voxel: {target:?}"
    );
}

/// The drop side of sticky retention: a node whose cell falls inside the
/// clearance zone of newly grown structure dies instead of persisting.
#[test]
fn sticky_node_dies_when_a_wall_grows_next_to_it() {
    let cfg = Config {
        wall_clearance_m: 0.2,
        ..test_config()
    };
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);

    let target = p
        .graph
        .nodes
        .iter()
        .map(|n| n.pos)
        .min_by(|a, b| {
            let da = (a.0 - 2.0).powi(2) + (a.1 - 2.0).powi(2);
            let db = (b.0 - 2.0).powi(2) + (b.1 - 2.0).powi(2);
            da.total_cmp(&db)
        })
        .unwrap();

    // A wall stack grows in the column right next to the node's cell.
    let b = RegionBounds {
        origin_x: target.0,
        origin_y: target.1,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let mut pts = slice(&all, &b, vs);
    for iz in 0..5 {
        pts.push((target.0 + vs, target.1, iz as f32 * vs + vs * 0.5));
    }
    p.update_region(&pts, &b, &cfg);

    assert!(
        !p.graph.nodes.iter().any(|n| n.pos == target),
        "sub-clearance node must be dropped, not kept: {target:?}"
    );
}

#[test]
fn sticky_nodes_survive_distant_changes() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before = node_coords(&p);

    // A junk voxel appears in one corner: a genuine local change.
    let b = RegionBounds {
        origin_x: 1.0,
        origin_y: 1.0,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let mut pts = slice(&all, &b, vs);
    pts.push((1.05, 1.05, 0.45));
    p.update_region(&pts, &b, &cfg);

    let after = node_coords(&p);
    let far = |c: &VoxelKey| {
        let x = c.0 as f32 * vs;
        let y = c.1 as f32 * vs;
        ((x - 1.0).powi(2) + (y - 1.0).powi(2)).sqrt() > 3.5
    };
    for c in before.iter().filter(|c| far(c)) {
        assert!(
            after.contains(c),
            "distant node {c:?} moved on a local change"
        );
    }
}

/// A point outside the region bounds must not enter the planner's voxel
/// map, where it could never be cleared and would inflate the rebuild box.
#[test]
fn region_update_ignores_points_outside_bounds() {
    let cfg = test_config();
    let bounds = RegionBounds {
        origin_x: 2.0,
        origin_y: 2.0,
        radius: 1.0,
        z_min: -1.0,
        z_max: 2.0,
    };
    let inside = (2.05, 2.05, 0.05);
    let outside = (10.05, 10.05, 0.05);

    let mut p = Planner::new(cfg.worker_threads);
    p.update_region(&[inside, outside], &bounds, &cfg);

    assert!(p.voxel_map.contains(&voxelize(inside, cfg.voxel_size)));
    assert!(!p.voxel_map.contains(&voxelize(outside, cfg.voxel_size)));
}

/// Floor 8m x 8m with a wall at x=4m that only a gap at y in [3.5, 4.5]
/// passes through, so crossing the wall is a non-trivial route.
fn big_world() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..80 {
        for iy in 0..80 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    for iy in 0..80 {
        if (35..45).contains(&iy) {
            continue;
        }
        for iz in 0..15 {
            pts.push((
                40.0 * vs + half,
                iy as f32 * vs + half,
                iz as f32 * vs + half,
            ));
        }
    }
    pts
}

fn slice(all: &[(f32, f32, f32)], b: &RegionBounds, vs: f32) -> Vec<(f32, f32, f32)> {
    all.iter()
        .copied()
        .filter(|&p| b.contains_voxel(voxelize(p, vs), vs))
        .collect()
}

fn path_len(w: &[(f32, f32, f32)]) -> f32 {
    w.windows(2)
        .map(|p| {
            let dx = p[1].0 - p[0].0;
            let dy = p[1].1 - p[0].1;
            let dz = p[1].2 - p[0].2;
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

type Pose = (f32, f32, f32);
const PLAN_PAIRS: [(Pose, Pose); 4] = [
    ((0.5, 0.5, 0.05), (7.5, 7.5, 0.05)),
    ((0.5, 7.5, 0.05), (7.5, 0.5, 0.05)),
    ((0.5, 0.5, 0.05), (0.5, 7.5, 0.05)),
    ((7.5, 0.5, 0.05), (7.5, 7.5, 0.05)),
];

fn assert_plans_equivalent(full: &Planner, region: &Planner, cfg: &Config) {
    for (s, g) in PLAN_PAIRS {
        let pf = full.plan(s, g, cfg);
        let pr = region.plan(s, g, cfg);
        assert_eq!(
            pf.is_some(),
            pr.is_some(),
            "path existence differs for {s:?} -> {g:?}"
        );
        if let (Some(pf), Some(pr)) = (pf, pr) {
            let (lf, lr) = (path_len(&pf), path_len(&pr));
            assert!(
                lr <= lf * PATH_LEN_RATIO + PATH_LEN_SLACK_M,
                "region path too long: {lr} vs {lf}"
            );
            assert!(
                lf <= lr * PATH_LEN_RATIO + PATH_LEN_SLACK_M,
                "full path too long: {lf} vs {lr}"
            );
        }
    }
}

/// Re-observing the same geometry must change nothing: no voxel, surface,
/// cell, node, or edge moves. This is the anti-jitter guarantee.
#[test]
fn region_reobserve_leaves_graph_bit_identical() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;

    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before_cells = cell_edges(&p);
    let before_nodes = node_coords(&p);
    let before_edges = node_edge_pairs(&p);

    for &(cx, cy) in &[(2.0, 2.0), (4.0, 4.0), (6.0, 3.0), (1.5, 7.0), (7.0, 7.0)] {
        let b = RegionBounds {
            origin_x: cx,
            origin_y: cy,
            radius: 1.2,
            z_min: -1.0,
            z_max: 2.0,
        };
        p.update_region(&slice(&all, &b, vs), &b, &cfg);
    }

    assert_eq!(
        cell_edges(&p),
        before_cells,
        "cells changed on re-observation"
    );
    assert_eq!(
        node_coords(&p),
        before_nodes,
        "nodes moved on re-observation"
    );
    assert_eq!(
        node_edge_pairs(&p),
        before_edges,
        "edges changed on re-observation"
    );
}

/// Build the planner purely from streamed local cylinders, as the live
/// pipeline does, and require equivalent planning to a one-shot full build.
#[test]
fn region_stream_only_plans_like_full() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;

    let mut full = Planner::new(cfg.worker_threads);
    full.update_global_map(&all, &cfg);

    let mut region = Planner::new(cfg.worker_threads);
    let mut cx = 0.5;
    while cx <= 7.5 {
        let mut cy = 0.5;
        while cy <= 7.5 {
            let b = RegionBounds {
                origin_x: cx,
                origin_y: cy,
                radius: 1.5,
                z_min: -1.0,
                z_max: 2.0,
            };
            let s = slice(&all, &b, vs);
            if !s.is_empty() {
                region.update_region(&s, &b, &cfg);
            }
            cy += 1.0;
        }
        cx += 1.0;
    }

    assert_eq!(
        voxel_set(&region),
        voxel_set(&full),
        "stream did not reconstruct the map"
    );
    assert_plans_equivalent(&full, &region, &cfg);
}

/// Floor split by a wall with a narrow 1-cell gap near x=1.0 and a wide gap
/// near x=4.5. Start and goal straddle the narrow gap.
fn two_gap_world() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..60 {
        for iy in 0..40 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    for ix in 0..60 {
        if ix == 10 || (40..50).contains(&ix) {
            continue;
        }
        for iz in 0..7 {
            pts.push((
                ix as f32 * vs + half,
                20.0 * vs + half,
                iz as f32 * vs + half,
            ));
        }
    }
    pts
}

/// The hard clearance floor must make the narrow gap impassable, forcing
/// the longer detour through the wide gap.
#[test]
fn hard_clearance_floor_avoids_narrow_gap() {
    let mut cfg = test_config();
    cfg.node_spacing_m = 0.8;
    let pts = two_gap_world();
    let start = (1.0, 1.0, 0.05);
    let goal = (1.0, 3.5, 0.05);
    let max_x = |w: &[(f32, f32, f32)]| w.iter().map(|p| p.0).fold(f32::MIN, f32::max);

    // No clearance: the shortest route slips straight through the narrow gap.
    cfg.wall_clearance_m = 0.0;
    let mut open = Planner::new(cfg.worker_threads);
    open.update_global_map(&pts, &cfg);
    let wp_open = open.plan(start, goal, &cfg).expect("open plan exists");

    // Clearance wider than the narrow gap: it is impassable, so detour wide.
    cfg.wall_clearance_m = 0.2;
    let mut safe = Planner::new(cfg.worker_threads);
    safe.update_global_map(&pts, &cfg);
    let wp_safe = safe.plan(start, goal, &cfg).expect("safe plan exists");

    assert!(max_x(&wp_open) < 2.0, "open path should use the near gap");
    assert!(
        max_x(&wp_safe) > 3.5,
        "safe path should detour to the wide gap: max_x={}",
        max_x(&wp_safe)
    );
    assert!(
        path_len(&wp_safe) > path_len(&wp_open) * 1.5,
        "safe route should be substantially longer: {} vs {}",
        path_len(&wp_safe),
        path_len(&wp_open)
    );
}

/// Every cell the smoothed path crosses, between waypoints included, must
/// clear the hard wall distance.
#[test]
fn final_path_clears_wall_distance() {
    let mut cfg = test_config();
    cfg.wall_clearance_m = 0.2;
    cfg.wall_buffer_m = 0.5;
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);

    let wp = p
        .plan((0.7, 4.0, 0.05), (7.3, 4.0, 0.05), &cfg)
        .expect("plan exists");
    let clearance: std::collections::HashMap<VoxelKey, f32> =
        p.surface_clearance().into_iter().collect();
    let vs = cfg.voxel_size;
    let key = |x: f32, y: f32, z: f32| {
        (
            (x / vs).floor() as i32,
            (y / vs).floor() as i32,
            (z / vs).round() as i32 - 1,
        )
    };

    // Interior waypoints are exact cell centers. Sample between them too.
    let interior = &wp[1..wp.len() - 1];
    assert!(interior.len() >= 2, "expected a multi-cell path");
    for pair in interior.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        for k in 0..=24 {
            let t = k as f32 / 24.0;
            let x = a.0 + t * (b.0 - a.0);
            let y = a.1 + t * (b.1 - a.1);
            let z = a.2 + t * (b.2 - a.2);
            if let Some(&c) = clearance.get(&key(x, y, z)) {
                assert!(
                    c >= cfg.wall_clearance_m - 1e-4,
                    "path point ({x:.2},{y:.2}) sits {c:.3} from a wall, under the {} clearance",
                    cfg.wall_clearance_m
                );
            }
        }
    }
}

/// Solid 0.3 m block, taller than the step threshold. The path must route
/// around it and never climb on.
fn block_world() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..40 {
        for iy in 0..12 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    // A solid block, 0.3 m tall, blocking the iy 0..6 lane around ix 18..22.
    for ix in 18..22 {
        for iy in 0..6 {
            for iz in 0..4 {
                pts.push((
                    ix as f32 * vs + half,
                    iy as f32 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
    }
    pts
}

#[test]
fn final_path_never_climbs_over_threshold_step() {
    let mut cfg = test_config();
    cfg.surface_closing_radius = 0.0;
    cfg.wall_clearance_m = 0.0;
    cfg.wall_buffer_m = 0.0;
    cfg.node_spacing_m = 0.5;
    let pts = block_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&pts, &cfg);

    let wp = p
        .plan((1.0, 0.5, 0.05), (3.9, 0.5, 0.05), &cfg)
        .expect("plan exists");

    // The block top is at z = 0.4. The floor surface point is z = 0.1. No
    // interior waypoint may land on the block.
    for w in &wp[1..wp.len() - 1] {
        assert!(
            w.2 < 0.25,
            "path climbed onto the 0.3 m block at {w:?}, exceeding the step threshold"
        );
    }
    // It had to detour out of the blocked lane (iy < 0.6).
    let max_y = wp.iter().map(|p| p.1).fold(f32::MIN, f32::max);
    assert!(
        max_y > 0.6,
        "path did not detour around the block: max_y={max_y}"
    );
}

/// Flat floor with a crossable 0.2 m ridge blocking ix 15 except a flat gap
/// at iy 10..12. Crossing is short but climbs two steps. The detour is flat.
/// Route choice is read from the xy lane, since smoothing flattens the ridge
/// waypoints away.
fn ridge_world() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..40 {
        for iy in 0..12 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    // A 0.2 m ridge cap at ix 15, iy 0..10: a 2-cell step up and back down.
    for iy in 0..10 {
        pts.push((15.0 * vs + half, iy as f32 * vs + half, 2.0 * vs + half));
    }
    pts
}

#[test]
fn step_penalty_diverts_path_around_ridge() {
    let mut cfg = test_config();
    cfg.surface_closing_radius = 0.0;
    cfg.wall_clearance_m = 0.0;
    cfg.wall_buffer_m = 0.0;
    cfg.node_spacing_m = 0.5;
    let pts = ridge_world();
    let start = (1.0, 0.5, 0.05);
    let goal = (2.9, 0.5, 0.05);
    let max_y = |w: &[(f32, f32, f32)]| w.iter().map(|p| p.1).fold(f32::MIN, f32::max);

    // No step penalty: the short route crosses the ridge low.
    cfg.step_penalty_weight = 0.0;
    let mut cheap = Planner::new(cfg.worker_threads);
    cheap.update_global_map(&pts, &cfg);
    let wp_cheap = cheap.plan(start, goal, &cfg).expect("plan exists");

    // Heavy step penalty: the flat detour to the iy 10 gap wins.
    cfg.step_penalty_weight = 30.0;
    let mut avoid = Planner::new(cfg.worker_threads);
    avoid.update_global_map(&pts, &cfg);
    let wp_avoid = avoid.plan(start, goal, &cfg).expect("plan exists");

    assert!(
        max_y(&wp_cheap) < 0.6,
        "with no step penalty the path should cross the ridge low: max_y={}",
        max_y(&wp_cheap)
    );
    assert!(
        max_y(&wp_avoid) > 0.9,
        "with a heavy step penalty the path should detour to the flat gap: max_y={}",
        max_y(&wp_avoid)
    );
}

/// Loading a whole cloud tile by tile must build exactly what a full
/// rebuild builds, and plan equivalently.
#[test]
fn full_map_load_matches_full_rebuild() {
    let cfg = test_config();
    let all = big_world();

    let mut full = Planner::new(cfg.worker_threads);
    full.update_global_map(&all, &cfg);

    let mut loaded = Planner::new(cfg.worker_threads);
    load_full_map(&mut loaded, &all, (0.5, 0.5), &cfg);

    assert_eq!(voxel_set(&loaded), voxel_set(&full), "voxel mismatch");
    assert_eq!(surface_set(&loaded), surface_set(&full), "surface mismatch");
    assert_eq!(
        cell_edges(&loaded),
        cell_edges(&full),
        "cell edges mismatch"
    );
    assert_plans_equivalent(&full, &loaded, &cfg);
}

/// Tiles must claim points by voxel center, so an off-center cloud loads
/// the same voxels a full rebuild quantizes.
#[test]
fn full_map_load_matches_full_rebuild_off_center() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let mut seed: u32 = 12345;
    let mut jitter = || {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (seed >> 8) as f32 / (1u32 << 24) as f32 * 0.8 * vs - 0.4 * vs
    };
    let all: Vec<(f32, f32, f32)> = big_world()
        .into_iter()
        .map(|(x, y, z)| (x + jitter(), y + jitter(), z))
        .collect();

    let mut full = Planner::new(cfg.worker_threads);
    full.update_global_map(&all, &cfg);

    let mut loaded = Planner::new(cfg.worker_threads);
    load_full_map(&mut loaded, &all, (0.5, 0.5), &cfg);

    assert_eq!(voxel_set(&loaded), voxel_set(&full), "voxel mismatch");
    assert_eq!(surface_set(&loaded), surface_set(&full), "surface mismatch");
}

/// A load sweeps every voxel absent from the cloud, including stale
/// geometry outside the cloud's xy extent and above its ceiling.
#[test]
fn full_map_load_sweeps_absent_voxels() {
    let cfg = test_config();
    let all = big_world();

    let mut junk = all.clone();
    junk.push((15.05, 15.05, 0.05));
    junk.push((2.05, 2.05, 5.05));
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&junk, &cfg);

    load_full_map(&mut p, &all, (0.5, 0.5), &cfg);

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&all, &cfg);
    assert_eq!(
        voxel_set(&p),
        voxel_set(&clean),
        "stale voxels survived the load"
    );
    assert_eq!(surface_set(&p), surface_set(&clean));
}

/// Reloading the same cloud is a no-op: nodes placed before the load
/// survive untouched.
#[test]
fn full_map_reload_leaves_graph_bit_identical() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before_cells = cell_edges(&p);
    let before_nodes = node_coords(&p);
    let before_edges = node_edge_pairs(&p);

    load_full_map(&mut p, &all, (4.0, 4.0), &cfg);

    assert_eq!(cell_edges(&p), before_cells, "cells changed on reload");
    assert_eq!(node_coords(&p), before_nodes, "nodes moved on reload");
    assert_eq!(node_edge_pairs(&p), before_edges, "edges changed on reload");
}

/// Distant sticky nodes survive a load that carries a small local change.
#[test]
fn full_map_load_keeps_distant_nodes_sticky() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before = node_coords(&p);

    let mut changed = all.clone();
    changed.push((1.05, 1.05, 0.45));
    load_full_map(&mut p, &changed, (1.0, 1.0), &cfg);

    let after = node_coords(&p);
    let far = |c: &VoxelKey| {
        let x = c.0 as f32 * vs;
        let y = c.1 as f32 * vs;
        ((x - 1.0).powi(2) + (y - 1.0).powi(2)).sqrt() > 3.5
    };
    for c in before.iter().filter(|c| far(c)) {
        assert!(after.contains(c), "distant node {c:?} moved on a load");
    }
}

/// The load takes its z band from the cloud, so geometry far above any
/// sensor overhead cap still lands.
#[test]
fn full_map_load_keeps_high_geometry() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let half = vs * 0.5;
    let mut all = big_world();
    for ix in 10..20 {
        for iy in 10..20 {
            all.push((
                ix as f32 * vs + half,
                iy as f32 * vs + half,
                30.0 * vs + half,
            ));
        }
    }
    let mut p = Planner::new(cfg.worker_threads);
    load_full_map(&mut p, &all, (0.5, 0.5), &cfg);
    assert!(
        p.voxel_map.contains(&(15, 15, 30)),
        "high platform truncated"
    );
}

/// Tiles come back nearest the given center first.
#[test]
fn partition_orders_tiles_near_center_first() {
    let cfg = test_config();
    let all = big_world();
    let p = Planner::new(cfg.worker_threads);
    let tiles = tiles_for(&p, &all, (0.0, 0.0), &cfg);
    assert!(tiles.len() >= 4);
    let d: Vec<f32> = tiles
        .iter()
        .map(|t| t.bounds.origin_x.powi(2) + t.bounds.origin_y.powi(2))
        .collect();
    assert!(
        d.windows(2).all(|w| w[0] <= w[1]),
        "tiles not sorted near-first: {d:?}"
    );
}

fn cyl(x: f32, y: f32, r: f32) -> RegionBounds {
    RegionBounds {
        origin_x: x,
        origin_y: y,
        radius: r,
        z_min: -1.0,
        z_max: 1.0,
    }
}

#[test]
fn map_load_skips_only_tiles_inside_applied_regions() {
    let cfg = test_config();
    let mut p = Planner::new(cfg.worker_threads);
    let tile = |x: f32| MapTile {
        bounds: cyl(x, 0.0, 1.0),
        points: Vec::new(),
    };
    p.load = Some(MapLoad::new(vec![
        tile(0.0),
        tile(2.0),
        tile(4.0),
        tile(6.0),
    ]));
    assert!(matches!(
        p.apply_next_tile(&cfg),
        LoadStep::Applied { remaining: 3 }
    ));

    // A region grazing the second tile does not cover it, so it applies.
    p.update_region(&[], &cyl(2.5, 0.0, 1.0), &cfg);
    assert!(matches!(
        p.apply_next_tile(&cfg),
        LoadStep::Applied { remaining: 2 }
    ));

    // A region covering the third tile makes it stale, so the last applies.
    p.update_region(&[], &cyl(4.0, 0.0, 1.5), &cfg);
    assert!(matches!(p.apply_next_tile(&cfg), LoadStep::Finished { .. }));
    assert!(!p.loading());
    assert!(matches!(p.apply_next_tile(&cfg), LoadStep::Idle));
}

#[test]
fn map_load_keeps_only_regions_no_other_covers() {
    let mut load = MapLoad::new(Vec::new());
    load.region_applied(cyl(0.0, 0.0, 2.0));
    load.region_applied(cyl(0.5, 0.0, 1.0));
    assert_eq!(load.regions.len(), 1, "a covered region is dropped");
    load.region_applied(cyl(0.0, 0.0, 5.0));
    assert_eq!(
        load.regions.len(),
        1,
        "a covering region replaces the one under it"
    );
    assert_eq!(load.regions[0].radius, 5.0);
    load.region_applied(cyl(9.0, 0.0, 1.0));
    assert_eq!(load.regions.len(), 2, "a disjoint region is kept");
}

/// A live region landing mid-load leaves nothing unloaded: the tiles it
/// straddles still apply, so the map ends equal to the cloud.
#[test]
fn full_map_load_covers_around_a_live_region() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    queue_load(&mut p, &all, (4.0, 4.0), &cfg);
    assert!(matches!(p.apply_next_tile(&cfg), LoadStep::Applied { .. }));

    let live = RegionBounds {
        origin_x: 4.0,
        origin_y: 4.0,
        radius: 3.0,
        z_min: -0.1,
        z_max: 2.0,
    };
    p.update_region(&slice(&all, &live, cfg.voxel_size), &live, &cfg);
    finish_load(&mut p, &cfg);
    assert!(!p.loading());

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&all, &cfg);
    assert_eq!(
        voxel_set(&p),
        voxel_set(&clean),
        "a tile straddling the live region went unloaded"
    );
    assert_eq!(surface_set(&p), surface_set(&clean));
}

/// big_world plus a 3x3 column box, 5 voxels tall, astride (4.0, 4.0).
fn boxed_world() -> Vec<(f32, f32, f32)> {
    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = big_world();
    for ix in 39..42 {
        for iy in 39..42 {
            for iz in 0..5 {
                pts.push((
                    ix as f32 * vs + half,
                    iy as f32 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
    }
    pts
}

/// A live region straddling the 2 m tiles around (4.0, 4.0) without
/// covering any of them.
fn straddling_live() -> RegionBounds {
    RegionBounds {
        origin_x: 4.0,
        origin_y: 4.0,
        radius: 1.5,
        z_min: -0.1,
        z_max: 2.0,
    }
}

/// A tile only partly under a live region must not paste snapshot
/// geometry back where the live update cleared it.
#[test]
fn tile_straddling_a_live_region_keeps_what_live_cleared() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let snapshot = boxed_world();
    let cleared = big_world();

    let mut p = Planner::new(cfg.worker_threads);
    queue_load(&mut p, &snapshot, (0.5, 0.5), &cfg);
    assert!(matches!(p.apply_next_tile(&cfg), LoadStep::Applied { .. }));

    let live = straddling_live();
    let tiles = &p.load.as_ref().expect("load pending").tiles;
    assert!(tiles.iter().all(|t| !live.covers_xy(&t.bounds)));
    assert!(tiles.iter().any(|t| live.intersects(&t.bounds)));
    p.update_region(&slice(&cleared, &live, vs), &live, &cfg);
    finish_load(&mut p, &cfg);
    assert!(!p.loading());

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&cleared, &cfg);
    assert_eq!(
        voxel_set(&p),
        voxel_set(&clean),
        "a straddling tile pasted the snapshot over the live region"
    );
    assert_eq!(surface_set(&p), surface_set(&clean));
}

/// A tile only partly under a live region must not delete what the live
/// update saw and the snapshot lacks.
#[test]
fn tile_straddling_a_live_region_keeps_what_live_saw() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let snapshot = big_world();
    let seen = boxed_world();

    let mut p = Planner::new(cfg.worker_threads);
    queue_load(&mut p, &snapshot, (0.5, 0.5), &cfg);
    assert!(matches!(p.apply_next_tile(&cfg), LoadStep::Applied { .. }));

    let live = straddling_live();
    p.update_region(&slice(&seen, &live, vs), &live, &cfg);
    finish_load(&mut p, &cfg);
    assert!(!p.loading());

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&seen, &cfg);
    assert_eq!(
        voxel_set(&p),
        voxel_set(&clean),
        "a straddling tile deleted what the live region saw"
    );
    assert_eq!(surface_set(&p), surface_set(&clean));
}

/// A tile a capped live region covers must not paste the snapshot back
/// over it, while its ceiling above the cap still loads.
#[test]
fn capped_live_region_still_makes_the_tile_under_it_stale() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let half = vs * 0.5;
    let mut snapshot = big_world();
    let mut cleared = snapshot.clone();
    for ix in 8..11 {
        for iy in 8..11 {
            for iz in 0..5 {
                snapshot.push((
                    ix as f32 * vs + half,
                    iy as f32 * vs + half,
                    iz as f32 * vs + half,
                ));
            }
        }
    }
    // A ceiling everywhere, above what any live region can reach.
    for ix in 0..80 {
        for iy in 0..80 {
            let p = (ix as f32 * vs + half, iy as f32 * vs + half, 3.0 + half);
            snapshot.push(p);
            cleared.push(p);
        }
    }

    let mut p = Planner::new(cfg.worker_threads);
    queue_load(&mut p, &snapshot, (1.0, 1.0), &cfg);

    // The box is gone by the time the live region lands.
    let live = RegionBounds::capped(1.0, 1.0, 3.0, -0.1, 5.0, 0.3, cfg.max_overhead_m);
    assert!(live.z_max < 3.0);
    p.update_region(&slice(&cleared, &live, vs), &live, &cfg);
    finish_load(&mut p, &cfg);

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&cleared, &cfg);
    assert_eq!(
        voxel_set(&p),
        voxel_set(&clean),
        "the tile under the live region pasted the snapshot back"
    );
    assert_eq!(surface_set(&p), surface_set(&clean));
}

#[test]
fn goal_on_subclearance_spur_still_plans() {
    let mut cfg = test_config();
    cfg.surface_closing_radius = 0.0;
    cfg.wall_clearance_m = 0.3;
    cfg.wall_buffer_m = 0.0;
    cfg.wall_buffer_weight = 0.0;
    cfg.node_spacing_m = 0.5;

    let vs = 0.1_f32;
    let half = vs * 0.5;
    let mut pts = Vec::new();
    for ix in 0..10 {
        for iy in 0..10 {
            pts.push((ix as f32 * vs + half, iy as f32 * vs + half, half));
        }
    }
    // A 1-wide spur off the open area: every spur cell is wall-adjacent so
    // none clears the clearance and the penalized Voronoi cannot own them.
    for ix in 10..16 {
        pts.push((ix as f32 * vs + half, 5.0 * vs + half, half));
    }

    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&pts, &cfg);

    let start = (0.45, 0.45, 0.0);
    let goal = (15.0 * vs + half, 5.0 * vs + half, 0.0);
    let wp = p
        .plan(start, goal, &cfg)
        .expect("goal on a sub-clearance spur still reaches its component node");
    let last = *wp.last().expect("path has waypoints");
    assert!((last.0 - goal.0).abs() < 1e-3 && (last.1 - goal.1).abs() < 1e-3);
}
