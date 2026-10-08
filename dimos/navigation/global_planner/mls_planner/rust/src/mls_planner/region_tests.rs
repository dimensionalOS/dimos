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
use crate::region_viz::{cell_of, RegionViz};
use std::collections::{BTreeMap, BTreeSet};

type SeedRegions = Vec<(RegionBounds, Vec<(f32, f32, f32)>)>;

/// Regions as the ray tracer hands a seeded map on, nearest center first.
fn seed_regions(
    points: &[(f32, f32, f32)],
    region_m: f32,
    center: (f32, f32),
    vs: f32,
) -> SeedRegions {
    let mut cells: BTreeMap<(i32, i32), (f32, f32)> = BTreeMap::new();
    for &(x, y, z) in points {
        let cell = ((x / region_m).floor() as i32, (y / region_m).floor() as i32);
        let band = cells.entry(cell).or_insert((z, z));
        band.0 = band.0.min(z);
        band.1 = band.1.max(z);
    }
    let mut regions: SeedRegions = cells
        .into_iter()
        .map(|((cx, cy), (z_lo, z_hi))| {
            let bounds = RegionBounds {
                origin_x: (cx as f32 + 0.5) * region_m,
                origin_y: (cy as f32 + 0.5) * region_m,
                radius: region_m * std::f32::consts::FRAC_1_SQRT_2 + vs,
                z_min: z_lo - vs,
                z_max: z_hi + vs,
            };
            let cloud = slice(points, &bounds, vs);
            (bounds, cloud)
        })
        .collect();
    let dist = |b: &RegionBounds| (b.origin_x - center.0).powi(2) + (b.origin_y - center.1).powi(2);
    regions.sort_by(|a, b| dist(&a.0).total_cmp(&dist(&b.0)));
    regions
}

/// Feed a world to the planner region by region, as a seed load arrives.
fn load_by_regions(p: &mut Planner, points: &[(f32, f32, f32)], center: (f32, f32), cfg: &Config) {
    for (bounds, cloud) in seed_regions(points, 2.0, center, cfg.voxel_size) {
        p.update_region(&cloud, &bounds, cfg);
    }
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
        goal_z_tolerance: 0.5,
        blocked_timeout_s: 2.0,
        viz_publish_hz: 2.0,
        viz_region_m: 4.0,
        viz_sweep_regions: 0,
        worker_threads: 4,
    }
}

#[test]
fn viz_reach_covers_the_node_window_and_a_relocated_node_edge() {
    let cfg = test_config();
    // 0.3 m of wall band at 0.1 m cells is 3 cells, plus 2 slack.
    assert_eq!(cfg.node_window_cells(), 5);
    // Plus a 1 m node spacing, 10 cells.
    assert_eq!(cfg.viz_reach_cells(), 15);
}

#[test]
fn a_clearance_change_at_the_edge_of_the_reach_is_due_and_one_beyond_is_not() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let half = vs * 0.5;
    // An 8 m by 2 m floor, so cells well past the reach exist.
    let floor: Vec<(f32, f32, f32)> = (0..80)
        .flat_map(|ix| (0..20).map(move |iy| (ix as f32 * vs + half, iy as f32 * vs + half, half)))
        .collect();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&floor, &cfg);

    let pitch = 10;
    let mut viz = RegionViz::new(pitch, cfg.viz_reach_cells(), 0);
    viz.mark_all();
    viz.tick(p.surface_clearance_iter(), p.edge_segment_iter());

    // A junk column lands at x = 2 m, which rewrites a window around it.
    let bounds = RegionBounds {
        origin_x: 2.05,
        origin_y: 1.05,
        radius: 0.15,
        z_min: -1.0,
        z_max: 1.0,
    };
    let mut cloud = slice(&floor, &bounds, vs);
    cloud.extend((3..8).map(|iz| (2.05, 1.05, iz as f32 * vs + half)));
    let window = p
        .update_region(&cloud, &bounds, &cfg)
        .expect("the junk changes voxels");
    viz.mark_window(window);

    // Clearance changes at the far edge of the reach and one cell beyond it.
    let far = window.1 + cfg.viz_reach_cells();
    let beyond = far + pitch;
    let fed: Vec<(VoxelKey, f32)> = p
        .surface_clearance()
        .into_iter()
        .map(|(key, c)| {
            (
                key,
                if key.0 == far || key.0 == beyond {
                    0.123
                } else {
                    c
                },
            )
        })
        .collect();
    let due: Vec<_> = viz
        .tick(fed.into_iter(), p.edge_segment_iter())
        .into_iter()
        .map(|(cell, _)| cell)
        .collect();
    assert!(
        due.contains(&cell_of((far, 0, 0), pitch)),
        "far edge cell not due: {due:?}"
    );
    assert!(
        !due.contains(&cell_of((beyond, 0, 0), pitch)),
        "cell beyond reach due: {due:?}"
    );
}

#[test]
fn region_bounds_capped_clamps_ceiling_to_sensor_overhead() {
    let region = |z_max: f32| RegionBounds {
        origin_x: 0.0,
        origin_y: 0.0,
        radius: 1.0,
        z_min: -1.0,
        z_max,
    };
    // A ceiling above sensor_z + max_overhead is pulled down to the cap.
    let capped = region(5.0).capped_at(0.5, 2.0);
    assert_eq!(capped.z_max, 2.5, "ceiling capped to sensor_z + overhead");
    // A ceiling already below the cap is left untouched.
    let low = region(1.0).capped_at(0.5, 2.0);
    assert_eq!(low.z_max, 1.0, "cap never raises a lower ceiling");
    assert_eq!(low.z_min, -1.0);
    assert_eq!(low.radius, 1.0);
}

#[test]
fn step_cells_floors_to_a_hard_bound() {
    let mut cfg = test_config();
    cfg.voxel_size = 0.08;
    // 0.15 / 0.08 = 1.875 floors to 1: a 2-voxel (0.16m) step exceeds 0.15m.
    cfg.step_threshold_m = 0.15;
    assert_eq!(cfg.step_cells(), 1);
    // 0.20 / 0.08 = 2.5 floors to 2, so 2-voxel steps are allowed.
    cfg.step_threshold_m = 0.20;
    assert_eq!(cfg.step_cells(), 2);
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

/// A seeded map fed region by region must build exactly what a full
/// rebuild builds, and plan equivalently.
#[test]
fn seed_regions_match_full_rebuild() {
    let cfg = test_config();
    let all = big_world();

    let mut full = Planner::new(cfg.worker_threads);
    full.update_global_map(&all, &cfg);

    let mut seeded = Planner::new(cfg.worker_threads);
    load_by_regions(&mut seeded, &all, (0.5, 0.5), &cfg);

    assert_eq!(voxel_set(&seeded), voxel_set(&full), "voxel mismatch");
    assert_eq!(surface_set(&seeded), surface_set(&full), "surface mismatch");
    assert_eq!(
        cell_edges(&seeded),
        cell_edges(&full),
        "cell edges mismatch"
    );
    assert_plans_equivalent(&full, &seeded, &cfg);
}

/// Regions claim points by voxel center, so an off-center cloud seeds the
/// same voxels a full rebuild quantizes.
#[test]
fn seed_regions_match_full_rebuild_off_center() {
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

    let mut seeded = Planner::new(cfg.worker_threads);
    load_by_regions(&mut seeded, &all, (0.5, 0.5), &cfg);

    assert_eq!(voxel_set(&seeded), voxel_set(&full), "voxel mismatch");
    assert_eq!(surface_set(&seeded), surface_set(&full), "surface mismatch");
}

/// Reseeding the same map is a no-op: nodes placed before it survive
/// untouched.
#[test]
fn reseeding_leaves_graph_bit_identical() {
    let cfg = test_config();
    let all = big_world();
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before_cells = cell_edges(&p);
    let before_nodes = node_coords(&p);
    let before_edges = node_edge_pairs(&p);

    load_by_regions(&mut p, &all, (4.0, 4.0), &cfg);

    assert_eq!(cell_edges(&p), before_cells, "cells changed on reseed");
    assert_eq!(node_coords(&p), before_nodes, "nodes moved on reseed");
    assert_eq!(node_edge_pairs(&p), before_edges, "edges changed on reseed");
}

#[test]
fn seed_regions_keep_distant_nodes_sticky() {
    let cfg = test_config();
    let all = big_world();
    let vs = cfg.voxel_size;
    let mut p = Planner::new(cfg.worker_threads);
    p.update_global_map(&all, &cfg);
    let before = node_coords(&p);

    let mut changed = all.clone();
    changed.push((1.05, 1.05, 0.45));
    load_by_regions(&mut p, &changed, (1.0, 1.0), &cfg);

    let after = node_coords(&p);
    let far = |c: &VoxelKey| {
        let x = c.0 as f32 * vs;
        let y = c.1 as f32 * vs;
        ((x - 1.0).powi(2) + (y - 1.0).powi(2)).sqrt() > 3.5
    };
    for c in before.iter().filter(|c| far(c)) {
        assert!(after.contains(c), "distant node {c:?} moved on a seed");
    }
}

/// A seed region's z band is the premap's own, so geometry far above any
/// sensor overhead cap still lands.
#[test]
fn seed_regions_keep_high_geometry() {
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
    load_by_regions(&mut p, &all, (0.5, 0.5), &cfg);
    assert!(
        p.voxel_map.contains(&(15, 15, 30)),
        "high platform truncated"
    );
}

/// A live update over a seeded area and a seed region over a live area both
/// end at the map's current state.
#[test]
fn live_and_seed_regions_converge_in_either_order() {
    let cfg = test_config();
    let vs = cfg.voxel_size;
    let all = big_world();
    let live = RegionBounds {
        origin_x: 4.0,
        origin_y: 4.0,
        radius: 1.5,
        z_min: -0.1,
        z_max: 2.0,
    };

    let mut seed_then_live = Planner::new(cfg.worker_threads);
    load_by_regions(&mut seed_then_live, &all, (0.5, 0.5), &cfg);
    seed_then_live.update_region(&slice(&all, &live, vs), &live, &cfg);

    let mut live_then_seed = Planner::new(cfg.worker_threads);
    live_then_seed.update_region(&slice(&all, &live, vs), &live, &cfg);
    load_by_regions(&mut live_then_seed, &all, (0.5, 0.5), &cfg);

    let mut clean = Planner::new(cfg.worker_threads);
    clean.update_global_map(&all, &cfg);
    assert_eq!(voxel_set(&seed_then_live), voxel_set(&clean));
    assert_eq!(voxel_set(&live_then_seed), voxel_set(&clean));
    assert_eq!(surface_set(&seed_then_live), surface_set(&clean));
    assert_eq!(surface_set(&live_then_seed), surface_set(&clean));
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
