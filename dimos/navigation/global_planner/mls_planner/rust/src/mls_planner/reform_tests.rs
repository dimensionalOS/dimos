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

//! Toy worlds for the lazy reform: a robot drives a known floor with drifting
//! odometry, GscPGO-style keyframes and closures are fed to the planner, and the
//! layers, links and positions are checked. Each toy also dumps its state as JSON
//! (to `$MLS_TOY_DUMP_DIR`, default `target/toy_dumps`) for the render page.

use std::collections::BTreeSet;
use std::fmt::Write as _;

use super::region_tests::bare;
use super::*;
use crate::lazy::{EventRecord, KeyframeRecord, Pose, Rect, NO_ANCHOR};

/// A toy world: the true floor, and where the robot really stood and thought it stood.
struct Toy {
    planner: Planner,
    cfg: Config,
    floor: Box<dyn Fn(f32, f32) -> bool>,
    /// Per keyframe: node id, true pose, raw odometry pose.
    keyframes: Vec<(u32, Pose, Pose)>,
    scan_radius_m: f32,
    cylinders: Vec<(f32, f32, f32)>,
    found: Vec<(u32, (i32, i32))>,
}

fn toy_config() -> Config {
    let mut cfg = test_config();
    cfg.reform = ReformConfig {
        enabled: true,
        tile_m: 2.0,
        tile_pad: 0,
        box_grid_m: 4.0,
        box_pad_m: 0.5,
        version_history: 4,
        min_shift_m: 0.0,
        migrate_per_update: 0,
    };
    // a wall clearance, so a layer seam misread as a wall shows up as impassable links
    cfg.wall_clearance_m = 2.0 * cfg.voxel_size;
    cfg
}

fn pose(x: f64, y: f64, yaw: f64) -> Pose {
    Pose::from_yaw([x, y, 0.0], yaw)
}

impl Toy {
    fn new(floor: impl Fn(f32, f32) -> bool + 'static) -> Self {
        let cfg = toy_config();
        Self {
            planner: Planner::new(cfg.worker_threads),
            cfg,
            floor: Box::new(floor),
            keyframes: Vec::new(),
            scan_radius_m: 1.6,
            cylinders: Vec::new(),
            found: Vec::new(),
        }
    }

    /// The floor voxel centers within `radius` of a true point.
    fn floor_near(&self, x: f32, y: f32, radius: f32) -> Vec<(f32, f32, f32)> {
        let vs = self.cfg.voxel_size;
        let half = vs * 0.5;
        let n = (radius / vs).ceil() as i32;
        let (cx, cy) = ((x / vs).floor() as i32, (y / vs).floor() as i32);
        let mut out = Vec::new();
        for ix in cx - n..=cx + n {
            for iy in cy - n..=cy + n {
                let (px, py) = (ix as f32 * vs + half, iy as f32 * vs + half);
                if (px - x).hypot(py - y) <= radius && (self.floor)(px, py) {
                    out.push((px, py, half));
                }
            }
        }
        out
    }

    /// Make a keyframe where the robot truly is (`true_pose`) while odometry says `raw`, scanning the floor around `look_at` (true frame), and feed the scan as the tracer would: in the planner's current frame.
    fn scan_at(&mut self, true_pose: Pose, raw: Pose, look_at: (f32, f32)) {
        let node_id = self.keyframes.len() as u32 + 100;
        let truth = self.floor_near(look_at.0, look_at.1, self.scan_radius_m);
        let to_local = true_pose.inverse();
        let mut b = [
            f32::INFINITY,
            f32::INFINITY,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::NEG_INFINITY,
            f32::NEG_INFINITY,
        ];
        for &(x, y, z) in &truth {
            let l = to_local.apply([x as f64, y as f64, z as f64]);
            for a in 0..3 {
                b[a] = b[a].min(l[a] as f32);
                b[a + 3] = b[a + 3].max(l[a] as f32);
            }
        }
        self.planner.add_keyframes(
            &[KeyframeRecord {
                node_id,
                ts: node_id as f64,
                raw,
                scan_box: b,
            }],
            &self.cfg,
        );
        self.keyframes.push((node_id, true_pose, raw));
        // The tracer places the scan where GscPGO now thinks the robot is, in the current layer's frame.
        let lazy = self.planner.lazy.as_ref().unwrap();
        let slot = lazy.table.slot(node_id).unwrap();
        let placed = lazy.table.frame_pose(slot).compose(&to_local);
        let points: Vec<(f32, f32, f32)> = truth
            .iter()
            .map(|&(x, y, z)| {
                let p = placed.apply([x as f64, y as f64, z as f64]);
                (p[0] as f32, p[1] as f32, p[2] as f32)
            })
            .collect();
        let center = placed.apply([look_at.0 as f64, look_at.1 as f64, 0.0]);
        let bounds = RegionBounds {
            origin_x: center[0] as f32,
            origin_y: center[1] as f32,
            // the points cover exactly this disc, as the tracer's region surfaces cover its bounds
            radius: self.scan_radius_m,
            z_min: -0.5,
            z_max: 1.0,
        };
        self.found.extend(self.lookup_rect(&bounds));
        self.cylinders
            .push((bounds.origin_x, bounds.origin_y, bounds.radius));
        self.planner
            .update_region(&bare(&points), &bounds, &self.cfg);
    }

    /// The stale submeshes a region update with these bounds would migrate.
    fn lookup_rect(&self, bounds: &RegionBounds) -> Vec<(u32, (i32, i32))> {
        let lazy = self.planner.lazy.as_ref().unwrap();
        let m = bounds.radius + self.planner.migrate_margin_m(&self.cfg);
        let rect: Rect = [
            bounds.origin_x - m,
            bounds.origin_x + m,
            bounds.origin_y - m,
            bounds.origin_y + m,
        ];
        lazy.layers.lookup(&lazy.table, rect)
    }

    /// A closure moving every keyframe to its true pose; `closed` pairs the keyframes at these list indices.
    fn close_to_truth(&mut self, closed: &[(usize, usize)]) -> ClosureOutcome {
        let lazy = self.planner.lazy.as_ref().unwrap();
        let partner = |i: usize| {
            closed.iter().find_map(|&(a, b)| {
                if a == i {
                    Some(self.keyframes[b].0)
                } else if b == i {
                    Some(self.keyframes[a].0)
                } else {
                    None
                }
            })
        };
        let records: Vec<EventRecord> = self
            .keyframes
            .iter()
            .enumerate()
            .map(|(i, &(node_id, truth, _))| {
                let pre = lazy.table.now(lazy.table.slot(node_id).unwrap());
                EventRecord {
                    node_id,
                    pre,
                    delta: truth.compose(&pre.inverse()),
                    version: None,
                    closed_with: partner(i),
                }
            })
            .collect();
        self.planner.on_closure(&records, &self.cfg).unwrap()
    }

    fn cells(&self) -> &SurfaceCells {
        &self.planner.graph.cells
    }

    /// Where a cell is now: its key in the current layer, or carried from its stale layer by its anchor.
    fn point_now(&self, id: CellId) -> [f64; 3] {
        let cells = self.cells();
        let vs = self.cfg.voxel_size as f64;
        let (ix, iy, iz) = cells.coord(id);
        let c = [
            (ix as f64 + 0.5) * vs,
            (iy as f64 + 0.5) * vs,
            (iz as f64 + 0.5) * vs,
        ];
        let lazy = self.planner.lazy.as_ref().unwrap();
        let layer = cells.layer(id);
        if layer == lazy.layers.current {
            return c;
        }
        let el = lazy
            .layers
            .cells
            .get(&id)
            .copied()
            .expect("a live cell has an element");
        lazy.layers.stale[&layer]
            .carry(el.anchor, &lazy.table)
            .apply(c)
    }

    /// Connected groups of live cells over their links, any layer.
    fn components(&self) -> usize {
        self.component_sizes().len()
    }

    /// The sizes of the connected groups of live cells, largest first.
    fn component_sizes(&self) -> Vec<usize> {
        let cells = self.cells();
        let ids: Vec<CellId> = cells.ids().collect();
        let mut parent: Vec<u32> = (0..cells.slot_capacity() as u32).collect();
        fn find(parent: &mut [u32], mut i: u32) -> u32 {
            while parent[i as usize] != i {
                parent[i as usize] = parent[parent[i as usize] as usize];
                i = parent[i as usize];
            }
            i
        }
        for &id in &ids {
            for e in cells.neighbors(id) {
                let (a, b) = (find(&mut parent, id), find(&mut parent, e.dest));
                parent[a as usize] = b;
            }
        }
        let mut sizes: std::collections::BTreeMap<u32, usize> = std::collections::BTreeMap::new();
        for &id in &ids {
            *sizes.entry(find(&mut parent, id)).or_default() += 1;
        }
        let mut out: Vec<usize> = sizes.into_values().collect();
        out.sort_unstable_by(|a, b| b.cmp(a));
        out
    }

    /// Links between cells in different layers.
    fn cross_layer_links(&self) -> usize {
        let cells = self.cells();
        cells
            .ids()
            .map(|id| {
                cells
                    .neighbors(id)
                    .iter()
                    .filter(|e| cells.layer(e.dest) != cells.layer(id))
                    .count()
            })
            .sum()
    }

    /// Live cells whose current position shares a voxel with another's, among those whose position falls in `area`.
    fn duplicates_in(&self, area: Rect) -> usize {
        let vs = self.cfg.voxel_size;
        let mut seen: BTreeSet<VoxelKey> = BTreeSet::new();
        let mut dup = 0;
        for id in self.cells().ids() {
            let p = self.point_now(id);
            let (x, y) = (p[0] as f32, p[1] as f32);
            if x < area[0] || x > area[1] || y < area[2] || y > area[3] {
                continue;
            }
            if !seen.insert(voxelize((x, y, p[2] as f32), vs)) {
                dup += 1;
            }
        }
        dup
    }

    /// The toy's state as JSON for the render page.
    fn dump(&self, name: &str) {
        let dir = std::env::var("MLS_TOY_DUMP_DIR").unwrap_or_else(|_| {
            concat!(env!("CARGO_MANIFEST_DIR"), "/target/toy_dumps").to_string()
        });
        let cells = self.cells();
        let lazy = self.planner.lazy.as_ref().unwrap();
        let mut j = String::new();
        let _ = write!(
            j,
            "{{\"name\":\"{name}\",\"current_layer\":{},\"voxel_size\":{},",
            lazy.layers.current, self.cfg.voxel_size
        );
        j.push_str("\"cells\":[");
        let ids: Vec<CellId> = cells.ids().collect();
        for (n, &id) in ids.iter().enumerate() {
            let p = self.point_now(id);
            let k = cells.coord(id);
            let anchor = lazy.layers.cells.get(&id).map_or(NO_ANCHOR, |e| e.anchor);
            let _ = write!(
                j,
                "{}{{\"id\":{id},\"layer\":{},\"key\":[{},{},{}],\"now\":[{:.3},{:.3},{:.3}],\"anchor\":{}}}",
                if n > 0 { "," } else { "" },
                cells.layer(id),
                k.0,
                k.1,
                k.2,
                p[0],
                p[1],
                p[2],
                anchor as i64
            );
        }
        j.push_str("],\"edges\":[");
        let mut first = true;
        for &id in &ids {
            for e in cells.neighbors(id) {
                if e.dest > id || !cells.neighbors(e.dest).iter().any(|b| b.dest == id) {
                    let _ = write!(
                        j,
                        "{}[{id},{},{}]",
                        if first { "" } else { "," },
                        e.dest,
                        u8::from(cells.layer(e.dest) != cells.layer(id))
                    );
                    first = false;
                }
            }
        }
        j.push_str("],\"keyframes\":[");
        for (n, &(node_id, truth, raw)) in self.keyframes.iter().enumerate() {
            let slot = lazy.table.slot(node_id).unwrap();
            let now = lazy.table.now(slot);
            let rect = lazy.table.current_rect(slot);
            let yaw = |p: &Pose| 2.0 * p.rotation[2].atan2(p.rotation[3]);
            let _ = write!(
                j,
                "{}{{\"node_id\":{node_id},\"true\":[{:.3},{:.3},{:.3}],\"raw\":[{:.3},{:.3},{:.3}],\"now\":[{:.3},{:.3},{:.3}],\"rect\":{}}}",
                if n > 0 { "," } else { "" },
                truth.translation[0], truth.translation[1], yaw(&truth),
                raw.translation[0], raw.translation[1], yaw(&raw),
                now.translation[0], now.translation[1], yaw(&now),
                rect.map_or("null".to_string(), |r| format!("[{:.3},{:.3},{:.3},{:.3}]", r[0], r[1], r[2], r[3])),
            );
        }
        j.push_str("],\"cylinders\":[");
        for (n, c) in self.cylinders.iter().enumerate() {
            let _ = write!(
                j,
                "{}[{:.3},{:.3},{:.3}]",
                if n > 0 { "," } else { "" },
                c.0,
                c.1,
                c.2
            );
        }
        j.push_str("],\"found\":[");
        for (n, (layer, tile)) in self.found.iter().enumerate() {
            let _ = write!(
                j,
                "{}[{layer},{},{}]",
                if n > 0 { "," } else { "" },
                tile.0,
                tile.1
            );
        }
        let _ = write!(j, "],\"tile_m\":{}}}", self.cfg.reform.tile_m);
        if std::fs::create_dir_all(&dir).is_ok() {
            let _ = std::fs::write(format!("{dir}/{name}.json"), j);
        }
    }
}

#[test]
fn many_small_closures_keep_one_layer_and_a_real_closure_starts_another() {
    let mut toy = Toy::new(|x, y| (0.0..12.0).contains(&x) && (0.0..2.0).contains(&y));
    for i in 0..4 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    let ids: Vec<u32> = toy.keyframes.iter().map(|k| k.0).collect();
    let nudge = |toy: &Toy, dy: f64, closed: bool| -> Vec<EventRecord> {
        let table = &toy.planner.lazy.as_ref().unwrap().table;
        ids.iter()
            .enumerate()
            .map(|(i, &node_id)| EventRecord {
                node_id,
                pre: table.now(table.slot(node_id).unwrap()),
                delta: pose(0.0, dy, 0.0),
                version: None,
                closed_with: (closed && i == 0).then_some(ids[3]),
            })
            .collect()
    };
    // Twenty solves each moving every keyframe a millimeter: far under half a voxel in all.
    let mut version = 0;
    for _ in 0..20 {
        let records = nudge(&toy, 0.001, false);
        let outcome = toy.planner.on_closure(&records, &toy.cfg).unwrap();
        assert!(!outcome.new_layer);
        version = outcome.closure.version;
    }
    assert_eq!(version, 20);
    assert_eq!(toy.planner.layer_count(), 1);
    assert_eq!(toy.planner.lazy.as_ref().unwrap().layers.current, 0);
    // A closure that pairs two keyframes starts a layer however small its move.
    let records = nudge(&toy, 0.001, true);
    let outcome = toy.planner.on_closure(&records, &toy.cfg).unwrap();
    assert!(outcome.new_layer);
    let lazy = toy.planner.lazy.as_ref().unwrap();
    assert_eq!(lazy.layers.current, 21);
    assert_eq!(lazy.table.frame_version(), 21);
    toy.dump("toy0_tiny_closures");
}

#[test]
fn a_closure_merge_spread_over_updates_ends_like_one_done_at_once() {
    // Toy 1's corridor out and back, with the merge migrated a few dozen elements per region update, so submeshes
    // move a strip at a time.
    let mut toy = Toy::new(|x, y| (0.0..12.0).contains(&x) && (0.0..2.0).contains(&y));
    toy.cfg.reform.migrate_per_update = 50;
    for i in 0..6 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    for i in (0..6).rev() {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(
            pose(x, 1.0, std::f64::consts::PI),
            pose(x, 4.0, std::f64::consts::PI),
            (x as f32, 1.0),
        );
    }
    assert!(toy.close_to_truth(&[(0, 11)]).new_layer);
    let queued = |toy: &Toy| {
        let lazy = toy.planner.lazy.as_ref().unwrap();
        lazy.pending.len() + lazy.urgent.len()
    };
    assert!(
        queued(&toy) > 0,
        "the closure left part of the merge for later updates"
    );
    // Scans far from the merge drain the queue; odometry still carries the 3 m jump.
    let (truth, raw) = (
        pose(11.0, 1.0, std::f64::consts::PI),
        pose(11.0, 4.0, std::f64::consts::PI),
    );
    let mut scans = 0;
    while queued(&toy) > 0 {
        toy.scan_at(truth, raw, (11.0, 1.0));
        scans += 1;
        assert!(scans < 500, "the queue drains");
        let migrated: u64 = toy
            .planner
            .last_stage_counts()
            .iter()
            .filter(|c| c.1 == "rekeyed" || c.1 == "migrated_voxels")
            .map(|c| c.2)
            .sum();
        // whole columns move, so the budget is passed by at most one column's few elements
        assert!(
            migrated <= 60,
            "one update migrates about the budget, not {migrated}"
        );
    }
    assert!(scans > 2, "the merge took several updates");
    assert_eq!(toy.components(), 1, "the merge joins them");
    assert_eq!(
        toy.duplicates_in([0.0, 3.9, 0.0, 2.0]),
        0,
        "one surface where they merged"
    );
}

#[test]
fn a_closure_merges_two_surfaces_and_keeps_links_to_the_rest() {
    // A 12 m corridor driven out and back; odometry jumps 3 m sideways at the turn, so the way back is mapped as a second corridor.
    let mut toy = Toy::new(|x, y| (0.0..12.0).contains(&x) && (0.0..2.0).contains(&y));
    for i in 0..6 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    for i in (0..6).rev() {
        let x = 1.0 + 2.0 * i as f64;
        let truth = pose(x, 1.0, std::f64::consts::PI);
        let raw = pose(x, 4.0, std::f64::consts::PI);
        toy.scan_at(truth, raw, (x as f32, 1.0));
    }
    assert_eq!(toy.components(), 2, "two corridors before the closure");
    let outcome = toy.close_to_truth(&[(0, 11)]);
    assert!(outcome.new_layer);
    toy.dump("toy1_merge");
    assert_eq!(blocked_seam_links(&toy), 0, "a layer seam reads as a wall");
    toy.planner.graph.node_edges.assert_indexed(toy.cells());
    assert_eq!(toy.components(), 1, "the merge joins them");
    assert!(
        toy.cross_layer_links() > 0,
        "links to the unmigrated rest survive"
    );
    assert_eq!(
        toy.duplicates_in([0.0, 3.9, 0.0, 2.0]),
        0,
        "one surface where they merged"
    );
    let current = toy.planner.lazy.as_ref().unwrap().layers.current;
    let merged = toy
        .cells()
        .ids()
        .filter(|&id| toy.cells().layer(id) == current)
        .count();
    assert!(merged > 0);
}

/// Odometry that drifts a little more each keyframe: raw = drift(k) * truth.
fn drift(k: usize) -> Pose {
    Pose::from_yaw([0.10 * k as f64, 0.06 * k as f64, 0.0], 0.003 * k as f64)
}

/// A 20 x 12 m loop of 2 m corridors with a corridor across its middle at x 9..11.
fn loop_floor(x: f32, y: f32) -> bool {
    let inside = (0.0..20.0).contains(&x) && (0.0..12.0).contains(&y);
    let band = x < 2.0 || x >= 18.0 || y < 2.0 || y >= 10.0;
    let cross = (9.0..11.0).contains(&x);
    inside && (band || cross)
}

/// The loop's centerline, counterclockwise from (1, 1): keyframe poses every 2 m.
fn loop_path() -> Vec<Pose> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let mut out = Vec::new();
    for i in 0..10 {
        out.push(pose(1.0 + 2.0 * i as f64, 1.0, 0.0));
    }
    for i in 1..6 {
        out.push(pose(19.0, 1.0 + 2.0 * i as f64, FRAC_PI_2));
    }
    for i in 1..10 {
        out.push(pose(19.0 - 2.0 * i as f64, 11.0, PI));
    }
    for i in 1..5 {
        out.push(pose(1.0, 11.0 - 2.0 * i as f64, -FRAC_PI_2));
    }
    out
}

#[test]
fn a_second_pass_crossing_the_old_run_reforms_what_it_meets() {
    second_pass_crossing_the_old_run(0);
}

#[test]
fn a_second_pass_on_a_migration_budget_reforms_what_it_meets_a_little_per_update() {
    second_pass_crossing_the_old_run(60);
}

/// Toy 2: the loop closed, then a second pass across its middle to the old run's far side at A, migrating at most
/// `budget` stale elements per update (0 = all); the scans at A repeat until nothing stale is queued.
fn second_pass_crossing_the_old_run(budget: u32) {
    let mut toy = Toy::new(loop_floor);
    toy.cfg.reform.migrate_per_update = budget;
    let path = loop_path();
    for (k, &truth) in path.iter().enumerate() {
        let t = truth.translation;
        toy.scan_at(truth, drift(k).compose(&truth), (t[0] as f32, t[1] as f32));
    }
    let last = path.len() - 1;
    let outcome = toy.close_to_truth(&[(0, last)]);
    assert!(outcome.new_layer);
    // A little past the closure the robot turns 90 degrees and crosses the loop's middle, toward the old run's far side at A.
    let along: Vec<Pose> = (1..5)
        .map(|i| pose(1.0 + 2.0 * i as f64, 1.0, 0.0))
        .collect();
    let mut across: Vec<Pose> = (1..6)
        .map(|i| pose(10.0, 1.0 + 2.0 * i as f64, std::f64::consts::FRAC_PI_2))
        .collect();
    let a = *across.last().unwrap();
    let (ax, ay) = (a.translation[0], a.translation[1]);
    // Until the second pass gets near, the old run at A is stale, displaced in its layer by the drift.
    let current = toy.planner.lazy.as_ref().unwrap().layers.current;
    let near_a: Vec<(CellId, [f64; 3])> = toy
        .cells()
        .ids()
        .filter(|&id| toy.cells().layer(id) < current)
        .map(|id| (id, toy.point_now(id)))
        .filter(|(_, p)| (p[0] - ax).hypot(p[1] - ay) < 1.0)
        .collect();
    assert!(
        !near_a.is_empty(),
        "the old run at A is stale before the second pass"
    );
    toy.dump("toy2_second_pass_before");
    across.pop();
    let queued = |toy: &Toy| {
        let lazy = toy.planner.lazy.as_ref().unwrap();
        lazy.pending.len() + lazy.urgent.len()
    };
    let mut at_a = 0;
    for truth in along.into_iter().chain(across).chain([a]) {
        let k = toy.keyframes.len();
        let t = truth.translation;
        toy.scan_at(truth, drift(k).compose(&truth), (t[0] as f32, t[1] as f32));
        if budget > 0 {
            let migrated: u64 = toy
                .planner
                .last_stage_counts()
                .iter()
                .filter(|c| c.1 == "rekeyed" || c.1 == "migrated_voxels")
                .map(|c| c.2)
                .sum();
            // whole columns move, so the budget is passed by at most one column's few elements
            assert!(
                migrated <= budget as u64 + 10,
                "an update migrated {migrated}"
            );
        }
    }
    while queued(&toy) > 0 {
        let k = toy.keyframes.len();
        toy.scan_at(a, drift(k).compose(&a), (ax as f32, ay as f32));
        at_a += 1;
        assert!(at_a < 400, "the queue drains");
    }
    toy.dump("toy2_second_pass");
    toy.planner.graph.node_edges.assert_indexed(toy.cells());
    // Each stale cell that was near A now has a current-layer cell where its anchor put it (ids recycle, so match by place).
    let vs = toy.cfg.voxel_size as f64;
    let reformed: Vec<[f64; 3]> = toy
        .cells()
        .ids()
        .filter(|&id| toy.cells().layer(id) == current)
        .map(|id| toy.point_now(id))
        .collect();
    for (id, then) in &near_a {
        assert!(
            reformed
                .iter()
                .any(|p| (p[0] - then[0]).hypot(p[1] - then[1]) < 1.5 * vs),
            "stale cell {id} at {then:?} is reformed in place"
        );
    }
    assert!(
        toy.cross_layer_links() > 0,
        "stretched links to the unvisited rest"
    );
    assert_eq!(
        toy.duplicates_in([
            ax as f32 - 1.0,
            ax as f32 + 1.0,
            ay as f32 - 1.0,
            ay as f32 + 0.9
        ]),
        0
    );
    let home = toy
        .planner
        .plan((ax as f32, ay as f32, 0.1), (1.0, 1.0, 0.1), &toy.cfg);
    assert!(home.is_some(), "a path from A back to the start");
}

#[test]
fn a_box_far_from_its_keyframe_finds_what_that_keyframe_saw() {
    let floor = |x: f32, y: f32| {
        ((0.0..8.0).contains(&x) && (0.0..2.0).contains(&y))
            || ((0.0..4.0).contains(&x) && (15.0..18.0).contains(&y))
    };
    let mut toy = Toy::new(floor);
    for i in 0..4 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    // One keyframe on the corridor whose lidar only reached a patch 15 m away.
    toy.scan_radius_m = 1.4;
    toy.scan_at(pose(3.0, 1.0, 0.0), pose(3.0, 1.0, 0.0), (2.0, 16.5));
    let telescope = toy.keyframes.len() - 1;
    // A closure moving everything 0.5 m along x.
    let shifted: Vec<(u32, Pose, Pose)> = toy
        .keyframes
        .iter()
        .map(|&(id, truth, raw)| {
            (
                id,
                Pose::from_yaw([0.5, 0.0, 0.0], 0.0).compose(&truth),
                raw,
            )
        })
        .collect();
    toy.keyframes = shifted;
    assert!(toy.close_to_truth(&[(0, 3)]).new_layer);
    let lazy = toy.planner.lazy.as_ref().unwrap();
    let patch: Rect = [0.5, 4.5, 15.0, 18.0];
    let slot = lazy.table.slot(toy.keyframes[telescope].0).unwrap();
    assert_eq!(
        lazy.table.candidates(patch),
        vec![slot],
        "only the telescope's box reaches the patch"
    );
    let found = lazy.layers.lookup(&lazy.table, patch);
    assert!(
        !found.is_empty(),
        "its tiles hold the patch's stale submeshes"
    );
    let current = lazy.layers.current;
    // A cylinder at the patch, in the current frame.
    let points: Vec<(f32, f32, f32)> = toy
        .floor_near(2.0, 16.5, 1.4)
        .into_iter()
        .map(|(x, y, z)| (x + 0.5, y, z))
        .collect();
    let bounds = RegionBounds {
        origin_x: 2.5,
        origin_y: 16.5,
        radius: 1.5,
        z_min: -0.5,
        z_max: 1.0,
    };
    toy.found.extend(toy.lookup_rect(&bounds));
    toy.cylinders
        .push((bounds.origin_x, bounds.origin_y, bounds.radius));
    toy.planner.update_region(&bare(&points), &bounds, &toy.cfg);
    toy.dump("toy3_telescope");
    let patch_cells: Vec<CellId> = toy
        .cells()
        .ids()
        .filter(|&id| {
            let p = toy.point_now(id);
            p[1] > 14.5
        })
        .collect();
    assert!(!patch_cells.is_empty());
    assert!(
        patch_cells
            .iter()
            .all(|&id| toy.cells().layer(id) == current),
        "the patch is reformed"
    );
    assert_eq!(toy.duplicates_in([0.0, 5.0, 14.5, 18.5]), 0);
}

#[test]
fn a_cloud_from_an_older_layer_is_carried_and_one_from_a_future_closure_is_held() {
    let mut toy = Toy::new(|x, y| (0.0..12.0).contains(&x) && (0.0..2.0).contains(&y));
    for i in 0..4 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.5, 0.0), (x as f32, 1.0));
    }
    assert!(toy.close_to_truth(&[(0, 3)]).new_layer);
    let version = toy.planner.reform_version();
    // The tracer still in the old frame: its points sit where the old layer had them, 0.5 m up in y.
    let stale: Vec<(f32, f32, f32)> = toy
        .floor_near(9.0, 1.0, 1.0)
        .into_iter()
        .map(|(x, y, z)| (x, y + 0.5, z))
        .collect();
    let bounds = RegionBounds {
        origin_x: 9.0,
        origin_y: 1.5,
        radius: 1.1,
        z_min: -0.5,
        z_max: 1.0,
    };
    let ahead = toy
        .planner
        .update_region_at_version(&bare(&stale), &bounds, version + 1, &toy.cfg);
    assert_eq!(ahead, Err(VersionGap::Ahead));
    let applied = toy
        .planner
        .update_region_at_version(&bare(&stale), &bounds, 0, &toy.cfg);
    assert!(applied.is_ok());
    // Carried into the current frame, the cloud lands on the true corridor, where the reformed cells are.
    let vs = toy.cfg.voxel_size;
    let current = toy.planner.lazy.as_ref().unwrap().layers.current;
    let k = voxelize((9.05, 1.05, 0.05), vs);
    let id = toy.cells().id(k).expect("a current cell at the true floor");
    assert_eq!(toy.cells().layer(id), current);
    assert!(
        toy.cells().id(voxelize((9.05, 2.45, 0.05), vs)).is_none(),
        "nothing at the old frame's place"
    );
    toy.dump("toy6_version_race");
}

/// Links across layers that came out impassable between cells clear of every wall: a layer seam misread as a wall.
fn blocked_seam_links(toy: &Toy) -> usize {
    let cells = toy.cells();
    let vs = toy.cfg.voxel_size;
    let reach = toy.cfg.wall_clearance_m + vs;
    let steps = (reach / vs).ceil() as i32;
    let open = |id: CellId| {
        let p = toy.point_now(id);
        (-steps..=steps).all(|i| {
            (-steps..=steps)
                .all(|k| (toy.floor)(p[0] as f32 + i as f32 * vs, p[1] as f32 + k as f32 * vs))
        })
    };
    cells
        .ids()
        .flat_map(|id| cells.neighbors(id).iter().map(move |e| (id, *e)))
        .filter(|(id, e)| {
            cells.layer(*id) != cells.layer(e.dest)
                && !e.cost.is_finite()
                && open(*id)
                && open(e.dest)
        })
        .count()
}

/// Live links whose ends sit more than `factor` voxels apart where they are now.
fn stretched_links(toy: &Toy, factor: f64) -> usize {
    let vs = toy.cfg.voxel_size as f64;
    let cells = toy.cells();
    cells
        .ids()
        .map(|id| {
            let a = toy.point_now(id);
            cells
                .neighbors(id)
                .iter()
                .filter(|e| {
                    let b = toy.point_now(e.dest);
                    (a[0] - b[0]).hypot(a[1] - b[1]) > factor * vs
                })
                .count()
        })
        .sum()
}

#[test]
fn a_bending_closure_through_a_mixed_anchor_surface_stretches_links_without_splitting_it() {
    // A 16 m corridor scanned every meter, so each stretch of floor is anchored to whichever keyframe first saw it.
    let mut toy = Toy::new(|x, y| (0.0..16.0).contains(&x) && (0.0..2.0).contains(&y));
    for k in 0..16 {
        let x = 0.5 + k as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    let anchors: BTreeSet<u32> = {
        let lazy = toy.planner.lazy.as_ref().unwrap();
        toy.cells()
            .ids()
            .filter_map(|id| lazy.layers.cells.get(&id).map(|e| e.anchor))
            .collect()
    };
    assert!(anchors.len() > 4, "one surface, many anchors");
    assert_eq!(toy.components(), 1, "{:?}", toy.component_sizes());
    // A closure that bends the corridor at its middle: the far half's keyframes slide 35 cm sideways, the near half's stay.
    for kf in toy.keyframes.iter_mut().skip(8) {
        kf.1 = pose(0.0, 0.35, 0.0).compose(&kf.1);
    }
    assert!(toy.close_to_truth(&[]).new_layer);
    // A scan at the bend reforms around it only: neighbors with different anchors land apart.
    toy.scan_at(pose(8.0, 1.0, 0.0), pose(8.0, 1.0, 0.0), (8.0, 1.0));
    toy.dump("toy4_mixed_anchors");
    toy.planner.graph.node_edges.assert_indexed(toy.cells());
    assert_eq!(toy.components(), 1, "stretched links keep one surface");
    assert!(
        toy.cross_layer_links() > 0,
        "links between the reformed bend and the stale ends"
    );
    assert!(
        stretched_links(&toy, 1.5) > 0,
        "links across the bend stretch past neighbors"
    );
    // The robot back at the start sees it fresh; the goal is in the stale far end.
    toy.scan_at(pose(1.0, 1.0, 0.0), pose(1.0, 1.0, 0.0), (1.0, 1.0));
    let plan = toy
        .planner
        .plan_or_truncate((1.0, 1.0, 0.1), (15.0, 1.35, 0.1), &toy.cfg);
    assert!(
        matches!(plan, Plan::Full(_)),
        "a path end to end across the stretched links, got {plan:?}"
    );
}

#[test]
fn a_closure_that_moves_route_cells_needs_a_replan_and_one_that_moves_none_does_not() {
    let mut toy = Toy::new(|x, y| (0.0..16.0).contains(&x) && (0.0..2.0).contains(&y));
    for k in 0..16 {
        let x = 0.5 + k as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    let (start, goal) = ((1.0, 1.0, 0.1), (15.0, 1.0, 0.1));
    let ids: Vec<u32> = toy.keyframes.iter().map(|k| k.0).collect();
    let nudge = |toy: &Toy| -> Vec<EventRecord> {
        let table = &toy.planner.lazy.as_ref().unwrap().table;
        ids.iter()
            .map(|&node_id| EventRecord {
                node_id,
                pre: table.now(table.slot(node_id).unwrap()),
                delta: pose(0.0, 0.001, 0.0),
                version: None,
                closed_with: None,
            })
            .collect()
    };
    assert!(matches!(
        toy.planner.plan_or_truncate(start, goal, &toy.cfg),
        Plan::Full(_)
    ));
    // a millimeter solve keeps the layer, and every route cell is in it: nothing on the route moved
    let records = nudge(&toy);
    assert!(
        !toy.planner
            .on_closure(&records, &toy.cfg)
            .unwrap()
            .new_layer
    );
    assert!(!toy.planner.replan_needed(start, goal, &toy.cfg));
    // a closure that bends the corridor starts a layer, carrying every route cell
    for kf in toy.keyframes.iter_mut().skip(8) {
        kf.1 = pose(0.0, 0.35, 0.0).compose(&kf.1);
    }
    assert!(toy.close_to_truth(&[]).new_layer);
    assert!(toy.planner.replan_needed(start, goal, &toy.cfg));
    // the robot sees the start fresh; a route from it over cells still stale, then a millimeter solve: those cells
    // are carried, so it is planned again
    toy.scan_at(pose(1.0, 1.0, 0.0), pose(1.0, 1.0, 0.0), (1.0, 1.0));
    let goal = (15.0, 1.35, 0.1);
    assert!(matches!(
        toy.planner.plan_or_truncate(start, goal, &toy.cfg),
        Plan::Full(_)
    ));
    let current = toy.cells().current_layer();
    let route = &toy.planner.route.as_ref().unwrap().cache;
    assert!(
        route.stale_cells(current).next().is_some(),
        "the route crosses stale cells"
    );
    assert!(!toy.planner.replan_needed(start, goal, &toy.cfg));
    let records = nudge(&toy);
    assert!(
        !toy.planner
            .on_closure(&records, &toy.cfg)
            .unwrap()
            .new_layer
    );
    assert_eq!(
        toy.planner.replan_reason(start, goal, &toy.cfg),
        Some(ReplanReason::RouteChanged)
    );
    assert!(matches!(
        toy.planner.plan_or_truncate(start, goal, &toy.cfg),
        Plan::Full(_)
    ));
    assert!(!toy.planner.replan_needed(start, goal, &toy.cfg));
}

#[test]
fn a_lookup_after_two_closures_finds_what_is_stale_since_each() {
    // Out along a 20 m corridor and back with odometry 3 m off; a first closure at the far end fixes that, then odometry slips 1 m more on the way back and a second closure halfway fixes it.
    let mut toy = Toy::new(|x, y| (0.0..20.0).contains(&x) && (0.0..2.0).contains(&y));
    use std::f64::consts::PI;
    for i in 0..10 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.0, 0.0), (x as f32, 1.0));
    }
    for x in [19.0, 17.0] {
        toy.scan_at(pose(x, 1.0, PI), pose(x, 4.0, PI), (x as f32, 1.0));
    }
    assert!(toy.close_to_truth(&[(9, 10)]).new_layer);
    let first = toy.planner.reform_version();
    for x in [15.0, 13.0, 11.0, 9.0] {
        toy.scan_at(pose(x, 1.0, PI), pose(x, 5.0, PI), (x as f32, 1.0));
    }
    assert!(toy.close_to_truth(&[(4, 15)]).new_layer);
    assert!(toy.planner.reform_version() > first);
    // In the second closure's frame, a lookup over the corridor finds the start, stale since version 0, and the way back at x 13..15, stale since the first closure.
    let corridor = RegionBounds {
        origin_x: 10.0,
        origin_y: 1.0,
        radius: 10.0,
        z_min: -0.5,
        z_max: 1.0,
    };
    let layers: BTreeSet<u32> = toy
        .lookup_rect(&corridor)
        .iter()
        .map(|&(layer, _)| layer)
        .collect();
    assert!(layers.contains(&0), "submeshes stale since version 0");
    assert!(
        layers.contains(&first),
        "submeshes stale since the first closure"
    );
    toy.found.clear();
    for x in [1.0, 3.0, 5.0, 13.0, 15.0] {
        toy.scan_at(pose(x, 1.0, PI), pose(x, 5.0, PI), (x as f32, 1.0));
    }
    toy.dump("toy5_two_closures");
    toy.planner.graph.node_edges.assert_indexed(toy.cells());
    assert_eq!(toy.components(), 1, "{:?}", toy.component_sizes());
    assert_eq!(toy.duplicates_in([0.0, 20.0, 0.0, 2.0]), 0);
}

#[test]
fn a_path_through_stale_cells_is_reported_where_they_are_now() {
    let mut toy = Toy::new(loop_floor);
    let path = loop_path();
    for (k, &truth) in path.iter().enumerate() {
        let t = truth.translation;
        toy.scan_at(truth, drift(k).compose(&truth), (t[0] as f32, t[1] as f32));
    }
    let last = path.len() - 1;
    assert!(toy.close_to_truth(&[(0, last)]).new_layer);
    // From the closed start to the loop's far corner, through cells still stale and drifted in their layer.
    let goal = (19.0, 11.0, 0.1);
    let plan = toy
        .planner
        .plan_or_truncate((1.0, 1.0, 0.1), goal, &toy.cfg);
    let Plan::Full(waypoints) = plan else {
        panic!("a full path to the far corner, got {plan:?}");
    };
    let slack = 2.0 * toy.cfg.voxel_size;
    for &(x, y, _) in &waypoints {
        let on_floor = [
            (-slack, -slack),
            (slack, -slack),
            (-slack, slack),
            (slack, slack),
            (0.0, 0.0),
        ]
        .iter()
        .any(|&(dx, dy)| loop_floor(x + dx, y + dy));
        assert!(on_floor, "waypoint ({x}, {y}) is on the true floor");
    }
}

#[test]
fn a_global_rebuild_never_drops_stale_layers() {
    let mut toy = Toy::new(|x, y| (0.0..20.0).contains(&x) && (0.0..2.0).contains(&y));
    for i in 0..10 {
        let x = 1.0 + 2.0 * i as f64;
        toy.scan_at(pose(x, 1.0, 0.0), pose(x, 1.5, 0.0), (x as f32, 1.0));
    }
    assert!(toy.close_to_truth(&[(0, 1)]).new_layer);
    let (layers, cells) = (toy.planner.layer_count(), toy.cells().ids().count());
    assert!(layers > 1);
    toy.planner
        .update_global_map(&bare(&[(1.0, 1.0, 0.05)]), &toy.cfg);
    assert_eq!(toy.planner.layer_count(), layers);
    assert_eq!(toy.cells().ids().count(), cells);
}

/// Per-closure bookkeeping on a big map: run with --ignored to print what a closure listing 20000 keyframes costs.
#[test]
#[ignore]
fn closure_bookkeeping_cost_on_a_big_map() {
    let keyframes = 20_000u32;
    let mut table = crate::lazy::KeyframeTable::new(toy_config().reform.table_params());
    for id in 0..keyframes {
        table.add(&KeyframeRecord {
            node_id: id,
            ts: id as f64,
            raw: pose(id as f64 * 0.5, 0.0, 0.0),
            scan_box: [-5.0, -5.0, -1.0, 5.0, 5.0, 2.0],
        });
    }
    let records: Vec<EventRecord> = (0..keyframes)
        .map(|id| EventRecord {
            node_id: id,
            pre: table.now(id),
            delta: pose(0.0, 0.001, 0.0),
            version: None,
            closed_with: None,
        })
        .collect();
    let started = std::time::Instant::now();
    let rounds = 20;
    for _ in 0..rounds {
        table.apply_event(&records);
        std::hint::black_box(table.shift_since_frame());
    }
    let per = started.elapsed().as_secs_f64() * 1e3 / rounds as f64;
    println!(
        "closure bookkeeping, {keyframes} keyframes: {per:.2} ms per event (apply + shift check)"
    );
}

/// Toy: after the loop closes, one scan over the old run's top corridor on a migration budget of one column leaves most of the old
/// run stale beside the new patch; every stale cell standing now in a column beside a current cell, within a step of
/// it, is linked to it, though the drift turned the old run's frame (so the stitch reads it through each anchor's carry).
#[test]
fn a_patch_over_a_turned_stale_run_links_every_stale_cell_beside_it_now() {
    let mut toy = Toy::new(loop_floor);
    toy.cfg.reform.migrate_per_update = 1;
    let path = loop_path();
    for (k, &truth) in path.iter().enumerate() {
        let t = truth.translation;
        toy.scan_at(truth, drift(k).compose(&truth), (t[0] as f32, t[1] as f32));
    }
    let last = path.len() - 1;
    assert!(toy.close_to_truth(&[(0, last)]).new_layer);
    // on the top corridor, which the run saw late, so far drifted
    let truth = pose(8.0, 11.0, std::f64::consts::PI);
    let k = toy.keyframes.len();
    toy.scan_at(truth, drift(k).compose(&truth), (8.0, 11.0));
    let cells = toy.cells();
    let current = toy.planner.lazy.as_ref().unwrap().layers.current;
    let vs = toy.cfg.voxel_size;
    let max_rise = toy.cfg.step_threshold_m + toy.cfg.plane_noise_m;
    let mut stale_at: std::collections::BTreeMap<(i32, i32), Vec<(CellId, f32)>> =
        Default::default();
    for id in cells.ids().filter(|&id| cells.layer(id) < current) {
        let p = toy
            .planner
            .stale_point(id)
            .expect("a stale cell has a place now");
        let k = voxelize(p, vs);
        stale_at.entry((k.0, k.1)).or_default().push((id, p.2));
    }
    let mut beside = 0;
    for a in cells.ids().filter(|&id| cells.layer(id) == current) {
        let (x, y, _) = cells.coord(a);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            for &(b, zb) in stale_at.get(&(x + dx, y + dy)).into_iter().flatten() {
                if (zb - cells.z(a)).abs() <= max_rise {
                    beside += 1;
                    assert!(
                        cells.neighbors(a).iter().any(|e| e.dest == b),
                        "current cell {a} at {:?} is not linked to stale cell {b} beside it",
                        cells.coord(a)
                    );
                }
            }
        }
    }
    assert!(beside > 0, "the patch meets the stale run");
}

/// One lookup over several areas finds what each area's lookup finds, together.
#[test]
fn a_lookup_over_several_areas_is_the_union_of_their_lookups() {
    let mut toy = Toy::new(loop_floor);
    let path = loop_path();
    for (k, &truth) in path.iter().enumerate() {
        let t = truth.translation;
        toy.scan_at(truth, drift(k).compose(&truth), (t[0] as f32, t[1] as f32));
    }
    let last = path.len() - 1;
    assert!(toy.close_to_truth(&[(0, last)]).new_layer);
    let lazy = toy.planner.lazy.as_ref().unwrap();
    let rects: Vec<Rect> = vec![
        [0.0, 3.0, 0.0, 3.0],
        [8.0, 12.0, 4.0, 6.0],
        [17.0, 20.0, 9.0, 12.0],
    ];
    let mut each: Vec<(u32, (i32, i32))> = rects
        .iter()
        .flat_map(|&r| lazy.layers.lookup(&lazy.table, r))
        .collect();
    each.sort_unstable();
    each.dedup();
    assert!(!each.is_empty());
    assert_eq!(lazy.layers.lookup_all(&lazy.table, &rects), each);
}
