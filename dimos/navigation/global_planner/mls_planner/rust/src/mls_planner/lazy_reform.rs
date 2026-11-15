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

//! The planner's side of the lazy reform: anchoring what it writes, freezing a
//! layer at each closure, and migrating stale submeshes into the current layer
//! when a closure or a region update reaches them.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;
use std::time::Instant;

use ahash::AHashMap;

use super::{Config, MapPoint, Planner, RegionBounds};
use crate::adjacency::{CellId, Edge, SurfaceCells};
use crate::columns::{ColumnEdit, ColumnMask, VoxelEdits};
use crate::lazy::{
    carry_plane, Closure, Element, EventRecord, FrozenLayer, KeyframeRecord, KeyframeTable, Layers,
    Pose, Rect, Submesh, Version,
};
use crate::nodes::NodeId;
use crate::surfaces::extract_surfaces_region;
use crate::voxel::{voxelize, VoxelKey};

/// The keyframe table and the version layers.
pub struct LazyState {
    pub(crate) table: KeyframeTable,
    pub(crate) layers: Layers,
    /// The planner's voxel edge, for placing stale cells.
    voxel_size: f32,
    /// Closing-box submeshes a closure has yet to migrate, reform.migrate_per_update elements per region update.
    pub(crate) pending: BTreeSet<(Version, (i32, i32))>,
    /// Submeshes a region update's lookup found around it, migrated before the pending ones.
    pub(crate) urgent: BTreeSet<(Version, (i32, i32))>,
    /// Per stale submesh, the frame it was built in and its cells (with their z now) by the column they stand in now:
    /// a stale cell only moves when a closure starts a frame, so a stitch reads the columns it needs, not the submesh.
    carried: AHashMap<(Version, (i32, i32)), (Version, CarriedCells)>,
}

/// Stale cells by the current-frame column they stand in, with their z there.
type CarriedCells = BTreeMap<(i32, i32), Vec<(CellId, f32)>>;

fn center(k: VoxelKey, voxel_size: f32) -> [f64; 3] {
    let v = voxel_size as f64;
    [
        (k.0 as f64 + 0.5) * v,
        (k.1 as f64 + 0.5) * v,
        (k.2 as f64 + 0.5) * v,
    ]
}

fn voxel_of(p: [f64; 3], voxel_size: f32) -> VoxelKey {
    voxelize((p[0] as f32, p[1] as f32, p[2] as f32), voxel_size)
}

impl LazyState {
    /// The element new data gets: anchored to the newest keyframe.
    fn element_at(&self, k: VoxelKey, voxel_size: f32) -> Element {
        self.layers
            .element(&self.table, self.table.latest(), center(k, voxel_size))
    }

    /// File a region update's voxel edits in the current layer.
    pub(crate) fn file_voxels(&mut self, edit: &ColumnEdit, voxel_size: f32) {
        let (ix, iy) = edit.col;
        for &iz in &edit.removed {
            self.layers.remove_voxel((ix, iy, iz));
        }
        for &iz in &edit.added {
            let k = (ix, iy, iz);
            let el = self.element_at(k, voxel_size);
            self.layers.add_voxel(k, el);
        }
    }

    /// File a patch's removed and added current-layer cells.
    pub(crate) fn file_cells(
        &mut self,
        cells: &SurfaceCells,
        removed: &[CellId],
        added: &[CellId],
        voxel_size: f32,
    ) {
        let current = self.layers.current;
        for &id in removed {
            self.layers.remove_cell(id, current);
        }
        for &id in added {
            if self.layers.cells.contains_key(&id) {
                continue;
            }
            let el = self.element_at(cells.coord(id), voxel_size);
            self.layers.add_cell(id, current, el);
        }
    }
}

impl LazyState {
    /// The stale submesh's cells by the column they stand in now, built once per frame. Cells it has lost since stay
    /// listed, so a reader checks each is still live and in the layer.
    fn carried_index(
        &mut self,
        key: (Version, (i32, i32)),
        frame: Version,
        cells: &SurfaceCells,
        voxel_size: f32,
    ) -> &CarriedCells {
        let fresh = self.carried.get(&key).is_some_and(|(f, _)| *f == frame);
        if !fresh {
            let mut index = CarriedCells::new();
            let (layer, _) = key;
            if let (Some(sub), Some(frozen)) = (
                self.layers.submeshes.get(&key),
                self.layers.stale.get(&layer),
            ) {
                // One carry per anchor, not per cell.
                let mut carries: AHashMap<u32, Pose> = AHashMap::new();
                for &id in &sub.cells {
                    if !cells.is_live(id) || cells.layer(id) != layer {
                        continue;
                    }
                    let Some(el) = self.layers.cells.get(&id) else {
                        continue;
                    };
                    let carry = carries
                        .entry(el.anchor)
                        .or_insert_with(|| frozen.carry(el.anchor, &self.table));
                    let (x, y, z) = cells.point(id, voxel_size);
                    let p = carry.apply([x as f64, y as f64, z as f64]);
                    let p = (p[0] as f32, p[1] as f32, p[2] as f32);
                    let k = voxelize(p, voxel_size);
                    index.entry((k.0, k.1)).or_default().push((id, p.2));
                }
            }
            self.carried.insert(key, (frame, index));
        }
        &self.carried[&key].1
    }

    /// Where a stale cell is now, carried by its anchor; None for a current-layer cell.
    pub(crate) fn stale_point(&self, cells: &SurfaceCells, id: CellId) -> Option<(f32, f32, f32)> {
        let layer = cells.layer(id);
        if layer == self.layers.current {
            return None;
        }
        let el = self.layers.cells.get(&id)?;
        let carry = self.layers.stale.get(&layer)?.carry(el.anchor, &self.table);
        let (x, y, z) = cells.point(id, self.voxel_size);
        let p = carry.apply([x as f64, y as f64, z as f64]);
        Some((p[0] as f32, p[1] as f32, p[2] as f32))
    }
}

/// A cell's key where it is now: its own in the current layer, its carried point's for a stale one.
pub(crate) fn key_now(lazy: Option<&LazyState>, cells: &SurfaceCells, id: CellId) -> VoxelKey {
    match lazy.and_then(|l| l.stale_point(cells, id).map(|p| voxelize(p, l.voxel_size))) {
        Some(k) => k,
        None => cells.coord(id),
    }
}

/// What a migration did to the current layer, for the repair after it.
#[derive(Default)]
struct Migrated {
    edits: VoxelEdits,
    /// Columns that gained a voxel or a cell.
    columns: BTreeSet<(i32, i32)>,
    /// Keys of cells that moved in or took a merged cell's links.
    rekeyed: Vec<VoxelKey>,
    /// Nodes given up by merged cells.
    gone: Vec<NodeId>,
    /// Every merged cell and the cell it merged into, for stored corridors.
    merged: Vec<(CellId, CellId)>,
}

impl Planner {
    /// The closure version the planner's map is in.
    pub fn reform_version(&self) -> Version {
        self.lazy.as_ref().map_or(0, |l| l.table.version())
    }

    /// Take GscPGO's new keyframes; the first one starts the lazy reform.
    pub fn add_keyframes(&mut self, records: &[KeyframeRecord], config: &Config) {
        if !config.reform.enabled {
            return;
        }
        let lazy = self.lazy.get_or_insert_with(|| LazyState {
            table: KeyframeTable::new(config.reform.table_params()),
            layers: Layers::new(config.reform.layer_params()),
            voxel_size: config.voxel_size,
            pending: BTreeSet::new(),
            urgent: BTreeSet::new(),
            carried: AHashMap::new(),
        });
        for record in records {
            lazy.table.add(record);
        }
    }

    /// Apply a loop closure to the keyframe table. One that closes a loop, or moves some keyframe past `reform.min_shift_m` since the current layer's frame, also freezes the current layer, starts the closure's, and migrates the submeshes inside the closing keyframes' boxes.
    pub fn on_closure(
        &mut self,
        records: &[EventRecord],
        config: &Config,
    ) -> Option<ClosureOutcome> {
        self.clock.start("closure");
        let lazy = self.lazy.as_mut()?;
        let stage = Instant::now();
        let closure = lazy.table.apply_event(records)?;
        let shift = lazy.table.shift_since_frame();
        self.clock.add("apply_event", stage);
        if closure.closing.is_empty() && shift <= config.reform.layer_shift_m(config.voxel_size) {
            if let Some(route) = &mut self.route {
                route.note_closure(self.graph.cells.current_layer());
            }
            return Some(ClosureOutcome {
                closure,
                new_layer: false,
                carry_newest: Pose::IDENTITY,
            });
        }
        let stage = Instant::now();
        // The anchors' poses in the frame of the layer being frozen.
        let anchor_poses = lazy
            .layers
            .anchors
            .iter()
            .map(|&a| {
                let pose = if (a as usize) < lazy.table.len() {
                    lazy.table.frame_pose(a)
                } else {
                    Pose::IDENTITY
                };
                (a, pose)
            })
            .collect();
        let newest = lazy.table.latest();
        let newest_then = newest.map(|slot| lazy.table.frame_pose(slot));
        lazy.table.start_frame();
        let carry_newest = match (newest, newest_then) {
            (Some(slot), Some(then)) => lazy.table.frame_pose(slot).compose(&then.inverse()),
            _ => Pose::IDENTITY,
        };
        let frozen = FrozenLayer {
            by_col: std::mem::take(&mut self.by_col),
            planes: std::mem::take(&mut self.planes),
            voxels: std::mem::take(&mut lazy.layers.voxels),
            surface_lookup: std::mem::take(&mut self.graph.surface_lookup),
            anchor_poses,
        };
        let old = lazy.layers.current;
        lazy.layers.anchors.clear();
        lazy.layers.stale.insert(old, frozen);
        lazy.layers.current = closure.version;
        self.graph.cells.begin_layer(closure.version);
        // The cached route is in the old layer's keys, and every cell on it is stale and carried now.
        self.last_path = None;
        self.route = None;
        self.clock.add("freeze", stage);

        let stage = Instant::now();
        let rects: Vec<Rect> = closure
            .closing
            .iter()
            .filter_map(|&slot| lazy.table.current_rect(slot))
            .collect();
        let found: BTreeSet<(Version, (i32, i32))> = rects
            .iter()
            .flat_map(|&rect| lazy.layers.lookup(&lazy.table, rect))
            .collect();
        lazy.pending.extend(found.difference(&lazy.urgent));
        self.clock.add("lookup", stage);
        self.clock
            .count("pending", lazy.pending.len() + lazy.urgent.len());
        let pool = Arc::clone(&self.pool);
        pool.install(|| self.migrate_pending(config));
        Some(ClosureOutcome {
            closure,
            new_layer: true,
            carry_newest,
        })
    }

    /// How many keyframes the lazy reform knows.
    pub fn keyframe_count(&self) -> usize {
        self.lazy.as_ref().map_or(0, |l| l.table.len())
    }

    /// How many version layers hold elements, the current one included.
    pub fn layer_count(&self) -> usize {
        self.lazy.as_ref().map_or(1, |l| 1 + l.layers.stale.len())
    }

    /// Queue the stale submeshes a region update is about to touch, out to the margin its repair reads, ahead of a
    /// closure's, then migrate the next reform.migrate_per_update elements of the queues.
    pub(super) fn reform_near(&mut self, bounds: &RegionBounds, config: &Config) {
        let margin = bounds.radius + self.migrate_margin_m(config);
        let Some(lazy) = self.lazy.as_mut() else {
            return;
        };
        let stage = Instant::now();
        let rect = [
            bounds.origin_x - margin,
            bounds.origin_x + margin,
            bounds.origin_y - margin,
            bounds.origin_y + margin,
        ];
        for key in lazy.layers.lookup(&lazy.table, rect) {
            lazy.pending.remove(&key);
            lazy.urgent.insert(key);
        }
        let outer = self.clock.enter("near");
        self.clock.add("lookup", stage);
        self.clock.enter(outer);
        self.migrate_pending(config);
    }

    /// The stale cell standing nearest `p` now, by where its anchor carries it, among the submeshes a lookup finds within
    /// the snap reach of `p` (and its height tolerance); None without one. For a goal no current-layer cell is near.
    pub(crate) fn nearest_stale_cell(&self, p: (f32, f32, f32), config: &Config) -> Option<CellId> {
        let lazy = self.lazy.as_ref()?;
        let reach = crate::planner::SNAP_SEARCH_RADIUS_M;
        let rect = [p.0 - reach, p.0 + reach, p.1 - reach, p.1 + reach];
        let cells = &self.graph.cells;
        let mut best: Option<(f32, CellId)> = None;
        for key in lazy.layers.lookup(&lazy.table, rect) {
            let Some(sub) = lazy.layers.submeshes.get(&key) else {
                continue;
            };
            for &id in &sub.cells {
                if !cells.is_live(id) {
                    continue;
                }
                let Some(q) = lazy.stale_point(cells, id) else {
                    continue;
                };
                let d2 = (q.0 - p.0).powi(2) + (q.1 - p.1).powi(2);
                if d2 > reach * reach || (q.2 - p.2).abs() > config.robot_height {
                    continue;
                }
                if best.is_none_or(|(bd, bid)| d2 < bd || d2 == bd && id < bid) {
                    best = Some((d2, id));
                }
            }
        }
        best.map(|(_, id)| id)
    }

    /// Where a stale layer's cell is in the current frame, by its anchor's pose then and now; None for a current-layer cell.
    pub(crate) fn stale_point(&self, id: CellId) -> Option<(f32, f32, f32)> {
        self.lazy.as_ref()?.stale_point(&self.graph.cells, id)
    }

    /// Why a plan from `start` to `goal` fails, as JSON: whether the cells connect over passable links; if not, where
    /// the start's component ends (by layer pair and whether a link exists), else the first place along the cell path
    /// the node graph lacks the edge between the two Voronoi owners.
    pub fn connectivity_json(
        &self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> String {
        use std::collections::VecDeque;
        let g = &self.graph;
        let vs = config.voxel_size;
        let snap = |p| {
            crate::planner::snap_pose_to_cell(&g.surface_lookup, p, vs, config.robot_height)
                .and_then(|k| g.cells.id(k))
        };
        let goal_cell = crate::planner::snap_goal_current(g, goal, vs, config.robot_height)
            .or_else(|| snap(goal));
        let (Some(a), Some(b)) = (snap(start), goal_cell) else {
            return format!(
                "{{\"snapped\":false,\"start\":{},\"goal\":{}}}",
                snap(start).is_some(),
                snap(goal).is_some()
            );
        };
        let mut pred: AHashMap<CellId, CellId> = AHashMap::new();
        pred.insert(a, a);
        let mut queue = VecDeque::from([a]);
        while let Some(c) = queue.pop_front() {
            if c == b {
                break;
            }
            for e in g.cells.neighbors(c) {
                if e.cost.is_finite() && !pred.contains_key(&e.dest) {
                    pred.insert(e.dest, c);
                    queue.push_back(e.dest);
                }
            }
        }
        let reached = pred.len();
        let now = |id: CellId| {
            self.stale_point(id)
                .unwrap_or_else(|| g.cells.point(id, vs))
        };
        if !pred.contains_key(&b) {
            let key = |id: CellId| voxelize(now(id), vs);
            let mut by_key: AHashMap<VoxelKey, Vec<CellId>> = AHashMap::new();
            for id in g.cells.ids() {
                by_key.entry(key(id)).or_default().push(id);
            }
            let mut pairs: std::collections::BTreeMap<(u32, u32, bool), (usize, [f32; 3])> =
                Default::default();
            for &c in pred.keys() {
                let k = key(c);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    for dz in -2..=2 {
                        for &o in by_key
                            .get(&(k.0 + dx, k.1 + dy, k.2 + dz))
                            .into_iter()
                            .flatten()
                        {
                            if pred.contains_key(&o) {
                                continue;
                            }
                            let linked = g.cells.neighbors(c).iter().any(|e| e.dest == o);
                            let p = now(c);
                            let entry = pairs
                                .entry((g.cells.layer(c), g.cells.layer(o), linked))
                                .or_insert((0, [p.0, p.1, p.2]));
                            entry.0 += 1;
                        }
                    }
                }
            }
            let list: Vec<String> = pairs
                .iter()
                .map(|((li, lo, linked), (n, p))| {
                    format!("{{\"in\":{li},\"out\":{lo},\"linked\":{linked},\"count\":{n},\"sample\":[{:.2},{:.2},{:.2}]}}", p[0], p[1], p[2])
                })
                .collect();
            // where the goal's component comes closest to the start's, by where cells are now
            let mut from_goal: AHashMap<CellId, ()> = AHashMap::new();
            from_goal.insert(b, ());
            let mut queue = VecDeque::from([b]);
            while let Some(c) = queue.pop_front() {
                for e in g.cells.neighbors(c) {
                    if e.cost.is_finite() && !from_goal.contains_key(&e.dest) {
                        from_goal.insert(e.dest, ());
                        queue.push_back(e.dest);
                    }
                }
            }
            let mut goal_keys: AHashMap<(i32, i32), Vec<CellId>> = AHashMap::new();
            for &c in from_goal.keys() {
                let k = key(c);
                goal_keys.entry((k.0, k.1)).or_default().push(c);
            }
            let mut closest: Option<(f32, CellId, CellId)> = None;
            for &c in pred.keys() {
                let k = key(c);
                for dx in -3..=3 {
                    for dy in -3..=3 {
                        for &o in goal_keys.get(&(k.0 + dx, k.1 + dy)).into_iter().flatten() {
                            let (p, q) = (now(c), now(o));
                            let d =
                                ((p.0 - q.0).powi(2) + (p.1 - q.1).powi(2) + (p.2 - q.2).powi(2))
                                    .sqrt();
                            if closest.is_none_or(|(best, _, _)| d < best) {
                                closest = Some((d, c, o));
                            }
                        }
                    }
                }
            }
            let near = closest.map_or("null".to_string(), |(d, c, o)| {
                let (p, q) = (now(c), now(o));
                let link = g.cells.neighbors(c).iter().find(|e| e.dest == o).map_or("none".to_string(), |e| format!("{}", e.cost));
                let w = |id: CellId| g.wall_state.dist.get(id as usize).copied().unwrap_or(f32::INFINITY);
                let filing = |id: CellId| -> String {
                    let Some(lazy) = self.lazy.as_ref() else { return "null".into() };
                    let Some(el) = lazy.layers.cells.get(&id) else { return "\"no element\"".into() };
                    let layer = g.cells.layer(id);
                    let filed = lazy.layers.submeshes.get(&(layer, el.tile)).is_some_and(|s| s.cells.contains(&id));
                    let p = now(id);
                    let r = 0.5f32;
                    let found = lazy.layers.lookup(&lazy.table, [p.0 - r, p.0 + r, p.1 - r, p.1 + r]);
                    let cands = lazy.table.candidates([p.0 - r, p.0 + r, p.1 - r, p.1 + r]);
                    format!(
                        "{{\"anchor\":{},\"tile\":[{},{}],\"filed\":{filed},\"tile_layers\":{:?},\"lookup_has\":{},\"candidates\":{:?}}}",
                        el.anchor, el.tile.0, el.tile.1,
                        lazy.layers.tiles.get(&el.tile).map(|t| t.iter().copied().collect::<Vec<_>>()).unwrap_or_default(),
                        found.contains(&(layer, el.tile)),
                        cands.iter().map(|&c| lazy.table.node_id(c)).collect::<Vec<_>>()
                    )
                };
                format!(
                    "{{\"gap_m\":{d:.3},\"start_side\":[{:.2},{:.2},{:.2}],\"goal_side\":[{:.2},{:.2},{:.2}],\"layers\":[{},{}],\"link_cost\":\"{link}\",\"walls\":[{:.2},{:.2}],\"start_side_links\":{},\"goal_side_links\":{},\"start_filing\":{},\"goal_filing\":{}}}",
                    p.0, p.1, p.2, q.0, q.1, q.2, g.cells.layer(c), g.cells.layer(o), w(c), w(o),
                    g.cells.neighbors(c).len(), g.cells.neighbors(o).len(), filing(c), filing(o)
                )
            });
            return format!("{{\"cells_connect\":false,\"reached\":{reached},\"goal_component\":{},\"closest\":{near},\"boundary\":[{}]}}", from_goal.len(), list.join(","));
        }
        let mut path = vec![b];
        while *path.last().unwrap() != a {
            path.push(pred[path.last().unwrap()]);
        }
        path.reverse();
        let owner = |c: CellId| g.cell_state.source.get(c as usize).copied();
        let mut gap = String::from("null");
        for w in path.windows(2) {
            if let (Some(oa), Some(ob)) = (owner(w[0]), owner(w[1])) {
                if oa != ob && g.node_edges.between(oa, ob).is_none() {
                    let (p, q) = (now(w[0]), now(w[1]));
                    gap = format!(
                        "{{\"at\":[{:.2},{:.2},{:.2}],\"next\":[{:.2},{:.2},{:.2}],\"layers\":[{},{}],\"owners\":[{oa},{ob}],\"owner_nodes\":[{},{}]}}",
                        p.0, p.1, p.2, q.0, q.1, q.2, g.cells.layer(w[0]), g.cells.layer(w[1]),
                        g.nodes.node_at(oa).is_some(), g.nodes.node_at(ob).is_some()
                    );
                    break;
                }
            }
        }
        format!("{{\"cells_connect\":true,\"reached\":{reached},\"path_cells\":{},\"first_gap\":{gap}}}", path.len())
    }

    /// The cells within `radius` of `center` where they are now, their links (flagged when across layers), the
    /// keyframes and the current layer's voxels, as the JSON the toy render page draws; `cylinders` (x, y, radius) and `found` (layer, v0 tile)
    /// come from the caller.
    pub fn snapshot_json(
        &self,
        name: &str,
        center: (f32, f32),
        radius: f32,
        cylinders: &[(f32, f32, f32)],
        found: &[(Version, (i32, i32))],
    ) -> String {
        use std::fmt::Write;
        let cells = &self.graph.cells;
        let Some(lazy) = self.lazy.as_ref() else {
            return String::new();
        };
        let vs = lazy.voxel_size;
        let now = |id: CellId| self.stale_point(id).unwrap_or_else(|| cells.point(id, vs));
        let near = |p: (f32, f32, f32)| (p.0 - center.0).hypot(p.1 - center.1) <= radius;
        let ids: Vec<CellId> = cells.ids().filter(|&id| near(now(id))).collect();
        let kept: BTreeSet<CellId> = ids.iter().copied().collect();
        let mut j = String::new();
        let _ = write!(
            j,
            "{{\"name\":\"{name}\",\"current_layer\":{},\"voxel_size\":{vs},\"cells\":[",
            lazy.layers.current
        );
        for (n, &id) in ids.iter().enumerate() {
            let (p, k) = (now(id), cells.coord(id));
            let anchor = lazy.layers.cells.get(&id).map_or(-1, |e| e.anchor as i64);
            let _ = write!(
                j,
                "{}{{\"id\":{id},\"layer\":{},\"key\":[{},{},{}],\"now\":[{:.3},{:.3},{:.3}],\"anchor\":{anchor},\"wall\":{:.3}}}",
                if n > 0 { "," } else { "" },
                cells.layer(id),
                k.0,
                k.1,
                k.2,
                p.0,
                p.1,
                p.2,
                self.graph.wall_state.dist.get(id as usize).copied().unwrap_or(99.0).min(99.0)
            );
        }
        j.push_str("],\"edges\":[");
        let mut first = true;
        for &id in &ids {
            for e in cells.neighbors(id) {
                if kept.contains(&e.dest)
                    && (e.dest > id || !cells.neighbors(e.dest).iter().any(|b| b.dest == id))
                {
                    let cross = u8::from(cells.layer(e.dest) != cells.layer(id));
                    let blocked = u8::from(!e.cost.is_finite());
                    let _ = write!(
                        j,
                        "{}[{id},{},{cross},{blocked}]",
                        if first { "" } else { "," },
                        e.dest
                    );
                    first = false;
                }
            }
        }
        j.push_str("],\"keyframes\":[");
        let yaw = |p: &Pose| 2.0 * p.rotation[2].atan2(p.rotation[3]);
        let mut first = true;
        for slot in 0..lazy.table.len() as u32 {
            let (raw, pose) = (lazy.table.raw(slot), lazy.table.now(slot));
            let t = pose.translation;
            if (t[0] as f32 - center.0).hypot(t[1] as f32 - center.1) > radius {
                continue;
            }
            let rect = lazy
                .table
                .current_rect(slot)
                .map_or("null".to_string(), |r| {
                    format!("[{:.3},{:.3},{:.3},{:.3}]", r[0], r[1], r[2], r[3])
                });
            let _ = write!(
                j,
                "{}{{\"node_id\":{},\"raw\":[{:.3},{:.3},{:.3}],\"now\":[{:.3},{:.3},{:.3}],\"rect\":{rect}}}",
                if first { "" } else { "," },
                lazy.table.node_id(slot),
                raw.translation[0],
                raw.translation[1],
                yaw(&raw),
                t[0],
                t[1],
                yaw(&pose)
            );
            first = false;
        }
        j.push_str("],\"cylinders\":[");
        for (n, c) in cylinders.iter().enumerate() {
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
        for (n, (layer, tile)) in found.iter().enumerate() {
            let _ = write!(
                j,
                "{}[{layer},{},{}]",
                if n > 0 { "," } else { "" },
                tile.0,
                tile.1
            );
        }
        // the current layer's voxels under the disc, by key
        j.push_str("],\"voxels\":[");
        let mut first = true;
        for (ix, iy, iz) in self.by_col.keys() {
            let (x, y) = ((ix as f32 + 0.5) * vs, (iy as f32 + 0.5) * vs);
            if (x - center.0).hypot(y - center.1) <= radius {
                let _ = write!(j, "{}[{ix},{iy},{iz}]", if first { "" } else { "," });
                first = false;
            }
        }
        let _ = write!(j, "],\"tile_m\":{}}}", lazy.layers.params.tile_m);
        j
    }

    /// A cell's key where it is now: its own in the current layer, its carried point's for a stale one.
    pub(crate) fn key_now(&self, id: CellId) -> VoxelKey {
        key_now(self.lazy.as_ref(), &self.graph.cells, id)
    }

    /// The ring around a touched area that must be in the current layer: what the surface extraction pads by and the node window reaches.
    pub(super) fn migrate_margin_m(&self, config: &Config) -> f32 {
        let pad = 2 * config.closing_passes() as i32;
        (pad + config.buffer_window_cells()) as f32 * config.voxel_size
    }

    /// Link current-layer cells in the columns around `keys` to stale cells beside them now, so a patch that meets
    /// an older pass it never shared a link with stays connected; returns the keys of the current cells it linked.
    pub(super) fn stitch_stale(&mut self, keys: &[VoxelKey], config: &Config) -> Vec<VoxelKey> {
        let Some(lazy) = self.lazy.as_mut() else {
            return Vec::new();
        };
        let current = lazy.layers.current;
        if keys.is_empty()
            || lazy
                .layers
                .submeshes
                .range(..(current, (i32::MIN, i32::MIN)))
                .next()
                .is_none()
        {
            return Vec::new();
        }
        let vs = config.voxel_size;
        const RING: [(i32, i32); 5] = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)];
        // The current cells in and beside the keys' columns link; a stale cell one column past them can be linked to.
        let columns: BTreeSet<(i32, i32)> = keys
            .iter()
            .flat_map(|k| RING.map(|(dx, dy)| (k.0 + dx, k.1 + dy)))
            .collect();
        let mut reach: ahash::AHashSet<(i32, i32)> = ahash::AHashSet::new();
        // One lookup per tile-sized block of columns.
        let block = ((lazy.layers.params.tile_m / vs).floor() as i32).max(1);
        let mut blocks: BTreeMap<(i32, i32), (i32, i32, i32, i32)> = BTreeMap::new();
        for &(x, y) in &columns {
            for (dx, dy) in RING {
                reach.insert((x + dx, y + dy));
            }
            let b = blocks
                .entry((x.div_euclid(block), y.div_euclid(block)))
                .or_insert((x, x, y, y));
            *b = (b.0.min(x), b.1.max(x), b.2.min(y), b.3.max(y));
        }
        let mut found: BTreeSet<(Version, (i32, i32))> = BTreeSet::new();
        for &(x0, x1, y0, y1) in blocks.values() {
            let rect = [
                (x0 - 1) as f32 * vs,
                (x1 + 2) as f32 * vs,
                (y0 - 1) as f32 * vs,
                (y1 + 2) as f32 * vs,
            ];
            found.extend(lazy.layers.lookup(&lazy.table, rect));
        }
        if found.is_empty() {
            return Vec::new();
        }
        self.clock.count("stitch_submeshes", found.len());
        let cells = &self.graph.cells;
        let frame = lazy.table.frame_version();
        let mut stale_at: BTreeMap<(i32, i32), Vec<(CellId, f32)>> = BTreeMap::new();
        let mut taken: ahash::AHashSet<CellId> = ahash::AHashSet::new();
        let mut scanned = 0usize;
        for key @ &(layer, _) in &found {
            let index = lazy.carried_index(*key, frame, cells, vs);
            // Each block's columns, one past the reach, by the column a stale cell stands in now.
            for &(x0, x1, y0, y1) in blocks.values() {
                for (&(x, y), at) in index.range((x0 - 2, i32::MIN)..=(x1 + 2, i32::MAX)) {
                    if y < y0 - 2 || y > y1 + 2 || !reach.contains(&(x, y)) {
                        continue;
                    }
                    for &(id, z) in at {
                        scanned += 1;
                        if cells.is_live(id) && cells.layer(id) == layer && taken.insert(id) {
                            stale_at.entry((x, y)).or_default().push((id, z));
                        }
                    }
                }
            }
        }
        self.clock.count("stitch_scanned", scanned);
        let max_rise = config.step_threshold_m + config.plane_noise_m;
        let mut links: Vec<(CellId, CellId, f32)> = Vec::new();
        for &(x, y) in &columns {
            for &iz in self.graph.surface_lookup.get(&(x, y)).into_iter().flatten() {
                let Some(a) = cells.id((x, y, iz)) else {
                    continue;
                };
                let za = cells.z(a);
                for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    for &(b, zb) in stale_at.get(&(x + dx, y + dy)).into_iter().flatten() {
                        let rise = (zb - za).abs();
                        if rise <= max_rise {
                            links.push((a, b, rise));
                        }
                    }
                }
            }
        }
        let mut linked: BTreeSet<VoxelKey> = BTreeSet::new();
        for (a, b, rise) in links {
            let cost = vs.hypot(rise);
            let cells = &mut self.graph.cells;
            cells.link(
                a,
                Edge {
                    dest: b,
                    base_cost: cost,
                    rise,
                    cost,
                },
            );
            cells.link(
                b,
                Edge {
                    dest: a,
                    base_cost: cost,
                    rise,
                    cost,
                },
            );
            linked.insert(cells.coord(a));
        }
        linked.into_iter().collect()
    }

    /// Migrate the next reform.migrate_per_update elements (voxels and cells) of the queued submeshes, the ones a
    /// region update found around it first; 0 migrates them all. A submesh larger than what is left of the budget
    /// moves a strip of its columns and stays queued.
    fn migrate_pending(&mut self, config: &Config) {
        let Some(lazy) = self.lazy.as_ref() else {
            return;
        };
        if lazy.urgent.is_empty() && lazy.pending.is_empty() {
            return;
        }
        let budget = match config.reform.migrate_per_update {
            0 => usize::MAX,
            n => n as usize,
        };
        let outer = self.clock.enter("pending");
        let stage = Instant::now();
        let caps = self.map_capacities();
        let migrated = self.migrate(budget, config);
        self.clock.add("migrate", stage);
        let grown = caps
            .iter()
            .zip(self.map_capacities())
            .filter(|(a, b)| *b > **a)
            .count();
        self.clock.count("maps_grown", grown);
        self.repair_migrated(migrated, config);
        self.clock.enter(outer);
    }

    /// Capacities of the maps a migration grows, to tell a rehash (a resize, or a rehash in place of a map full of
    /// tombstones, both of which raise it) from the work itself.
    fn map_capacities(&self) -> [usize; 4] {
        let lazy = self.lazy.as_ref();
        [
            self.planes.capacity(),
            self.graph.surface_lookup.capacity(),
            lazy.map_or(0, |l| l.layers.voxels.capacity()),
            lazy.map_or(0, |l| l.layers.cells.capacity()),
        ]
    }

    /// Move up to `budget` elements of the queued submeshes into the current layer by their anchors' poses then and now.
    fn migrate(&mut self, budget: usize, config: &Config) -> Migrated {
        let vs = config.voxel_size;
        let min_normal_z = config.min_normal_z();
        let params = config.placement_params();
        let mut out = Migrated::default();
        let mut flipped: BTreeSet<(i32, i32)> = BTreeSet::new();
        let mut changed: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
        let lazy = self.lazy.as_mut().expect("migrate needs the lazy state");
        let current = lazy.layers.current;
        let mut left = budget;
        let (mut submeshes, mut voxels_moved) = (0usize, 0usize);
        while left > 0 {
            let (key, urgent) = match lazy.urgent.pop_first() {
                Some(key) => (key, true),
                None => match lazy.pending.pop_first() {
                    Some(key) => (key, false),
                    None => break,
                },
            };
            let (layer, tile) = key;
            let Some(mut sub) = lazy.layers.submeshes.remove(&key) else {
                continue;
            };
            let Some(mut frozen) = lazy.layers.stale.remove(&layer) else {
                continue;
            };
            submeshes += 1;
            let (strip_voxels, strip_cells) = take_strip(&mut sub, left, &self.graph.cells);
            left -= (strip_voxels.len() + strip_cells.len()).min(left);
            if sub.is_empty() {
                lazy.carried.remove(&key);
                if let Some(layers) = lazy.layers.tiles.get_mut(&tile) {
                    layers.remove(&layer);
                    if layers.is_empty() {
                        lazy.layers.tiles.remove(&tile);
                    }
                }
            } else {
                for k in &strip_voxels {
                    if let Some(el) = frozen.voxels.get(k) {
                        sub.count(el.anchor, -1);
                    }
                }
                for id in &strip_cells {
                    if let Some(el) = lazy.layers.cells.get(id) {
                        sub.count(el.anchor, -1);
                    }
                }
                lazy.layers.submeshes.insert(key, sub);
                if urgent {
                    lazy.urgent.insert(key);
                } else {
                    lazy.pending.insert(key);
                }
            }
            voxels_moved += strip_voxels.len();
            for k in strip_voxels {
                let Some(el) = frozen.voxels.remove(&k) else {
                    continue;
                };
                let plane = frozen.planes.remove(&k);
                frozen.by_col.remove(k);
                let carry = frozen.carry(el.anchor, &lazy.table);
                let to = voxel_of(carry.apply(center(k, vs)), vs);
                if self.by_col.contains(to) {
                    continue;
                }
                let col = (to.0, to.1);
                if self.by_col.get(col).is_none() {
                    flipped.insert(col);
                } else {
                    changed.entry(col).or_default().push(to.2);
                }
                self.by_col.add(to);
                if let Some(p) = plane.and_then(|p| carry_plane(p, k, to, &carry, vs, min_normal_z))
                {
                    self.planes.insert(to, p);
                }
                lazy.layers.add_voxel(to, el);
                out.columns.insert(col);
            }
            for id in strip_cells {
                if !self.graph.cells.is_live(id) || self.graph.cells.layer(id) != layer {
                    continue;
                }
                let Some(&el) = lazy.layers.cells.get(&id) else {
                    continue;
                };
                let from = self.graph.cells.coord(id);
                if let Some(zs) = frozen.surface_lookup.get_mut(&(from.0, from.1)) {
                    zs.retain(|&z| z != from.2);
                    if zs.is_empty() {
                        frozen.surface_lookup.remove(&(from.0, from.1));
                    }
                }
                let carry = frozen.carry(el.anchor, &lazy.table);
                let to = voxel_of(carry.apply(center(from, vs)), vs);
                out.columns.insert((to.0, to.1));
                out.rekeyed.push(to);
                match self.graph.cells.id(to) {
                    None => {
                        self.graph.cells.move_to_layer(id, to);
                        let zs = self.graph.surface_lookup.entry((to.0, to.1)).or_default();
                        if let Err(at) = zs.binary_search(&to.2) {
                            zs.insert(at, to.2);
                        }
                        self.graph
                            .nodes
                            .rekey_cell(&self.graph.cells, &params, id, from);
                        lazy.layers.cells.remove(&id);
                        lazy.layers.add_cell(id, current, el);
                    }
                    Some(into) => {
                        merge_cell(&mut self.graph.cells, id, into);
                        out.merged.push((id, into));
                        out.gone.extend(self.graph.nodes.transfer(
                            &self.graph.cells,
                            &params,
                            id,
                            from,
                            into,
                        ));
                        self.graph.cells.remove_id(id);
                        lazy.layers.cells.remove(&id);
                    }
                }
            }
            let emptied = frozen.by_col.voxel_count() == 0
                && lazy
                    .layers
                    .submeshes
                    .range((layer, (i32::MIN, i32::MIN))..=(layer, (i32::MAX, i32::MAX)))
                    .next()
                    .is_none();
            if !emptied {
                lazy.layers.stale.insert(layer, frozen);
            }
        }
        out.edits.flipped = flipped.into_iter().collect();
        out.edits.changed = changed
            .into_iter()
            .filter(|(col, _)| !out.edits.flipped.contains(col))
            .map(|(col, mut heights)| {
                heights.sort_unstable();
                heights.dedup();
                (col, heights)
            })
            .collect();
        out.edits.changed.sort_unstable_by_key(|&(col, _)| col);
        self.clock.count("submeshes", submeshes);
        self.clock.count("migrated_voxels", voxels_moved);
        self.clock.count("migrated_columns", out.columns.len());
        self.clock.count("rekeyed", out.rekeyed.len());
        out
    }

    /// Re-extract the surface around what migrated and repair edges, nodes and walls there, as a region update would.
    fn repair_migrated(&mut self, migrated: Migrated, config: &Config) {
        let Some((&first, _)) = migrated.columns.iter().next().map(|c| (c, ())) else {
            return;
        };
        let stage = Instant::now();
        self.graph.node_edges.redirect_cells(&migrated.merged);
        if let Some(route) = &mut self.route {
            route.note_cells(migrated.merged.iter().map(|&(id, _)| id));
        }
        self.clock.add("redirect", stage);
        let stage = Instant::now();
        let pad = 2 * config.closing_passes() as i32;
        let mut bbox = (first.0, first.0, first.1, first.1);
        for &(x, y) in &migrated.columns {
            bbox = (bbox.0.min(x), bbox.1.max(x), bbox.2.min(y), bbox.3.max(y));
        }
        let mut footprint = ColumnMask::new(bbox, 2 * pad);
        for &col in &migrated.columns {
            footprint.set(col);
        }
        let write = footprint.dilated(pad);
        self.clock.add("mask", stage);
        self.clock.count("mask_area", write.area());
        let stage = Instant::now();
        let terrain = config.terrain(&self.by_col, &self.planes);
        let new_cells = extract_surfaces_region(
            &terrain,
            config.closing_passes(),
            config.max_closing_box_cells,
            &write,
        );
        self.clock.add("extract", stage);
        let stage = Instant::now();
        let (added, removed) = self.replace_surface_region(&write, &new_cells);
        self.clock.add("replace_surface", stage);
        self.rebuild_region_graph(
            added,
            removed,
            &migrated.rekeyed,
            &migrated.edits,
            migrated.gone,
            config.node_window_cells(),
            config,
        );
    }

    /// A region update from a cloud stamped with closure `version`: one from an older layer is carried into the current layer's frame by the newest keyframe's move between the frames, a newer one is refused for the caller to hold until its closure arrives.
    pub fn update_region_at_version(
        &mut self,
        points: &[MapPoint],
        bounds: &RegionBounds,
        version: Version,
        config: &Config,
    ) -> Result<Option<crate::columns::ColumnWindow>, VersionGap> {
        self.clock.start("region");
        let window = config.node_window_cells();
        let Some(lazy) = &self.lazy else {
            return Ok(self.update_region_within(points, bounds, config, window));
        };
        if version > lazy.table.version() {
            return Err(VersionGap::Ahead);
        }
        // A version since the current layer's frame was fixed is within reform.min_shift_m of it.
        if version >= lazy.table.frame_version() {
            return Ok(self.update_region_within(points, bounds, config, window));
        }
        let Some(anchor) = lazy.table.latest() else {
            return Ok(self.update_region_within(points, bounds, config, window));
        };
        let stage = Instant::now();
        let Some(then) = lazy.table.pose_at(version, anchor) else {
            return Err(VersionGap::Forgotten);
        };
        let carry = lazy.table.frame_pose(anchor).compose(&then.inverse());
        let carried: Vec<MapPoint> = points
            .iter()
            .map(|&((x, y, z), fit)| {
                let p = carry.apply([x as f64, y as f64, z as f64]);
                let n = carry.rotate([fit[0] as f64, fit[1] as f64, fit[2] as f64]);
                (
                    (p[0] as f32, p[1] as f32, p[2] as f32),
                    [n[0] as f32, n[1] as f32, n[2] as f32, fit[3]],
                )
            })
            .collect();
        let mid = 0.5 * (bounds.z_min + bounds.z_max) as f64;
        let origin = carry.apply([bounds.origin_x as f64, bounds.origin_y as f64, mid]);
        let dz = origin[2] - mid;
        let carried_bounds = RegionBounds {
            origin_x: origin[0] as f32,
            origin_y: origin[1] as f32,
            z_min: (bounds.z_min as f64 + dz) as f32,
            z_max: (bounds.z_max as f64 + dz) as f32,
            ..*bounds
        };
        self.clock.add("carry", stage);
        Ok(self.update_region_within(&carried, &carried_bounds, config, window))
    }
}

/// What a closure did to the planner.
#[derive(Clone, Debug)]
pub struct ClosureOutcome {
    pub closure: Closure,
    /// Whether it froze the current layer and started a new one.
    pub new_layer: bool,
    /// How the newest keyframe moved between the old layer's frame and the new one's, which a held goal rides.
    pub carry_newest: Pose,
}

/// Why a stamped cloud cannot be applied now.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VersionGap {
    /// Stamped with a closure not yet received: hold it.
    Ahead,
    /// Stamped with a version older than the poses kept: drop it.
    Forgotten,
}

/// Take the elements of `sub` in its lowest columns by (x, y), all of them when they fit in `budget`; whole columns,
/// so a column's voxels and cells move together and the budget is passed by at most one column.
fn take_strip(
    sub: &mut Submesh,
    budget: usize,
    cells: &SurfaceCells,
) -> (Vec<VoxelKey>, Vec<CellId>) {
    if sub.voxels.len() + sub.cells.len() <= budget {
        return (
            std::mem::take(&mut sub.voxels).into_iter().collect(),
            std::mem::take(&mut sub.cells).into_iter().collect(),
        );
    }
    let column = |k: VoxelKey| (k.0, k.1);
    let mut columns: Vec<(i32, i32)> = sub
        .voxels
        .iter()
        .map(|&k| column(k))
        .chain(sub.cells.iter().map(|&id| column(cells.coord(id))))
        .collect();
    let (_, &mut cut, _) = columns.select_nth_unstable(budget.max(1) - 1);
    // Voxel keys order by column first.
    let voxels: Vec<VoxelKey> = sub
        .voxels
        .iter()
        .copied()
        .take_while(|&k| column(k) <= cut)
        .collect();
    for k in &voxels {
        sub.voxels.remove(k);
    }
    let taken: Vec<CellId> = sub
        .cells
        .iter()
        .copied()
        .filter(|&id| column(cells.coord(id)) <= cut)
        .collect();
    for id in &taken {
        sub.cells.remove(id);
    }
    (voxels, taken)
}

/// Give `into` every link of `from` it lacks, both ways, so links to the rest of the map survive the merge.
fn merge_cell(cells: &mut SurfaceCells, from: CellId, into: CellId) {
    let outbound: Vec<Edge> = cells.neighbors(from).to_vec();
    for e in outbound {
        if e.dest == into || !cells.is_live(e.dest) {
            continue;
        }
        cells.link(into, Edge { dest: e.dest, ..e });
        if let Some(back) = cells
            .neighbors(e.dest)
            .iter()
            .find(|b| b.dest == from)
            .copied()
        {
            cells.link(e.dest, Edge { dest: into, ..back });
        }
    }
}
