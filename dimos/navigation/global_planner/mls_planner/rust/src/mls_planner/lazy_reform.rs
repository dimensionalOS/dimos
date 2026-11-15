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

use std::collections::BTreeSet;
use std::sync::Arc;

use ahash::AHashMap;

use super::{Config, MapPoint, Planner, RegionBounds};
use crate::adjacency::{CellId, Edge, SurfaceCells};
use crate::columns::{ColumnEdit, ColumnMask, VoxelEdits};
use crate::lazy::{
    carry_plane, Closure, Element, EventRecord, FrozenLayer, KeyframeRecord, KeyframeTable, Layers,
    Pose, Rect, Version,
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
}

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
    /// Old key -> new key of every moved cell, for stored corridors.
    renamed: AHashMap<VoxelKey, VoxelKey>,
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
        let lazy = self.lazy.as_mut()?;
        let closure = lazy.table.apply_event(records)?;
        let shift = lazy.table.shift_since_frame();
        if closure.closing.is_empty() && shift <= config.reform.layer_shift_m(config.voxel_size) {
            return Some(ClosureOutcome {
                closure,
                new_layer: false,
                carry_newest: Pose::IDENTITY,
            });
        }
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
        // The cached route is in the old layer's keys.
        self.last_path = None;

        let rects: Vec<Rect> = closure
            .closing
            .iter()
            .filter_map(|&slot| lazy.table.current_rect(slot))
            .collect();
        let pool = Arc::clone(&self.pool);
        pool.install(|| self.reform_rects(&rects, config));
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

    /// Migrate the stale submeshes a region update is about to touch, out to the margin its repair reads.
    pub(super) fn reform_near(&mut self, bounds: &RegionBounds, config: &Config) {
        if self.lazy.is_none() {
            return;
        }
        let margin = bounds.radius + self.migrate_margin_m(config);
        let rect = [
            bounds.origin_x - margin,
            bounds.origin_x + margin,
            bounds.origin_y - margin,
            bounds.origin_y + margin,
        ];
        self.reform_rects(&[rect], config);
    }

    /// Migrate the stale submeshes within reach of a plan's start or goal, so it snaps to the current layer: the goal tolerance plus a node spacing around it.
    pub fn reform_around(&mut self, p: (f32, f32, f32), config: &Config) {
        if self.lazy.is_none() {
            return;
        }
        let reach = config.goal_tolerance + config.node_spacing_m;
        let rect = [p.0 - reach, p.0 + reach, p.1 - reach, p.1 + reach];
        let pool = Arc::clone(&self.pool);
        pool.install(|| self.reform_rects(&[rect], config));
    }

    /// Where a stale layer's cell is in the current frame, by its anchor's pose then and now; None for a current-layer cell.
    pub(crate) fn stale_point(&self, id: CellId) -> Option<(f32, f32, f32)> {
        let lazy = self.lazy.as_ref()?;
        let layer = self.graph.cells.layer(id);
        if layer == lazy.layers.current {
            return None;
        }
        let el = lazy.layers.cells.get(&id)?;
        let carry = lazy.layers.stale.get(&layer)?.carry(el.anchor, &lazy.table);
        let (x, y, z) = self.graph.cells.point(id, lazy.voxel_size);
        let p = carry.apply([x as f64, y as f64, z as f64]);
        Some((p[0] as f32, p[1] as f32, p[2] as f32))
    }

    /// A cell's key where it is now: its own in the current layer, its carried point's for a stale one.
    pub(crate) fn key_now(&self, id: CellId) -> VoxelKey {
        match (self.stale_point(id), &self.lazy) {
            (Some(p), Some(lazy)) => voxelize(p, lazy.voxel_size),
            _ => self.graph.cells.coord(id),
        }
    }

    /// The ring around a touched area that must be in the current layer: what the surface extraction pads by and the node window reaches.
    pub(super) fn migrate_margin_m(&self, config: &Config) -> f32 {
        let pad = 2 * config.closing_passes() as i32;
        (pad + config.buffer_window_cells()) as f32 * config.voxel_size
    }

    /// Look up and migrate the stale submeshes under each current-frame rectangle, then repair the current layer around them.
    fn reform_rects(&mut self, rects: &[Rect], config: &Config) {
        let Some(lazy) = &self.lazy else {
            return;
        };
        let found: BTreeSet<(Version, (i32, i32))> = rects
            .iter()
            .flat_map(|&rect| lazy.layers.lookup(&lazy.table, rect))
            .collect();
        if found.is_empty() {
            return;
        }
        let migrated = self.migrate(&found, config);
        self.repair_migrated(migrated, config);
    }

    /// Move every element of the found submeshes into the current layer by its anchor's pose then and now.
    fn migrate(&mut self, found: &BTreeSet<(Version, (i32, i32))>, config: &Config) -> Migrated {
        let vs = config.voxel_size;
        let min_normal_z = config.min_normal_z();
        let params = config.placement_params();
        let mut out = Migrated::default();
        let mut flipped: BTreeSet<(i32, i32)> = BTreeSet::new();
        let mut changed: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
        let lazy = self.lazy.as_mut().expect("migrate needs the lazy state");
        let current = lazy.layers.current;
        for &(layer, tile) in found {
            let Some(sub) = lazy.layers.submeshes.remove(&(layer, tile)) else {
                continue;
            };
            if let Some(layers) = lazy.layers.tiles.get_mut(&tile) {
                layers.remove(&layer);
                if layers.is_empty() {
                    lazy.layers.tiles.remove(&tile);
                }
            }
            let Some(mut frozen) = lazy.layers.stale.remove(&layer) else {
                continue;
            };
            for &k in &sub.voxels {
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
            for &id in &sub.cells {
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
                        out.renamed.insert(from, to);
                    }
                    Some(into) => {
                        merge_cell(&mut self.graph.cells, id, into);
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
        out
    }

    /// Re-extract the surface around what migrated and repair edges, nodes and walls there, as a region update would.
    fn repair_migrated(&mut self, migrated: Migrated, config: &Config) {
        let Some((&first, _)) = migrated.columns.iter().next().map(|c| (c, ())) else {
            return;
        };
        if !migrated.renamed.is_empty() {
            self.graph.node_edges.rename_cells(&migrated.renamed);
        }
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
        let terrain = config.terrain(&self.by_col, &self.planes);
        let new_cells = extract_surfaces_region(
            &terrain,
            config.closing_passes(),
            config.max_closing_box_cells,
            &write,
        );
        let (added, removed) = self.replace_surface_region(&write, &new_cells);
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
        let Some(lazy) = &self.lazy else {
            return Ok(self.update_region(points, bounds, config));
        };
        if version > lazy.table.version() {
            return Err(VersionGap::Ahead);
        }
        // A version since the current layer's frame was fixed is within reform.min_shift_m of it.
        if version >= lazy.table.frame_version() {
            return Ok(self.update_region(points, bounds, config));
        }
        let Some(anchor) = lazy.table.latest() else {
            return Ok(self.update_region(points, bounds, config));
        };
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
        Ok(self.update_region(&carried, &carried_bounds, config))
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
