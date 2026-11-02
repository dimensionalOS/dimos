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

//! Config and the owned-state Planner that builds and queries the MLS graph.

use std::sync::Arc;
use std::time::{Duration, Instant};

use ahash::{AHashMap, AHashSet};
use dimos_module::{debug_throttled, native_config, warn_throttled, worker_pool};
use rayon::prelude::*;
use validator::ValidationError;

use crate::adjacency::{
    build_surface_cells, build_surface_lookup, rebuild_edges_around, CellId, SurfaceCells,
    SurfaceLookup,
};
use crate::columns::{
    diff_column, ColumnBuckets, ColumnEdit, ColumnIz, ColumnMask, ColumnWindow, VoxelEdits,
};
use crate::dijkstra::RepairWindow;
use crate::edges::{build_node_edges, build_node_edges_region, edges_to_segments, PlannerGraph};
use crate::nodes::{
    apply_wall_safe_penalty_region, place_nodes, place_nodes_region, relocate_nodes,
    wall_distances_region, PlacementParams, HOLE_SPAN_CELLS,
};
use crate::planner;
use crate::surfaces::{extract_surfaces, extract_surfaces_region};
use crate::voxel::{voxelize, VoxelKey};

#[native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_wall_buffer"))]
pub struct Config {
    pub world_frame: String,
    /// Frame whose tf pose in the world frame is the planning start.
    pub base_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub robot_height: f32,
    /// Subtracted from the start pose z before snapping to a surface. 0 when
    /// the publisher already ground-projects.
    #[validate(range(min = 0.0))]
    pub start_z_offset_m: f32,
    /// Ignore surface more than this far above the sensor.
    #[validate(range(min = 0.0))]
    pub max_overhead_m: f32,
    /// Radius in meters of the morphological closing that fills small holes in
    /// the extracted surface. Fills holes up to twice this wide.
    #[validate(range(min = 0.0))]
    pub surface_closing_radius: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub node_spacing_m: f32,
    /// Hard clearance. Cells closer than this to a wall or edge are impassable.
    #[validate(range(min = 0.0))]
    pub wall_clearance_m: f32,
    /// Width of the soft standoff zone beyond the clearance. Paths prefer to stay
    /// clearance + buffer from walls.
    #[validate(range(min = 0.0))]
    pub wall_buffer_m: f32,
    /// Peak soft wall penalty at the clearance edge: the cost multiplier there is
    /// 1 + this, decaying to 1 at the outer edge of the buffer zone.
    #[validate(range(min = 0.0))]
    pub wall_buffer_weight: f32,
    /// Max traversable vertical step. Taller steps are impassable.
    #[validate(range(min = 0.0))]
    pub step_threshold_m: f32,
    /// Soft cost added per meter of vertical climb.
    #[validate(range(min = 0.0))]
    pub step_penalty_weight: f32,
    /// Ground-plane distance from goal at which the planner stops replanning.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f32,
    /// Rate cap for the surface_map / nodes / node_edges viz artifacts. 0
    /// disables them entirely. The path output is unthrottled.
    #[validate(range(min = 0.0))]
    pub viz_publish_hz: f32,
    /// Edge of the square cells the surface and edge viz publish by.
    #[validate(range(exclusive_min = 0.0))]
    pub viz_cell_m: f32,
    /// Unchanged cells republished per tick, round robin. 0 turns the sweep off.
    pub viz_sweep_cells: u32,
    /// Worker threads for parallel planner work.
    #[validate(range(min = 1))]
    pub worker_threads: u32,
}

/// The soft wall penalty needs a non-zero zone to act in.
fn validate_wall_buffer(config: &Config) -> Result<(), ValidationError> {
    if config.wall_buffer_weight > 0.0 && config.wall_buffer_m == 0.0 {
        return Err(ValidationError::new(
            "wall_buffer_weight requires wall_buffer_m > 0",
        ));
    }
    Ok(())
}

impl Config {
    /// Number of dilation and erosion passes for the closing radius.
    pub fn closing_passes(&self) -> u32 {
        (self.surface_closing_radius / self.voxel_size).ceil() as u32
    }

    /// Robot-height headroom in cells, the clear space a cell needs to be standable.
    pub fn headroom_cells(&self) -> i32 {
        (self.robot_height / self.voxel_size).ceil() as i32
    }

    /// Max traversable vertical step in cells.
    pub fn step_cells(&self) -> i32 {
        (self.step_threshold_m / self.voxel_size).floor() as i32
    }

    /// Radius of the node window around a changed cell: the hard clearance
    /// plus slack. Costs in the wider penalty band lag until a change reaches them.
    pub fn node_window_cells(&self) -> i32 {
        (self.wall_clearance_m / self.voxel_size).ceil() as i32 + WINDOW_SLACK_CELLS
    }

    /// Radius of the ball for a repair that keeps wall buffer costs exact: the
    /// whole penalty band plus slack.
    pub fn buffer_window_cells(&self) -> i32 {
        ((self.wall_clearance_m + self.wall_buffer_m) / self.voxel_size).ceil() as i32
            + WINDOW_SLACK_CELLS
    }

    /// Columns past a rewritten window the viz must reread: the widest repair
    /// window plus one node spacing for the edges of a relocated node.
    pub fn viz_reach_cells(&self) -> i32 {
        self.buffer_window_cells().max(self.node_window_cells())
            + (self.node_spacing_m / self.voxel_size).ceil() as i32
    }

    /// Config-derived scalars for node placement.
    pub fn placement_params(&self) -> PlacementParams {
        PlacementParams {
            clearance_cells: self.headroom_cells(),
            step_cells: self.step_cells(),
            voxel_size: self.voxel_size,
            node_spacing_m: self.node_spacing_m,
            wall_clearance_m: self.wall_clearance_m,
            wall_buffer_m: self.wall_buffer_m,
            wall_buffer_weight: self.wall_buffer_weight,
            step_penalty_weight: self.step_penalty_weight,
        }
    }
}

pub(crate) const DEBUG_LOG_PERIOD: Duration = Duration::from_secs(5);

/// Cells a repair window reaches past the band it must keep exact.
const WINDOW_SLACK_CELLS: i32 = 2;

/// How far around its start, goal and route a plan re-keys cells a warp moved.
const PLAN_REBIN_M: f32 = 3.0;

/// Re-key-and-replan rounds before a plan settles for the route it has.
const PLAN_REBIN_PASSES: usize = 3;

pub(crate) fn ms_since(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1e3
}

/// Largest region box a local map update allocates over: 10M columns is a
/// radius of ~126 m at 8 cm voxels, already four times a lidar's range, and
/// costs ~300 MB of per-column scratch. A larger one is a bad pose or bound.
pub const MAX_REGION_COLUMNS: f64 = 1e7;

/// Cylindrical region the planner re-derives from a local map slice.
#[derive(Clone, Copy)]
pub struct RegionBounds {
    pub origin_x: f32,
    pub origin_y: f32,
    pub radius: f32,
    pub z_min: f32,
    pub z_max: f32,
}

impl RegionBounds {
    /// This region with its ceiling capped to `max_overhead_m` above the sensor.
    pub fn capped_at(self, sensor_z: f32, max_overhead_m: f32) -> Self {
        RegionBounds {
            z_max: self.z_max.min(sensor_z + max_overhead_m),
            ..self
        }
    }

    fn contains_voxel(&self, (kx, ky, kz): VoxelKey, voxel_size: f32) -> bool {
        let half = voxel_size * 0.5;
        let z = kz as f32 * voxel_size + half;
        if z < self.z_min || z > self.z_max {
            return false;
        }
        let dx = kx as f32 * voxel_size + half - self.origin_x;
        let dy = ky as f32 * voxel_size + half - self.origin_y;
        dx * dx + dy * dy <= self.radius * self.radius
    }

    /// Columns in the cylinder's bounding box, what a region update allocates over.
    pub fn column_count(&self, voxel_size: f32) -> f64 {
        let side = (2.0 * self.radius as f64 / voxel_size as f64).ceil() + 1.0;
        side * side
    }

    /// Why this region can't be ingested, None when it can: a non-finite field,
    /// an inverted height range, or a box past MAX_REGION_COLUMNS.
    pub fn defect(&self, voxel_size: f32) -> Option<&'static str> {
        let fields = [
            self.origin_x,
            self.origin_y,
            self.radius,
            self.z_min,
            self.z_max,
        ];
        if fields.iter().any(|v| !v.is_finite()) {
            return Some("a non-finite field");
        }
        if self.radius < 0.0 || self.z_min > self.z_max {
            return Some("a negative radius or inverted height range");
        }
        if self.column_count(voxel_size) > MAX_REGION_COLUMNS {
            return Some("a box too large to allocate over");
        }
        None
    }

    /// Inclusive voxel-column bounding box of the cylinder in the xy plane.
    fn column_bbox(&self, voxel_size: f32) -> ColumnWindow {
        let inv = 1.0 / voxel_size;
        let x0 = ((self.origin_x - self.radius) * inv).floor() as i32;
        let x1 = ((self.origin_x + self.radius) * inv).floor() as i32;
        let y0 = ((self.origin_y - self.radius) * inv).floor() as i32;
        let y1 = ((self.origin_y + self.radius) * inv).floor() as i32;
        (x0, x1, y0, y1)
    }
}

pub struct Planner {
    // The planner owns its worker pool, so its thread setting cannot collide
    // with other components sharing the process.
    pool: Arc<rayon::ThreadPool>,
    graph: PlannerGraph,
    voxel_map: AHashSet<VoxelKey>,
    by_col: ColumnIz,
    // Last successful path and its goal, for safe truncation when a later
    // replan finds no full path.
    last_path: Option<((f32, f32, f32), Vec<VoxelKey>)>,
    // Last goal and whether a full plan reached it, so replan outcomes log
    // on transitions instead of every cycle.
    last_result: Option<((f32, f32, f32), bool)>,
    // When each voxel landed, for a warp that tells overlapping passes apart.
    // Off unless asked for.
    voxel_times: Option<VoxelTimes>,
}

/// Seconds since `epoch` at which each voxel was added, and the clock edits
/// are stamped with.
pub(crate) struct VoxelTimes {
    epoch: Option<f64>,
    now: f32,
    by_key: AHashMap<VoxelKey, f32>,
    /// Where a warped voxel's point sits off its center, so the next warp moves
    /// that point and a sub-voxel shift adds up instead of being lost. Absent is zero.
    residual: AHashMap<VoxelKey, [f32; 3]>,
}

impl VoxelTimes {
    fn stamp(&mut self, t: f64) {
        let epoch = *self.epoch.get_or_insert(t);
        self.now = (t - epoch) as f32;
    }

    /// Absolute time of a voxel, NaN when unknown.
    fn get(&self, k: &VoxelKey) -> f64 {
        match (self.by_key.get(k), self.epoch) {
            (Some(&t), Some(epoch)) => epoch + t as f64,
            _ => f64::NAN,
        }
    }
}

/// How big the planner's containers are, for the periodic memory log.
#[derive(Clone, Debug, Default)]
pub struct PlannerSizes {
    pub voxels: usize,
    pub column_tiles: usize,
    pub surface_cells: usize,
    pub surface_slots: usize,
    pub surface_edges: usize,
    pub nodes: usize,
    pub node_edges: usize,
    pub dijkstra_slots: usize,
    pub voxel_times: usize,
}

impl Planner {
    /// Voxel centers and surface cell points, flat xyz triples, for offline checks.
    pub fn dump_centers(&self, voxel_size: f32) -> (Vec<f32>, Vec<f32>) {
        let half = voxel_size * 0.5;
        let voxels = self
            .placed_voxels(voxel_size)
            .flat_map(|(x, y, z)| [x, y, z].map(|k| k as f32 * voxel_size + half))
            .collect();
        let surface = self
            .placed_surface(voxel_size)
            .flat_map(|(ix, iy, iz)| {
                let (x, y, z) = crate::voxel::surface_point_xyz(ix, iy, iz, voxel_size);
                [x, y, z]
            })
            .collect();
        (voxels, surface)
    }

    /// Each voxel's key where its point sits, which a warp can move off its own key.
    pub fn placed_voxels(&self, voxel_size: f32) -> impl Iterator<Item = VoxelKey> + '_ {
        self.voxel_map.iter().map(move |&k| {
            let r = self
                .voxel_times
                .as_ref()
                .and_then(|t| t.residual.get(&k).copied())
                .unwrap_or([0.0; 3]);
            voxelize(
                (
                    (k.0 as f32 + 0.5) * voxel_size + r[0],
                    (k.1 as f32 + 0.5) * voxel_size + r[1],
                    (k.2 as f32 + 0.5) * voxel_size + r[2],
                ),
                voxel_size,
            )
        })
    }

    /// Each surface cell's key where its point sits.
    pub fn placed_surface(&self, voxel_size: f32) -> impl Iterator<Item = VoxelKey> + '_ {
        let cells = &self.graph.cells;
        cells.ids().map(move |id| cells.placed(id, voxel_size))
    }

    pub fn sizes(&self) -> PlannerSizes {
        PlannerSizes {
            voxels: self.voxel_map.len(),
            column_tiles: self.by_col.tile_count(),
            surface_cells: self.graph.cells.live_len(),
            surface_slots: self.graph.cells.slot_capacity(),
            surface_edges: self.graph.cells.edge_count(),
            nodes: self.graph.nodes.len(),
            node_edges: self.graph.node_edges.len(),
            dijkstra_slots: self.graph.cell_state.dist.len() + self.graph.wall_state.dist.len(),
            voxel_times: self.voxel_times.as_ref().map_or(0, |t| t.by_key.len()),
        }
    }

    pub fn new(worker_threads: u32) -> Self {
        Self {
            pool: worker_pool(worker_threads),
            graph: PlannerGraph::default(),
            voxel_map: AHashSet::new(),
            by_col: ColumnIz::default(),
            last_path: None,
            last_result: None,
            voxel_times: None,
        }
    }

    /// Record when each voxel lands, for warps. Costs a map entry per voxel.
    pub fn track_voxel_times(&mut self) {
        self.voxel_times.get_or_insert_with(|| VoxelTimes {
            epoch: None,
            now: 0.0,
            by_key: AHashMap::new(),
            residual: AHashMap::new(),
        });
    }

    /// The time, in seconds, the next edits are stamped with.
    pub fn set_clock(&mut self, t: f64) {
        if let Some(times) = &mut self.voxel_times {
            times.stamp(t);
        }
    }

    pub fn update_global_map(&mut self, points: &[(f32, f32, f32)], config: &Config) {
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            self.voxel_map.clear();
            for &p in points {
                self.voxel_map.insert(voxelize(p, voxel_size));
            }
            if let Some(times) = &mut self.voxel_times {
                let now = times.now;
                times.by_key = self.voxel_map.iter().map(|&k| (k, now)).collect();
                times.residual.clear();
            }
            self.rebuild_from_voxels(config);
        });
    }

    /// Re-derive surfaces, cells, nodes and edges from the whole voxel map.
    fn rebuild_from_voxels(&mut self, config: &Config) {
        let mut surface: Vec<VoxelKey> = Vec::new();
        extract_surfaces(
            &self.voxel_map,
            config.headroom_cells(),
            config.closing_passes(),
            &mut self.by_col,
            &mut surface,
        );
        build_surface_lookup(&surface, &mut self.graph.surface_lookup);
        self.rebuild_graph(config);
    }

    /// Update planner artifacts within a local region instead of rebuilding
    /// from the whole map. Returns the inclusive column window the surface
    /// was rewritten in, or None when no voxel changed.
    pub fn update_region(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        config: &Config,
    ) -> Option<ColumnWindow> {
        self.update_region_within(local_points, bounds, config, config.node_window_cells())
    }

    /// update_region for a region of a seeded map. It repairs out to the full
    /// wall buffer, so the costs around a seeded wall are exact across region
    /// borders.
    pub fn update_seed_region(
        &mut self,
        points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        config: &Config,
    ) -> Option<ColumnWindow> {
        self.update_region_within(points, bounds, config, config.buffer_window_cells())
    }

    fn update_region_within(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        config: &Config,
        window_steps: i32,
    ) -> Option<ColumnWindow> {
        if let Some(defect) = bounds.defect(config.voxel_size) {
            warn_throttled!(
                Duration::from_secs(5),
                defect,
                origin_x = bounds.origin_x,
                origin_y = bounds.origin_y,
                radius = bounds.radius,
                z_min = bounds.z_min,
                z_max = bounds.z_max,
                columns = bounds.column_count(config.voxel_size),
                points = local_points.len(),
                "Rejected a region update with bad bounds."
            );
            return None;
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let clearance = config.headroom_cells();
            let pad = (2 * config.closing_passes()) as i32;

            // Re-key what a warp moved out of its keys before the region's data meets it.
            if self.voxel_times.is_some() {
                let margin = config.buffer_window_cells() as f32 * voxel_size;
                self.rebin_near(
                    (bounds.origin_x, bounds.origin_y),
                    bounds.radius + margin,
                    config,
                );
            }
            let stage = Instant::now();
            let edits = self.replace_region_voxels(local_points, bounds, voxel_size);
            let diff_ms = ms_since(stage);
            // No voxel changed, so surfaces and the graph are untouched.
            if edits.is_empty() {
                return None;
            }

            // A changed column shifts surfaces only within pad of it, and
            // the extraction reads one more pad around that.
            let bbox = bounds.column_bbox(voxel_size);
            let mut footprint = ColumnMask::new(bbox, 2 * pad);
            for col in edits.columns() {
                footprint.set(col);
            }
            let write = footprint.dilated(pad);
            let stage = Instant::now();
            let new_cells =
                extract_surfaces_region(&self.by_col, clearance, config.closing_passes(), &write);
            let extract_ms = ms_since(stage);
            let stage = Instant::now();
            let (added, removed) = self.replace_surface_region(&write, &new_cells);
            let replace_ms = ms_since(stage);
            let (cells_added, cells_removed) = (added.len(), removed.len());

            let stage = Instant::now();
            self.rebuild_region_graph(added, removed, &edits, &[], window_steps, config);
            debug_throttled!(
                DEBUG_LOG_PERIOD,
                diff_ms,
                extract_ms,
                replace_ms,
                graph_ms = ms_since(stage),
                bbox_columns = (bbox.1 - bbox.0 + 1) as i64 * (bbox.3 - bbox.2 + 1) as i64,
                changed_columns = edits.columns().count(),
                cells_added,
                cells_removed,
                "region update stages"
            );
            write.bounds()
        })
    }

    /// Patch changed cells, then repair nodes and edges around the change.
    /// A voxel edit can change a wall scan without changing any surface cell,
    /// so the repair runs for the edits too. `moves` are nodes, by list index,
    /// to carry to a target cell, as a warp does.
    fn rebuild_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        edits: &VoxelEdits,
        moves: &[(usize, VoxelKey)],
        window_steps: i32,
        config: &Config,
    ) {
        let step = config.step_cells();
        // A moving node's coordinate, while its cell id still resolves.
        let mut relocations: Vec<(usize, VoxelKey, VoxelKey)> = moves
            .iter()
            .map(|&(ni, target)| {
                let cell = self.graph.nodes[ni].cell_id;
                (ni, self.graph.cells.coord(cell), target)
            })
            .collect();
        let patch = self.patch_cells(&added, &removed);
        let mut seeds = added;
        seeds.extend_from_slice(&removed);
        if seeds.is_empty() && edits.is_empty() && moves.is_empty() {
            return;
        }
        let moving: AHashSet<usize> = moves.iter().map(|&(ni, _)| ni).collect();
        relocations.extend(
            patch
                .dead_nodes
                .iter()
                .filter(|(ni, _)| !moving.contains(ni))
                .map(|&(ni, coord)| (ni, coord, coord)),
        );

        let affected = self.changed_neighborhood(&seeds, step);
        let graph = &mut self.graph;
        rebuild_edges_around(
            &mut graph.cells,
            &graph.surface_lookup,
            &affected,
            config.voxel_size,
            step,
        );
        let params = config.placement_params();
        let mut gone_nodes = relocate_nodes(
            &graph.cells,
            &graph.surface_lookup,
            &mut graph.nodes,
            &relocations,
            &params,
        );
        // A moved node's region and edges change around both its ends.
        let mut roots = affected;
        for &(_, from, to) in &relocations[..moves.len()] {
            roots.extend(graph.cells.id(from));
            roots.extend(graph.cells.id(to));
        }
        let window_cells = self.node_window(edits, &roots, window_steps, config);
        if window_cells.is_empty() && patch.removed.is_empty() && gone_nodes.is_empty() {
            return;
        }
        let graph = &mut self.graph;
        let window = RepairWindow::new(
            &graph.cells,
            &window_cells,
            &mut graph.cluster_index,
            &mut graph.node_scratch.seen,
        );
        wall_distances_region(
            &graph.cells,
            &self.by_col,
            &params,
            &window,
            &mut graph.wall_state,
        );
        gone_nodes.extend(place_nodes_region(
            &graph.cells,
            &params,
            &patch.added,
            &window,
            &graph.wall_state.dist,
            &graph.cell_state,
            &mut graph.nodes,
        ));
        apply_wall_safe_penalty_region(
            &mut graph.cells,
            &graph.wall_state.dist,
            &params,
            &window_cells,
            &mut graph.node_scratch,
        );
        build_node_edges_region(
            &graph.cells,
            &graph.nodes,
            &window,
            &patch.removed,
            &gone_nodes,
            &mut graph.cell_state,
            &mut graph.node_edges,
        );
    }

    /// Apply the surface cell changes to the cell store.
    fn patch_cells(&mut self, added: &[VoxelKey], removed: &[VoxelKey]) -> CellPatch {
        let graph = &mut self.graph;
        // Removal frees cell ids and the inserts recycle them, so a node id
        // captured here is not stable. Capture dead nodes by coordinate while
        // their ids still resolve.
        let dead_nodes: Vec<(usize, VoxelKey)> = removed
            .iter()
            .filter_map(|&c| {
                let i = graph.nodes.node_at(graph.cells.id(c)?)?;
                Some((i, c))
            })
            .collect();
        let removed: Vec<CellId> = removed
            .iter()
            .filter_map(|&c| graph.cells.remove(c))
            .collect();
        let added: Vec<CellId> = added.iter().map(|&c| graph.cells.insert(c)).collect();
        CellPatch {
            added,
            removed,
            dead_nodes,
        }
    }

    /// Replace the cylinder's voxels with the local map points, ignoring
    /// points outside it. Returns the voxel edits.
    fn replace_region_voxels(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        voxel_size: f32,
    ) -> VoxelEdits {
        let incoming: Vec<VoxelKey> = local_points
            .par_iter()
            .filter(|p| p.0.is_finite() && p.1.is_finite() && p.2.is_finite())
            .map(|&p| voxelize(p, voxel_size))
            .filter(|&k| bounds.contains_voxel(k, voxel_size))
            .collect();
        let bbox = bounds.column_bbox(voxel_size);
        let buckets = ColumnBuckets::new(&incoming, bbox);

        let by_col = &self.by_col;
        let edits: Vec<ColumnEdit> = ColumnIz::tiles_covering(bbox)
            .into_par_iter()
            .flat_map_iter(|tile| {
                let mut local: Vec<ColumnEdit> = Vec::new();
                for (col, old) in by_col.tile_columns_in(tile, bbox) {
                    let new = buckets.column(col);
                    if old.is_empty() && new.is_empty() {
                        continue;
                    }
                    if let Some(edit) =
                        diff_column(col, old, new, |k| bounds.contains_voxel(k, voxel_size))
                    {
                        local.push(edit);
                    }
                }
                local
            })
            .collect();
        self.apply_column_edits(&edits)
    }

    /// Apply per-column voxel removals and additions to the map.
    fn apply_column_edits(&mut self, edits: &[ColumnEdit]) -> VoxelEdits {
        let mut voxel_edits = VoxelEdits::default();
        for edit in edits {
            let (ix, iy) = edit.col;
            let was_empty = self.by_col.get(edit.col).is_none();
            for &iz in &edit.removed {
                self.voxel_map.remove(&(ix, iy, iz));
                self.by_col.remove((ix, iy, iz));
            }
            for &iz in &edit.added {
                self.voxel_map.insert((ix, iy, iz));
                self.by_col.add((ix, iy, iz));
            }
            if let Some(times) = &mut self.voxel_times {
                // Fresh data sits where it was observed, with no residual.
                for &iz in &edit.removed {
                    times.by_key.remove(&(ix, iy, iz));
                    times.residual.remove(&(ix, iy, iz));
                }
                for &iz in &edit.added {
                    times.by_key.insert((ix, iy, iz), times.now);
                    times.residual.remove(&(ix, iy, iz));
                }
            }
            if was_empty != self.by_col.get(edit.col).is_none() {
                voxel_edits.flipped.push(edit.col);
            } else {
                let mut heights: Vec<i32> =
                    edit.removed.iter().chain(&edit.added).copied().collect();
                heights.sort_unstable();
                voxel_edits.changed.push((edit.col, heights));
            }
        }
        voxel_edits
    }

    /// Replace the surface_lookup entries for write-mask columns whose cells
    /// changed, leaving identical columns untouched. Returns the added and
    /// removed cells so only the affected parts of the graph get patched.
    fn replace_surface_region(
        &mut self,
        write: &ColumnMask,
        new_cells: &[VoxelKey],
    ) -> (Vec<VoxelKey>, Vec<VoxelKey>) {
        let Some(bbox) = write.bounds() else {
            return (Vec::new(), Vec::new());
        };
        let new_by_col = ColumnBuckets::new(new_cells, bbox);

        let lookup = &self.graph.surface_lookup;
        let changed: Vec<((i32, i32), Vec<i32>)> = write
            .rows()
            .into_par_iter()
            .flat_map_iter(|row| {
                let mut local: Vec<((i32, i32), Vec<i32>)> = Vec::new();
                for col in write.row_columns(row) {
                    let old = lookup.get(&col).map(Vec::as_slice).unwrap_or(&[]);
                    let new = new_by_col.column(col);
                    if old != new {
                        local.push((col, new.to_vec()));
                    }
                }
                local
            })
            .collect();

        let mut added: Vec<VoxelKey> = Vec::new();
        let mut removed: Vec<VoxelKey> = Vec::new();
        for (col, new_zs) in changed {
            let old_zs = self
                .graph
                .surface_lookup
                .get(&col)
                .map(Vec::as_slice)
                .unwrap_or(&[]);
            for &iz in &new_zs {
                if old_zs.binary_search(&iz).is_err() {
                    added.push((col.0, col.1, iz));
                }
            }
            for &iz in old_zs {
                if new_zs.binary_search(&iz).is_err() {
                    removed.push((col.0, col.1, iz));
                }
            }
            if new_zs.is_empty() {
                self.graph.surface_lookup.remove(&col);
            } else {
                self.graph.surface_lookup.insert(col, new_zs);
            }
        }
        (added, removed)
    }

    /// Rebuild all cells from surface_lookup, then nodes and edges.
    fn rebuild_graph(&mut self, config: &Config) {
        let voxel_size = config.voxel_size;
        let step = config.step_cells();

        build_surface_cells(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            voxel_size,
            step,
        );
        self.rebuild_nodes(config);
    }

    /// The changed cells and the live cells one column step from them, each
    /// once. Their adjacency is rebuilt and the node window grows from them.
    fn changed_neighborhood(&mut self, changed: &[VoxelKey], step_dz: i32) -> Vec<CellId> {
        let graph = &mut self.graph;
        let lookup = &graph.surface_lookup;
        let cells = &graph.cells;
        graph.node_scratch.ensure_capacity(cells.slot_capacity());
        let seen = &mut graph.node_scratch.seen;
        let mut out: Vec<CellId> = Vec::new();
        for &(ix, iy, iz) in changed {
            for (dx, dy) in [(0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)] {
                let Some(zs) = lookup.get(&(ix + dx, iy + dy)) else {
                    continue;
                };
                for &nz in zs {
                    if (nz - iz).abs() > step_dz {
                        continue;
                    }
                    if let Some(id) = cells.id((ix + dx, iy + dy, nz)) {
                        if !seen[id as usize] {
                            seen[id as usize] = true;
                            out.push(id);
                        }
                    }
                }
            }
        }
        for &id in &out {
            seen[id as usize] = false;
        }
        out
    }

    /// Live cells within the node-graph margin of the changed cells.
    fn node_window(
        &mut self,
        edits: &VoxelEdits,
        roots: &[CellId],
        steps: i32,
        config: &Config,
    ) -> Vec<CellId> {
        let graph = &mut self.graph;
        let wall_roots = wall_seed_roots(
            &graph.surface_lookup,
            &self.by_col,
            &graph.cells,
            edits,
            config,
        );
        graph
            .node_scratch
            .ensure_capacity(graph.cells.slot_capacity());
        bfs_ball(
            &graph.cells,
            roots.iter().copied().chain(wall_roots),
            steps,
            &mut graph.node_scratch.seen,
        )
    }

    /// Full rebuild of nodes and node edges from the current cells.
    fn rebuild_nodes(&mut self, config: &Config) {
        place_nodes(
            &mut self.graph.cells,
            &self.by_col,
            &config.placement_params(),
            &mut self.graph.wall_state,
            &mut self.graph.nodes,
        );

        build_node_edges(
            &self.graph.cells,
            &self.graph.nodes,
            &mut self.graph.cell_state,
            &mut self.graph.node_edges,
        );
    }

    pub fn plan(
        &self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> Option<Vec<(f32, f32, f32)>> {
        if self.graph.nodes.is_empty() {
            return None;
        }
        planner::plan(&self.graph, start, goal, config)
            .map(|(wp, cells)| self.placed_waypoints(wp, &cells, config.voxel_size))
    }

    /// Re-key the cells a warp moved around the start and the goal, so a plan
    /// finds them where they are. The rest of the route is planned over the
    /// keys the cells kept and placed by their offsets.
    pub fn rebin_for_plan(
        &mut self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) {
        if self.voxel_times.is_none() {
            return;
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            self.rebin_near((start.0, start.1), PLAN_REBIN_M, config);
            self.rebin_near((goal.0, goal.1), PLAN_REBIN_M, config);
        });
    }

    /// Where the cached route's cells that a warp moved off their keys sit, in the ground plane.
    fn displaced_path_points(&self, voxel_size: f32) -> Vec<(f32, f32)> {
        let Some((_, path)) = &self.last_path else {
            return Vec::new();
        };
        let cells = &self.graph.cells;
        path.iter()
            .filter_map(|&k| cells.id(k))
            .filter(|&id| cells.placed(id, voxel_size) != cells.coord(id))
            .map(|id| {
                let (ix, iy, _) = cells.coord(id);
                let o = cells.offset(id);
                (
                    (ix as f32 + 0.5) * voxel_size + o[0],
                    (iy as f32 + 0.5) * voxel_size + o[1],
                )
            })
            .collect()
    }

    /// Plan to the goal, or follow the cached path as far as it is still safe.
    /// Returns the waypoints, empty when nothing ahead is traversable (stop).
    pub fn plan_or_truncate(
        &mut self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> Vec<(f32, f32, f32)> {
        self.rebin_for_plan(start, goal, config);
        let mut waypoints = self.plan_or_truncate_keys(start, goal, config);
        // A route through cells a warp moved without re-keying: re-key along it and plan again,
        // so the route's geometry reads one consistent grid.
        for _ in 0..PLAN_REBIN_PASSES {
            let displaced = self.displaced_path_points(config.voxel_size);
            if displaced.is_empty() {
                break;
            }
            let pool = Arc::clone(&self.pool);
            pool.install(|| self.rebin_around(&displaced, PLAN_REBIN_M, config));
            waypoints = self.plan_or_truncate_keys(start, goal, config);
        }
        let path = self
            .last_path
            .as_ref()
            .map(|(_, cells)| cells.clone())
            .unwrap_or_default();
        self.placed_waypoints(waypoints, &path, config.voxel_size)
    }

    fn plan_or_truncate_keys(
        &mut self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> Vec<(f32, f32, f32)> {
        if !self.graph.nodes.is_empty() {
            if let Some((waypoints, cells)) = planner::plan(&self.graph, start, goal, config) {
                if self.last_result != Some((goal, true)) {
                    tracing::info!(?goal, waypoints = waypoints.len(), "full path to goal");
                }
                self.last_result = Some((goal, true));
                self.last_path = Some((goal, cells));
                return waypoints;
            }
        }
        if self.last_result != Some((goal, false)) {
            tracing::warn!(
                ?goal,
                "no full path to goal, following any cached path while safe"
            );
        }
        self.last_result = Some((goal, false));
        match &self.last_path {
            Some((cached_goal, cells)) if *cached_goal == goal => {
                planner::truncate_to_safe(&self.graph, cells, start, config)
            }
            _ => Vec::new(),
        }
    }

    pub fn graph(&self) -> &PlannerGraph {
        &self.graph
    }

    /// Corridor segments of every node edge, for visualization.
    pub fn edge_segments(&self) -> Vec<(VoxelKey, VoxelKey, f32)> {
        self.pool
            .install(|| edges_to_segments(self.graph.node_edges.edges()))
    }

    /// The same segments without materializing them.
    pub fn edge_segment_iter(&self) -> impl Iterator<Item = (VoxelKey, VoxelKey, f32)> + '_ {
        self.graph.node_edges.edges().iter().flat_map(|edge| {
            edge.chain
                .windows(2)
                .map(move |pair| (pair[0], pair[1], edge.cost))
        })
    }

    pub fn surface(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.graph
            .surface_lookup
            .iter()
            .flat_map(|(&(ix, iy), zs)| zs.iter().map(move |&iz| (ix, iy, iz)))
    }

    /// Surface cells paired with their wall clearance, the distance to the
    /// nearest untraversable edge. Unreached cells report +inf.
    pub fn surface_clearance(&self) -> Vec<(VoxelKey, f32)> {
        self.surface_clearance_iter().collect()
    }

    pub fn surface_clearance_iter(&self) -> impl Iterator<Item = (VoxelKey, f32)> + '_ {
        let dist = &self.graph.wall_state.dist;
        self.graph.cells.ids().map(move |id| {
            let d = dist.get(id as usize).copied().unwrap_or(f32::INFINITY);
            (self.graph.cells.coord(id), d)
        })
    }

    pub fn voxel_count(&self) -> usize {
        self.voxel_map.len()
    }

    pub fn voxel_keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.voxel_map.iter().copied()
    }
}

/// The cell store changes of one region update.
struct CellPatch {
    added: Vec<CellId>,
    removed: Vec<CellId>,
    /// The nodes whose cell died, by list index and the dead coordinate.
    dead_nodes: Vec<(usize, VoxelKey)>,
}

/// Surface cells whose wall adjacency a voxel edit can flip without any
/// surface change, up to HOLE_SPAN_CELLS from the edited column.
fn wall_seed_roots(
    lookup: &SurfaceLookup,
    by_col: &ColumnIz,
    cells: &SurfaceCells,
    edits: &VoxelEdits,
    config: &Config,
) -> Vec<CellId> {
    let step_dz = config.step_cells();
    let reach = config.headroom_cells() + step_dz;
    let reached = |heights: &[i32], nz: i32| {
        let first = heights.partition_point(|&iz| iz < nz - step_dz);
        heights.get(first).is_some_and(|&iz| iz <= nz + reach)
    };
    let mut roots: Vec<CellId> = Vec::new();
    let mut scan_from = |ix: i32, iy: i32, heights: Option<&[i32]>| {
        for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
            for k in 1..=HOLE_SPAN_CELLS {
                let col = (ix + dx * k, iy + dy * k);
                if let Some(zs) = lookup.get(&col) {
                    for &nz in zs {
                        if heights.is_some_and(|heights| !reached(heights, nz)) {
                            continue;
                        }
                        if let Some(id) = cells.id((col.0, col.1, nz)) {
                            roots.push(id);
                        }
                    }
                }
                if by_col.get(col).is_some() {
                    break;
                }
            }
        }
    };
    for ((ix, iy), heights) in &edits.changed {
        scan_from(*ix, *iy, Some(heights));
    }
    for &(ix, iy) in &edits.flipped {
        scan_from(ix, iy, None);
    }
    roots
}

/// The roots and every cell within `steps` hops of one over cell adjacency,
/// each once. `seen` is a dense all-false mask and is left all-false.
fn bfs_ball(
    cells: &SurfaceCells,
    roots: impl Iterator<Item = CellId>,
    steps: i32,
    seen: &mut [bool],
) -> Vec<CellId> {
    let mut ball: Vec<CellId> = Vec::new();
    for id in roots {
        if !seen[id as usize] {
            seen[id as usize] = true;
            ball.push(id);
        }
    }
    let mut frontier: Vec<CellId> = ball.clone();
    for _ in 0..steps {
        if frontier.is_empty() {
            break;
        }
        let mut next: Vec<CellId> = Vec::new();
        for &u in &frontier {
            for e in cells.neighbors(u) {
                let i = e.dest as usize;
                if !seen[i] {
                    seen[i] = true;
                    ball.push(e.dest);
                    next.push(e.dest);
                }
            }
        }
        frontier = next;
    }
    for &id in &ball {
        seen[id as usize] = false;
    }
    ball
}

mod warp;
pub use warp::WarpReport;

#[cfg(test)]
mod region_tests;
