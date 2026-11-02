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

use ahash::AHashSet;
use dimos_module::{debug_throttled, native_config, worker_pool};
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
    apply_wall_safe_penalty_region, place_nodes, place_nodes_region, relocate_dead_nodes,
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

pub(crate) fn ms_since(start: Instant) -> f64 {
    start.elapsed().as_secs_f64() * 1e3
}

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
}

impl Planner {
    pub fn new(worker_threads: u32) -> Self {
        Self {
            pool: worker_pool(worker_threads),
            graph: PlannerGraph::default(),
            voxel_map: AHashSet::new(),
            by_col: ColumnIz::default(),
            last_path: None,
            last_result: None,
        }
    }

    pub fn update_global_map(&mut self, points: &[(f32, f32, f32)], config: &Config) {
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let clearance = config.headroom_cells();

            self.voxel_map.clear();
            for &p in points {
                self.voxel_map.insert(voxelize(p, voxel_size));
            }

            let mut surface: Vec<VoxelKey> = Vec::new();
            extract_surfaces(
                &self.voxel_map,
                clearance,
                config.closing_passes(),
                &mut self.by_col,
                &mut surface,
            );
            build_surface_lookup(&surface, &mut self.graph.surface_lookup);

            self.rebuild_graph(config);
        });
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
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let clearance = config.headroom_cells();
            let pad = (2 * config.closing_passes()) as i32;

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
            self.rebuild_region_graph(added, removed, &edits, window_steps, config);
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
    /// so the repair runs for the edits too.
    fn rebuild_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        edits: &VoxelEdits,
        window_steps: i32,
        config: &Config,
    ) {
        let step = config.step_cells();
        let patch = self.patch_cells(&added, &removed);
        let mut seeds = added;
        seeds.extend_from_slice(&removed);
        if seeds.is_empty() && edits.is_empty() {
            return;
        }

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
        let mut gone_nodes = relocate_dead_nodes(
            &graph.cells,
            &graph.surface_lookup,
            &mut graph.nodes,
            &patch.dead_nodes,
            &params,
        );
        let window_cells = self.node_window(edits, &affected, window_steps, config);
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

        let mut voxel_edits = VoxelEdits::default();
        for edit in &edits {
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
            &mut self.graph.node_scratch,
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
        planner::plan(&self.graph, start, goal, config).map(|(wp, _)| wp)
    }

    /// Plan to the goal, or follow the cached path as far as it is still safe.
    /// Returns the waypoints, empty when nothing ahead is traversable (stop).
    pub fn plan_or_truncate(
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

#[cfg(test)]
mod region_tests;
