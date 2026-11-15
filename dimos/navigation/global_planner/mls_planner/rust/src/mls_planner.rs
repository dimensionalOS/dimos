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

use ahash::AHashMap;
use dimos_module::{debug_throttled, native_config, worker_pool};
use rayon::prelude::*;
use validator::{Validate, ValidationError};

use crate::adjacency::{
    build_surface_cells, build_surface_lookup, rebuild_edges_around, CellId, SurfaceCells,
    SurfaceLookup,
};
use crate::columns::{
    diff_column, ColumnBuckets, ColumnEdit, ColumnIz, ColumnMask, ColumnWindow, VoxelEdits,
};
use crate::dijkstra::RepairWindow;
use crate::edges::{build_node_edges, build_node_edges_region, relink_failed, PlannerGraph};
use crate::lazy::ReformConfig;
use crate::nodes::{
    apply_wall_safe_penalty_region, place_nodes, place_nodes_region, relocate_nodes,
    wall_distances_region, NodeId, PlacementParams, HOLE_SPAN_CELLS,
};
use crate::planner;
use crate::region_viz::SurfaceEntry;
use crate::surfaces::{extract_surfaces, extract_surfaces_region, RISER_REACH};
use crate::terrain::{PlaneMap, SurfacePlane, Terrain};
use crate::voxel::{voxelize, VoxelKey, Xyz};

/// An incoming voxel center and its tracer plane: normal xyz, then offset from the center, NaN without one.
pub type MapPoint = ((f32, f32, f32), [f32; 4]);

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
    /// Steepest voxel plane, in degrees from level, a cell takes its height from; steeper stand at their top face.
    #[validate(range(min = 0.0, exclusive_max = 90.0))]
    pub max_surface_slope_deg: f32,
    /// How far fit noise alone moves a plane's height; plane moves and rises this small are ignored.
    #[validate(range(min = 0.0))]
    pub plane_noise_m: f32,
    /// Soft cost added per meter of vertical climb.
    #[validate(range(min = 0.0))]
    pub step_penalty_weight: f32,
    /// Ground-plane distance from goal at which the planner stops replanning.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f32,
    /// Vertical distance from goal within which the planner counts arrival.
    #[validate(range(exclusive_min = 0.0))]
    pub goal_z_tolerance: f32,
    /// Seconds a goal may go without a safe path before it is aborted.
    #[validate(range(min = 0.0))]
    pub blocked_timeout_s: f32,
    /// Seconds a followed goal may go without the robot moving before it is aborted.
    #[validate(range(min = 0.0))]
    pub progress_timeout_s: f32,
    /// Ground-plane meters the robot must move within the progress timeout window.
    #[validate(range(exclusive_min = 0.0))]
    pub progress_min_m: f32,
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
    /// Whether and how the planner follows GscPGO's loop closures.
    #[validate(nested)]
    pub reform: ReformConfig,
    /// With no full route publish no path, instead of following the cached route to a standoff short of the blockage.
    pub require_full_route: bool,
    /// A region update spanning more columns is a bad pose or bound, refused before it allocates per-column scratch.
    #[validate(range(min = 1))]
    pub max_region_columns: u64,
    /// A surface cluster spanning a box of more cells is left unclosed, as only a stray voxel makes one.
    #[validate(range(min = 1))]
    pub max_closing_box_cells: u64,
    /// A replan to the same goal keeps the last route from this far past both where the new cell path rejoins it and the last change on it; only the stretch before is pulled and smoothed again, m.
    #[validate(range(min = 0.0))]
    pub route_reuse_slack_m: f32,
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

/// How a plan toward the goal ended, with the waypoints to follow.
#[derive(Debug, PartialEq)]
pub enum Plan {
    /// A full path to the goal.
    Full(Vec<Xyz>),
    /// No full path, so the cached one cut where it stops being safe.
    Truncated(Vec<Xyz>),
    /// No full path and nothing safe ahead.
    Blocked,
}

impl Config {
    /// Number of dilation and erosion passes for the closing radius.
    pub fn closing_passes(&self) -> u32 {
        (self.surface_closing_radius / self.voxel_size).ceil() as u32
    }

    /// Robot-height headroom in cells: the farthest above a cell a voxel can still block it.
    pub fn headroom_cells(&self) -> i32 {
        (self.robot_height / self.voxel_size).ceil() as i32
    }

    /// The surface model over `by_col` and `planes` with this robot's limits.
    pub(crate) fn terrain<'a>(&self, by_col: &'a ColumnIz, planes: &'a PlaneMap) -> Terrain<'a> {
        Terrain {
            by_col,
            planes,
            voxel_size: self.voxel_size,
            robot_height: self.robot_height,
            step_m: self.step_threshold_m,
            noise_m: self.plane_noise_m,
            max_slope: self.max_slope(),
        }
    }

    /// The steepest slope a cell's plane may have, dz per meter.
    pub(crate) fn max_slope(&self) -> f32 {
        self.max_surface_slope_deg.to_radians().tan()
    }

    /// The least normal z of a plane a cell takes its height from.
    pub fn min_normal_z(&self) -> f32 {
        self.max_surface_slope_deg.to_radians().cos()
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
            step_m: self.step_threshold_m,
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

/// Wall time of one stage of the last update or closure call, under the phase it ran in (region, pending, near, closure).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StageMs {
    pub phase: &'static str,
    pub stage: &'static str,
    pub ms: f64,
}

/// Per-stage wall time and sizes of the last update_region* or on_closure call, for offline profiling; nothing the planner computes reads it.
#[derive(Default)]
pub(crate) struct StageClock {
    phase: &'static str,
    stages: Vec<StageMs>,
    counts: Vec<(&'static str, &'static str, u64)>,
}

impl StageClock {
    pub(crate) fn start(&mut self, phase: &'static str) {
        self.phase = phase;
        self.stages.clear();
        self.counts.clear();
    }

    /// Enter a phase, returning the one to restore.
    pub(crate) fn enter(&mut self, phase: &'static str) -> &'static str {
        std::mem::replace(&mut self.phase, phase)
    }

    /// Add the time since `since` to `stage` in the current phase.
    pub(crate) fn add(&mut self, stage: &'static str, since: Instant) {
        let ms = ms_since(since);
        let phase = self.phase;
        match self
            .stages
            .iter_mut()
            .find(|s| s.phase == phase && s.stage == stage)
        {
            Some(s) => s.ms += ms,
            None => self.stages.push(StageMs { phase, stage, ms }),
        }
    }

    /// Add `n` to the count `name` in the current phase.
    pub(crate) fn count(&mut self, name: &'static str, n: usize) {
        let phase = self.phase;
        match self.counts.iter_mut().find(|c| c.0 == phase && c.1 == name) {
            Some(c) => c.2 += n as u64,
            None => self.counts.push((phase, name, n as u64)),
        }
    }
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
        self.contains_height(kz, voxel_size) && self.contains_column((kx, ky), voxel_size)
    }

    /// Whether a voxel `kz` cells up sits within the z band.
    fn contains_height(&self, kz: i32, voxel_size: f32) -> bool {
        let z = (kz as f32 + 0.5) * voxel_size;
        z >= self.z_min && z <= self.z_max
    }

    /// Whether a column's center lies within the cylinder's radius.
    fn contains_column(&self, (kx, ky): (i32, i32), voxel_size: f32) -> bool {
        let dx = (kx as f32 + 0.5) * voxel_size - self.origin_x;
        let dy = (ky as f32 + 0.5) * voxel_size - self.origin_y;
        dx * dx + dy * dy <= self.radius * self.radius
    }

    /// Columns in the cylinder's bounding box, what a region update allocates over.
    pub fn column_count(&self, voxel_size: f32) -> f64 {
        let side = (2.0 * self.radius as f64 / voxel_size as f64).ceil() + 1.0;
        side * side
    }

    /// Why this region can't be ingested, None when it can: a non-finite field, an inverted height range, or a box past max_columns.
    pub fn defect(&self, voxel_size: f32, max_columns: u64) -> Option<&'static str> {
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
        if self.column_count(voxel_size) > max_columns as f64 {
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
    /// Every mapped voxel with its usable plane.
    planes: PlaneMap,
    by_col: ColumnIz,
    // Last successful path and its goal: followed while safe when a replan finds no full path.
    last_path: Option<((f32, f32, f32), Vec<VoxelKey>)>,
    // Last goal and whether a full plan reached it, so replan outcomes log
    // on transitions instead of every cycle.
    last_result: Option<((f32, f32, f32), bool)>,
    // Whether the last region update had bad bounds, so a bad pose warns once.
    region_rejected: bool,
    // The keyframe table and version layers, once the first keyframe arrives with config.reform.enabled.
    lazy: Option<lazy_reform::LazyState>,
    // The last full route and what changed on it since, for the next replan to skip or reuse.
    route: Option<route::RouteWatch>,
    // Stage times of the last update or closure call.
    clock: StageClock,
}

/// How big the planner's containers are.
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
}

impl Planner {
    pub fn sizes(&self) -> PlannerSizes {
        PlannerSizes {
            voxels: self.by_col.voxel_count(),
            column_tiles: self.by_col.tile_count(),
            surface_cells: self.graph.cells.live_len(),
            surface_slots: self.graph.cells.slot_capacity(),
            surface_edges: self.graph.cells.edge_count(),
            nodes: self.graph.nodes.len(),
            node_edges: self.graph.node_edges.len(),
            dijkstra_slots: self.graph.cell_state.dist.len() + self.graph.wall_state.dist.len(),
        }
    }

    pub fn new(worker_threads: u32) -> Self {
        Self {
            pool: worker_pool(worker_threads),
            graph: PlannerGraph::default(),
            planes: PlaneMap::new(),
            by_col: ColumnIz::default(),
            last_path: None,
            last_result: None,
            region_rejected: false,
            lazy: None,
            route: None,
            clock: StageClock::default(),
        }
    }

    /// Wall time per stage of the last update_region* or on_closure call, in the order first reached.
    pub fn last_stage_ms(&self) -> &[StageMs] {
        &self.clock.stages
    }

    /// Sizes the last update_region* or on_closure call worked on: (phase, name, count).
    pub fn last_stage_counts(&self) -> &[(&'static str, &'static str, u64)] {
        &self.clock.counts
    }

    /// Rebuild the map from a whole point set. Refused once the lazy reform holds stale layers, which a rebuild would drop.
    pub fn update_global_map(&mut self, points: &[MapPoint], config: &Config) {
        if self.layer_count() > 1 {
            tracing::warn!("Refused a global map rebuild: it would drop the stale layers the lazy reform holds.");
            return;
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let min_normal_z = config.min_normal_z();
            let keys: Vec<VoxelKey> = points
                .iter()
                .map(|&(p, _)| voxelize(p, voxel_size))
                .collect();
            self.planes = points
                .iter()
                .zip(&keys)
                .filter_map(|(&(_, fit), &key)| {
                    Some((key, SurfacePlane::from_fit(fit, min_normal_z)?))
                })
                .collect();
            self.by_col = ColumnIz::from_voxels(&keys);
            self.rebuild_from_voxels(config);
        });
    }

    /// Re-derive surfaces, cells, nodes and edges from the whole voxel map.
    fn rebuild_from_voxels(&mut self, config: &Config) {
        let mut surface: Vec<VoxelKey> = Vec::new();
        let terrain = config.terrain(&self.by_col, &self.planes);
        extract_surfaces(
            &terrain,
            config.closing_passes(),
            config.max_closing_box_cells,
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
        local_points: &[MapPoint],
        bounds: &RegionBounds,
        config: &Config,
    ) -> Option<ColumnWindow> {
        self.clock.start("region");
        self.update_region_within(local_points, bounds, config, config.node_window_cells())
    }

    /// update_region for a region of a seeded map. It repairs out to the full
    /// wall buffer, so the costs around a seeded wall are exact across region
    /// borders.
    pub fn update_seed_region(
        &mut self,
        points: &[MapPoint],
        bounds: &RegionBounds,
        config: &Config,
    ) -> Option<ColumnWindow> {
        self.clock.start("region");
        self.update_region_within(points, bounds, config, config.buffer_window_cells())
    }

    fn update_region_within(
        &mut self,
        local_points: &[MapPoint],
        bounds: &RegionBounds,
        config: &Config,
        window_steps: i32,
    ) -> Option<ColumnWindow> {
        let defect = bounds.defect(config.voxel_size, config.max_region_columns);
        let first_rejected = defect.is_some() && !self.region_rejected;
        self.region_rejected = defect.is_some();
        if let Some(defect) = defect {
            if !first_rejected {
                return None;
            }
            tracing::warn!(
                defect,
                origin_x = bounds.origin_x,
                origin_y = bounds.origin_y,
                radius = bounds.radius,
                z_min = bounds.z_min,
                z_max = bounds.z_max,
                columns = bounds.column_count(config.voxel_size),
                points = local_points.len(),
                "Rejected a region update with bad bounds, and will until one is good."
            );
            return None;
        }
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let pad = (2 * config.closing_passes()) as i32;

            // Bring the stale parts of the map near the region into the current layer first.
            self.reform_near(bounds, config);
            let stage = Instant::now();
            let edits = self.replace_region_voxels(local_points, bounds, config);
            let diff_ms = ms_since(stage);
            self.clock.add("replace_voxels", stage);
            // No voxel changed, so surfaces and the graph are untouched.
            if edits.is_empty() {
                return None;
            }

            // A changed column shifts surfaces only within pad of it (and a
            // riser bridge one column more), and the extraction reads as much again.
            let bbox = bounds.column_bbox(voxel_size);
            let reach = pad + RISER_REACH;
            let mut footprint = ColumnMask::new(bbox, 2 * reach);
            for col in edits.columns() {
                footprint.set(col);
            }
            let write = footprint.dilated(reach);
            let stage = Instant::now();
            let terrain = config.terrain(&self.by_col, &self.planes);
            let new_cells = extract_surfaces_region(
                &terrain,
                config.closing_passes(),
                config.max_closing_box_cells,
                &write,
            );
            let extract_ms = ms_since(stage);
            self.clock.add("extract", stage);
            self.clock.count("mask_area", write.area());
            let stage = Instant::now();
            let (added, removed) = self.replace_surface_region(&write, &new_cells);
            let replace_ms = ms_since(stage);
            self.clock.add("replace_surface", stage);
            let (cells_added, cells_removed) = (added.len(), removed.len());

            let stage = Instant::now();
            self.rebuild_region_graph(
                added,
                removed,
                &[],
                &edits,
                Vec::new(),
                window_steps,
                config,
            );
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

    /// Patch changed cells and repair nodes and edges around them and the edits; `moves` carry nodes to target cells and `rekeyed` keys get their adjacency rebuilt.
    #[allow(clippy::too_many_arguments)]
    fn rebuild_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        rekeyed: &[VoxelKey],
        edits: &VoxelEdits,
        gone: Vec<NodeId>,
        window_steps: i32,
        config: &Config,
    ) {
        self.repair_region_graph(added, removed, rekeyed, edits, gone, window_steps, config);
        let stage = Instant::now();
        self.graph.cells.recycle_freed();
        self.clock.add("recycle", stage);
    }

    #[allow(clippy::too_many_arguments)]
    fn repair_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        rekeyed: &[VoxelKey],
        edits: &VoxelEdits,
        gone: Vec<NodeId>,
        window_steps: i32,
        config: &Config,
    ) {
        let stage = Instant::now();
        let patch = self.patch_cells(&added, &removed, &edits.retuned, config);
        self.clock.add("patch_cells", stage);
        let mut seeds = added;
        seeds.extend_from_slice(&removed);
        seeds.extend_from_slice(rekeyed);
        seeds.extend(patch.retuned.iter().map(|&id| self.graph.cells.coord(id)));
        let stage = Instant::now();
        let stitched = self.stitch_stale(&seeds, config);
        self.clock.add("stitch_stale", stage);
        self.clock.count("seeds", seeds.len());
        seeds.extend(stitched);
        if seeds.is_empty() && edits.is_empty() && gone.is_empty() {
            return;
        }
        let relocations: Vec<(usize, VoxelKey, VoxelKey)> = patch
            .dead_nodes
            .iter()
            .map(|&(ni, coord)| (ni, coord, coord))
            .collect();

        let reach = config
            .terrain(&self.by_col, &self.planes)
            .step_reach_cells();
        let stage = Instant::now();
        let affected = self.changed_neighborhood(&seeds, reach);
        self.clock.add("neighborhood", stage);
        let stage = Instant::now();
        let graph = &mut self.graph;
        rebuild_edges_around(
            &mut graph.cells,
            &graph.surface_lookup,
            &affected,
            &config.terrain(&self.by_col, &self.planes),
        );
        self.clock.add("rebuild_edges", stage);
        let stage = Instant::now();
        let params = config.placement_params();
        let mut gone_nodes = relocate_nodes(
            &graph.cells,
            &graph.surface_lookup,
            &mut graph.nodes,
            &relocations,
            &params,
        );
        gone_nodes.extend(gone);
        self.clock.add("relocate", stage);
        let stage = Instant::now();
        let window_cells = self.node_window(edits, &affected, window_steps, config);
        self.clock.add("node_window", stage);
        self.clock.count("affected_cells", affected.len());
        self.clock.count("window_cells", window_cells.len());
        let stage = Instant::now();
        if let Some(route) = &mut self.route {
            let (lazy, cells) = (self.lazy.as_ref(), &self.graph.cells);
            route.note_cells(
                [
                    &patch.added,
                    &patch.removed,
                    &patch.retuned,
                    &affected,
                    &window_cells,
                ]
                .into_iter()
                .flatten()
                .copied(),
            );
            let now = |&id: &CellId| {
                let k = lazy_reform::key_now(lazy, cells, id);
                (k.0, k.1)
            };
            route.note_columns(
                seeds
                    .iter()
                    .map(|k| (k.0, k.1))
                    .chain(affected.iter().chain(&window_cells).map(now)),
            );
        }
        self.clock.add("route_note", stage);
        if window_cells.is_empty() && patch.removed.is_empty() && gone_nodes.is_empty() {
            return;
        }
        let stage = Instant::now();
        let graph = &mut self.graph;
        let window = RepairWindow::new(
            &graph.cells,
            &window_cells,
            &mut graph.cluster_index,
            &mut graph.node_scratch.seen,
        );
        self.clock.add("repair_window", stage);
        let stage = Instant::now();
        let lazy = self.lazy.as_ref();
        let cells = &graph.cells;
        wall_distances_region(
            cells,
            &config.terrain(&self.by_col, &self.planes),
            &window,
            &|id| lazy_reform::key_now(lazy, cells, id),
            &mut graph.wall_state,
        );
        self.clock.add("wall_field", stage);
        let stage = Instant::now();
        gone_nodes.extend(place_nodes_region(
            &graph.cells,
            &params,
            &patch.added,
            &window,
            &graph.wall_state.dist,
            &graph.cell_state,
            &mut graph.nodes,
        ));
        self.clock.add("place_nodes", stage);
        let stage = Instant::now();
        apply_wall_safe_penalty_region(
            &mut graph.cells,
            &graph.wall_state.dist,
            &params,
            &window_cells,
            &mut graph.node_scratch,
        );
        self.clock.add("wall_penalty", stage);
        let stage = Instant::now();
        let failed = build_node_edges_region(
            &graph.cells,
            &graph.nodes,
            &window,
            &patch.removed,
            &gone_nodes,
            &mut graph.cell_state,
            &mut graph.node_edges,
        );
        drop(window);
        self.clock.add("node_edges", stage);
        if !failed.is_empty() {
            let stage = Instant::now();
            let report = relink_failed(
                &graph.cells,
                &graph.nodes,
                &failed,
                &mut graph.cell_state,
                &mut graph.node_edges,
                &mut graph.cluster_index,
                &mut graph.node_scratch.seen,
            );
            tracing::debug!(
                pairs = report.pairs,
                cells = report.cells,
                unlinked = report.unlinked,
                failed_again = report.failed_again,
                ms = ms_since(stage),
                "relinked node pairs whose corridor walk failed"
            );
            self.clock.add("relink_failed", stage);
        }
    }

    /// Apply the surface cell changes, and the new surface of kept cells whose plane moved.
    fn patch_cells(
        &mut self,
        added: &[VoxelKey],
        removed: &[VoxelKey],
        retuned: &[VoxelKey],
        config: &Config,
    ) -> CellPatch {
        let graph = &mut self.graph;
        let terrain = config.terrain(&self.by_col, &self.planes);
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
        let added: Vec<CellId> = added
            .iter()
            .map(|&c| graph.cells.insert(c, terrain.surface(c)))
            .collect();
        if let Some(lazy) = &mut self.lazy {
            lazy.file_cells(&graph.cells, &removed, &added, config.voxel_size);
        }
        let mut retuned_cells: Vec<CellId> = Vec::new();
        for &c in retuned {
            if graph.cells.id(c).is_some() {
                let id = graph.cells.insert(c, terrain.surface(c));
                graph.nodes.follow_cell(&graph.cells, id, config.voxel_size);
                retuned_cells.push(id);
            }
        }
        CellPatch {
            added,
            removed,
            retuned: retuned_cells,
            dead_nodes,
        }
    }

    /// Replace the cylinder's voxels and planes with the local map points,
    /// ignoring points outside it; a plane moved less than plane_noise_m keeps
    /// its old fit, so the map matches a full rebuild within that. Returns the voxel edits.
    fn replace_region_voxels(
        &mut self,
        local_points: &[MapPoint],
        bounds: &RegionBounds,
        config: &Config,
    ) -> VoxelEdits {
        let (voxel_size, min_normal_z) = (config.voxel_size, config.min_normal_z());
        let inside = |&(p, _): &MapPoint| {
            if !(p.0.is_finite() && p.1.is_finite() && p.2.is_finite()) {
                return None;
            }
            let k = voxelize(p, voxel_size);
            bounds.contains_voxel(k, voxel_size).then_some(k)
        };
        let keys: Vec<VoxelKey> = local_points.par_iter().filter_map(inside).collect();
        let bbox = bounds.column_bbox(voxel_size);
        let buckets = ColumnBuckets::new(&keys, bbox);

        let by_col = &self.by_col;
        let edits: Vec<ColumnEdit> = ColumnIz::tiles_covering(bbox)
            .into_par_iter()
            .flat_map_iter(|tile| {
                let mut local: Vec<ColumnEdit> = Vec::new();
                for (col, old) in by_col.tile_columns_in(tile, bbox) {
                    let new = buckets.column(col);
                    if old.is_empty() && new.is_empty() || !bounds.contains_column(col, voxel_size)
                    {
                        continue;
                    }
                    let in_band = |(_, _, kz)| bounds.contains_height(kz, voxel_size);
                    if let Some(edit) = diff_column(col, old, new, in_band) {
                        local.push(edit);
                    }
                }
                local
            })
            .collect();
        // Each moved plane, and whether its voxel was already mapped.
        let planes = &self.planes;
        let moved: Vec<(VoxelKey, Option<SurfacePlane>, bool)> = local_points
            .par_iter()
            .filter_map(|point| {
                let k @ (ix, iy, iz) = inside(point)?;
                let plane = SurfacePlane::from_fit(point.1, min_normal_z);
                let old = planes.get(&k).copied();
                SurfacePlane::moved(old, plane, voxel_size, config.plane_noise_m).then(|| {
                    let existed = by_col
                        .get((ix, iy))
                        .is_some_and(|zs| zs.binary_search(&iz).is_ok());
                    (k, plane, existed)
                })
            })
            .collect();
        self.apply_column_edits(&edits, moved, voxel_size)
    }

    /// Apply per-column voxel removals and additions to the map, then the moved planes (key, new plane, whether the voxel was mapped before).
    fn apply_column_edits(
        &mut self,
        edits: &[ColumnEdit],
        moved: Vec<(VoxelKey, Option<SurfacePlane>, bool)>,
        voxel_size: f32,
    ) -> VoxelEdits {
        let mut voxel_edits = VoxelEdits::default();
        let mut changed: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
        for edit in edits {
            let (ix, iy) = edit.col;
            let was_empty = self.by_col.get(edit.col).is_none();
            for &iz in &edit.removed {
                self.planes.remove(&(ix, iy, iz));
                self.by_col.remove((ix, iy, iz));
            }
            for &iz in &edit.added {
                self.by_col.add((ix, iy, iz));
            }
            if let Some(lazy) = &mut self.lazy {
                lazy.file_voxels(edit, voxel_size);
            }
            if was_empty != self.by_col.get(edit.col).is_none() {
                voxel_edits.flipped.push(edit.col);
            } else {
                changed
                    .entry(edit.col)
                    .or_default()
                    .extend(edit.removed.iter().chain(&edit.added));
            }
        }
        for (key @ (ix, iy, iz), plane, existed) in moved {
            match plane {
                Some(plane) => self.planes.insert(key, plane),
                None => self.planes.remove(&key),
            };
            // A new voxel's plane rides in with its edit, so only a kept one retunes.
            if existed {
                changed.entry((ix, iy)).or_default().push(iz);
                voxel_edits.retuned.push(key);
            }
        }
        voxel_edits.changed = changed
            .into_iter()
            .map(|(col, mut heights)| {
                heights.sort_unstable();
                heights.dedup();
                (col, heights)
            })
            .collect();
        voxel_edits.changed.sort_unstable_by_key(|&(col, _)| col);
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
        self.route = None;
        build_surface_cells(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            &config.terrain(&self.by_col, &self.planes),
        );
        self.graph.cells.recycle_freed();
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
            &config.terrain(&self.by_col, &self.planes),
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
            &config.terrain(&self.by_col, &self.planes),
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

    /// The path from start to goal over the whole graph, stale layers included, in current-frame points; the start snaps in the current layer, the goal there or else to the stale cell standing nearest it.
    pub fn plan(
        &self,
        start: (f32, f32, f32),
        goal: (f32, f32, f32),
        config: &Config,
    ) -> Option<Vec<(f32, f32, f32)>> {
        if self.graph.nodes.is_empty() {
            return None;
        }
        let (point, nearest) = (
            |id| self.stale_point(id),
            |p| self.nearest_stale_cell(p, config),
        );
        let stale = planner::Stale {
            point: &point,
            nearest: &nearest,
        };
        planner::plan(&self.graph, start, goal, config, stale).map(|(wp, _)| wp)
    }

    /// Plan to the goal; with no full route follow the cached path while safe, or nothing with require_full_route.
    pub fn plan_or_truncate(&mut self, start: Xyz, goal: Xyz, config: &Config) -> Plan {
        let planned = if self.graph.nodes.is_empty() {
            None
        } else {
            let (point, nearest) = (
                |id| self.stale_point(id),
                |p| self.nearest_stale_cell(p, config),
            );
            let stale = planner::Stale {
                point: &point,
                nearest: &nearest,
            };
            let reuse = self.route.as_ref().map(|r| (&r.cache, &r.changes));
            planner::plan_reusing(&self.graph, start, goal, config, stale, reuse)
        };
        self.route = None;
        if let Some(((waypoints, cells), cache)) = planned {
            self.route = Some(route::RouteWatch::new(cache, config.voxel_size));
            if self.last_result != Some((goal, true)) {
                tracing::info!(?goal, waypoints = waypoints.len(), "full path to goal");
            }
            self.last_result = Some((goal, true));
            self.last_path = Some((goal, cells));
            return Plan::Full(waypoints);
        }
        if self.last_result != Some((goal, false)) {
            if config.require_full_route {
                tracing::warn!(?goal, "no full path to goal, no path");
            } else {
                tracing::warn!(
                    ?goal,
                    "no full path to goal, following any cached path while safe"
                );
            }
        }
        self.last_result = Some((goal, false));
        if config.require_full_route {
            self.last_path = None;
            return Plan::Blocked;
        }
        let waypoints = match &self.last_path {
            Some((cached_goal, cells)) if *cached_goal == goal => {
                planner::truncate_to_safe(&self.graph, cells, start, config)
            }
            _ => Vec::new(),
        };
        if waypoints.is_empty() {
            Plan::Blocked
        } else {
            Plan::Truncated(waypoints)
        }
    }

    pub fn graph(&self) -> &PlannerGraph {
        &self.graph
    }

    /// Corridor segments of every node edge, for visualization.
    pub fn edge_segments(&self) -> Vec<(VoxelKey, VoxelKey, f32)> {
        self.edge_segment_iter().collect()
    }

    /// Node-edge corridor segments, each end where its cell is now.
    pub fn edge_segment_iter(&self) -> impl Iterator<Item = (VoxelKey, VoxelKey, f32)> + '_ {
        let now = |id: CellId| self.key_now(id);
        self.graph.node_edges.edges().iter().flat_map(move |edge| {
            edge.chain
                .windows(2)
                .filter(|pair| {
                    self.graph.cells.is_live(pair[0]) && self.graph.cells.is_live(pair[1])
                })
                .map(move |pair| (now(pair[0]), now(pair[1]), edge.cost))
        })
    }

    /// Surface cells with their surface height and wall clearance (nearest untraversable edge, +inf unreached), each where it is now.
    pub fn surface_clearance(&self) -> Vec<SurfaceEntry> {
        self.surface_clearance_iter().collect()
    }

    pub fn surface_clearance_iter(&self) -> impl Iterator<Item = SurfaceEntry> + '_ {
        let dist = &self.graph.wall_state.dist;
        self.graph.cells.ids().map(move |id| {
            let d = dist.get(id as usize).copied().unwrap_or(f32::INFINITY);
            match self.stale_point(id) {
                Some(p) => (self.key_now(id), p.2, d),
                None => (self.graph.cells.coord(id), self.graph.cells.z(id), d),
            }
        })
    }

    pub fn voxel_count(&self) -> usize {
        self.by_col.voxel_count()
    }

    pub fn voxel_keys(&self) -> impl Iterator<Item = VoxelKey> + '_ {
        self.by_col.keys()
    }
}

/// The cell store changes of one region update.
struct CellPatch {
    added: Vec<CellId>,
    removed: Vec<CellId>,
    /// Kept cells whose plane moved.
    retuned: Vec<CellId>,
    /// The nodes whose cell died, by list index and the dead coordinate.
    dead_nodes: Vec<(usize, VoxelKey)>,
}

/// Surface cells whose wall adjacency a voxel edit can flip without any
/// surface change, up to HOLE_SPAN_CELLS from the edited column.
fn wall_seed_roots(
    lookup: &SurfaceLookup,
    terrain: &Terrain,
    cells: &SurfaceCells,
    edits: &VoxelEdits,
    config: &Config,
) -> Vec<CellId> {
    let step_dz = terrain.step_reach_cells();
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
                if terrain.by_col.get(col).is_some() {
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

mod lazy_reform;
mod route;
pub use lazy_reform::{ClosureOutcome, VersionGap};
pub use route::ReplanReason;

#[cfg(test)]
pub(crate) fn test_config() -> Config {
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
        max_surface_slope_deg: 45.0,
        plane_noise_m: 0.01,
        step_penalty_weight: 0.0,
        goal_tolerance: 0.3,
        goal_z_tolerance: 0.5,
        blocked_timeout_s: 2.0,
        progress_timeout_s: 10.0,
        progress_min_m: 0.2,
        viz_publish_hz: 2.0,
        viz_cell_m: 4.0,
        viz_sweep_cells: 0,
        worker_threads: 4,
        reform: ReformConfig::off(),
        require_full_route: false,
        max_region_columns: 10_000_000,
        max_closing_box_cells: 250_000_000,
        route_reuse_slack_m: 3.0,
    }
}

#[cfg(test)]
mod region_tests;

#[cfg(test)]
mod plane_tests;

#[cfg(test)]
mod reform_tests;

#[cfg(test)]
mod route_tests;
