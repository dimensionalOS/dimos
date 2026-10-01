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

use ahash::{AHashMap, AHashSet};
use dimos_module::{native_config, worker_pool};
use rayon::prelude::*;
use validator::ValidationError;

use crate::adjacency::{build_surface_cells, build_surface_lookup, rebuild_edges_around, CellId};
use crate::edges::{build_node_edges, build_node_edges_region, edges_to_segments, PlannerGraph};
use crate::nodes::{
    place_nodes, place_nodes_region, relocate_dead_nodes, PlacementParams, HOLE_SPAN_CELLS,
};
use crate::planner;
use crate::surfaces::{
    add_to_by_col, extract_surfaces, extract_surfaces_region, remove_from_by_col, ColumnIz,
};
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
    /// Edge of the square cells the surface and edge viz publish by. A tick
    /// sends the cells that changed, so a viewer never takes the whole map.
    #[validate(range(exclusive_min = 0.0))]
    pub viz_region_m: f32,
    /// Unchanged cells republished per tick, round robin, so a viewer that
    /// joined late or lost a message fills in. 0 turns the sweep off.
    pub viz_sweep_regions: u32,
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

    /// Cells a changed cell can touch through wall distances: the penalty
    /// band plus slack, the radius of the node window's BFS ball.
    pub fn node_window_cells(&self) -> i32 {
        const SLACK_CELLS: i32 = 2;
        let buffer_cells =
            ((self.wall_clearance_m + self.wall_buffer_m) / self.voxel_size).ceil() as i32;
        buffer_cells + SLACK_CELLS
    }

    /// Columns past a rewritten window the viz must reread: the node window,
    /// plus one node spacing for the edges of a relocated node.
    pub fn viz_reach_cells(&self) -> i32 {
        self.node_window_cells() + (self.node_spacing_m / self.voxel_size).ceil() as i32
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
    /// Region cylinder with its ceiling capped to `max_overhead_m` above the
    /// sensor.
    pub fn capped(
        origin_x: f32,
        origin_y: f32,
        radius: f32,
        z_min: f32,
        z_max: f32,
        sensor_z: f32,
        max_overhead_m: f32,
    ) -> Self {
        RegionBounds {
            origin_x,
            origin_y,
            radius,
            z_min,
            z_max: z_max.min(sensor_z + max_overhead_m),
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
    fn column_bbox(&self, voxel_size: f32) -> (i32, i32, i32, i32) {
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
    ) -> Option<(i32, i32, i32, i32)> {
        let pool = Arc::clone(&self.pool);
        pool.install(|| {
            let voxel_size = config.voxel_size;
            let clearance = config.headroom_cells();
            let pad = (2 * config.closing_passes()) as i32;

            // No voxel changed, so surfaces and the graph are untouched.
            let (bx0, bx1, by0, by1) =
                self.replace_region_voxels(local_points, bounds, voxel_size)?;

            // A changed column shifts surfaces only within pad of it.
            let write = (bx0 - pad, bx1 + pad, by0 - pad, by1 + pad);
            let new_cells =
                extract_surfaces_region(&self.by_col, clearance, config.closing_passes(), write);
            let (added, removed) = self.replace_surface_region(write, &new_cells);

            self.rebuild_region_graph(added, removed, config);
            Some(write)
        })
    }

    /// Patch changed cells, then repair nodes and edges around the change.
    /// A no-op when no surface cell changed.
    fn rebuild_region_graph(
        &mut self,
        added: Vec<VoxelKey>,
        removed: Vec<VoxelKey>,
        config: &Config,
    ) {
        let step = config.step_cells();
        // Removal frees cell ids and the insert loop below recycles them, so a
        // node id captured here is not stable. Capture doomed nodes by
        // coordinate while their ids still resolve.
        let removed_set: AHashSet<VoxelKey> = removed.iter().copied().collect();
        let dead_nodes: Vec<(usize, VoxelKey)> = self
            .graph
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (i, self.graph.cells.coord(n.cell_id)))
            .filter(|(_, c)| removed_set.contains(c))
            .collect();
        for &c in &removed {
            self.graph.cells.remove(c);
        }
        let mut added_ids: Vec<CellId> = Vec::with_capacity(added.len());
        for &c in &added {
            added_ids.push(self.graph.cells.insert(c));
        }
        let mut seeds = added;
        seeds.extend_from_slice(&removed);
        if seeds.is_empty() {
            return;
        }

        rebuild_edges_around(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            &seeds,
            config.voxel_size,
            step,
        );
        let params = config.placement_params();
        relocate_dead_nodes(
            &self.graph.cells,
            &self.graph.surface_lookup,
            &mut self.graph.nodes,
            &dead_nodes,
            &params,
        );
        let window = self.node_window(&seeds, config);
        place_nodes_region(
            &mut self.graph.cells,
            &self.by_col,
            &params,
            &added_ids,
            &window,
            &mut self.graph.wall_state,
            &mut self.graph.node_scratch,
            &mut self.graph.nodes,
        );
        build_node_edges_region(
            &self.graph.cells,
            &self.graph.nodes,
            &window,
            &mut self.graph.cell_state,
            &mut self.graph.node_edges,
            &mut self.graph.node_adj,
        );
    }

    /// Replace the cylinder's voxels with the local map points, ignoring
    /// points outside it. Returns the column bbox of changed voxels.
    fn replace_region_voxels(
        &mut self,
        local_points: &[(f32, f32, f32)],
        bounds: &RegionBounds,
        voxel_size: f32,
    ) -> Option<(i32, i32, i32, i32)> {
        let new_set: AHashSet<VoxelKey> = local_points
            .iter()
            .map(|&p| voxelize(p, voxel_size))
            .collect();

        let (x0, x1, y0, y1) = bounds.column_bbox(voxel_size);
        let by_col = &self.by_col;
        let stale: Vec<VoxelKey> = (x0..(x1 + 1))
            .into_par_iter()
            .flat_map_iter(|ix| {
                let mut local: Vec<VoxelKey> = Vec::new();
                for iy in y0..=y1 {
                    let Some(zs) = by_col.get(&(ix, iy)) else {
                        continue;
                    };
                    for &iz in zs {
                        let k = (ix, iy, iz);
                        if bounds.contains_voxel(k, voxel_size) && !new_set.contains(&k) {
                            local.push(k);
                        }
                    }
                }
                local
            })
            .collect();

        let mut bb = ChangeBounds::new();
        for &k in &stale {
            bb.add(k.0, k.1);
            self.voxel_map.remove(&k);
            remove_from_by_col(&mut self.by_col, k);
        }
        for &k in &new_set {
            if !bounds.contains_voxel(k, voxel_size) {
                continue;
            }
            if self.voxel_map.insert(k) {
                bb.add(k.0, k.1);
                add_to_by_col(&mut self.by_col, k);
            }
        }
        bb.bounds()
    }

    /// Replace the surface_lookup entries for write-box columns whose cells
    /// changed, leaving identical columns untouched. Returns the added and
    /// removed cells so only the affected parts of the graph get patched.
    fn replace_surface_region(
        &mut self,
        write: (i32, i32, i32, i32),
        new_cells: &[VoxelKey],
    ) -> (Vec<VoxelKey>, Vec<VoxelKey>) {
        let (x0, x1, y0, y1) = write;
        let mut new_by_col: AHashMap<(i32, i32), Vec<i32>> = AHashMap::new();
        for &(ix, iy, iz) in new_cells {
            new_by_col.entry((ix, iy)).or_default().push(iz);
        }
        for zs in new_by_col.values_mut() {
            zs.sort_unstable();
            zs.dedup();
        }

        let lookup = &self.graph.surface_lookup;
        let changed: Vec<((i32, i32), Vec<i32>)> = (x0..(x1 + 1))
            .into_par_iter()
            .flat_map_iter(|ix| {
                let mut local: Vec<((i32, i32), Vec<i32>)> = Vec::new();
                for iy in y0..=y1 {
                    let col = (ix, iy);
                    let old = lookup.get(&col).map(Vec::as_slice).unwrap_or(&[]);
                    let new = new_by_col.get(&col).map(Vec::as_slice).unwrap_or(&[]);
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

    /// Live cells within the node-graph margin of the changed cells, walked
    /// as a BFS ball over cell adjacency from roots covering everything a
    /// change can directly touch.
    fn node_window(&mut self, changed: &[VoxelKey], config: &Config) -> Vec<CellId> {
        // Wall distances only matter out to the penalty band, so the ball
        // covers the buffer reach of the changed cells plus slack.
        let steps = config.node_window_cells();
        let step_dz = config.step_cells();

        let graph = &mut self.graph;
        let lookup = &graph.surface_lookup;
        let cells = &graph.cells;
        graph.node_scratch.ensure_capacity(cells.slot_capacity());
        let seen = &mut graph.node_scratch.seen;
        let mut ball: Vec<CellId> = Vec::new();
        let mut frontier: Vec<CellId> = Vec::new();
        let mut insert = |id: CellId, ball: &mut Vec<CellId>, frontier: &mut Vec<CellId>| {
            if !seen[id as usize] {
                seen[id as usize] = true;
                ball.push(id);
                frontier.push(id);
            }
        };

        // Roots: the changed cells themselves, plus the live column neighbors
        // that carry the change when the cell itself was removed.
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
                        insert(id, &mut ball, &mut frontier);
                    }
                }
            }
        }

        // Wall-seed scans cross up to HOLE_SPAN_CELLS empty columns, so a
        // change can flip wall adjacency that far away. Root the first
        // surfaced column each direction, whole when its existence flipped.
        let headroom = config.headroom_cells();
        for &(ix, iy, iz) in changed {
            let flipped = lookup.get(&(ix, iy)).is_none_or(|zs| zs.len() <= 1);
            let (z_lo, z_hi) = (iz - headroom - step_dz, iz + step_dz);
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                for k in 1..=HOLE_SPAN_CELLS {
                    let col = (ix + dx * k, iy + dy * k);
                    let Some(zs) = lookup.get(&col) else {
                        continue;
                    };
                    for &nz in zs {
                        if !flipped && !(z_lo..=z_hi).contains(&nz) {
                            continue;
                        }
                        if let Some(id) = cells.id((col.0, col.1, nz)) {
                            insert(id, &mut ball, &mut frontier);
                        }
                    }
                    break;
                }
            }
        }

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
            &mut self.graph.node_adj,
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
            .install(|| edges_to_segments(&self.graph.node_edges))
    }

    /// The same segments without materializing them.
    pub fn edge_segment_iter(&self) -> impl Iterator<Item = (VoxelKey, VoxelKey, f32)> + '_ {
        self.graph.node_edges.iter().flat_map(|edge| {
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

/// Running inclusive xy bounding box of changed columns.
struct ChangeBounds {
    min_x: i32,
    max_x: i32,
    min_y: i32,
    max_y: i32,
    any: bool,
}

impl ChangeBounds {
    fn new() -> Self {
        Self {
            min_x: i32::MAX,
            max_x: i32::MIN,
            min_y: i32::MAX,
            max_y: i32::MIN,
            any: false,
        }
    }

    fn add(&mut self, ix: i32, iy: i32) {
        self.any = true;
        self.min_x = self.min_x.min(ix);
        self.max_x = self.max_x.max(ix);
        self.min_y = self.min_y.min(iy);
        self.max_y = self.max_y.max(iy);
    }

    fn bounds(&self) -> Option<(i32, i32, i32, i32)> {
        self.any
            .then_some((self.min_x, self.max_x, self.min_y, self.max_y))
    }
}

#[cfg(test)]
mod region_tests;
