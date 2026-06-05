// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! Config and the owned-state Planner that builds and queries the MLS graph.

use std::time::Instant;

use ahash::AHashSet;
use serde::Deserialize;
use validator::Validate;

use crate::adjacency::{build_surface_cells, build_surface_lookup};
use crate::edges::{build_node_edges, PlannerGraph};
use crate::nodes::place_nodes;
use crate::planner;
use crate::surfaces::{extract_surfaces, ColumnIz};
use crate::voxel::{voxelize, VoxelKey};

#[derive(Debug, Deserialize, Validate)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub world_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub robot_height: f32,
    #[validate(range(min = 0))]
    pub surface_dilation_passes: u32,
    #[validate(range(min = 0))]
    pub surface_erosion_passes: u32,
    #[validate(range(exclusive_min = 0.0))]
    pub node_spacing_m: f32,
    #[validate(range(min = 0.0))]
    pub node_wall_buffer_m: f32,
    #[validate(range(min = 0.0))]
    pub node_step_threshold_m: f32,
}

/// Wall-clock cost of each stage of the most recent full rebuild, in milliseconds.
#[derive(Default, Clone, Copy)]
pub struct RebuildTimings {
    pub voxelize_ms: f64,
    pub surfaces_ms: f64,
    pub graph_ms: f64,
}

#[derive(Default)]
pub struct Planner {
    graph: PlannerGraph,
    voxel_map: AHashSet<VoxelKey>,
    by_col: ColumnIz,
    surface: Vec<VoxelKey>,
    last_timings: RebuildTimings,
}

impl Planner {
    pub fn update_global_map(&mut self, points: &[(f32, f32, f32)], config: &Config) {
        let voxel_size = config.voxel_size;
        let clearance = (config.robot_height / voxel_size).ceil() as i32;
        let step = (config.node_step_threshold_m / voxel_size).floor() as i32;

        let t_voxelize = Instant::now();
        self.voxel_map.clear();
        for &p in points {
            self.voxel_map.insert(voxelize(p, voxel_size));
        }
        let voxelize_ms = ms(t_voxelize.elapsed());

        let t_surfaces = Instant::now();
        extract_surfaces(
            &self.voxel_map,
            clearance,
            config.surface_dilation_passes,
            config.surface_erosion_passes,
            &mut self.by_col,
            &mut self.surface,
        );
        let surfaces_ms = ms(t_surfaces.elapsed());

        let t_graph = Instant::now();
        build_surface_lookup(&self.surface, &mut self.graph.surface_lookup);
        build_surface_cells(
            &mut self.graph.cells,
            &self.graph.surface_lookup,
            voxel_size,
            step,
        );

        place_nodes(
            &mut self.graph.cells,
            voxel_size,
            config.node_spacing_m,
            config.node_wall_buffer_m,
            &mut self.graph.cell_state,
            &mut self.graph.nodes,
        );

        build_node_edges(
            &self.graph.cells,
            &self.graph.nodes,
            &mut self.graph.cell_state,
            &mut self.graph.node_edges,
            &mut self.graph.node_adj,
        );
        let graph_ms = ms(t_graph.elapsed());

        self.last_timings = RebuildTimings {
            voxelize_ms,
            surfaces_ms,
            graph_ms,
        };
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
        planner::plan(
            &self.graph,
            start,
            goal,
            config.voxel_size,
            config.robot_height,
            config.node_spacing_m,
            config.node_step_threshold_m,
        )
    }

    pub fn graph(&self) -> &PlannerGraph {
        &self.graph
    }

    pub fn surface(&self) -> &[VoxelKey] {
        &self.surface
    }

    pub fn voxel_count(&self) -> usize {
        self.voxel_map.len()
    }

    pub fn last_timings(&self) -> RebuildTimings {
        self.last_timings
    }
}

fn ms(d: std::time::Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}
