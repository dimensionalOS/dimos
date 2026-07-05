// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

//! wasm-bindgen bindings for the interactive browser demo: the JS side feeds
//! synthetic obstacle points + a goal, gets back the costmap raster and the
//! planned path each frame.

use wasm_bindgen::prelude::*;

use crate::costmap::{self, CostmapConfig};
use crate::solver::{self, SolverConfig};

#[wasm_bindgen]
pub struct DemoPlanner {
    costmap_cfg: CostmapConfig,
    solver_cfg: SolverConfig,
    map: Option<costmap::Costmap>,
    prev: Option<Vec<(f32, f32)>>,
}

#[wasm_bindgen]
impl DemoPlanner {
    #[wasm_bindgen(constructor)]
    pub fn new(resolution: f32, half_extent: f32) -> DemoPlanner {
        DemoPlanner {
            costmap_cfg: CostmapConfig {
                resolution,
                half_extent,
                ..CostmapConfig::default()
            },
            solver_cfg: SolverConfig::default(),
            map: None,
            prev: None,
        }
    }

    /// Rebuild the costmap from flat [x0,y0,z0, x1,y1,z1, ...] points around
    /// the robot.
    pub fn update_terrain(&mut self, points: &[f32], robot_x: f32, robot_y: f32, robot_z: f32) {
        let pts: Vec<[f32; 3]> = points
            .chunks_exact(3)
            .map(|c| [c[0], c[1], c[2]])
            .collect();
        self.map = Some(costmap::build(
            &pts,
            (robot_x, robot_y, robot_z),
            robot_z,
            &self.costmap_cfg,
        ));
    }

    /// Plan toward the goal; returns flat [x0,y0,yaw0, x1,y1,yaw1, ...].
    pub fn plan(
        &mut self,
        robot_x: f32,
        robot_y: f32,
        robot_yaw: f32,
        goal_x: f32,
        goal_y: f32,
        speed: f32,
    ) -> Vec<f32> {
        let Some(map) = self.map.as_ref() else {
            return Vec::new();
        };
        let route = vec![(robot_x, robot_y), (goal_x, goal_y)];
        let plan = solver::plan(
            map,
            &route,
            &[],
            (robot_x, robot_y, robot_yaw),
            speed,
            self.prev.as_deref(),
            &self.solver_cfg,
        );
        if plan.poses.len() >= 2 {
            self.prev = Some(plan.poses.iter().map(|p| (p.0, p.1)).collect());
        }
        plan.poses.iter().flat_map(|p| [p.0, p.1, p.2]).collect()
    }

    /// Costmap raster (i8 costs, row-major) + metadata for rendering.
    pub fn costmap_cells(&self) -> Vec<i8> {
        self.map.as_ref().map(|m| m.cost.clone()).unwrap_or_default()
    }
    pub fn costmap_width(&self) -> usize {
        self.map.as_ref().map(|m| m.width).unwrap_or(0)
    }
    pub fn costmap_height(&self) -> usize {
        self.map.as_ref().map(|m| m.height).unwrap_or(0)
    }
    pub fn costmap_origin_x(&self) -> f32 {
        self.map.as_ref().map(|m| m.origin.0).unwrap_or(0.0)
    }
    pub fn costmap_origin_y(&self) -> f32 {
        self.map.as_ref().map(|m| m.origin.1).unwrap_or(0.0)
    }
    pub fn costmap_resolution(&self) -> f32 {
        self.map.as_ref().map(|m| m.resolution).unwrap_or(0.0)
    }
}
