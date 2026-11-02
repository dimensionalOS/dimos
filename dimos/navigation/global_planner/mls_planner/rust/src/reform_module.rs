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

//! The MLS planner that follows a pose-graph optimizer's loop closures: on
//! each `loop_closure_event` it warps its whole graph, goal included, into
//! the corrected frame in place instead of waiting for a corrected map.

use std::time::Duration;

use dimos_module::{warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::geometry_msgs::{PointStamped, PoseStamped};
use lcm_msgs::nav_msgs::Path;
use lcm_msgs::sensor_msgs::PointCloud2;
use tracing::debug;
use validator::ValidationError;

use crate::mls_planner::Config;
use crate::module::{Intake, WorkerOutputs, WorkerTasks};
use crate::reform::{decode_loop_closure_event, decode_pose_graph, ReformParams, ReformState};

/// The MLS planner's config plus how closures warp it.
#[dimos_module::native_config]
#[derive(Clone)]
#[validate(schema(function = "validate_wall_buffer"))]
pub struct ReformConfig {
    pub world_frame: String,
    pub base_frame: String,
    #[validate(range(exclusive_min = 0.0))]
    pub voxel_size: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub robot_height: f32,
    #[validate(range(min = 0.0))]
    pub start_z_offset_m: f32,
    #[validate(range(min = 0.0))]
    pub max_overhead_m: f32,
    #[validate(range(min = 0.0))]
    pub surface_closing_radius: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub node_spacing_m: f32,
    #[validate(range(min = 0.0))]
    pub wall_clearance_m: f32,
    #[validate(range(min = 0.0))]
    pub wall_buffer_m: f32,
    #[validate(range(min = 0.0))]
    pub wall_buffer_weight: f32,
    #[validate(range(min = 0.0))]
    pub step_threshold_m: f32,
    #[validate(range(min = 0.0))]
    pub step_penalty_weight: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub goal_tolerance: f32,
    #[validate(range(min = 0.0))]
    pub viz_publish_hz: f32,
    #[validate(range(exclusive_min = 0.0))]
    pub viz_cell_m: f32,
    pub viz_sweep_cells: u32,
    #[validate(range(min = 1))]
    pub worker_threads: u32,
    /// Spatial width of the blend between keyframe corrections, m. 0 picks
    /// twice the mean keyframe spacing.
    #[validate(range(min = 0.0))]
    pub reform_blend_sigma_m: f64,
    /// Temporal width of the blend, s, which keeps the passes of a revisited
    /// place apart. 0 picks twice the mean keyframe interval.
    #[validate(range(min = 0.0))]
    pub reform_blend_time_sigma_s: f64,
    /// A keyframe whose correction moves its surroundings less than this
    /// waits for a later closure, so near no-op closures cost nothing.
    #[validate(range(min = 0.0))]
    pub reform_min_shift_m: f64,
    /// Past this fraction of the voxels moving in one closure, the graph is
    /// rebuilt from the warped voxels instead of repaired around them.
    #[validate(range(min = 0.0))]
    pub reform_rebuild_fraction: f64,
}

fn validate_wall_buffer(config: &ReformConfig) -> Result<(), ValidationError> {
    if config.wall_buffer_weight > 0.0 && config.wall_buffer_m == 0.0 {
        return Err(ValidationError::new(
            "wall_buffer_weight requires wall_buffer_m > 0",
        ));
    }
    Ok(())
}

impl ReformConfig {
    pub fn planner(&self) -> Config {
        Config {
            world_frame: self.world_frame.clone(),
            base_frame: self.base_frame.clone(),
            voxel_size: self.voxel_size,
            robot_height: self.robot_height,
            start_z_offset_m: self.start_z_offset_m,
            max_overhead_m: self.max_overhead_m,
            surface_closing_radius: self.surface_closing_radius,
            node_spacing_m: self.node_spacing_m,
            wall_clearance_m: self.wall_clearance_m,
            wall_buffer_m: self.wall_buffer_m,
            wall_buffer_weight: self.wall_buffer_weight,
            step_threshold_m: self.step_threshold_m,
            step_penalty_weight: self.step_penalty_weight,
            goal_tolerance: self.goal_tolerance,
            viz_publish_hz: self.viz_publish_hz,
            viz_cell_m: self.viz_cell_m,
            viz_sweep_cells: self.viz_sweep_cells,
            worker_threads: self.worker_threads,
        }
    }

    pub fn reform(&self) -> ReformParams {
        ReformParams {
            blend_sigma_m: self.reform_blend_sigma_m,
            blend_time_sigma_s: self.reform_blend_time_sigma_s,
            min_shift_m: self.reform_min_shift_m,
            rebuild_fraction: self.reform_rebuild_fraction,
        }
    }
}

#[derive(Module)]
#[module(name = "mls_reform_planner", setup = spawn_worker, teardown = stop_worker)]
pub struct MlsReformPlanner {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = PointCloud2::decode, handler = on_local_map)]
    local_map: Input<PointCloud2>,

    #[input(decode = PoseStamped::decode, handler = on_region_bounds)]
    region_bounds: Input<PoseStamped>,

    #[input(decode = PointCloud2::decode, handler = on_seed_map)]
    seed_map: Input<PointCloud2>,

    #[input(decode = PoseStamped::decode, handler = on_seed_bounds)]
    seed_bounds: Input<PoseStamped>,

    #[input(decode = PointStamped::decode, handler = on_goal)]
    goal: Input<PointStamped>,

    // Every keyframe's pose, for the anchors that hold the unmoved parts.
    #[input(decode = PointCloud2::decode, handler = on_pose_graph)]
    pose_graph: Input<PointCloud2>,

    // Per closure, each keyframe it moved: its pre pose and the delta.
    #[input(decode = PointCloud2::decode, handler = on_loop_closure_event)]
    loop_closure_event: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    surface_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    nodes: Output<PointCloud2>,

    #[output(encode = Path::encode, msg = "LineSegments3D")]
    node_edges: Output<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    #[config]
    config: ReformConfig,

    intake: Intake,
    tasks: WorkerTasks,
    reform: ReformState,
}

impl MlsReformPlanner {
    async fn spawn_worker(&mut self) {
        let outputs = WorkerOutputs {
            surface_map: self.surface_map.clone(),
            nodes: self.nodes.clone(),
            node_edges: self.node_edges.clone(),
            path: self.path.clone(),
        };
        self.tasks = self
            .intake
            .spawn(self.tf.clone(), self.config.planner(), outputs, true);
    }

    async fn stop_worker(&mut self) {
        self.tasks.stop();
    }

    async fn on_global_map(&mut self, msg: PointCloud2) {
        self.intake.global_map(msg);
    }

    async fn on_local_map(&mut self, msg: PointCloud2) {
        self.intake.local_map(msg);
    }

    async fn on_region_bounds(&mut self, msg: PoseStamped) {
        self.intake.region_bounds(msg);
    }

    async fn on_seed_map(&mut self, msg: PointCloud2) {
        self.intake.seed_map(msg);
    }

    async fn on_seed_bounds(&mut self, msg: PoseStamped) {
        self.intake.seed_bounds(msg);
    }

    async fn on_goal(&mut self, msg: PointStamped) {
        self.intake.goal(msg);
    }

    async fn on_pose_graph(&mut self, msg: PointCloud2) {
        match decode_pose_graph(&msg) {
            Ok(records) => self.reform.add_keyframes(&records),
            Err(e) => warn_throttled!(
                Duration::from_secs(1),
                error = %e,
                "Failed to decode pose_graph, dropped it."
            ),
        }
    }

    async fn on_loop_closure_event(&mut self, msg: PointCloud2) {
        let moved = match decode_loop_closure_event(&msg) {
            Ok(moved) => moved,
            Err(e) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %e,
                    "Failed to decode loop_closure_event, dropped it."
                );
                return;
            }
        };
        match self.reform.closure(&moved, &self.config.reform()) {
            Some(warp) => self.intake.queue_warp(warp),
            None => debug!(
                keyframes = moved.len(),
                "Loop closure moved nothing past reform_min_shift_m, held for the next."
            ),
        }
    }
}
