// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

use dimos_module::{run, Input, LcmTransport, Module, Output};
use lcm_msgs::nav_msgs::{Odometry, Path};
use lcm_msgs::sensor_msgs::PointCloud2;
use serde::Deserialize;
use tracing::info;

#[allow(dead_code)] // fields populated incrementally as algorithm stages land
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Config {
    world_frame: String,
    voxel_size: f32,
    robot_height: f32,
    surface_dilation_passes: u32,
    surface_erosion_passes: u32,
    node_spacing_m: f32,
    node_wall_buffer_m: f32,
    node_step_threshold_m: f32,
}

#[allow(dead_code)] // outputs wired up incrementally as algorithm stages land
#[derive(Module)]
#[module(setup = setup)]
struct MlsPlanner {
    #[input(decode = PointCloud2::decode, handler = on_global_map)]
    global_map: Input<PointCloud2>,

    #[input(decode = Odometry::decode, handler = on_start_pose)]
    start_pose: Input<Odometry>,

    #[input(decode = Odometry::decode, handler = on_goal_pose)]
    goal_pose: Input<Odometry>,

    #[output(encode = PointCloud2::encode)]
    surface_map: Output<PointCloud2>,

    #[output(encode = PointCloud2::encode)]
    nodes: Output<PointCloud2>,

    #[output(encode = Path::encode)]
    node_edges: Output<Path>,

    #[output(encode = Path::encode)]
    path: Output<Path>,

    #[config]
    config: Config,

    height_cells: i32,
    step_cells: i32,
}

impl MlsPlanner {
    async fn setup(&mut self) {
        let cfg = &self.config;
        if !cfg.voxel_size.is_finite() || cfg.voxel_size <= 0.0 {
            panic!(
                "mls_planner: voxel_size must be > 0, got {}",
                cfg.voxel_size
            );
        }
        if !cfg.robot_height.is_finite() || cfg.robot_height <= 0.0 {
            panic!(
                "mls_planner: robot_height must be > 0, got {}",
                cfg.robot_height
            );
        }
        if !cfg.node_spacing_m.is_finite() || cfg.node_spacing_m <= 0.0 {
            panic!(
                "mls_planner: node_spacing_m must be > 0, got {}",
                cfg.node_spacing_m
            );
        }
        if !cfg.node_wall_buffer_m.is_finite() || cfg.node_wall_buffer_m < 0.0 {
            panic!(
                "mls_planner: node_wall_buffer_m must be >= 0, got {}",
                cfg.node_wall_buffer_m
            );
        }
        if !cfg.node_step_threshold_m.is_finite() || cfg.node_step_threshold_m < 0.0 {
            panic!(
                "mls_planner: node_step_threshold_m must be >= 0, got {}",
                cfg.node_step_threshold_m
            );
        }

        self.height_cells = (cfg.robot_height / cfg.voxel_size).ceil() as i32;
        self.step_cells = (cfg.node_step_threshold_m / cfg.voxel_size).floor() as i32;

        info!(
            world_frame = %cfg.world_frame,
            voxel_size = cfg.voxel_size,
            robot_height = cfg.robot_height,
            height_cells = self.height_cells,
            step_cells = self.step_cells,
            "mls_planner ready",
        );
    }

    async fn on_global_map(&mut self, msg: PointCloud2) {
        let n = (msg.width as usize) * (msg.height as usize);
        info!(points = n, "global_map stub: not yet implemented");
    }

    async fn on_start_pose(&mut self, msg: Odometry) {
        let p = &msg.pose.pose.position;
        info!(
            x = p.x,
            y = p.y,
            z = p.z,
            "start_pose stub: not yet implemented"
        );
    }

    async fn on_goal_pose(&mut self, msg: Odometry) {
        let p = &msg.pose.pose.position;
        info!(
            x = p.x,
            y = p.y,
            z = p.z,
            "goal_pose stub: not yet implemented"
        );
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("failed to create LCM transport");
    run::<MlsPlanner, _>(transport)
        .await
        .expect("mls_planner run failed");
}
