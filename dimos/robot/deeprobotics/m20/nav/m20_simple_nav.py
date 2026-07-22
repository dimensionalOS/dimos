#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Simple M20 navigation stack.

This blueprint consumes the M20 onboard SLAM/LIO outputs that are bridged into
DimOS as ``slam_aligned_points`` and ``slam_odom``. It does not subscribe to the
front/rear raw lidar topics directly.
"""

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.obstacle_avoidance.module import (
    ObstacleAvoidance,
    ObstacleAvoidanceConfig,
)
from dimos.navigation.replanning_a_star.module import (
    ReplanningAStarPlanner,
    ReplanningAStarPlannerConfig,
)
from dimos.robot.deeprobotics.m20.blueprints.basic import m20, rerun
from dimos.robot.deeprobotics.m20.rrd_replay import M20RrdReplay
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.utils.config import load_config_mapping

voxel_size = 0.05
m20_width_clearance = 0.45
# SLAM odometry is the robot head pose. The head is about 1.3 m above the
# bottom, so overhead clearance must reflect the full bottom-to-head height.
m20_height_clearance = 0.8
m20_overhead_safety_margin = 0.2
m20_overhead_clearance = m20_height_clearance + m20_overhead_safety_margin
m20_max_step_height = 0.15
m20_rotation_diameter = 1.2
m20_safe_radius_margin = 0.1
map_save_dir = Path(__file__).resolve().parent / "map_save"
map_save_path = map_save_dir / "m20_accumulated_map.pcd"
M20_SIMPLE_NAV_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config/m20_simple_nav.yaml"


def _load_m20_simple_nav_config() -> tuple[dict[str, Any], dict[str, Any]]:
    payload = load_config_mapping(M20_SIMPLE_NAV_CONFIG_PATH)
    try:
        planner_values = payload["replanningastarplanner"]
        obstacle_avoidance_values = payload["obstacleavoidance"]
    except KeyError:
        raise ValueError(
            f"Config file {M20_SIMPLE_NAV_CONFIG_PATH} must define "
            "replanningastarplanner and obstacleavoidance"
        ) from None

    planner_config = ReplanningAStarPlannerConfig.model_validate(planner_values)
    obstacle_avoidance_config = ObstacleAvoidanceConfig.model_validate(obstacle_avoidance_values)
    return (
        planner_config.model_dump(include=set(planner_values)),
        obstacle_avoidance_config.model_dump(include=set(obstacle_avoidance_values)),
    )


M20_SIMPLE_NAV_PLANNER_CONFIG, M20_SIMPLE_NAV_OBSTACLE_AVOIDANCE_CONFIG = (
    _load_m20_simple_nav_config()
)

_m20_slam_ray_tracer = RayTracingVoxelMap.blueprint(
    executable="target/release/voxel_ray_tracing",
    build_command="cargo build --release --bin voxel_ray_tracing",
    voxel_size=voxel_size,
    max_range=8.0,
    shadow_depth=0.1,
    min_health=-1,
    max_health=10,
    emit_every=2,
    ray_subsample=1,
    global_emit_every=1,
    auto_build=True,
    support_min=0,
    # M20's onboard SLAM publishes this cloud already registered in map frame.
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "slam_odom"),
    ]
)

# _m20_pointcloud_map_save = PointCloudMapSave.blueprint(
#     translation_threshold_m=0.5,
#     rotation_threshold_rad=math.radians(15.0),
#     voxel_size=voxel_size,
#     save_path=str(map_save_path),
# ).remappings(
#     [
#         (PointCloudMapSave, "lidar", "slam_aligned_points"),
#         (PointCloudMapSave, "odometry", "slam_odom"),
#         (PointCloudMapSave, "global_map", "dimos/m20_saved_pointcloud_map"),
#     ]
# )

if global_config.replay:
    m20_source = autoconnect(
        rerun,
        M20RrdReplay.blueprint(),
        M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
    )
else:
    m20_source = m20

m20_simple_nav = autoconnect(
    m20_source,
    _m20_slam_ray_tracer,
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=m20_overhead_clearance,
            can_climb=m20_max_step_height,
            ignore_noise=0.08,
            smoothing=1.5,
            min_gradient_neighbors=2,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=m20_width_clearance,
        robot_rotation_diameter=m20_rotation_diameter,
        **M20_SIMPLE_NAV_PLANNER_CONFIG,
    ).remappings(
        [
            (ReplanningAStarPlanner, "odometry", "slam_odom"),
            (ReplanningAStarPlanner, "nav_cmd_vel", "raw_nav_cmd_vel"),
        ]
    ),
    ObstacleAvoidance.blueprint(**M20_SIMPLE_NAV_OBSTACLE_AVOIDANCE_CONFIG).remappings(
        [(ObstacleAvoidance, "odometry", "slam_odom")]
    ),
    MovementManager.blueprint(),
).global_config(
    n_workers=11,
    robot_model="m20",
    robot_ip="10.21.31.103",
    robot_width=m20_width_clearance,
    robot_rotation_diameter=m20_rotation_diameter,
)
