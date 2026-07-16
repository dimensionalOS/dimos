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

"""Basic M20 navigation with a fresh, in-memory map on every startup.

The onboard M20 bridge publishes ``slam_aligned_points`` in the map frame and
``slam_odom`` as the robot pose in that same frame. This blueprint accumulates
the already-registered point clouds directly, following the Go2 navigation
pipeline without exploration, patrolling, or map persistence.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.deeprobotics.m20.blueprints.basic import m20, rerun
from dimos.robot.deeprobotics.m20.rrd_replay import M20RrdReplay
from dimos.robot.deeprobotics.m20.tf import M20TF

voxel_size = 0.05
m20_width_clearance = 0.45
m20_height_clearance = 0.6
m20_overhead_safety_margin = 0.1
m20_max_step_height = 0.15
m20_rotation_diameter = 1.2
m20_safe_radius_margin = 0.1

if global_config.replay:
    m20_source = autoconnect(
        rerun,
        M20RrdReplay.blueprint(),
        M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
    )
else:
    m20_source = m20

m20_simple_nav2 = autoconnect(
    m20_source,
    VoxelGridMapper.blueprint(
        voxel_size=voxel_size,
        emit_every=5,
        frame_id="map",
        carve_columns=True,
    ).remappings([(VoxelGridMapper, "lidar", "slam_aligned_points")]),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=m20_height_clearance + m20_overhead_safety_margin,
            can_climb=m20_max_step_height,
            ignore_noise=0.1,
            smoothing=1.5,
            min_gradient_neighbors=4,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=m20_width_clearance,
        robot_rotation_diameter=m20_rotation_diameter,
    ).remappings([(ReplanningAStarPlanner, "odometry", "slam_odom")]),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="m20", robot_ip="10.21.31.103")
