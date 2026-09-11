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

"""Simple G1 nav stack: onboard sensors, raytracing costmap, and A* replanning."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_onboard import _unitree_g1_onboard
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_vis import unitree_g1_vis
from dimos.robot.unitree.g1.config import G1

assert G1.height_clearance is not None and G1.width_clearance is not None

g1_overhead_safety_margin = 0.2
g1_overhead_clearance = G1.height_clearance + g1_overhead_safety_margin
g1_max_step_height = 0.10
g1_rotation_diameter = 0.8
voxel_resolution = 0.05
g1_safe_radius_margin = 0.6

unitree_g1_nav_simple = autoconnect(
    _unitree_g1_onboard,
    RayTracingVoxelMap.blueprint(voxel_size=voxel_resolution),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_resolution,
            can_pass_under=g1_overhead_clearance,
            can_climb=g1_max_step_height,
        ),
        initial_safe_radius_meters=G1.width_clearance + g1_safe_radius_margin,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=G1.width_clearance,
        robot_rotation_diameter=g1_rotation_diameter,
    ),
    MovementManager.blueprint(),
    unitree_g1_vis,
).global_config(n_workers=10, robot_model="unitree_g1")
