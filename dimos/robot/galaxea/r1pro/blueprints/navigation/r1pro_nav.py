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

"""R1 Pro click-to-drive navigation: raytracing costmap + A* replanning.

Mirrors ``unitree_g1_nav_simple``, fed by the connection's chassis lidar
and wheel odometry. Wheel odometry drifts with slip — good enough for
room-scale click-to-drive; swap in a LIO source for anything larger.

Usage:
    dimos run r1pro-nav        # click a goal in the rerun viewer
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_coordinator

# First-pass R1 Pro clearances — tune on the robot.
_r1pro_width = 0.65  # chassis width + margin (m)
_r1pro_rotation_diameter = 0.9  # in-place rotation footprint (m)
_r1pro_overhead_clearance = 1.9  # torso-up height + margin (m)
_r1pro_max_step_height = 0.03  # wheeled base: essentially flat floors only (m)
_voxel_resolution = 0.05

r1pro_nav = autoconnect(
    r1pro_coordinator,
    RayTracingVoxelMap.blueprint(voxel_size=_voxel_resolution),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=_voxel_resolution,
            can_pass_under=_r1pro_overhead_clearance,
            can_climb=_r1pro_max_step_height,
        ),
        initial_safe_radius_meters=_r1pro_width + 0.4,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=_r1pro_width,
        robot_rotation_diameter=_r1pro_rotation_diameter,
    ),
    MovementManager.blueprint(),
).global_config(n_workers=8)

__all__ = ["r1pro_nav"]
