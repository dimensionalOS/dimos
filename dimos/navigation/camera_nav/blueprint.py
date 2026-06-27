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

"""Camera-only navigation pipeline (monocular).

Architecture
------------
    color_image + camera_info
        → MonocularDepthModule        — DepthAnythingV2 metric depth (Z in metres)
        → DepthAccumulatorModule      — ICP SLAM, pose-graph, persistent 3D map
        → CostMapper                  — project 3D map → 2D occupancy grid
        → WavefrontFrontierExplorer   — pick next unexplored frontier goal
        → ReplanningAStarPlanner      — A* path from odom to goal, replan on change
        → MovementManager             — issue velocity commands to robot

Accuracy note — monocular depth
--------------------------------
DepthAnythingV2-Metric-Indoor depth error is approximately 8–12 cm at 1–3 m and
degrades beyond 5 m. This is sufficient for obstacle avoidance and room-scale
exploration. For centimetre-level precision use ZED stereo (< 2 cm error at 5 m)
via ``camera_nav_zed_stack`` in blueprint_zed.py.

Usage
-----
Compose on top of any robot blueprint that exposes ``color_image``, ``camera_info``,
and odometry TF::

    from dimos.navigation.camera_nav.blueprint import camera_nav_stack
    my_robot_camera_nav = autoconnect(my_robot_blueprint, camera_nav_stack)

For validation without a robot (webcam smoke-test)::

    python -m dimos.navigation.camera_nav.validate
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.monocular_depth_module import MonocularDepthModule

camera_nav_stack = autoconnect(
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    CostMapper.blueprint(algo="height_cost"),
    WavefrontFrontierExplorer.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
)
