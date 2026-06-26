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

"""Camera-only navigation pipeline.

Replaces LiDAR with monocular depth estimation + frame accumulation.
Compose on top of any robot blueprint that exposes ``color_image`` and
robot odometry TF::

    from dimos.navigation.camera_nav.blueprint import camera_nav_stack

    my_robot_camera_nav = autoconnect(
        my_robot_blueprint,      # provides color_image, odom TF
        camera_nav_stack,        # depth → accumulate → costmap → plan → move
    )

For the full agentic loop also add ``WavefrontFrontierExplorer``,
``NavigationSkillContainer``, and ``McpServer``.

Validation (webcam smoke-test, no robot needed)::

    python -m dimos.navigation.camera_nav.validate
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.depth.accumulator import DepthAccumulatorModule
from dimos.perception.depth.monocular_depth_module import MonocularDepthModule

camera_nav_stack = autoconnect(
    MonocularDepthModule.blueprint(),
    DepthAccumulatorModule.blueprint(),
    CostMapper.blueprint(algo="height_cost"),
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
)
