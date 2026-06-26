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

"""Go2 nav stack with holonomic trajectory control and hot replanning.

Uses ``DannavPlanner`` with ``replan_on_costmap_update=True``. For the default
differential stack see ``unitree_go2``.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import GeneralOccupancyConfig
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.dannav.module import DannavPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2_dannav = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(emit_every=1),
    CostMapper.blueprint(
        algo="general",
        config=GeneralOccupancyConfig(
            resolution=0.05,
            min_height=0.08,
            max_height=2.0,
            mark_free_radius=0.2,
        ),
    ),
    DannavPlanner.blueprint(
        replan_on_costmap_update=True,
    ),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2", robot_width=0.25)

__all__ = ["unitree_go2_dannav"]
