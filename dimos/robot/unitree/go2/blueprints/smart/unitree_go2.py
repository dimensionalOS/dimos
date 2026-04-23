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

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.smart_nav.modules.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=7, robot_model="unitree_go2")

__all__ = ["unitree_go2"]
