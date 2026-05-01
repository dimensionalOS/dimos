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

from pathlib import Path

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels import VoxelGridMapper
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.nav_stack.modules.movement_manager.movement_manager import MovementManager
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

unitree_go2 = autoconnect(
    unitree_go2_basic,
    VoxelGridMapper.blueprint(),
    CostMapper.blueprint(),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
    PatrollingModule.blueprint(),
    MovementManager.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")


class Go2MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_go2.db"


class Go2Memory(Recorder):
    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    config: Go2MemoryConfig


unitree_go2_memory = autoconnect(
    unitree_go2,
    Go2Memory.blueprint(),
).global_config(n_workers=11)

__all__ = ["unitree_go2", "unitree_go2_memory"]
