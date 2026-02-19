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

"""Smart blueprint for Deep Robotics M20: connection + SLAM + navigation.

This is the PoC target blueprint — M20 with full autonomous navigation:
    M20Connection → VoxelGridMapper → CostMapper → A* Planner → Frontier Explorer

Reference: M20 Software Development Guide (nell/relay/docs/software-development-guide.md)
"""

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.deeprobotics.m20.blueprints.basic.m20_minimal import m20_minimal

m20_smart = autoconnect(
    m20_minimal,
    voxel_mapper(voxel_size=0.1),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(
    n_dask_workers=6,
    robot_model="deeprobotics_m20",
    robot_width=0.3,
    robot_rotation_diameter=0.6,
)

__all__ = ["m20_smart"]
