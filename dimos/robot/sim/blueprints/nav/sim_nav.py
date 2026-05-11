# Copyright 2026 Dimensional Inc.
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

"""DimSim navigation blueprint — basic + mapping + planning + exploration."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import (
    HeightCostConfig,
)
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.sim.blueprints.basic.sim_basic import sim_basic

sim_nav = autoconnect(
    sim_basic,
    VoxelGridMapper.blueprint(voxel_size=0.1),
    CostMapper.blueprint(
        algo="height_cost", config=HeightCostConfig(can_pass_under=1.5, smoothing=2.0)
    ),
    ReplanningAStarPlanner.blueprint(),
    WavefrontFrontierExplorer.blueprint(),
).global_config(n_workers=6, robot_model="dimsim")

__all__ = ["sim_nav"]
