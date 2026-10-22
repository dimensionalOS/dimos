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

"""G1 simulation using GR00T control, shared navigation and frontier exploration."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.experimental.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.unitree.g1.blueprints.navigation._g1_navigation import g1_navigation
from dimos.robot.unitree.g1.controllers.groot.blueprint import groot_control
from dimos.robot.unitree.g1.visualization.rerun import g1_viewer

_simulated = global_config.simulation == "mujoco"

unitree_g1_basic_sim = autoconnect(
    groot_control(simulated=_simulated),
    g1_navigation(simulated=_simulated),
    g1_viewer(global_config.viewer, simulated=_simulated, joint_topic="g1_joints"),
    WavefrontFrontierExplorer.blueprint(),
)
