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

"""Stock G1 velocity control and navigation in the shared MuJoCo world.

Run with ``dimos --simulation mujoco --viewer none run unitree-g1-basic-sim``.
The same G1Connection is used by the stock hardware blueprint.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic import unitree_g1_basic


def _require_simulation() -> str | None:
    if global_config.simulation != "mujoco":
        return "Select --simulation mujoco before loading the stock G1 simulation blueprint."
    return None


unitree_g1_basic_sim = autoconnect(
    unitree_g1_basic,
    ReplanningAStarPlanner.blueprint(),
    MovementManager.blueprint(),
).requirements(_require_simulation)
