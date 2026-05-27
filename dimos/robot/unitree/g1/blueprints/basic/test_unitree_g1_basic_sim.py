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

from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_basic_sim import unitree_g1_basic_sim
from dimos.robot.unitree.g1.mujoco_sim import G1SimConnection


def test_unitree_g1_basic_sim_routes_navigation_and_teleop_to_cmd_vel() -> None:
    modules = {atom.module for atom in unitree_g1_basic_sim.blueprints}

    assert G1SimConnection in modules
    assert ReplanningAStarPlanner in modules
    assert MovementManager in modules
    assert (
        unitree_g1_basic_sim.remapping_map[(MovementManager, "way_point")]
        == "_mgr_way_point_unused"
    )

    movement_manager = next(
        atom for atom in unitree_g1_basic_sim.blueprints if atom.module is MovementManager
    )
    streams = {(stream.name, stream.direction) for stream in movement_manager.streams}

    assert ("tele_cmd_vel", "in") in streams
    assert ("nav_cmd_vel", "in") in streams
    assert ("cmd_vel", "out") in streams
