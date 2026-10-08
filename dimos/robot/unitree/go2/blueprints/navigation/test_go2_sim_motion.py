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

"""The simulator runs the stack the robot runs, unmodified."""

from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.navigation.go2_motion_stack import _go2_motion_stack
from dimos.robot.unitree.go2.blueprints.navigation.go2_sim_motion import go2_sim_motion
from dimos.robot.unitree.go2.dds.blueprints import go2_dds_nav


def _atoms(blueprint: Blueprint) -> dict[str, BlueprintAtom]:
    return {atom.name: atom for atom in blueprint.active_blueprints}


def _remappings(blueprint: Blueprint, name: str) -> dict[tuple[str, str], object]:
    return {key: target for key, target in blueprint.remapping_map.items() if key[0] == name}


def test_sim_composes_the_stack_the_robot_runs() -> None:
    sim, robot = _atoms(go2_sim_motion), _atoms(go2_dds_nav)
    for name in _atoms(_go2_motion_stack):
        assert (sim[name].module, sim[name].kwargs) == (robot[name].module, robot[name].kwargs)
        assert _remappings(go2_sim_motion, name) == _remappings(go2_dds_nav, name)


def test_one_movement_manager() -> None:
    assert [a.module for a in go2_sim_motion.active_blueprints].count(MovementManager) == 1


def test_one_worker_per_module_with_gossip_off() -> None:
    overrides = go2_sim_motion.global_config_overrides
    assert overrides["n_workers"] == len(go2_sim_motion.active_blueprints)
    assert overrides["zenoh_gossip"] is False
