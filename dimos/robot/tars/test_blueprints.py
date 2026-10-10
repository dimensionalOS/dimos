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

"""Wiring of the TARS blueprints."""

import pytest

pytest.importorskip("tars_sdk")

from dimos.core.coordination.blueprints import Blueprint
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.tars.blueprints import tars_sim, tars_sim_keyboard_teleop, tars_sim_nav
from dimos.robot.tars.height_crop import HeightCrop
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop


def _remap(blueprint: Blueprint, module: type, port: str) -> object:
    names = {a.module: a.name for a in blueprint.active_blueprints}
    return blueprint.remapping_map.get((names[module], port))


@pytest.mark.parametrize("blueprint", [tars_sim, tars_sim_keyboard_teleop, tars_sim_nav])
def test_one_movement_manager_owns_cmd_vel(blueprint: Blueprint) -> None:
    assert [a.module for a in blueprint.active_blueprints].count(MovementManager) == 1


def test_keyboard_teleop_goes_through_movement_manager() -> None:
    assert _remap(tars_sim_keyboard_teleop, KeyboardTeleop, "cmd_vel") == "tele_cmd_vel"


def test_map_is_built_from_registered_lidar() -> None:
    assert _remap(tars_sim, VoxelGridMapper, "lidar") == "registered_lidar"


def test_costmap_reads_the_height_cropped_map() -> None:
    modules = [a.module for a in tars_sim_nav.active_blueprints]
    assert HeightCrop in modules
    assert _remap(tars_sim_nav, CostMapper, "global_map") == "nav_map"
