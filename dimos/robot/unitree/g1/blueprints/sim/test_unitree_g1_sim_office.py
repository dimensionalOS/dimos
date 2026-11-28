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

from dataclasses import replace

import pytest

from dimos.core.coordination.blueprints import config_defaults
from dimos.core.hardware_module import HardwareModule
from dimos.core.tf_module import TfModule

pytest.importorskip("pxr")
# Office 555 and the G1 are Git LFS, over the hosted runner's pull cap.
pytestmark = pytest.mark.self_hosted


def test_one_g1_stands_on_office_555s_main_spot_with_its_walker_and_sensors_on_its_mounts() -> None:
    from dimos.robot.unitree.g1.blueprints.sim.unitree_g1_sim_office import unitree_g1_sim_office

    solid = unitree_g1_sim_office.solidify()
    atoms = [
        replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
        for atom in solid.active_blueprints
    ]
    HardwareModule.blueprint_checks(atoms)
    TfModule.blueprint_checks(atoms)
    names = {atom.name for atom in atoms}
    assert {
        "office555",
        "main/g1bodysim",
        "main/g1walker",
        "main/odomtf",
        "main/lidar/livoxsim",
        "main/camera/simcamera",
    } <= names
    # cmd_vel reaches its walker from outside its namespace.
    walker = next(atom for atom in atoms if atom.name == "main/g1walker")
    shared = {s.name for s in walker.streams if (walker.name, s.name) not in solid.remapping_map}
    assert "cmd_vel" in shared


def test_three_g1s_stand_on_its_main_left_and_right_spots_and_only_main_takes_cmd_vel() -> None:
    from dimos.robot.unitree.g1.blueprints.sim.unitree_g1_sim_office_three import (
        unitree_g1_sim_office_three,
    )

    solid = unitree_g1_sim_office_three.solidify()
    atoms = [
        replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
        for atom in solid.active_blueprints
    ]
    HardwareModule.blueprint_checks(atoms)
    TfModule.blueprint_checks(atoms)
    names = {atom.name for atom in atoms}
    for spot in ("main", "left", "right"):
        assert {f"{spot}/g1bodysim", f"{spot}/g1walker", f"{spot}/camera/simcamera"} <= names
    assert "watching/simcamera" in names
    walkers = {atom.name: atom for atom in atoms if atom.name.endswith("/g1walker")}
    cmd_vel = {name: solid.remapping_map.get((name, "cmd_vel"), "cmd_vel") for name in walkers}
    assert cmd_vel["main/g1walker"] == "cmd_vel"
    assert cmd_vel["left/g1walker"] != "cmd_vel" and cmd_vel["right/g1walker"] != "cmd_vel"
