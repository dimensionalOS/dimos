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


def test_the_go2_sits_on_office_555s_main_spot_with_its_streams_where_a_real_one_has_them() -> None:
    from dimos.robot.unitree.go2.blueprints.sim.unitree_go2_sim_office import unitree_go2_sim_office

    atoms = [
        replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
        for atom in unitree_go2_sim_office.solidify().active_blueprints
    ]
    HardwareModule.blueprint_checks(atoms)
    TfModule.blueprint_checks(atoms)
    names = {atom.name for atom in atoms}
    assert {"main/go2webrtcsim", "main/odomtf", "office555"} <= names
    # A namespaced stream is remapped under the namespace; a shared one keeps its own name.
    namespaced = unitree_go2_sim_office.solidify().remapping_map
    go2 = next(atom for atom in atoms if atom.name == "main/go2webrtcsim")
    shared = {stream.name for stream in go2.streams if (go2.name, stream.name) not in namespaced}
    assert {"cmd_vel", "odom", "lidar", "color_image", "camera_info"} <= shared
