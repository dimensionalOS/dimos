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

"""Surface selection measures collision geometry without modifying the live model."""

import mujoco
import numpy as np
import pytest

from dimos.robot.galaxea.r1pro.home_surfaces import low_surface_destination
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES


@pytest.fixture
def floor_scene():
    bodies = "".join(
        f'<body name="{n}" pos="10 10 10"/>' for n in ("base_link", "task_bin", *PACKING_BODIES)
    )
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom name="floor" type="box" size="5 5 .02" pos="0 0 -.02"/>'
        '<geom name="visual_only" type="box" size=".5 .5 .1" pos="-1 -2 .8" contype="0" conaffinity="0"/>'
        + bodies
        + "</worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_floor_uses_physical_support_and_does_not_edit_live_geometry(floor_scene):
    model, data = floor_scene
    groups = model.geom_group.copy()
    qpos = data.qpos.copy()
    result = low_surface_destination(model, data, "floor")
    assert result["support_geom"] == "floor"
    assert result["tray_position"] == pytest.approx([-1.0, -2.0, 0.0])
    np.testing.assert_array_equal(model.geom_group, groups)
    np.testing.assert_array_equal(data.qpos, qpos)


def test_obstruction_at_floor_destination_is_reported_before_motion(floor_scene):
    model, data = floor_scene
    geom = model.geom("visual_only")
    model.geom_contype[geom.id] = 1
    with pytest.raises(RuntimeError, match="no sufficiently level support"):
        low_surface_destination(model, data, "floor")


def test_missing_bed_does_not_substitute_floor(floor_scene):
    model, data = floor_scene
    with pytest.raises(RuntimeError, match="bed"):
        low_surface_destination(model, data, "bed")
