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

"""A blueprint's robot leaves its own scene and stands in another environment."""

import math

import mujoco
import numpy as np
import pytest

from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.sim2.geometry.mujoco import compile_scene
from dimos.sim2.scenes.compose import robot_in_environment
from dimos.sim2.scenes.mjcf import environment_from_mjcf
from dimos.sim2.scenes.test_mjcf import ROOM
from dimos.sim2.scenes.urdf import Component, prepare_scene

# A cart with one driven lift, and the shed it was authored beside.
BOT = """
<robot name="bot">
  <link name="bot/base">
    <inertial><mass value="10"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
    <visual><geometry><box size="0.4 0.4 0.2"/></geometry></visual>
    <collision><geometry><box size="0.4 0.4 0.2"/></geometry></collision>
  </link>
  <link name="bot/mast">
    <inertial><mass value="1"/><inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
    <visual><origin xyz="0 0 0.2"/><geometry><box size="0.05 0.05 0.4"/></geometry></visual>
  </link>
  <joint name="bot/lift" type="prismatic">
    <parent link="bot/base"/><child link="bot/mast"/><origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="0.3" effort="100" velocity="1"/>
  </joint>
</robot>
"""
SHED = """
<robot name="shed">
  <link name="shed/wall">
    <visual><geometry><box size="2 0.1 2"/></geometry></visual>
    <collision><geometry><box size="2 0.1 2"/></geometry></collision>
  </link>
</robot>
"""


@pytest.fixture
def robot_scene(tmp_path):
    (tmp_path / "bot.urdf").write_text(BOT)
    (tmp_path / "shed.urdf").write_text(SHED)
    return prepare_scene(
        {
            "bot": Component(
                RobotModel.from_file(tmp_path / "bot.urdf"),
                xyz=(5, 5, 0.11),
                floating=True,
                drives={"bot/lift": (100.0, 10.0, 0.0)},
            ),
            "shed": Component(RobotModel.from_file(tmp_path / "shed.urdf"), xyz=(5, 6, 1)),
        }
    )


@pytest.fixture
def room(tmp_path):
    (tmp_path / "room.xml").write_text(ROOM)
    environment_from_mjcf(tmp_path / "room.xml", tmp_path / "room")
    return UsdModel.from_file(tmp_path / "room" / "scene.usda")


def test_the_robot_stands_in_the_room_with_its_drive_and_nothing_of_its_old_scene(
    robot_scene, room
):
    composed = robot_in_environment(
        robot_scene, ["bot/lift"], room, place=(0.0, -2.0, 0.0), heading=math.pi / 2
    )
    model = compile_scene(composed)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = {model.body(i).name for i in range(model.nbody)}
    assert {"bot/base", "bot/mast", "env/crate", "env/door", "env/static"} <= names
    assert "shed/wall" not in names
    # The room has its own slab, and the plane of the robot's old scene is gone.
    assert not (model.geom_type == mujoco.mjtGeom.mjGEOM_PLANE).any()
    base = model.body("bot/base").id
    # It keeps the height it had above its own floor, at the new place and heading.
    np.testing.assert_allclose(data.xpos[base], [0, -2, 0.11], atol=1e-6)
    np.testing.assert_allclose(data.xmat[base].reshape(3, 3)[:, 0], [0, 1, 0], atol=1e-6)
    assert model.actuator("bot/lift").trnid[0] == model.joint("bot/lift").id
    # It drops the last centimetre onto the room's slab and rests there.
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert data.xpos[base][2] == pytest.approx(0.1, abs=0.005)


def test_a_scene_whose_driven_joints_are_not_in_one_part_is_refused(robot_scene, room):
    with pytest.raises(ValueError, match="driven joints are in 0 parts"):
        robot_in_environment(robot_scene, ["other/joint"], room)
