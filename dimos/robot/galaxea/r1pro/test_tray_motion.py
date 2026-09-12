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

"""Unloading must check the gripper opening sweep against the supported tray."""

import mujoco
import pytest

from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.tray_motion import TrayMotion, TrayWaypoint


@pytest.fixture
def opening_motion():
    links = []
    for i, name in enumerate(R1PRO_PICK_PLACE_JOINTS):
        joint = (
            f'<joint name="{name}" type="slide" axis="0 -1 0" range="-.1 .1"/>'
            if i == len(R1PRO_PICK_PLACE_JOINTS) - 1
            else f'<joint name="{name}" type="hinge" range="-3 3"/>'
        )
        links.append(
            f'<body name="link_{i}">{joint}'
            '<inertial pos="0 0 0" mass=".1" diaginertia=".001 .001 .001"/>'
        )
    chain = "".join(links) + (
        '<geom name="finger" type="box" pos="0 .22 .55" size=".04 .008 .02"/>'
        + "</body>" * len(links)
    )
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body name="base_link" pos="0 0 .3">'
        '<joint name="r1pro/base_x" type="slide" axis="1 0 0"/>'
        '<joint name="r1pro/base_y" type="slide" axis="0 1 0"/>'
        '<joint name="r1pro/base_yaw" type="hinge" axis="0 0 1"/>'
        '<geom type="box" size=".1 .1 .1"/>'
        + chain
        + '</body><body name="task_bin" pos="0 .175 .85">'
        '<freejoint name="task_tray_free"/>'
        '<geom name="handle" type="box" size=".05 .012 .012"/>'
        "</body></worldbody><actuator>"
        '<position name="r1pro/base_x" joint="r1pro/base_x"/>'
        '<position name="r1pro/base_y" joint="r1pro/base_y"/>'
        '<position name="r1pro/base_yaw" joint="r1pro/base_yaw"/>'
        "</actuator></mujoco>"
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    motion = TrayMotion(model, data, cargo_bodies=())
    target = motion.initial.copy()
    target[-1] = 0.05
    return motion, data, [TrayWaypoint("unload_release", target.tolist(), 1.0)]


def test_unloading_rejects_opening_a_finger_into_the_tray_handle(opening_motion):
    motion, data, points = opening_motion
    with pytest.raises(RuntimeError, match="contacts the supported tray"):
        motion._checked(points, data, allow_tray_contact=False)
    assert data.joint(R1PRO_PICK_PLACE_JOINTS[-1]).qpos[0] == 0.0


def test_tray_grasping_can_still_intentionally_contact_its_handles(opening_motion):
    motion, data, points = opening_motion
    assert motion._checked(points, data) == points
