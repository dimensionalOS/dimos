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

"""Release checks distinguish actual support from incidental side contact."""

import mujoco
import pytest

from dimos.robot.galaxea.r1pro.bottle_motion import bottle_contacts


@pytest.mark.parametrize(("support", "expected"), [("table", ["table"]), ("wall", [])])
def test_bottle_contact_requires_an_upward_support_normal(support, expected):
    fixture = (
        '<geom name="table" type="box" pos="0 0 -.01" size=".3 .3 .01"/>'
        if support == "table"
        else '<geom name="wall" type="box" pos=".034 0 .07" size=".01 .3 .3"/>'
    )
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><option gravity="0 0 0"/><worldbody>'
        + fixture
        + '<body name="task_bottle" pos="0 0 .069">'
        '<freejoint name="task_bottle_free"/><geom type="cylinder" size=".025 .07" mass=".1"/>'
        '</body><body name="base_link" pos="5 0 0">'
        '<joint name="r1pro/right_gripper" type="slide" axis="1 0 0"/>'
        '<geom name="right_finger_pad1" type="sphere" size=".01" mass=".1"/>'
        '<geom name="right_finger_pad2" type="sphere" size=".01" pos="0 .1 0" mass=".1"/>'
        "</body></worldbody></mujoco>"
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon > 0
    state = bottle_contacts(model, data, 0)
    assert state["support_geoms"] == expected
