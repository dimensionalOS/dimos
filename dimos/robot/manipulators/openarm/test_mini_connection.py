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

"""Joy-Con pause keeps arm-only leaders off the follower stream."""

import pytest

from dimos.robot.manipulators.openarm.mini_connection import OpenArmMiniJoyConConnection
from dimos.teleop.webxr.controller_types import Buttons


async def test_plus_toggles_only_on_press_and_does_not_publish_trigger_grippers(mocker):
    connection = OpenArmMiniJoyConConnection()
    try:
        publish = mocker.patch.object(connection.joint_state, "publish")
        left = mocker.patch.object(connection.left_gripper_command, "publish")
        right = mocker.patch.object(connection.right_gripper_command, "publish")
        pressed = Buttons()
        pressed.right_menu = True
        readings = {"left": {f"joint_{i}": 0.0 for i in range(1, 8)}}
        connection.publish_reading(readings)
        assert publish.call_count == 1
        await connection.handle_teleop_buttons(pressed)
        await connection.handle_teleop_buttons(pressed)
        connection.publish_reading(readings)
        assert publish.call_count == 1
        await connection.handle_teleop_buttons(Buttons())
        await connection.handle_teleop_buttons(pressed)
        readings["left"]["joint_1"] = 0.3
        connection.publish_reading(readings)
        assert publish.call_count == 2
        assert publish.call_args.args[0].position[0] == pytest.approx(0.3)
        left.assert_not_called()
        right.assert_not_called()
    finally:
        connection.stop()
