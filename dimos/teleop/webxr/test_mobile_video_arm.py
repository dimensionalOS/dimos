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

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.teleop.webxr.controller_types import ThumbstickState, WebXRControllerState
from dimos.teleop.webxr.extensions import MobileVideoArmTeleopModule


def _controller(
    *,
    is_left: bool,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    thumbstick_press: bool = False,
) -> WebXRControllerState:
    return WebXRControllerState(
        is_left=is_left,
        thumbstick_press=thumbstick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def test_mobile_arm_teleop_publishes_yaw_drive_and_one_neutral_stop(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_y=-1.0)
        right = _controller(is_left=False, stick_x=0.5)

        module._publish_cmd_vel(left, right)
        moving = publish.call_args.args[0]
        assert isinstance(moving, Twist)
        assert moving.linear.x == pytest.approx(module.config.linear_scale)
        assert moving.linear.y == 0.0
        assert moving.angular.z == pytest.approx(-0.5 * module.config.yaw_scale)

        idle_left = _controller(is_left=True)
        idle_right = _controller(is_left=False)
        module._publish_cmd_vel(idle_left, idle_right)
        module._publish_cmd_vel(idle_left, idle_right)

        assert publish.call_count == 2
        assert publish.call_args.args[0] == Twist.zero()
    finally:
        module.stop()


def test_mobile_arm_teleop_strafe_mode_and_deadzone(mocker) -> None:
    module = MobileVideoArmTeleopModule(right_stick_mode="strafe")
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_x=0.5, stick_y=0.1)
        right = _controller(is_left=False, stick_x=-0.5)

        module._publish_cmd_vel(left, right)

        moving = publish.call_args.args[0]
        assert moving.linear.x == 0.0
        assert moving.linear.y == pytest.approx(0.5 * module.config.strafe_scale)
        assert moving.angular.z == pytest.approx(-0.5 * module.config.yaw_scale)
    finally:
        module.stop()


def test_mobile_arm_teleop_stick_press_publishes_one_stop_per_press(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_y=-1.0)
        pressed = _controller(is_left=False, thumbstick_press=True)

        module._publish_cmd_vel(left, pressed)
        module._publish_cmd_vel(left, pressed)

        assert publish.call_count == 1
        assert publish.call_args.args[0] == Twist.zero()
    finally:
        module.stop()
