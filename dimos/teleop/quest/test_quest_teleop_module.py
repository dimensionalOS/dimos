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

from collections.abc import Iterator

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.teleop.quest.quest_extensions import (
    ArmTeleopModule,
    HandTeleopModule,
    MobileVideoArmTeleopModule,
)
from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule
from dimos.teleop.quest.quest_types import (
    Hand,
    QuestControllerState,
    ThumbstickState,
)


@pytest.fixture
def module() -> Iterator[QuestTeleopModule]:
    module = QuestTeleopModule(server_port=9443)
    try:
        yield module
    finally:
        module.stop()


def test_quest_web_server_is_initialized_during_start(module: QuestTeleopModule, mocker) -> None:
    web_interface = mocker.patch("dimos.teleop.quest.quest_teleop_module.RobotWebInterface")
    setup_routes = mocker.patch.object(module, "_setup_routes")
    start_server = mocker.patch.object(module, "_start_server")
    start_control_loop = mocker.patch.object(module, "_start_control_loop")

    module.start()

    web_interface.assert_called_once_with(host="0.0.0.0", port=9443)
    setup_routes.assert_called_once_with()
    start_server.assert_called_once_with()
    start_control_loop.assert_called_once_with()


def test_translation_scale_changes_pose_delta(module: QuestTeleopModule) -> None:
    module._initial_poses[Hand.RIGHT] = PoseStamped(position=[1.0, 2.0, 3.0])
    module._current_poses[Hand.RIGHT] = PoseStamped(position=[1.2, 1.5, 4.0])

    module._set_translation_scale(2.0)

    output = module._get_output_pose(Hand.RIGHT)
    assert output is not None
    assert output.position.x == pytest.approx(0.4)
    assert output.position.y == pytest.approx(-1.0)
    assert output.position.z == pytest.approx(2.0)


@pytest.mark.parametrize("translation_scale", [0.0, -1.0, float("inf")])
def test_translation_scale_must_be_positive_and_finite(
    module: QuestTeleopModule, translation_scale: float
) -> None:
    with pytest.raises(ValueError):
        module._set_translation_scale(translation_scale)

    assert module._translation_scale == 1.0


def test_arm_teleop_publishes_absolute_controller_pose() -> None:
    module = ArmTeleopModule()
    try:
        pose = PoseStamped(frame_id="left", position=[1.0, 2.0, 3.0])
        module._current_poses[Hand.LEFT] = pose
        module._initial_poses[Hand.LEFT] = PoseStamped(position=[0.5, 0.5, 0.5])

        assert module._get_output_pose(Hand.LEFT) is pose
    finally:
        module.stop()


def test_hand_teleop_pinch_toggles_engagement(mocker) -> None:
    module = HandTeleopModule()
    try:
        publish = mocker.patch.object(module.teleop_buttons, "publish")
        module._current_poses[Hand.RIGHT] = mocker.Mock()
        module._controllers[Hand.RIGHT] = QuestControllerState(
            is_left=False, primary=True, trigger=1.0
        )

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        assert publish.call_args.args[0].right_trigger_analog == pytest.approx(1.0)

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]

        module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
        module._handle_engage()
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=True)
        module._handle_engage()

        assert not module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert not publish.call_args.args[0].right_primary
    finally:
        module.stop()


def _controller(
    *,
    is_left: bool,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    thumbstick_press: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
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
