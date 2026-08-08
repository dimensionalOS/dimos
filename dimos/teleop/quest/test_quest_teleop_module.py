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
from dimos.teleop.quest.quest_extensions import ArmTeleopModule, HandTeleopModule
from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState


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
