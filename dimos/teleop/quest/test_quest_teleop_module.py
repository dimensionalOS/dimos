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
from types import SimpleNamespace

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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


def test_unknown_joy_controller_identity_is_rejected(module: QuestTeleopModule, mocker) -> None:
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )

    with pytest.raises(ValueError, match="Unexpected frame_id"):
        module._on_joy_bytes(b"data")


def test_final_client_disconnect_clears_state_and_publishes_safe_buttons(
    module: QuestTeleopModule, mocker
) -> None:
    first = mocker.MagicMock()
    second = mocker.MagicMock()
    published = []
    module.teleop_buttons.subscribe(published.append)
    pose = mocker.MagicMock(spec=PoseStamped)
    with module._lock:
        for hand in Hand:
            module._is_engaged[hand] = True
            module._initial_poses[hand] = pose
            module._current_poses[hand] = pose
            module._controllers[hand] = QuestControllerState(primary=True)
    module._connected_clients = {first, second}

    module._client_disconnected(first)
    assert module.get_status().left_engaged is True

    module._client_disconnected(second)

    status = module.get_status()
    assert status.left_engaged is False
    assert status.right_engaged is False
    assert status.left_pose is None
    assert status.right_pose is None
    assert status.buttons.data == 0
    assert published[-1].data == 0
