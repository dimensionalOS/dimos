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

from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule


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
