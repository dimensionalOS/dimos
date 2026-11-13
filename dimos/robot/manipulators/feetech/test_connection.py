# Copyright 2025-2026 Dimensional Inc.
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

import threading
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.feetech.connection import FeetechConnection
from dimos.robot.manipulators.feetech.robot import FeetechBus, FeetechRobot


class _Leaders(FeetechConnection):
    robot = FeetechRobot(
        buses=(FeetechBus("left", ("joint_1", "joint_7")), FeetechBus("right", ("joint_1",)))
    )

    def serial_ports(self) -> dict[str, str]:
        return {}


def test_connection_publishes_every_joint_under_its_source(mocker: Any) -> None:
    connection = _Leaders(mock_positions={"left_joint_7": 0.5})
    published = threading.Event()
    messages: list[JointState] = []

    def record(message: JointState) -> None:
        messages.append(message)
        published.set()

    mocker.patch.object(connection.joint_state, "publish", side_effect=record)
    connection.start()
    try:
        assert published.wait(2.0)
    finally:
        connection.stop()

    assert messages[0].name == [
        "leader/left_joint_1",
        "leader/left_joint_7",
        "leader/right_joint_1",
    ]
    assert messages[0].position == pytest.approx([0.0, 0.5, 0.0])
