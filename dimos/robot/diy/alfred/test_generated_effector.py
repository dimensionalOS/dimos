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

from types import SimpleNamespace
from unittest.mock import MagicMock

from dimos_generated.nav_msgs.msg import Odometry
import pytest

from dimos.msgs.geometry import yaw
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel


@pytest.mark.asyncio
async def test_wheel_odometry_generated_frame_inversion_and_local_velocity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = AlfredHighLevel()
    readings = iter(
        [
            {"translation": [0.0, 0.0], "rotation": 0.0},
            {"translation": [1.0, -2.0], "rotation": 0.0},
        ]
    )
    client = MagicMock()

    def get_odometry(_request: object) -> SimpleNamespace:
        reading = next(readings)
        return SimpleNamespace(done=lambda: True, result=lambda: reading)

    client.get_odometry.side_effect = get_odometry
    stamps = iter([100.0, 101.0])
    monkeypatch.setattr(
        "dimos.robot.diy.alfred.effector_high_level.time",
        SimpleNamespace(time=lambda: next(stamps)),
    )
    received: list[Odometry] = []

    def publish(value: Odometry) -> None:
        received.append(Odometry.decode(value.encode()))
        if len(received) == 2:
            module._odometry_stop.set()

    module.wheel_odometry = MagicMock()
    module.wheel_odometry.publish.side_effect = publish

    async def wait_or_stop(_seconds: float) -> None:
        return None

    monkeypatch.setattr(module, "_wait_or_stop", wait_or_stop)
    await module._poll_wheel_odometry(client)
    assert len(received) == 2
    first, last = received
    assert first.header.stamp.sec == 100
    assert last.header.stamp.sec == 101
    assert last.header.frame_id == "wheel_odom"
    assert last.child_frame_id == "base_link"
    assert (last.pose.pose.position.x, last.pose.pose.position.y) == (1.0, 2.0)
    assert yaw(last.pose.pose.orientation) == 0.0
    assert (last.twist.twist.linear.x, last.twist.twist.linear.y, last.twist.twist.angular.z) == (
        1.0,
        2.0,
        0.0,
    )
    client.set_target_velocity.assert_not_called()
