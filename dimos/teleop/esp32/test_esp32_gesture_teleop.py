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

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.esp32.esp32_gesture_teleop import Esp32GestureTeleop


@pytest.fixture
def module() -> Iterator[Esp32GestureTeleop]:
    module = Esp32GestureTeleop(address="d4:e9:f4:e2:79:02")
    try:
        yield module
    finally:
        module.stop()


def published(publish) -> list[Joy]:  # type: ignore[no-untyped-def]
    return [call.args[0] for call in publish.call_args_list]


def test_gesture_is_published_as_a_tap_on_joy(module: Esp32GestureTeleop, mocker) -> None:
    publish = mocker.patch.object(module.joystick, "publish")

    module._on_payload(bytes([0, 0, 1, 20, 1]), now=0.0)  # baseline
    module._on_payload(bytes([0, 1, 2, 35, 2]), now=0.1)  # STAND

    press, release = published(publish)
    assert press.buttons == [0, 1]
    assert release.buttons == [0, 0]
    assert press.axes == pytest.approx([0.35])


def test_repeated_advertisements_publish_once(module: Esp32GestureTeleop, mocker) -> None:
    publish = mocker.patch.object(module.joystick, "publish")

    module._on_payload(bytes([0, 0, 2, 20, 1]), now=0.0)
    for i in range(5):
        module._on_payload(bytes([0, i, 1, 20, 2]), now=0.1 + i / 10)

    assert [msg.buttons for msg in published(publish)] == [[1, 0], [0, 0]]


def test_malformed_payload_publishes_nothing(module: Esp32GestureTeleop, mocker) -> None:
    publish = mocker.patch.object(module.joystick, "publish")
    module._on_payload(bytes(3), now=0.0)
    publish.assert_not_called()


def test_other_boards_are_ignored(module: Esp32GestureTeleop, mocker) -> None:
    on_payload = mocker.patch.object(module, "_on_payload")
    other = mocker.Mock(address="AA:BB:CC:DD:EE:FF")
    module._on_advertisement(other, mocker.Mock(manufacturer_data={0xFFFF: bytes(5)}))
    on_payload.assert_not_called()


def test_start_requires_an_address() -> None:
    module = Esp32GestureTeleop()
    try:
        with pytest.raises(ValueError):
            module.start()
    finally:
        module.stop()
