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

import os
import struct
from typing import Any

import pytest

from dimos.hardware.controllers.joycon.joycon_module import JoyconConnectionModule, _JoyConDevices

ZR = 313
PLUS = 315


def test_zr_held_closes_the_right_jaw_at_the_jaw_speed(mocker: Any) -> None:
    joycons = JoyconConnectionModule(jaw_speed_per_s=3.0)
    left = mocker.patch.object(joycons.left_gripper_command, "publish")
    right = mocker.patch.object(joycons.right_gripper_command, "publish")
    try:
        for _ in range(2):
            joycons.publish_reading({"left": None, "right": frozenset({ZR})}, dt=0.1)
    finally:
        joycons.stop()

    # 0.2 s at 3 openings per second: from open (1) to 0.4.
    assert right.call_args.args[0].data == pytest.approx(0.4)
    left.assert_not_called()


def test_each_plus_press_flips_follow_once(mocker: Any) -> None:
    joycons = JoyconConnectionModule()
    follow = mocker.patch.object(joycons.follow, "publish")
    held, released = frozenset({PLUS}), frozenset[int]()
    try:
        for keys in (held, held, released, held):
            joycons.publish_reading({"left": None, "right": keys}, dt=0.02)
    finally:
        joycons.stop()

    assert [call.args[0].data for call in follow.call_args_list] == [False, True]


def test_a_press_and_release_between_two_reads_is_still_seen() -> None:
    reader, writer = os.pipe()
    os.set_blocking(reader, False)
    devices = _JoyConDevices()
    devices._fds = {"right": reader}
    devices._next_scan = float("inf")
    try:
        for value in (1, 0):
            os.write(writer, struct.pack("llHHi", 0, 0, 1, PLUS, value))
        first, second = devices.read()["right"], devices.read()["right"]
    finally:
        os.close(reader)
        os.close(writer)

    assert first == frozenset({PLUS})
    assert second == frozenset()
