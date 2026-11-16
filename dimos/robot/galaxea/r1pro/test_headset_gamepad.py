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

import math

import pytest

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.galaxea.r1pro.headset_gamepad import STANDARD_LAYOUT, joycon_readings
from dimos.teleop.joycon.mapping import LEFT_JOYCON_BUTTONS, RIGHT_JOYCON_BUTTONS, JoyConReading


def pad(
    *down: int, axes=(0.0, 0.0, 0.0, 0.0), buttons: int = 18, layout: str = STANDARD_LAYOUT
) -> Joy:
    return Joy(frame_id=layout, axes=list(axes), buttons=[int(i in down) for i in range(buttons)])


def test_each_button_of_the_standard_layout_is_the_joycon_button_at_its_place():
    # The right four by place: bottom, right, left, top. Then shoulders, triggers, minus and plus,
    # stick presses, the four directions, home, capture.
    labels = ["B", "A", "Y", "X", "L", "R", "ZL", "ZR", "minus", "plus", "stick", "stick"]
    labels += ["up", "down", "left", "right", "home", "capture"]
    right_side = {0, 1, 2, 3, 5, 7, 9, 11, 16}
    for index, label in enumerate(labels):
        left, right = joycon_readings(pad(index))
        mine, other = (right, left) if index in right_side else (left, right)
        assert mine.pressed == {label} and other.pressed == frozenset(), index
        assert label in (RIGHT_JOYCON_BUTTONS if index in right_side else LEFT_JOYCON_BUTTONS)
    left, right = joycon_readings(pad(3, 6, 7, 12, 11))
    assert left.pressed == {"ZL", "up"} and right.pressed == {"X", "ZR", "stick"}
    assert right.stick_pressed and not left.stick_pressed


def test_the_sticks_are_the_first_four_axes_as_they_are():
    left, right = joycon_readings(pad(axes=(0.25, -1.0, -0.5, 0.75)))
    assert left == JoyConReading(frozenset(), (0.25, -1.0))
    assert right == JoyConReading(frozenset(), (-0.5, 0.75))


def test_a_pad_without_home_and_capture_is_still_the_pair():
    left, right = joycon_readings(pad(3, 15, buttons=16))
    assert right.pressed == {"X"} and left.pressed == {"right"}


@pytest.mark.parametrize(
    "sample",
    [
        pad(layout=""),
        pad(layout="xr-standard"),
        pad(buttons=15),
        pad(axes=(0.0, 0.0)),
        pad(axes=(0.0, math.nan, 0.0, 0.0)),
        pad(axes=(0.0, 1.5, 0.0, 0.0)),
    ],
)
def test_a_pad_this_layout_does_not_describe_is_no_joycon_pair(sample):
    assert joycon_readings(sample) is None
