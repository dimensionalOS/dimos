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

"""A gamepad that a headset's page reads, as the Joy-Con pair that JoyLo takes.

A headset that tracks hands and no controllers can have the pair of Joy-Cons
paired to it in place of this host. Its browser gives the pair to the page as
one gamepad in the W3C "standard" layout, and the page sends that pad as it is:
the four stick axes, and each button as 0 or 1. Here each button gets the label
JoyLo knows it by.

The layout is by place, not by print: button 3 is the top one of the right
four, which a Joy-Con prints X on. A stick is right and back positive, as a
Joy-Con reading has it.
"""

from __future__ import annotations

import math

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.joycon.mapping import JoyConReading

#: The ``frame_id`` of a pad in the layout this module reads.
STANDARD_LAYOUT = "standard"

_LEFT = {
    4: "L",
    6: "ZL",
    8: "minus",
    10: "stick",
    12: "up",
    13: "down",
    14: "left",
    15: "right",
    17: "capture",
}
_RIGHT = {0: "B", 1: "A", 2: "Y", 3: "X", 5: "R", 7: "ZR", 9: "plus", 11: "stick", 16: "home"}
# Both sticks, and the buttons up to the last direction button. Home and capture
# are past these: a browser that does not give them gives a pad without them.
_AXES, _BUTTONS = 4, 16


def joycon_readings(sample: Joy) -> tuple[JoyConReading, JoyConReading] | None:
    """The left and the right Joy-Con of a pad, or None for a pad this layout does not describe."""
    if (
        sample.frame_id != STANDARD_LAYOUT
        or len(sample.axes) < _AXES
        or len(sample.buttons) < _BUTTONS
    ):
        return None
    axes = [float(value) for value in sample.axes[:_AXES]]
    if not all(math.isfinite(value) and abs(value) <= 1.0 + 1e-6 for value in axes):
        return None
    down = [bool(button) for button in sample.buttons]

    def side(labels: dict[int, str], x: float, y: float) -> JoyConReading:
        pressed = frozenset(label for i, label in labels.items() if i < len(down) and down[i])
        return JoyConReading(pressed, (x, y))

    return side(_LEFT, axes[0], axes[1]), side(_RIGHT, axes[2], axes[3])
