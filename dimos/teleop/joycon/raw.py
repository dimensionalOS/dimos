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

"""Lossless, versioned Joy-Con samples on the standard Joy transport.

Axes are left X/Y then right X/Y, positive right/down as in hid-nintendo.
Buttons follow each controller's printed labels; the final two bits report
presence. A missing controller is distinct from a present, released controller.
"""

import math

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.joycon.mapping import (
    JOYCON_SIDES,
    LEFT_JOYCON_BUTTONS,
    RIGHT_JOYCON_BUTTONS,
    JoyConReading,
    JoyConSide,
)

FRAME_ID = "dimos/joycon/v1"
BUTTON_LABELS = (LEFT_JOYCON_BUTTONS, RIGHT_JOYCON_BUTTONS)
BUTTON_COUNT = sum(map(len, BUTTON_LABELS)) + 2


def encode_readings(left: JoyConReading | None, right: JoyConReading | None) -> Joy:
    readings = (left, right)
    return Joy(
        frame_id=FRAME_ID,
        axes=[axis for reading in readings for axis in (reading.stick if reading else (0, 0))],
        buttons=[
            int(reading is not None and label in reading.pressed)
            for reading, labels in zip(readings, BUTTON_LABELS, strict=True)
            for label in labels
        ]
        + [int(reading is not None) for reading in readings],
    )


def decode_readings(sample: Joy) -> dict[JoyConSide, JoyConReading | None]:
    if (
        sample.frame_id != FRAME_ID
        or len(sample.axes) != 4
        or len(sample.buttons) != BUTTON_COUNT
        or any(not math.isfinite(axis) or abs(axis) > 1 for axis in sample.axes)
        or any(bit not in (0, 1) for bit in sample.buttons)
    ):
        raise ValueError("invalid Joy-Con v1 sample")
    result: dict[JoyConSide, JoyConReading | None] = {}
    offset = 0
    for i, side in enumerate(JOYCON_SIDES):
        labels = BUTTON_LABELS[i]
        result[side] = (
            JoyConReading(
                frozenset(label for j, label in enumerate(labels) if sample.buttons[offset + j]),
                (sample.axes[2 * i], sample.axes[2 * i + 1]),
            )
            if sample.buttons[-2 + i]
            else None
        )
        offset += len(labels)
    return result
