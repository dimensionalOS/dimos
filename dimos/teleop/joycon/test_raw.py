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

import pytest

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.joycon.mapping import LEFT_JOYCON_BUTTONS, RIGHT_JOYCON_BUTTONS, JoyConReading
from dimos.teleop.joycon.raw import decode_readings, encode_readings


def test_wire_roundtrip_retains_every_button_stick_and_presence():
    left = JoyConReading(frozenset(LEFT_JOYCON_BUTTONS), (-1, 0.25))
    right = JoyConReading(frozenset(RIGHT_JOYCON_BUTTONS), (0.5, 1))
    sample = Joy.lcm_decode(encode_readings(left, right).lcm_encode())
    assert decode_readings(sample) == {"left": left, "right": right}
    assert decode_readings(encode_readings(None, JoyConReading())) == {
        "left": None,
        "right": JoyConReading(),
    }


@pytest.mark.parametrize("axes", [[], [0, 0, 0], [0, 0, 0, float("nan")], [0, 0, 0, 2]])
def test_malformed_sticks_are_rejected(axes):
    sample = encode_readings(JoyConReading(), JoyConReading())
    sample.axes = axes
    with pytest.raises(ValueError, match="invalid Joy-Con"):
        decode_readings(sample)
