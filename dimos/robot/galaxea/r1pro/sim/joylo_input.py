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

"""Explicit simulation input for scripted checks without physical controllers."""

import math

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.galaxea.r1pro.joylo import ARMS
from dimos.teleop.joycon.mapping import LEFT_JOYCON_BUTTONS, RIGHT_JOYCON_BUTTONS, JoyConReading
from dimos.teleop.joycon.raw import encode_readings


class JoyloSimInput(Module):
    leader_readings: Out[JointState]
    joycon_sample: Out[Joy]

    @rpc
    def send(
        self,
        joints: list[float],
        left_buttons: list[str],
        right_buttons: list[str],
        axes: list[float],
        left_present: bool = True,
        right_present: bool = True,
    ) -> None:
        """Send one input sample; call at 50 Hz while commanding the simulation.

        joints: seven left then seven right calibrated leader radians.
        axes: left X/Y then right X/Y, positive right/down, each in [-1, 1].
        No samples means the robot-side input timeout latches a pause.
        """
        if len(joints) != len(ARMS) or not all(map(math.isfinite, joints)):
            raise ValueError("14 finite leader joint values required")
        if len(axes) != 4 or any(not math.isfinite(v) or abs(v) > 1 for v in axes):
            raise ValueError("four finite normalized stick axes required")
        if set(left_buttons) - set(LEFT_JOYCON_BUTTONS) or set(right_buttons) - set(
            RIGHT_JOYCON_BUTTONS
        ):
            raise ValueError("unknown Joy-Con button label")
        self.leader_readings.publish(JointState(name=list(ARMS), position=joints))
        self.joycon_sample.publish(
            encode_readings(
                JoyConReading(frozenset(left_buttons), (axes[0], axes[1]))
                if left_present
                else None,
                JoyConReading(frozenset(right_buttons), (axes[2], axes[3]))
                if right_present
                else None,
            )
        )
