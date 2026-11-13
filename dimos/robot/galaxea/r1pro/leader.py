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

"""Read the seven-motor OpenArm leaders through the shared Feetech connection."""

import math

from dimos.robot.manipulators.feetech.robot import FeetechBus, FeetechMotor, FeetechRobot
from dimos.robot.manipulators.openarm.mini_connection import (
    OpenArmMiniConnection,
    OpenArmMiniConnectionConfig,
)
from dimos.teleop.joycon.mapping import JoyConSide


class R1ProLeaderConfig(OpenArmMiniConnectionConfig):
    source: str = "r1pro"
    enabled_sides: tuple[JoyConSide, ...] = ("left", "right")


class R1ProLeaderConnection(OpenArmMiniConnection):
    config: R1ProLeaderConfig
    # These are raw leader readings, not OpenArm follower targets. Do not clamp
    # them to the follower before JoyLo's discontinuity guard. Its turn history
    # preserves absolute tracking across +/-pi, including after a pause.
    robot = FeetechRobot(
        buses=tuple(
            FeetechBus(
                name=side,
                motors=tuple(
                    FeetechMotor(f"joint_{i}", f"{side}_arm_joint{i}", -math.tau, math.tau)
                    for i in range(1, 8)
                ),
            )
            for side in ("left", "right")
        )
    )

    def serial_ports(self) -> dict[str, str]:
        return {
            side: port
            for side, port in super().serial_ports().items()
            if side in self.config.enabled_sides
        }

    def publish_reading(self, readings: dict[str, dict[str, float]]) -> None:
        super().publish_reading(
            {side: values for side, values in readings.items() if side in self.config.enabled_sides}
        )
