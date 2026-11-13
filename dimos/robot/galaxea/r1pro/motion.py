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

"""R1 motion groups shared by every coordinator command source."""

from dimos.control.motion_limits import MotionGroup
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES
from dimos.robot.galaxea.r1pro.joylo import ARMS, JAWS, TORSO


def motion_groups(
    *,
    arm_rate: float = 0.75,
    arm_ceiling: float = 1.0,
    torso_rate: float = 0.3,
    torso_ceiling: float = 0.5,
    base_speed: float = 0.3,
    yaw_speed: float = 0.6,
) -> dict[str, MotionGroup]:
    return {
        "arm_rate": MotionGroup(
            joints=ARMS, rate=arm_rate, ceiling=arm_ceiling, label="Arm tracking"
        ),
        "torso_rate": MotionGroup(
            joints=TORSO,
            rate=torso_rate,
            ceiling=torso_ceiling,
            label="Torso tracking",
            coordinated=True,
        ),
        "base_speed": MotionGroup(
            joints=R1PRO_BODY_AXES[:2],
            rate=base_speed,
            ceiling=0.5,
            unit="m/s",
            label="Drive speed",
        ),
        "yaw_speed": MotionGroup(
            joints=R1PRO_BODY_AXES[2:], rate=yaw_speed, ceiling=2.0, label="Turn speed"
        ),
        "gripper_rate": MotionGroup(
            joints=JAWS, rate=0.3, ceiling=0.3, unit="m/s", label="Gripper opening"
        ),
    }
