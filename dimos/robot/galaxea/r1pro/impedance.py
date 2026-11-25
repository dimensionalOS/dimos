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

"""Arm impedance settings shared by the R1 Pro simulation and hardware trials."""

from dimos.control.coordinator import TaskConfig
from dimos.robot.galaxea.r1pro.joylo import ARMS


def joylo_impedance_task(arm_rate: float) -> TaskConfig:
    """JoyLo profile with a fixed arm spring and a 2 Nm spring torque cap."""
    return TaskConfig(
        name="leader_impedance",
        type="joint_impedance",
        joint_names=list(ARMS),
        priority=10,
        stream_bind={"joint_command": "leader_joint_command"},
        params={
            "stiffness": dict.fromkeys(ARMS, 45.0),
            "damping": dict.fromkeys(ARMS, 6.0),
            "max_torque": dict.fromkeys(ARMS, 2.0),
            "velocity_limits": dict.fromkeys(ARMS, arm_rate),
        },
    )
