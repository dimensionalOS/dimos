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

"""Standalone Alfred pillar coordinator blueprint.

dimos run alfred-pillar                                          # the Nano on its by-id path
dimos run alfred-pillar --pillarconnection.device-path /dev/ttyUSB1
"""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JOINT_TRAJECTORY_TASK_NAME,
    joint_trajectory_task,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.alfred_model import ALFRED_V1_MODEL
from dimos.robot.diy.alfred.channels import channel_spec
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
)

# Streamed joint_command targets go through the canonical trajectory task (#3610).
PILLAR_TASK_NAME = JOINT_TRAJECTORY_TASK_NAME
PILLAR_LIFT_VELOCITY_LIMIT_M_S = 0.1

# PillarConnection onto a coordinator's ports for the robot "pillar".
PILLAR_CONNECTION_PORTS = [
    (PillarConnection, "position_command", f"{PILLAR_HARDWARE_ID}_position_command"),
    (PillarConnection, "joint_state", f"{PILLAR_HARDWARE_ID}_joint_state"),
]


class AlfredPillarCoordinator(ControlCoordinator):
    """Drives the pillar through its connection module."""

    pillar_position_command: Out[JointState]
    pillar_joint_state: In[JointState]


alfred_pillar = (
    autoconnect(
        PillarConnection.blueprint(),
        AlfredPillarCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=ALFRED_V1_MODEL,
            tasks=[
                joint_trajectory_task(
                    [PILLAR_LIFT_JOINT],
                    velocity_limits={PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S},
                ),
            ],
        ),
    )
    .remappings(PILLAR_CONNECTION_PORTS)
    .transports(
        {
            ("joint_command", JointState): channel_spec(
                f"/{PILLAR_HARDWARE_ID}/joint_command", JointState
            ),
            ("coordinator_joint_state", JointState): channel_spec(
                f"/{PILLAR_HARDWARE_ID}/joints", JointState
            ),
        }
    )
)
