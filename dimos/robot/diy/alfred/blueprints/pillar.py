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
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.diy.alfred.channels import channel_spec
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)

# Streamed joint_command targets go through the canonical trajectory task (#3610).
PILLAR_TASK_NAME = JOINT_TRAJECTORY_TASK_NAME
PILLAR_LIFT_VELOCITY_LIMIT_M_S = 0.1

_pillar_hardware = pillar_hardware()


def pillar_motor_transports() -> dict[tuple[str, type], TransportSpec]:
    """The connection's two streams, pinned to the channels the whole-body adapter uses."""
    return {
        ("motor_command", MotorCommandArray): channel_spec(
            f"/{PILLAR_HARDWARE_ID}/motor_command", MotorCommandArray
        ),
        ("motor_states", JointState): channel_spec(
            f"/{PILLAR_HARDWARE_ID}/motor_states", JointState
        ),
    }


alfred_pillar = autoconnect(
    PillarConnection.blueprint(),
    ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_pillar_hardware],
        tasks=[
            joint_trajectory_task(
                [PILLAR_LIFT_JOINT],
                velocity_limits={PILLAR_LIFT_JOINT: PILLAR_LIFT_VELOCITY_LIMIT_M_S},
            ),
        ],
    ),
).transports(
    {
        **pillar_motor_transports(),
        ("joint_command", JointState): channel_spec(
            f"/{PILLAR_HARDWARE_ID}/joint_command", JointState
        ),
        ("coordinator_joint_state", JointState): channel_spec(
            f"/{PILLAR_HARDWARE_ID}/joints", JointState
        ),
    }
)
