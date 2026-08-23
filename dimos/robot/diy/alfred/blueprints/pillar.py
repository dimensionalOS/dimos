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

"""Standalone Alfred pillar coordinator blueprint."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PillarConnection,
    pillar_hardware,
)

PILLAR_SERVO_TASK_NAME = "servo_pillar"

_pillar_hardware = pillar_hardware()

alfred_pillar = autoconnect(
    PillarConnection.blueprint(),
    ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_pillar_hardware],
        tasks=[
            TaskConfig(
                name=PILLAR_SERVO_TASK_NAME,
                type="servo",
                joint_names=[PILLAR_LIFT_JOINT],
                priority=10,
                auto_start=True,
            ),
        ],
    ),
).transports(
    {
        ("motor_command", MotorCommandArray): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/motor_command", MotorCommandArray
        ),
        ("motor_states", JointState): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/motor_states", JointState
        ),
        ("joint_command", JointState): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/joint_command", JointState
        ),
        ("coordinator_joint_state", JointState): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/joints", JointState
        ),
    }
)
