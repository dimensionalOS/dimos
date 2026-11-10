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

"""G1 canonical joint connection and bounded trajectory coordinator."""

from __future__ import annotations

import os

from dimos.control.connection.connection_module import ConnectionConfig as JointConnectionConfig
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
from dimos.robot.unitree.g1.wholebody_connection import (
    G1WholeBodyConnection,
    G1WholeBodyConnectionConfig,
)


class _G1Coordinator(ControlCoordinator):
    g1_position_command: Out[JointState]
    g1_joint_state: In[JointState]
    g1_imu: In[Imu]


_declaration = G1WholeBodyConnectionConfig(
    release_sport_mode=True, network_interface=os.getenv("ROBOT_INTERFACE", "")
)
unitree_g1_coordinator = (
    autoconnect(
        G1WholeBodyConnection.blueprint(**_declaration.model_dump()),
        _G1Coordinator.blueprint(
            instance_name="ControlCoordinator",
            tick_rate=500,
            connections=[
                JointConnectionConfig(
                    **_declaration.model_dump(include=set(JointConnectionConfig.model_fields))
                )
            ],
            tasks=[
                joint_trajectory_task(
                    G1_JOINT_NAMES, velocity_limits={name: 1.0 for name in G1_JOINT_NAMES}
                )
            ],
        ),
    )
    .remappings(
        [
            (G1WholeBodyConnection, "position_command", "g1_position_command"),
            (_G1Coordinator, "g1_joint_state", "joint_state"),
            (_G1Coordinator, "g1_imu", "imu"),
        ]
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/g1/joints", JointState),
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("imu", Imu): LCMTransport("/g1/imu", Imu),
        }
    )
)
