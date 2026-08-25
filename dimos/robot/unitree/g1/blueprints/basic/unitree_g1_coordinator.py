#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""G1 ControlCoordinator connected to the DDS connection module by typed ports.

Mirrors `unitree_go2_coordinator.py`. Run with `ROBOT_INTERFACE=<nic> dimos run unitree-g1-coordinator`.
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType, make_humanoid_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.port_adapter import PortWholeBodyAdapter
from dimos.hardware.whole_body.spec import WholeBodyAdapter
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

_g1_joints = make_humanoid_joints("g1")


class _G1Coordinator(ControlCoordinator):
    motor_states: In[JointState]
    imu: In[Imu]
    motor_command: Out[MotorCommandArray]
    g1_joints: Out[JointState]

    def _create_whole_body_adapter(self, component: HardwareComponent) -> WholeBodyAdapter:
        return PortWholeBodyAdapter(
            dof=len(component.joints),
            motor_states=self.motor_states,
            imu=self.imu,
            motor_command=self.motor_command,
        )


# ROBOT_INTERFACE pins cyclonedds to a NIC; required on multi-NIC hosts.
unitree_g1_coordinator = autoconnect(
    G1WholeBodyConnection.blueprint(
        release_sport_mode=True,
        network_interface=os.getenv("ROBOT_INTERFACE", ""),
    ),
    _G1Coordinator.blueprint(
        instance_name="ControlCoordinator",
        publish_robot_joint_states=True,
        tick_rate=500,
        hardware=[
            HardwareComponent(
                hardware_id="g1",
                hardware_type=HardwareType.WHOLE_BODY,
                joints=_g1_joints,
                adapter_type="module",
            ),
        ],
        tasks=[
            TaskConfig(
                name="servo_g1",
                type="servo",
                joint_names=_g1_joints,
                priority=10,
            ),
        ],
    ),
).transports(
    {
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("g1_joints", JointState): LCMTransport("/g1/joints", JointState),
    }
)
