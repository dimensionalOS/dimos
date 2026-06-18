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

"""Go2 ControlCoordinator: Go2WholeBodyConnection Module + servo task via LCM bridge.

Run with `ROBOT_INTERFACE=<nic> dimos run unitree-go2-wholebody-coordinator`.
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType, make_quadruped_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.go2.wholebody_connection import Go2WholeBodyConnection

_go2_joints = make_quadruped_joints("go2")

# Per-joint PD gains, applied by ConnectedWholeBody when converting
# position commands → MotorCommand
_KP = (25.0,) * 12
_KD = (0.5,) * 12

# ROBOT_INTERFACE pins cyclonedds to a NIC; required on multi-NIC hosts.
unitree_go2_wholebody_coordinator = (
    autoconnect(
        Go2WholeBodyConnection.blueprint(
            release_sport_mode=True,
            network_interface=os.getenv("ROBOT_INTERFACE", ""),
        ),
        ControlCoordinator.blueprint(
            tick_rate=500,
            hardware=[
                HardwareComponent(
                    hardware_id="go2",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=_go2_joints,
                    adapter_type="transport_lcm",
                    wb_config=WholeBodyConfig(kp=_KP, kd=_KD),
                ),
            ],
            tasks=[
                TaskConfig(
                    name="servo_go2",
                    type="servo",
                    joint_names=_go2_joints,
                    priority=10,
                ),
            ],
        ),
    )
    # No remappings: Module stream names (motor_states/imu/motor_command) don't
    # collide with ControlCoordinator's (joint_state/joint_command/...).
    .transports(
        {
            ("motor_states", JointState): LCMTransport("/go2/motor_states", JointState),
            ("imu", Imu): LCMTransport("/go2/imu", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/go2/motor_command", MotorCommandArray
            ),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("joint_command", JointState): LCMTransport("/go2/joint_command", JointState),
        }
    )
)


__all__ = ["unitree_go2_wholebody_coordinator"]
