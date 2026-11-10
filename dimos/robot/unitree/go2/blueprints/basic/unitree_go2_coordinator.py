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

"""Unitree Go2 ControlCoordinator with concrete canonical joint connection ports.

Usage:
    dimos run unitree-go2-coordinator
    dimos --simulation run unitree-go2-coordinator
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig as JointConnectionConfig
from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.go2.connection import GO2_BASE_JOINTS, ConnectionConfig, GO2Connection
from dimos.robot.unitree.go2.coordinator import Go2Coordinator

_go2_joints = list(GO2_BASE_JOINTS)

unitree_go2_coordinator = (
    autoconnect(
        GO2Connection.blueprint(obstacle_avoidance=False),
        Go2Coordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[
                JointConnectionConfig(
                    **ConnectionConfig().model_dump(include=set(JointConnectionConfig.model_fields))
                )
            ],
            tasks=[
                TaskConfig(
                    name="vel_go2",
                    type="twist_velocity",
                    joint_names=_go2_joints,
                    params={
                        "axes": ["linear.x", "linear.y", "angular.z"],
                        "yaw_joint": GO2_BASE_JOINTS[2],
                    },
                    priority=10,
                ),
            ],
        ),
    )
    .remappings(
        [
            (GO2Connection, "cmd_vel", "go2_cmd_vel"),
            (GO2Connection, "velocity_command", "go2_velocity_command"),
            (Go2Coordinator, "go2_joint_state", "joint_state"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
        }
    )
)
