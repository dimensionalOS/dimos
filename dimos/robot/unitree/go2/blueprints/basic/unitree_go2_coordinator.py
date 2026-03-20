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

"""Unitree Go2 ControlCoordinator over WebRTC via transport adapter.

GO2Connection owns the WebRTC connection (sensors + actuation).
ControlCoordinator communicates via LCM topics using TransportTwistAdapter.

    Twist → coordinator twist_command → TransportTwistAdapter
          → LCM /go2/cmd_vel → GO2Connection → connection.move()

Usage:
    dimos run unitree-go2-coordinator               # real robot (WebRTC)
    dimos --simulation run unitree-go2-coordinator   # MuJoCo sim
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig, control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.go2.connection import GO2Connection, go2_connection

_go2_joints = make_twist_base_joints("go2")

unitree_go2_coordinator = autoconnect(
    go2_connection(),
    control_coordinator(
        hardware=[
            HardwareComponent(
                hardware_id="go2",
                hardware_type=HardwareType.BASE,
                joints=_go2_joints,
                adapter_type="transport_lcm",
            ),
        ],
        tasks=[
            TaskConfig(
                name="vel_go2",
                type="velocity",
                joint_names=_go2_joints,
                priority=10,
            ),
        ],
    ),
).remappings(
    [
        # Rename GO2Connection's cmd_vel so autoconnect doesn't wire teleop directly to it.
        # The adapter publishes to /go2/cmd_vel, GO2Connection reads from it.
        (GO2Connection, "cmd_vel", "go2_cmd_vel"),
        # Rename GO2Connection's odom so the adapter reads it on /go2/odom.
        (GO2Connection, "odom", "go2_odom"),
    ]
).transports(
    {
        # Teleop/nav/agent publish cmd_vel, coordinator reads twist_command — same topic
        ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        # Adapter writes here → GO2Connection reads (via remapped name)
        ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
        # GO2Connection publishes odom here → adapter reads it
        ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
        # Coordinator publishes joint state
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
    }
)

__all__ = ["unitree_go2_coordinator"]
