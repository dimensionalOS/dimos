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

"""Unitree Go2 keyboard teleop via ControlCoordinator (DDS/SDK2 path).

WASD keys -> Twist -> coordinator twist_command -> UnitreeGo2TwistAdapter (DDS).

Usage:
    dimos run unitree-go2-keyboard-teleop
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_go2_joints = make_twist_base_joints("go2")

unitree_go2_keyboard_teleop = (
    autoconnect(
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="go2",
                    hardware_type=HardwareType.BASE,
                    joints=_go2_joints,
                    adapter_type="unitree_go2",
                    adapter_kwargs={"rage_mode": False},
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
        KeyboardTeleop.blueprint(),
    )
    .transports(
        {
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(obstacle_avoidance=True)
)

__all__ = ["unitree_go2_keyboard_teleop"]
