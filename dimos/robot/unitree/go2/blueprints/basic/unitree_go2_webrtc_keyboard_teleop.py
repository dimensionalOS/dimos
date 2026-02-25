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

"""Unitree Go2 keyboard teleop via ControlCoordinator over WebRTC.

WASD keys → Twist → coordinator twist_command → UnitreeGo2WebRTCAdapter.

Uses WebRTC for wireless control instead of DDS (hardwired ethernet).

Controls:
    W/S: Forward/backward (linear.x)
    Q/E: Strafe left/right (linear.y)
    A/D: Turn left/right (angular.z)
    Shift: 2x boost
    Ctrl: 0.5x slow
    Space: Emergency stop
    ESC: Quit

Usage:
    dimos --simulation run unitree-go2-webrtc-keyboard-teleop   # MuJoCo sim
    dimos run unitree-go2-webrtc-keyboard-teleop                # real hardware (WebRTC)
"""

from __future__ import annotations

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.keyboard_teleop import keyboard_teleop

if global_config.simulation:
    from dimos.robot.unitree.go2.connection import go2_connection

    _go2 = go2_connection()
    _teleop = keyboard_teleop()
else:
    import os

    from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
    from dimos.control.coordinator import TaskConfig, control_coordinator
    from dimos.core.transport import LCMTransport
    from dimos.msgs.geometry_msgs import Twist
    from dimos.msgs.sensor_msgs import JointState

    _go2_joints = make_twist_base_joints("go2")

    _go2 = control_coordinator(
        hardware=[
            HardwareComponent(
                hardware_id="go2",
                hardware_type=HardwareType.BASE,
                joints=_go2_joints,
                adapter_type="unitree_go2_webrtc",
                address=os.getenv("ROBOT_IP", "192.168.12.1"),
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
    ).transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        }
    )

    _teleop = keyboard_teleop()

unitree_go2_webrtc_keyboard_teleop = autoconnect(_go2, _teleop)

__all__ = ["unitree_go2_webrtc_keyboard_teleop"]
