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

"""Unitree Go2 keyboard teleop via ControlCoordinator.

WASD keys → Twist → coordinator twist_command → UnitreeGo2Adapter.

Controls:
    W/S: Forward/backward (linear.x)
    Q/E: Strafe left/right (linear.y)
    A/D: Turn left/right (angular.z)
    Shift: 2x boost
    Ctrl: 0.5x slow
    Space: Emergency stop
    ESC: Quit

Usage:
    dimos --simulation run unitree-go2-keyboard-teleop   # MuJoCo sim
    dimos run unitree-go2-keyboard-teleop                # real hardware
"""

from __future__ import annotations

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.unitree.keyboard_teleop import keyboard_teleop

if global_config.simulation:
    from dimos.robot.unitree.go2.connection import go2_connection

    # go2_connection automatically uses MujocoConnection when simulation=True.
    # keyboard_teleop.cmd_vel is wired directly to go2_connection.cmd_vel
    # by autoconnect — no LCM needed.
    _go2 = go2_connection()
    _teleop = keyboard_teleop()
else:
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
                adapter_type="unitree_go2",
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

    _teleop = keyboard_teleop().transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
        }
    )

unitree_go2_keyboard_teleop = autoconnect(_go2, _teleop)

__all__ = ["unitree_go2_keyboard_teleop"]
