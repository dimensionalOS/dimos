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

"""Robot-side OpenArm/Joy-Con teleop through CC's Galaxea ROS connection.

Run the operator-only blueprint on the laptop, in the same Zenoh domain.
Starts paused. The existing vendor stack must be started and checked explicitly.
"""

import math

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import _transports
from dimos.robot.galaxea.r1pro.connection import R1ProConnection, R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo import ARMS, TORSO, JoyloConfig
from dimos.robot.galaxea.r1pro.joylo_coordinator import R1ProJoyloCoordinator, joylo_tasks
from dimos.robot.galaxea.r1pro.joylo_module import R1ProJoylo

_declaration = R1ProConnectionConfig(
    boot_vendor_stack=False,
    tracking_speed=1.0,
    enable_wrist_color=True,
    color_publish_hz=10.0,
    enable_wrist_depth=False,
    publish_odom_tf=True,
)
_profile = JoyloConfig(
    leader_angle_period=math.tau,
    torso_link_lengths=(0.4, 0.3),
    arm_rate=0.75,
    max_arm_rate=1.0,
    torso_rate=0.3,
    max_torso_rate=0.5,
    base_speed=0.3,
    yaw_speed=0.6,
)

r1pro_joylo_real = (
    autoconnect(
        R1ProConnection.blueprint(**_declaration.model_dump()),
        R1ProJoylo.blueprint(**_profile.model_dump()),
        R1ProJoyloCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[
                ConnectionConfig(
                    source=_declaration.source,
                    model=_declaration.model,
                    joint_commands=_declaration.joint_commands,
                    deadman_timeout_s=_declaration.deadman_timeout_s,
                )
            ],
            tasks=joylo_tasks(
                {
                    **dict.fromkeys(ARMS, _profile.max_arm_rate),
                    **dict.fromkeys(TORSO, _profile.max_torso_rate),
                }
            ),
        ),
    )
    .remappings(
        [
            (R1ProConnection, "joint_state", "robot_feedback"),
            (R1ProJoyloCoordinator, "r1pro_joint_state", "robot_feedback"),
            (R1ProConnection, "position_command", "r1pro_position_command"),
            (R1ProConnection, "velocity_command", "r1pro_velocity_command"),
            (R1ProJoyloCoordinator, "twist_command", "cmd_vel"),
            (R1ProJoylo, "cmd_vel", "joylo_cmd_vel"),
            (R1ProJoyloCoordinator, "joylo_twist_command", "joylo_cmd_vel"),
        ]
    )
    .transports(_transports)
    .global_config(viewer="none", transport="zenoh", n_workers=3)
)
