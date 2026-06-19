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

"""Teleop blueprints for testing and deployment.

Single sim/real blueprints — pass `--simulation` to run inside MuJoCo, omit for real
hardware. The underlying coordinator blueprints branch on `global_config.simulation`.
"""

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.control.blueprints.teleop import (
    coordinator_teleop_dual,
    coordinator_teleop_piper,
    coordinator_teleop_xarm6,
    coordinator_teleop_xarm7,
)
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.quest.quest_extensions import (
    ArmTeleopModule,
    Go2TeleopModule,
    VideoArmTeleopModule,
)
from dimos.teleop.quest.quest_types import Buttons
from dimos.visualization.vis_module import vis_module

# Arm teleop with press-and-hold engage (has rerun viz)
teleop_quest_rerun = autoconnect(
    ArmTeleopModule.blueprint(),
    vis_module("rerun"),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport.spec(
            "/teleop/left_delta", PoseStamped
        ),
        ("right_controller_output", PoseStamped): LCMTransport.spec(
            "/teleop/right_delta", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
    }
)


# XArm7 teleop (sim with --simulation, real otherwise): right controller -> xarm7
teleop_quest_xarm7 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm7,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
    }
)


# XArm7 teleop + camera streaming into the Quest scene as a panel.
teleop_quest_xarm7_video = autoconnect(
    VideoArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm7,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
        ("color_image", Image): LCMTransport.spec("/teleop/color_image", Image),
    }
)


# Piper teleop (sim with --simulation, real otherwise): left controller -> piper arm
teleop_quest_piper = autoconnect(
    ArmTeleopModule.blueprint(task_names={"left": "teleop_piper"}),
    coordinator_teleop_piper,
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
    }
)


# XArm6 teleop (sim with --simulation, real otherwise): right controller -> xarm6
teleop_quest_xarm6 = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm6,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
    }
)


# Dual arm teleop: right -> piper, left -> xarm6 (TeleopIK, real-only)
teleop_quest_dual = autoconnect(
    ArmTeleopModule.blueprint(task_names={"right": "teleop_piper", "left": "teleop_xarm"}),
    coordinator_teleop_dual,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("left_controller_output", PoseStamped): LCMTransport.spec(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport.spec("/teleop/buttons", Buttons),
    }
)


# Go2 quadruped: thumbstick velocity teleop + camera streamed to the headset.
teleop_quest_go2 = (
    autoconnect(
        Go2TeleopModule.blueprint(),
        GO2Connection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport.spec("/cmd_vel", Twist),
            ("color_image", Image): pSHMTransport.spec(
                "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    )
    .global_config(robot_model="unitree_go2")
)


__all__ = [
    "teleop_quest_dual",
    "teleop_quest_go2",
    "teleop_quest_piper",
    "teleop_quest_rerun",
    "teleop_quest_xarm6",
    "teleop_quest_xarm7",
    "teleop_quest_xarm7_video",
]
