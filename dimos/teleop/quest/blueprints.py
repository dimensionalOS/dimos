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

"""Teleop blueprints for testing and deployment."""

from dimos.control.blueprints import (
    coordinator_teleop_dual,
    coordinator_teleop_piper,
    coordinator_teleop_xarm6,
)
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.teleop.quest.quest_extensions import arm_teleop_module, visualizing_teleop_module
from dimos.teleop.quest.quest_types import Buttons

# -----------------------------------------------------------------------------
# Quest Teleop Blueprints
# -----------------------------------------------------------------------------

# Arm teleop with press-and-hold engage
arm_teleop = autoconnect(
    arm_teleop_module(),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)

# Arm teleop with Rerun visualization
arm_teleop_visualizing = autoconnect(
    visualizing_teleop_module(),
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport("/teleop/left_delta", PoseStamped),
        ("right_controller_output", PoseStamped): LCMTransport("/teleop/right_delta", PoseStamped),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# -----------------------------------------------------------------------------
# Teleop wired to Coordinator (TeleopIK)
# -----------------------------------------------------------------------------

# Single XArm6 teleop: right controller -> xarm6
# Usage: dimos run arm-teleop-xarm6

arm_teleop_xarm6 = autoconnect(
    arm_teleop_module(task_names={"right": "teleop_xarm"}),
    coordinator_teleop_xarm6,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# Single Piper teleop: left controller -> piper arm
# Usage: dimos run arm-teleop-piper
arm_teleop_piper = autoconnect(
    arm_teleop_module(task_names={"left": "teleop_piper"}),
    coordinator_teleop_piper,
).transports(
    {
        ("left_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


# Dual arm teleop: right -> piper, left -> xarm6 (TeleopIK)
arm_teleop_dual = autoconnect(
    arm_teleop_module(task_names={"right": "teleop_piper", "left": "teleop_xarm"}),
    coordinator_teleop_dual,
).transports(
    {
        ("right_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("left_controller_output", PoseStamped): LCMTransport(
            "/coordinator/cartesian_command", PoseStamped
        ),
        ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
    }
)


__all__ = [
    "arm_teleop",
    "arm_teleop_dual",
    "arm_teleop_piper",
    "arm_teleop_visualizing",
    "arm_teleop_xarm6",
]
