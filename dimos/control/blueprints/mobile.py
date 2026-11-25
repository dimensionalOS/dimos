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

"""Mobile manipulation coordinator blueprints.

Usage:
    dimos run coordinator-mock-twist-base                # Mock holonomic base
    dimos run coordinator-mobile-manip-mock              # Mock arm + base
    dimos run coordinator-flowbase                       # FlowBase holonomic base (Portal RPC)
    dimos run coordinator-flowbase-keyboard-teleop       # FlowBase + WASD pygame teleop
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import MockJointConnection, mock_base_model
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.flowbase.connection import (
    FLOWBASE_BASE_JOINTS,
    FlowBaseConnection,
    flowbase_joint_connection,
)
from dimos.robot.manipulators.common.mock import mock_arm
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_MOCK_BASE = ConnectionConfig(
    source="base",
    model=mock_base_model("base"),
    joint_commands={
        f"base/base_{axis}": (Interface.VELOCITY,) for axis in ("forward", "lateral", "turn")
    },
)
_mock_base_connection = MockJointConnection.blueprint(
    **{**_MOCK_BASE.model_dump(), "instance_name": "base"}
).remappings(
    [
        ("base", "velocity_command", "base_velocity_command"),
        ("base", "joint_state", "base_joint_state"),
    ]
)
_vel_base = TaskConfig(
    name="vel_base",
    type="twist_velocity",
    joint_names=list(_MOCK_BASE.joint_commands),
    params={"axes": ["linear.x", "linear.y", "angular.z"]},
    priority=10,
)


class FlowBaseCoordinator(ControlCoordinator):
    """Body speeds out to one FlowBase named "base", and its readings in,
    through ``FlowBaseConnection``."""

    base_velocity_command: Out[JointState]
    base_joint_state: In[JointState]


# The connection module's ports onto the coordinator's, and Twist in on cmd_vel.
# The coordinator owns cmd_vel, so the connection's own Twist input is moved off it.
_FLOWBASE_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (FlowBaseConnection, "cmd_vel", "base_cmd_vel"),
    (FlowBaseConnection, "velocity_command", "base_velocity_command"),
    (FlowBaseCoordinator, "base_joint_state", "joint_state"),
    (FlowBaseCoordinator, "twist_command", "cmd_vel"),
]


def _flowbase_coordinator() -> Blueprint:
    """A FlowBase behind a coordinator, driven by Twist on ``cmd_vel``."""
    return autoconnect(
        FlowBaseCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[flowbase_joint_connection()],
            tasks=[
                TaskConfig(
                    name="vel_base",
                    type="twist_velocity",
                    joint_names=list(FLOWBASE_BASE_JOINTS),
                    params={"axes": ["linear.x", "linear.y", "angular.z"]},
                    priority=10,
                ),
            ],
        ),
        FlowBaseConnection.blueprint(),
    )


class _MockBaseCoordinator(ControlCoordinator):
    base_velocity_command: Out[JointState]
    base_joint_state: In[JointState]


# Mock holonomic base: forward, lateral and turn speeds, driven by Twist on cmd_vel
coordinator_mock_twist_base = autoconnect(
    _MockBaseCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_MOCK_BASE],
        tasks=[_vel_base],
    ),
    _mock_base_connection,
).remappings([(_MockBaseCoordinator, "twist_command", "cmd_vel")])

# FlowBase holonomic base over Portal RPC
coordinator_flowbase = _flowbase_coordinator().remappings(_FLOWBASE_REMAPS)

# FlowBase + WASD pygame keyboard teleop. Shift boosts to 0.8 m/s, the base's top speed.
coordinator_flowbase_keyboard_teleop = autoconnect(
    _flowbase_coordinator(),
    KeyboardTeleop.blueprint(boost_multiplier=1.6),
).remappings(_FLOWBASE_REMAPS)


class _MockMobileManipCoordinator(_MockBaseCoordinator):
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


_arm, _arm_connection = mock_arm("arm", 7)

# Mock 7-joint arm + mock holonomic base
coordinator_mobile_manip_mock = autoconnect(
    _MockMobileManipCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_arm, _MOCK_BASE],
        tasks=[joint_trajectory_task([f"arm/joint{i + 1}" for i in range(7)]), _vel_base],
    ),
    _arm_connection,
    _mock_base_connection,
).remappings([(_MockMobileManipCoordinator, "twist_command", "cmd_vel")])
