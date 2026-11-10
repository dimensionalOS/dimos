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

"""Coordinator types and explicit composition helpers for Damiao deployments.

Blueprints wire a coordinator and a connection themselves with ``autoconnect``,
exactly like the xArm ones. The only shared helper is the declaration both must
agree on: the simulation swap. The connection is driven directly, so there is
no command stream to wire between them.
"""

from __future__ import annotations

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.damiao.robot import DamiaoRobot


class DamiaoCoordinator(ControlCoordinator):
    """One connection named arm, including a robot with two synchronized arms."""

    hosted_connections = True
    #: Declares the position interface to ConnectionSource. A hosted cycle
    #: writes the connection directly and does not publish commands here.
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper inputs, with joint commands over streams."""

    hosted_connections = True
    #: Declares the position interface to ConnectionSource. A hosted cycle
    #: writes the connection directly and does not publish commands here.
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTeleopCoordinator(TeleopControlCoordinator):
    """WebXR inputs with a direct connection owning the robot."""

    hosted_connections = True
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def mock_initial_positions(robot: DamiaoRobot) -> dict[str, float]:
    """The simulated start: the robot's home posture with its jaws closed.

    Loads the robot's model, to find where each jaw is closed.
    """
    home = robot.home_joints or [0.0] * len(robot.arm_joint_names)
    travel = robot.gripper_travel()
    closed = [travel[joint][0] for joint in robot.gripper_joint_names]
    return dict(zip(robot.joint_names, [*home, *closed], strict=True))


def arm_connection(connection: type[DamiaoConnection], *, teach: bool = False) -> Blueprint:
    """The deployed connection, or an independent mock when ``--simulation`` is on.

    Blueprints call this at the point they compose their modules, so the
    simulation swap is visible next to the coordinator it feeds instead of
    hiding inside a general-purpose composer.
    """
    if global_config.simulation:
        return MockJointConnection.blueprint(
            source="arm",
            model=connection.robot.model,
            initial_positions=mock_initial_positions(connection.robot),
            hosted=True,
        )
    return connection.blueprint(source="arm", teach=teach, hosted=True)


def arm_remaps(
    connection: type[DamiaoConnection],
    coordinator: type[ControlCoordinator],
) -> list[tuple[type[ModuleBase], str, str]]:
    """Wire the coordinator's joint-state input to its connection's readings.

    The mock that ``arm_connection`` substitutes publishes the same port, so
    the remap does not need to name a different module while simulating. There
    is no command stream: the hosted cycle calls the connection directly.
    """
    del connection
    return [(coordinator, "arm_joint_state", "joint_state")]
