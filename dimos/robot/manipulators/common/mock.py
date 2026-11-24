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

"""Mock manipulator coordinator blueprints."""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import (
    MockJointConnection,
    MockJointConnectionConfig,
    mock_arm_model,
)
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState


def mock_arm(robot: str, joints: int) -> tuple[ConnectionConfig, Blueprint]:
    """A mock arm named ``robot`` with ``joints`` turning joints, as the coordinator's
    declaration and its connection, wired onto ports ``<robot>_position_command``
    and ``<robot>_joint_state``."""
    declaration = MockJointConnectionConfig(source=robot, model=mock_arm_model(joints))
    connection = MockJointConnection.blueprint(
        **{**declaration.model_dump(), "instance_name": robot}
    ).remappings(
        [
            (robot, "position_command", f"{robot}_position_command"),
            (robot, "joint_state", f"{robot}_joint_state"),
        ]
    )
    return (
        ConnectionConfig(
            source=robot, model=declaration.model, joint_commands=declaration.joint_commands
        ),
        connection,
    )


class _MockCoordinator(ControlCoordinator):
    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


_arm, _arm_connection = mock_arm("arm", 7)

coordinator_mock = autoconnect(
    _MockCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_arm],
        tasks=[joint_trajectory_task([f"arm/joint{i + 1}" for i in range(7)])],
    ),
    _arm_connection,
)


class _DualMockCoordinator(ControlCoordinator):
    left_arm_position_command: Out[JointState]
    left_arm_joint_state: In[JointState]
    right_arm_position_command: Out[JointState]
    right_arm_joint_state: In[JointState]


_left, _left_connection = mock_arm("left_arm", 7)
_right, _right_connection = mock_arm("right_arm", 6)

coordinator_dual_mock = autoconnect(
    _DualMockCoordinator.blueprint(
        instance_name="ControlCoordinator",
        connections=[_left, _right],
        tasks=[
            joint_trajectory_task(
                [f"left_arm/joint{i + 1}" for i in range(7)]
                + [f"right_arm/joint{i + 1}" for i in range(6)]
            ),
        ],
    ),
    _left_connection,
    _right_connection,
)
