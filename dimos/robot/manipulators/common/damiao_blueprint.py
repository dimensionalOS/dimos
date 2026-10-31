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

"""Compose direct Damiao connections with stream-only control coordinators."""

from collections.abc import Sequence

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.common.coordinators import ArmTwistCoordinator
from dimos.robot.manipulators.common.damiao_connection import DamiaoConnection
from dimos.robot.urdf_manager import UrdfManager


class DamiaoCoordinator(ControlCoordinator):
    """One connection named arm, including a robot with two synchronized arms."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTwistCoordinator(ArmTwistCoordinator):
    """End-effector twists and gripper inputs, with joint commands over streams."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]


class DamiaoTeleopCoordinator(TeleopControlCoordinator):
    """WebXR inputs with a direct connection owning the robot."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


def connect_damiao(
    connection: type[DamiaoConnection],
    *,
    coordinator: type[ControlCoordinator] = DamiaoCoordinator,
    tasks: Sequence[TaskConfig],
) -> Blueprint:
    """Wire one connection and coordinator using the same deployed model."""
    return autoconnect(
        coordinator.blueprint(
            instance_name="ControlCoordinator",
            model=connection.models().control,
            tasks=list(tasks),
        ),
        connection.blueprint(source="arm"),
    ).remappings(
        [
            (connection, "position_command", "arm_position_command"),
            (coordinator, "arm_joint_state", "joint_state"),
        ]
    )


def damiao_connection_control(connection: type[DamiaoConnection]) -> Blueprint:
    """Trajectories and normalized grippers with the robot model served to clients."""
    deployed = connection.models()
    sides = ("left", "right") if len(deployed.gripper_joints) == 2 else ("right",)
    tasks = [
        joint_trajectory_task(deployed.joints),
        *[
            TaskConfig(
                name=f"arm_{joint.rsplit('/', 1)[1]}",
                type="gripper",
                joint_names=[joint],
                priority=20,
                stream_bind={"gripper_command": f"{side}_gripper_command"},
            )
            for joint, side in zip(deployed.gripper_joints, sides, strict=True)
        ],
    ]
    return autoconnect(
        connect_damiao(connection, tasks=tasks),
        UrdfManager.blueprint(model=deployed.control),
    )
