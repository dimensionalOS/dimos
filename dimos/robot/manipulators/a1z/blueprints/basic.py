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

"""Basic Galaxea A1Z coordinator and planner blueprints, through ``A1ZConnection``."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators._modeling import joint_names
from dimos.robot.manipulators.a1z.config import A1Z_DOF, a1z_model, make_a1z_model_config
from dimos.robot.manipulators.a1z.connection import A1ZConnection
from dimos.robot.manipulators.common.blueprints import planner


class A1ZCoordinator(ControlCoordinator):
    """Joint targets out to one A1Z named "arm", and its joints in, through
    ``A1ZConnection``."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
A1Z_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (A1ZConnection, "position_command", "arm_position_command"),
    (A1ZCoordinator, "arm_joint_state", "joint_state"),
]


def a1z_coordinator() -> Blueprint:
    """Run planned joint trajectories on an A1Z and open and close its gripper,
    through ``A1ZConnection``.

    The coordinator names the joints ``arm/arm_joint1`` ... and ``arm/gripper``;
    a planner driving it uses ``make_a1z_model_config(robot="arm")`` and
    ``gripper_tasks={"manipulator": "arm_gripper"}``. Wire it up with
    ``.remappings(A1Z_REMAPS)``. The CAN port is given at run time, after the
    blueprint name (``--a1zconnection.can-port a1zcan``) or as
    ``A1ZCONNECTION__CAN_PORT``.
    """
    return autoconnect(
        A1ZCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=a1z_model(robot="arm"),
            tasks=[
                joint_trajectory_task(joint_names(A1Z_DOF, prefix="arm/arm_joint")),
                # arm/gripper is open at its upper end, the task's default.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=["arm/gripper"],
                    priority=20,
                ),
            ],
        ),
        A1ZConnection.blueprint(),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
coordinator_a1z = a1z_coordinator().remappings(A1Z_REMAPS)
a1z_planner_coordinator = autoconnect(
    planner(model=make_a1z_model_config(robot="arm"), gripper_tasks={"manipulator": "arm_gripper"}),
    a1z_coordinator(),
).remappings(A1Z_REMAPS)
