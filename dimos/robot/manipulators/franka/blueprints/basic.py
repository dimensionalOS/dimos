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

"""Franka Research 3 coordinator and planner blueprints.

Run on the real arm (FCI activated in Desk, Execution mode):

    dimos run fr3-planner-coordinator --frankaconnection.ip 172.16.0.2
"""

from __future__ import annotations

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import movable_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect
from dimos.core.module import ModuleBase
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.franka.config import (
    FR3_FINGER_JOINT,
    fr3_joint_names,
    fr3_model,
    make_fr3_model_config,
)
from dimos.robot.manipulators.franka.connection import FrankaConnection


class FrankaCoordinator(ControlCoordinator):
    """Joint targets out to one FR3 named "arm", and its joints in, through
    ``FrankaConnection``."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]


# The connection module's ports, onto the coordinator's ports for the robot named "arm".
FRANKA_REMAPS: list[tuple[type[ModuleBase], str, str]] = [
    (FrankaConnection, "position_command", "arm_position_command"),
    (FrankaCoordinator, "arm_joint_state", "joint_state"),
]


def franka_coordinator() -> MetaBlueprint:
    """Run planned joint trajectories on an FR3 and open and close its Franka
    Hand, through ``FrankaConnection``.

    The coordinator names the joints ``arm/fr3_joint1`` ... and
    ``arm/fr3_finger_joint1``; a planner driving it uses the model from
    ``make_fr3_model_config(robot="arm")``, whose joints are named the same,
    and ``gripper_tasks={"manipulator": "arm_gripper"}``.

    Wire it up with ``.remappings(FRANKA_REMAPS)``. The arm's FCI address is
    given at run time, after the blueprint name (``--frankaconnection.ip
    172.16.0.2``) or as ``FRANKACONNECTION__IP``.
    """
    model = fr3_model(robot="arm")
    return autoconnect(
        FrankaCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[
                ConnectionConfig(
                    source="arm",
                    model=model,
                    joint_commands={j.name: (Interface.POSITION,) for j in movable_joints(model)},
                )
            ],
            tasks=[
                joint_trajectory_task(fr3_joint_names("arm")),
                # fr3_finger_joint1 is open at 0.04 m.
                TaskConfig(
                    name="arm_gripper",
                    type="gripper",
                    joint_names=[f"arm/{FR3_FINGER_JOINT}"],
                    priority=20,
                    params={"open_at": "upper"},
                ),
            ],
        ),
        FrankaConnection.blueprint(),
    )


def _planner_coordinator() -> MetaBlueprint:
    """Plan and run motions of an FR3 with the Franka Hand. Wire it up with
    ``.remappings(FRANKA_REMAPS)``; see ``franka_coordinator``."""
    return autoconnect(
        planner(
            model=make_fr3_model_config(robot="arm"),
            gripper_tasks={"manipulator": "arm_gripper"},
        ),
        franka_coordinator(),
    )


# The blueprint registry finds a blueprint only by a line ending in a call like `.remappings`.
fr3_planner_coordinator = _planner_coordinator().remappings(FRANKA_REMAPS)
