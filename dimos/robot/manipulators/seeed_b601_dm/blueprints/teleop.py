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

"""Seeed B601-DM keyboard jog and planner over its direct connection.

``--can-port`` takes the Damiao USB-CAN bridge's serial port or a SocketCAN
interface. ``--simulation`` composes a ``MockJointConnection`` instead and
never opens a bus.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.common.topics import EEF_TWIST_TASK_NAME
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTwistCoordinator,
    arm_connection,
    arm_declaration,
    arm_remaps,
)
from dimos.robot.manipulators.seeed_b601_dm.blueprints.basic import B601_DM_PLANNING_MODEL
from dimos.robot.manipulators.seeed_b601_dm.config import b601_dm_gripper_task
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(B601_DM_PLANNING_MODEL.joint_names),
        priority=priority,
        params={"robot_model": B601_DM_PLANNING_MODEL, "target_frame": "end_link"},
    )


# The jog (priority 20) preempts a planned trajectory (10) while a key is held.
keyboard_teleop_seeed_b601_dm_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(
        model=B601_DM_PLANNING_MODEL,
        gripper_tasks={"manipulator": "b601_dm_gripper"},
        visualization={"backend": "viser"},
        trajectory_parametrization={"backend": "simple_trapezoid"},
    ),
    autoconnect(
        DamiaoTwistCoordinator.blueprint(
            instance_name="ControlCoordinator",
            connections=[arm_declaration(B601DMConnection)],
            tasks=[
                _eef_twist_task(priority=20),
                joint_trajectory_task([f"arm/{name}" for name in B601_DM_ARM_JOINTS], priority=10),
                b601_dm_gripper_task(),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(B601DMConnection),
    ).remappings(arm_remaps(B601DMConnection, DamiaoTwistCoordinator)),
)
