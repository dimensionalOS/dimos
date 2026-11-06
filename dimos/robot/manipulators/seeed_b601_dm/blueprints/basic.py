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

"""Basic Seeed B601-DM coordinator and planner blueprints.

The arm runs on its direct Damiao connection: ``--can-port`` takes the Damiao
USB-CAN bridge's serial port or a SocketCAN interface. ``--simulation``
composes a ``MockJointConnection`` instead and never opens a bus.
"""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.damiao.damiao_blueprint import connect_damiao
from dimos.robot.manipulators.seeed_b601_dm.config import b601_dm_gripper_task
from dimos.robot.manipulators.seeed_b601_dm.connection import B601DMConnection
from dimos.robot.manipulators.seeed_b601_dm.coordinator import SeeedB601DMCoordinator
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS

B601_DM_PLANNING_MODEL = B601DMConnection.planning_model("arm")


def _tasks() -> list[TaskConfig]:
    return [
        joint_trajectory_task([f"arm/{name}" for name in B601_DM_ARM_JOINTS]),
        b601_dm_gripper_task(),
    ]


def seeed_b601_dm_control() -> Blueprint:
    """Trajectories and normalized gripper control over the direct connection."""
    return connect_damiao(
        B601DMConnection,
        coordinator=SeeedB601DMCoordinator,
        tasks=_tasks(),
    ).remappings([])


seeed_b601_dm_planner_coordinator = autoconnect(
    planner(
        gripper_tasks={"manipulator": "b601_dm_gripper"},
        model=B601_DM_PLANNING_MODEL,
        visualization={"backend": "viser"},
        trajectory_parametrization={
            "backend": "simple_trapezoid",
        },
    ),
    connect_damiao(
        B601DMConnection,
        coordinator=SeeedB601DMCoordinator,
        tasks=_tasks(),
    ),
)

coordinator_seeed_b601_dm = connect_damiao(
    B601DMConnection,
    coordinator=SeeedB601DMCoordinator,
    tasks=_tasks(),
).remappings([])
