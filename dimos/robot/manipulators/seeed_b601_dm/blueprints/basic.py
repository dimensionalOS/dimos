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

Mock hardware by default. Pass ``--can-port`` with the Damiao USB-CAN
bridge's serial port or a SocketCAN interface to drive the real arm.
"""

from __future__ import annotations

from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.seeed_b601_dm.config import (
    b601_dm_gripper_task,
    b601_dm_model_config,
)
from dimos.robot.manipulators.seeed_b601_dm.coordinator import SeeedB601DMCoordinator
from dimos.robot.manipulators.seeed_b601_dm.joints import B601_DM_ARM_JOINTS

seeed_b601_dm_planner_coordinator = autoconnect(
    planner(
        gripper_tasks={"manipulator": "b601_dm_gripper"},
        model=b601_dm_model_config(),
        visualization={"backend": "viser"},
        trajectory_parametrization={
            "backend": "simple_trapezoid",
        },
    ),
    SeeedB601DMCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[joint_trajectory_task(list(B601_DM_ARM_JOINTS)), b601_dm_gripper_task()],
    ),
)

coordinator_seeed_b601_dm = SeeedB601DMCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tasks=[joint_trajectory_task(list(B601_DM_ARM_JOINTS)), b601_dm_gripper_task()],
)
