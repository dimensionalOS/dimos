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

"""B601-DM coordinator with arm trajectory and normalized gripper tasks."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.robot.manipulators.seeed_b601_dm.config import (
    B601_DM_ARM_JOINTS,
    B601_DM_GRIPPER_JOINT,
    B601_DM_HARDWARE_ID,
)
from dimos.robot.manipulators.seeed_b601_dm.coordinator import B601DMCoordinator

coordinator_seeed_b601_dm = B601DMCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tasks=[
        joint_trajectory_task(list(B601_DM_ARM_JOINTS)),
        TaskConfig(
            name=f"{B601_DM_HARDWARE_ID}_gripper",
            type="gripper",
            joint_names=[B601_DM_GRIPPER_JOINT],
            priority=20,
        ),
    ],
)
