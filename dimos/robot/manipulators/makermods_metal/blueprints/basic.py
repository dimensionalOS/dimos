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

"""Metal Arm coordinator with arm trajectory and normalized gripper tasks."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_GRIPPER_JOINT,
    METAL_HARDWARE_ID,
)
from dimos.robot.manipulators.makermods_metal.coordinator import MetalCoordinator

coordinator_makermods_metal = MetalCoordinator.blueprint(
    instance_name="ControlCoordinator",
    tasks=[
        joint_trajectory_task(list(METAL_ARM_JOINTS)),
        TaskConfig(
            name=f"{METAL_HARDWARE_ID}_gripper",
            type="gripper",
            joint_names=[METAL_GRIPPER_JOINT],
            priority=20,
        ),
    ],
)
