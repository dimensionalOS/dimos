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

"""The main Metal coordinator entry, using its direct connection."""

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.robot.manipulators.damiao.damiao_blueprint import connect_damiao
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection

coordinator_makermods_metal = connect_damiao(
    MetalConnection,
    tasks=[
        joint_trajectory_task([f"arm/{name}" for name in MetalConnection.planning.joint_names]),
        TaskConfig(
            name="metal_gripper",
            type="gripper",
            joint_names=["arm/gripper"],
            priority=20,
        ),
    ],
).remappings([])
