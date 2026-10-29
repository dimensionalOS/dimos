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

"""Dual-CAN Metal deployment configuration."""

from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.stream import In
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.dual_makermods_metal.config import dual_metal_hardware


class DualMetalCoordinatorConfig(ControlCoordinatorConfig):
    left_can_port: str | None = None
    right_can_port: str | None = None


class DualMetalCoordinator(ControlCoordinator):
    config: DualMetalCoordinatorConfig
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]

    def _setup_from_config(self) -> None:
        self.config.hardware = dual_metal_hardware(
            left_can_port=self.config.left_can_port,
            right_can_port=self.config.right_can_port,
        )
        super()._setup_from_config()
