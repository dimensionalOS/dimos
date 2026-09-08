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

"""A-750 local serial connection configuration."""

from dataclasses import replace

from dimos.control.components import HardwareComponent
from dimos.robot.manipulators.a750.config import a750_hardware
from dimos.robot.manipulators.common.connection import SingleArmCoordinator


class A750Coordinator(SingleArmCoordinator):
    def _resolve_hardware(self, component: HardwareComponent) -> HardwareComponent:
        resolved = a750_hardware(address=self.config.address)
        return replace(
            component,
            adapter_type=resolved.adapter_type,
            address=resolved.address,
            limits=component.limits if resolved.adapter_type == "mock" else None,
        )
