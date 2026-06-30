# Copyright 2025-2026 Dimensional Inc.
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

"""Shared Damiao actuator/runtime adapters."""

from dimos.hardware.damiao.arm_adapter import DamiaoArmAdapter
from dimos.hardware.damiao.runtime import DamiaoBindingUnavailableError, DamiaoRobotRuntime
from dimos.hardware.damiao.specs import (
    DamiaoArmSpec,
    DamiaoBusSpec,
    DamiaoJointGroupSpec,
    DamiaoMotorSpec,
    DamiaoRobotSpec,
)
from dimos.hardware.damiao.whole_body_adapter import DamiaoWholeBodyAdapter

__all__ = [
    "DamiaoArmAdapter",
    "DamiaoArmSpec",
    "DamiaoBindingUnavailableError",
    "DamiaoBusSpec",
    "DamiaoJointGroupSpec",
    "DamiaoMotorSpec",
    "DamiaoRobotRuntime",
    "DamiaoRobotSpec",
    "DamiaoWholeBodyAdapter",
]
