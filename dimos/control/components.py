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

"""Hardware component schema for the ControlCoordnator."""

from dataclasses import dataclass, field
from enum import Enum

HardwareId = str
JointName = str


class HardwareType(Enum):
    MANIPULATOR = "manipulator"
    BASE = "base"
    GRIPPER = "gripper"


@dataclass(frozen=True)
class JointState:
    """State of a single joint."""

    position: float
    velocity: float
    effort: float


@dataclass
class HardwareComponent:
    """Configuration for a hardware component.

    Attributes:
        hardware_id: Unique identifier, also used as joint name prefix
        hardware_type: Type of hardware (MANIPULATOR, BASE, GRIPPER)
        joints: List of joint names (e.g., ["arm_joint1", "arm_joint2", ...])
        backend_type: Backend type ("mock", "xarm", "piper")
        address: Connection address - IP for TCP, port for CAN
        auto_enable: Whether to auto-enable servos
    """

    hardware_id: HardwareId
    hardware_type: HardwareType
    joints: list[JointName] = field(default_factory=list)
    backend_type: str = "mock"
    address: str | None = None
    auto_enable: bool = True


def make_joints(hardware_id: HardwareId, dof: int) -> list[JointName]:
    """Create joint names for hardware.

    Args:
        hardware_id: The hardware identifier (e.g., "left_arm")
        dof: Degrees of freedom

    Returns:
        List of joint names like ["left_arm_joint1", "left_arm_joint2", ...]
    """
    return [f"{hardware_id}_joint{i + 1}" for i in range(dof)]


__all__ = [
    "HardwareComponent",
    "HardwareId",
    "HardwareType",
    "JointName",
    "JointState",
    "make_joints",
]
