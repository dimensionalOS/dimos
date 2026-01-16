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

"""Hardware component schema for the ControlCoordinator."""

from dataclasses import dataclass
from enum import Enum

from dimos.hardware.manipulators.spec import ControlMode

HardwareId = str
JointName = str
TaskName = str


class HardwareType(Enum):
    MANIPULATOR = "manipulator"
    BASE = "base"
    GRIPPER = "gripper"


class JointType(Enum):
    REVOLUTE = "revolute"  # Rotary with limits (radians)
    PRISMATIC = "prismatic"  # Linear with limits (meters)
    CONTINUOUS = "continuous"  # Rotary no limits (wheels)
    VELOCITY = "velocity"  # Velocity-only (base vx/vy/wz)


@dataclass(frozen=True)
class JointConfig:
    joint_name: JointName
    joint_type: JointType
    supported_modes: tuple[ControlMode, ...]
    limits: tuple[float, float] | None = None
    default_on_timeout: float | None = None  # None=hold, 0.0=zero velocity


@dataclass
class HardwareComponent:
    """Configuration for a hardware component.

    Attributes:
        hardware_id: Unique hardware identifier (e.g., "arm", "left_arm")
        hardware_type: Type of hardware (MANIPULATOR, BASE, GRIPPER)
        joints: List of joint names (e.g., ["arm_joint1", "arm_joint2", ...])
        adapter_type: Adapter type ("mock", "xarm", "piper")
        address: Connection address - IP for TCP, port for CAN
        auto_enable: Whether to auto-enable servos
    """

    hardware_id: HardwareId
    hardware_type: HardwareType
    joints: list[JointName] = field(default_factory=list)
    adapter_type: str = "mock"
    address: str | None = None
    auto_enable: bool = True
    description: str = ""


def make_joints(
    hardware_id: HardwareId,
    dof: int,
    joint_type: JointType = JointType.REVOLUTE,
    supported_modes: tuple[ControlMode, ...] = (ControlMode.POSITION, ControlMode.SERVO_POSITION),
) -> list[JointConfig]:
    """Create joint configs for hardware.

    Args:
        hardware_id: The hardware identifier (e.g., "left_arm")
        dof: Degrees of freedom
        joint_type: Type of joints (default: REVOLUTE)
        supported_modes: Control modes the joints support

    Returns:
        List of JointConfig with names like "left_arm_joint1", "left_arm_joint2", ...
    """
    return [
        JointConfig(
            joint_name=f"{hardware_id}_joint{i + 1}",
            joint_type=joint_type,
            supported_modes=supported_modes,
        )
        for i in range(dof)
    ]


__all__ = [
    "HardwareComponent",
    "HardwareId",
    "HardwareType",
    "JointConfig",
    "JointName",
    "JointState",
    "TaskName",
    "make_joints",
]
