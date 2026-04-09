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

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

HardwareId = str
JointName = str
TaskName = str


def split_joint_name(joint_name: str) -> tuple[str, str]:
    """Split a coordinator joint name into (hardware_id, suffix).

    Example: "left_arm/joint1" -> ("left_arm", "joint1")
    """
    parts = joint_name.split("/", 1)
    if len(parts) != 2:
        raise ValueError(f"Joint name '{joint_name}' missing separator '/'")
    return parts[0], parts[1]


class HardwareType(Enum):
    MANIPULATOR = "manipulator"
    BASE = "base"


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
        hardware_type: Type of hardware (MANIPULATOR, BASE)
        joints: List of joint names (e.g., ["arm/joint1", "arm/joint2", ...])
        adapter_type: Adapter type ("mock", "xarm", "piper")
        address: Connection address - IP for TCP, port for CAN
        auto_enable: Whether to auto-enable servos
        gripper_joints: Joints that use adapter gripper methods (separate from joints).
    """

    hardware_id: HardwareId
    hardware_type: HardwareType
    joints: list[JointName] = field(default_factory=list)
    adapter_type: str = "mock"
    address: str | None = None
    auto_enable: bool = True
    gripper_joints: list[JointName] = field(default_factory=list)
    adapter_kwargs: dict[str, Any] = field(default_factory=dict)

    @property
    def all_joints(self) -> list[JointName]:
        """All joints: arm joints + gripper joints."""
        return self.joints + self.gripper_joints


def make_gripper_joints(hardware_id: HardwareId) -> list[JointName]:
    """Create gripper joint names for a hardware device.

    Args:
        hardware_id: The hardware identifier (e.g., "arm")

    Returns:
        List of joint names like ["arm/gripper"]
    """
    return [f"{hardware_id}/gripper"]


def make_joints(hardware_id: HardwareId, dof: int) -> list[JointName]:
    """Create joint names for hardware.

    Args:
        hardware_id: The hardware identifier (e.g., "left_arm")
        dof: Degrees of freedom

    Returns:
        List of joint names like ["left_arm/joint1", "left_arm/joint2", ...]
    """
    return [f"{hardware_id}/joint{i + 1}" for i in range(dof)]


# Maps virtual joint suffix → (Twist group, Twist field)
TWIST_SUFFIX_MAP: dict[str, tuple[str, str]] = {
    "vx": ("linear", "x"),
    "vy": ("linear", "y"),
    "vz": ("linear", "z"),
    "wx": ("angular", "x"),
    "wy": ("angular", "y"),
    "wz": ("angular", "z"),
}

_DEFAULT_TWIST_SUFFIXES = ["vx", "vy", "wz"]


def make_twist_base_joints(
    hardware_id: HardwareId,
    suffixes: list[str] | None = None,
) -> list[JointName]:
    """Create virtual joint names for a twist base.

    Args:
        hardware_id: The hardware identifier (e.g., "base")
        suffixes: Velocity DOF suffixes. Defaults to ["vx", "vy", "wz"] (holonomic).

    Returns:
        List of joint names like ["base/vx", "base/vy", "base/wz"]
    """
    suffixes = suffixes or _DEFAULT_TWIST_SUFFIXES
    for s in suffixes:
        if s not in TWIST_SUFFIX_MAP:
            raise ValueError(f"Unknown twist suffix '{s}'. Valid: {list(TWIST_SUFFIX_MAP)}")
    return [f"{hardware_id}/{s}" for s in suffixes]


__all__ = [
    "TWIST_SUFFIX_MAP",
    "HardwareComponent",
    "HardwareId",
    "HardwareType",
    "JointName",
    "JointState",
    "TaskName",
    "make_gripper_joints",
    "make_joints",
    "make_twist_base_joints",
    "split_joint_name",
]
