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
from pathlib import Path
from typing import Any

from dimos.hardware.whole_body.spec import WholeBodyConfig

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
    WHOLE_BODY = "whole_body"


@dataclass(frozen=True)
class JointState:
    """State of a single joint."""

    position: float
    velocity: float
    effort: float


@dataclass
class HardwareComponent:
    """Configuration for a hardware component.

    ``all_joints`` is the stored list and is the adapter's array order;
    ``arm_joints`` and ``gripper_joints`` are derived views.

    Attributes:
        hardware_id: Unique identifier, also used as joint name prefix
        hardware_type: Type of hardware (MANIPULATOR, BASE)
        all_joints: Every joint name in adapter order, gripper entries last
            (e.g., ["arm/joint1", ..., "arm/joint6", "arm/gripper"])
        gripper_dof: How many trailing entries of ``all_joints`` are the
            gripper. 0 means no gripper.
        adapter_type: Adapter type ("mock", "xarm", "piper")
        address: Connection address - IP for TCP, port for CAN
        auto_enable: Whether to auto-enable servos
        domain_id: DDS domain ID for adapters that use DDS transport
            (e.g. Unitree G1). Real robot uses 0; unitree_mujoco sim
            defaults to 1. Ignored by non-DDS adapters.
        adapter_kwargs: Generic untyped kwargs forwarded to the adapter
            constructor — use for adapter-specific knobs that don't
            belong in the spec.
        wb_config: Whole-body-specific config (PD gains etc.).  Populate
            on hardware_type=WHOLE_BODY components.  Keeps WB-only knobs
            off the generic HardwareComponent shared by manipulators,
            bases, and grippers.
    """

    hardware_id: HardwareId
    hardware_type: HardwareType
    all_joints: list[JointName] = field(default_factory=list)
    gripper_dof: int = 0
    adapter_type: str = "mock"
    address: str | Path | None = None
    auto_enable: bool = True
    domain_id: int = 0
    adapter_kwargs: dict[str, Any] = field(default_factory=dict)
    wb_config: WholeBodyConfig | None = None

    def __post_init__(self) -> None:
        if self.gripper_dof < 0:
            raise ValueError(
                f"HardwareComponent '{self.hardware_id}': gripper_dof must be >= 0, "
                f"got {self.gripper_dof}"
            )
        if self.gripper_dof > len(self.all_joints):
            raise ValueError(
                f"HardwareComponent '{self.hardware_id}': gripper_dof {self.gripper_dof} "
                f"exceeds all_joints length {len(self.all_joints)}"
            )

    @property
    def _gripper_start(self) -> int:
        # len - gripper_dof, never negative slicing: -0 == 0 would report a
        # gripper-less arm as having no arm joints.
        return len(self.all_joints) - self.gripper_dof

    @property
    def arm_joints(self) -> list[JointName]:
        """The non-gripper joints, in adapter order."""
        return self.all_joints[: self._gripper_start]

    @property
    def gripper_joints(self) -> list[JointName]:
        """The trailing gripper joints, in adapter order. Empty without a gripper."""
        return self.all_joints[self._gripper_start :]


def make_gripper_joints(hardware_id: HardwareId, count: int = 1) -> list[JointName]:
    """Create gripper joint names for a hardware device.

    Args:
        hardware_id: The hardware identifier (e.g., "arm")
        count: Number of actuated gripper joints

    Returns:
        ``["arm/gripper"]``, or ``["hand/gripper1", ..., "hand/gripperN"]``
    """
    if count < 1:
        raise ValueError(f"make_gripper_joints({hardware_id!r}): count must be >= 1, got {count}")
    if count == 1:
        return [f"{hardware_id}/gripper"]
    return [f"{hardware_id}/gripper{i + 1}" for i in range(count)]


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


_HUMANOID_29DOF_JOINTS = [
    # Left leg (0-5)
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    # Right leg (6-11)
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    # Waist (12-14)
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    # Left arm (15-21)
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    # Right arm (22-28)
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]


def make_humanoid_joints(hardware_id: HardwareId) -> list[JointName]:
    """Create joint names for a 29-DOF humanoid.

    Covers 6-DOF legs, 3-DOF waist, and 7-DOF arms.

    Args:
        hardware_id: The hardware identifier (e.g., "g1")

    Returns:
        List of 29 joint names like ["g1/left_hip_pitch", ..., "g1/right_wrist_yaw"]
    """
    return [f"{hardware_id}/{j}" for j in _HUMANOID_29DOF_JOINTS]
