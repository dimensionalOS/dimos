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

"""Agilex Piper robot configuration."""

from __future__ import annotations

from typing import Any

from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# Piper gripper collision exclusions (parallel jaw gripper)
# The gripper fingers (link7, link8) can touch each other and gripper_base
PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]


def piper(
    name: str = "piper",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    y_offset: float = 0.0,
    **overrides: Any,
) -> RobotConfig:
    """Create a Piper robot configuration.

    Piper has 6 revolute joints (joint1-joint6) for the arm and 2 prismatic
    joints (joint7, joint8) for the parallel jaw gripper.

    Args:
        name: Robot identifier.
        adapter_type: Hardware adapter ("mock", "piper").
        address: CAN port (e.g., "can0").
        y_offset: Y-axis offset for base pose (multi-arm setups).
        **overrides: Override any RobotConfig field.
    """
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": LfsPath("piper_description") / "urdf/piper_description.xacro",
        "end_effector_link": "gripper_base",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": [f"joint{i}" for i in range(1, 7)],
        "base_link": "arm_base",
        "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "base_pose": [0, y_offset, 0, 0, 0, 0, 1],
        "package_paths": {
            "piper_description": LfsPath("piper_description"),
            "piper_gazebo": LfsPath("piper_description"),
        },
        "xacro_args": {},
        "auto_convert_meshes": True,
        "collision_exclusion_pairs": PIPER_GRIPPER_COLLISION_EXCLUSIONS,
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = ["PIPER_GRIPPER_COLLISION_EXCLUSIONS", "piper"]
