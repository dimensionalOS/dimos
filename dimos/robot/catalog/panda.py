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

"""Franka Emika Panda robot configuration."""

from __future__ import annotations

from typing import Any

from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# Panda gripper collision exclusions (parallel jaw gripper)
PANDA_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("hand", "left_finger"),
    ("hand", "right_finger"),
    ("left_finger", "right_finger"),
    ("link7", "hand"),
]


def panda(
    name: str = "panda",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Create a Franka Emika Panda robot configuration (7 DOF).

    Args:
        name: Robot identifier (must contain 'panda' for VAMP auto-detection).
        adapter_type: Hardware adapter ("mock").
        address: Connection address.
        **overrides: Override any RobotConfig field.
    """
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": LfsPath("panda_description") / "urdf/panda.urdf",
        "end_effector_link": "link7",
        "adapter_type": adapter_type,
        "address": address,
        "joint_names": [f"joint{i}" for i in range(1, 8)],
        "base_link": "link0",
        "home_joints": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        "auto_convert_meshes": False,
        "max_velocity": 2.0,
        "max_acceleration": 4.0,
        "collision_exclusion_pairs": PANDA_GRIPPER_COLLISION_EXCLUSIONS,
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = ["PANDA_GRIPPER_COLLISION_EXCLUSIONS", "panda"]
