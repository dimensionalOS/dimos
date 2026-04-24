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

# Copyright 2025-2026 Dimensional Inc.
# Copyright 2026 Aleksandr Dobkin
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

"""Agilex A-750 robot configuration."""

from __future__ import annotations

import math
from typing import Any

from dimos.robot.config import GripperConfig, RobotConfig
from dimos.utils.data import LfsPath

# Pre-built MJCF for Pinocchio FK (xacro not supported by Pinocchio)
A750_FK_MODEL = LfsPath("a750_description/urdf/a750_rev1_no_gripper.urdf")

# A-750 gripper collision exclusions (parallel jaw gripper)
# The gripper fingers (link7, link8) can touch each other and gripper_base
# from a750_moveit_config/config/a750-rev1.srdf
A750_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("base_link", "link1"),
    ("base_link", "link2"),
    ("left_finger_link", "link3"),
    ("left_finger_link", "link4"),
    ("left_finger_link", "link5"),
    ("left_finger_link", "link6"),
    ("left_finger_link", "right_finger_link"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link2", "link4"),
    ("link3", "link4"),
    ("link3", "link5"),
    ("link3", "right_finger_link"),
    ("link4", "link5"),
    ("link4", "link6"),
    ("link4", "right_finger_link"),
    ("link5", "link6"),
    ("link5", "right_finger_link"),
    ("link6", "right_finger_link"),
]


def a750(
    name: str = "a750",
    *,
    adapter_type: str = "mock",
    device_path: str | None = None,
    **overrides: Any,
) -> RobotConfig:
    """Create an A-750 robot configuration.

    A-750 has 6 revolute joints (joint1-joint6) for the arm and 2 prismatic
    joints (joint7, joint8) for the parallel jaw gripper.

    Args:
        name: Robot identifier.
        adapter_type: Hardware adapter ("mock", "a750").
        device_path: Device path (e.g., "/dev/ttyACM0").
        **overrides: Override any RobotConfig field.
    """
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": LfsPath("a750_description") / "urdf/a750_rev1.urdf",
        "end_effector_link": "gripper_base",
        "adapter_type": adapter_type,
        "device_path": device_path,
        "joint_names": [f"joint{i}" for i in range(1, 7)],
        "base_link": "base_link",
        "home_joints": [0.0, 0.0, -math.radians(90), 0.0, 0.0, 0.0],
        "base_pose": [0, 0, 0, 0, 0, 0, 1],
        "package_paths": {
            "a750_description": LfsPath("a750_description"),
            "a750_gazebo": LfsPath("a750_description"),
        },
        "xacro_args": {},
        "auto_convert_meshes": True,
        "collision_exclusion_pairs": A750_GRIPPER_COLLISION_EXCLUSIONS,
        "gripper": GripperConfig(
            type="a750",
            joints=["finger"],
            collision_exclusions=A750_GRIPPER_COLLISION_EXCLUSIONS,
            open_position=0.06,
            close_position=0.02,
        ),
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)


__all__ = ["A750_FK_MODEL", "A750_GRIPPER_COLLISION_EXCLUSIONS", "a750"]
