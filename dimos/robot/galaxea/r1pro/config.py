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

"""Galaxea R1 Pro planning-model configuration (Drake side; hardware wiring
lives in ``connection.py``).

The full-body description (vendor ``r1pro_2026`` URDF + meshes) lives in the
LFS store; set ``R1PRO_DESCRIPTION`` to override with a local checkout.
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose
from dimos.utils.data import LfsPath


def _description_root() -> Path:
    override = os.getenv("R1PRO_DESCRIPTION")
    if override:
        return Path(override)
    return LfsPath("r1_pro_description")


R1PRO_MODEL_PATH = _description_root() / "urdf" / "r1pro_2026.urdf"

# Collision exclusion pairs — structural mesh overlaps in the full-body URDF
# plus gripper parallel-linkage exclusions.
R1PRO_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    # Chassis ↔ wheels (mesh overlap at zero pose)
    ("base_link", "wheel_motor_link1"),
    ("base_link", "wheel_motor_link2"),
    ("base_link", "wheel_motor_link3"),
    ("base_link", "steer_motor_link1"),
    ("base_link", "steer_motor_link2"),
    ("base_link", "steer_motor_link3"),
    # Torso ↔ arm shoulders (tight mesh fit)
    ("torso_link4", "left_arm_link1"),
    ("torso_link4", "right_arm_link1"),
    ("torso_link4", "left_arm_base_link"),
    ("torso_link4", "right_arm_base_link"),
    # Non-adjacent arm links that overlap at zero pose (link5 ↔ link7)
    ("left_arm_link5", "left_arm_link7"),
    ("right_arm_link5", "right_arm_link7"),
    # Left gripper
    ("left_arm_link7", "left_gripper_link"),
    ("left_gripper_link", "left_gripper_finger_link1"),
    ("left_gripper_link", "left_gripper_finger_link2"),
    ("left_gripper_finger_link1", "left_gripper_finger_link2"),
    ("left_gripper_link", "left_realsense_link"),
    ("left_arm_link7", "left_realsense_link"),
    # Right gripper
    ("right_arm_link7", "right_gripper_link"),
    ("right_gripper_link", "right_gripper_finger_link1"),
    ("right_gripper_link", "right_gripper_finger_link2"),
    ("right_gripper_finger_link1", "right_gripper_finger_link2"),
    ("right_gripper_link", "right_realsense_link"),
    ("right_arm_link7", "right_realsense_link"),
]


def make_r1pro_arm_model_config(side: str = "left") -> RobotModelConfig:
    """Planning model for one R1 Pro arm (7 DOF) out of the full-body URDF."""
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    urdf_joints = [f"{side}_arm_joint{i}" for i in range(1, 8)]
    root = _description_root()

    return RobotModelConfig(
        name=f"{side}_arm",
        model_path=R1PRO_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=urdf_joints,
        end_effector_link=f"{side}_arm_link7",
        base_link="base_link",
        # The vendor URDF references its meshes via package://r1pro_urdf.
        package_paths={"r1pro_urdf": root},
        auto_convert_meshes=True,
        collision_exclusion_pairs=R1PRO_COLLISION_EXCLUSIONS,
        max_velocity=0.5,
        max_acceleration=1.0,
        joint_name_mapping={f"r1pro/{j}": j for j in urdf_joints},
        coordinator_task_name=f"traj_{side}_arm",
        home_joints=[0.0] * 7,
    )


__all__ = [
    "R1PRO_COLLISION_EXCLUSIONS",
    "R1PRO_MODEL_PATH",
    "make_r1pro_arm_model_config",
]
