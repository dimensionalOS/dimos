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


"""Galaxea R1 Pro planning model (hardware wiring lives in ``connection.py``).

The vendor description is fetched from the pinned upstream URDF repo; set
``R1PRO_DESCRIPTION`` to point at a local checkout instead.
"""

from __future__ import annotations

import os
from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.galaxea.r1pro.joints import (
    LEFT_ARM_JOINTS,
    PASSIVE_JOINTS,
    RIGHT_ARM_JOINTS,
    UPPER_BODY_JOINTS,
    coordinator_name,
)

R1PRO_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url="https://github.com/userguide-galaxea/URDF",
    ref="2e5d31e1784481a34d178006c0d0e18e0a84a82a",
)


def _description_root() -> Path:
    override = os.getenv("R1PRO_DESCRIPTION")
    if override:
        return Path(override)
    return R1PRO_DESCRIPTION_SOURCE / "R1Pro" / "urdf_r1pro_g1z_2026"


R1PRO_PACKAGE_ROOT = _description_root()
R1PRO_MODEL_PATH = R1PRO_PACKAGE_ROOT / "urdf" / "r1pro_2026.urdf"

# Structural mesh overlaps in the full-body URDF plus the gripper parallel
# linkages, which legitimately intersect. Wrist camera links follow the
# upstream naming (d405 + gmsl), not the vendor build's single realsense link.
R1PRO_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("base_link", "wheel_motor_link1"),
    ("base_link", "wheel_motor_link2"),
    ("base_link", "wheel_motor_link3"),
    ("base_link", "steer_motor_link1"),
    ("base_link", "steer_motor_link2"),
    ("base_link", "steer_motor_link3"),
    ("torso_link4", "left_arm_link1"),
    ("torso_link4", "right_arm_link1"),
    ("torso_link4", "left_arm_base_link"),
    ("torso_link4", "right_arm_base_link"),
    ("left_arm_link5", "left_arm_link7"),
    ("right_arm_link5", "right_arm_link7"),
    ("left_arm_link7", "left_gripper_link"),
    ("left_gripper_link", "left_gripper_finger_link1"),
    ("left_gripper_link", "left_gripper_finger_link2"),
    ("left_gripper_finger_link1", "left_gripper_finger_link2"),
    ("left_gripper_link", "left_d405_link"),
    ("left_arm_link7", "left_d405_link"),
    ("left_gripper_link", "left_gmsl_link"),
    ("left_arm_link7", "left_gmsl_link"),
    ("right_arm_link7", "right_gripper_link"),
    ("right_gripper_link", "right_gripper_finger_link1"),
    ("right_gripper_link", "right_gripper_finger_link2"),
    ("right_gripper_finger_link1", "right_gripper_finger_link2"),
    ("right_gripper_link", "right_d405_link"),
    ("right_arm_link7", "right_d405_link"),
    ("right_gripper_link", "right_gmsl_link"),
    ("right_arm_link7", "right_gmsl_link"),
]


R1PRO_MODEL = (
    RobotModel.from_file(
        R1PRO_MODEL_PATH,
        package_paths={"r1pro_urdf": R1PRO_PACKAGE_ROOT},
    )
    .with_fixed_joints(*PASSIVE_JOINTS)
    .with_renamed_joints({joint: coordinator_name(joint) for joint in UPPER_BODY_JOINTS})
)


def make_r1pro_model_config() -> RobotModelConfig:
    """Full-body collision model with the two arms as planning groups.

    The chassis is welded at the URDF root: base motion is commanded through the
    coordinator's chassis velocity task, not planned.
    """
    return RobotModelConfig(
        model=R1PRO_MODEL,
        joint_names=[coordinator_name(joint) for joint in UPPER_BODY_JOINTS],
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=tuple(coordinator_name(j) for j in LEFT_ARM_JOINTS),
                base_link="left_arm_base_link",
                tip_link="left_gripper_link",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=tuple(coordinator_name(j) for j in RIGHT_ARM_JOINTS),
                base_link="right_arm_base_link",
                tip_link="right_gripper_link",
            ),
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=R1PRO_COLLISION_EXCLUSIONS,
        max_velocity=0.5,
        max_acceleration=1.0,
        home_joints=[0.0] * len(UPPER_BODY_JOINTS),
    )
