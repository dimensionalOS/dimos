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

"""Pinned OpenArm geometry and canonical planning configuration."""

from __future__ import annotations

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource

OPENARM_DESCRIPTION_URL = "https://github.com/enactic/openarm_description"
OPENARM_DESCRIPTION_REF = "1fba2cbc05001f05b4514120b70130b4ac06f409"

OPENARM_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url=OPENARM_DESCRIPTION_URL,
    ref=OPENARM_DESCRIPTION_REF,
)
OPENARM_DESCRIPTION_ROOT = OPENARM_DESCRIPTION_SOURCE / "."
OPENARM_BIMANUAL_XACRO = (
    OPENARM_DESCRIPTION_SOURCE
    / "assets"
    / "robot"
    / "openarm_v2.0"
    / "urdf"
    / "openarm_v20.urdf.xacro"
)

OPENARM_BIMANUAL_MODEL = (
    RobotModel.from_file(
        OPENARM_BIMANUAL_XACRO,
        package_paths={"openarm_description": OPENARM_DESCRIPTION_ROOT},
        xacro_args={
            "robot_preset": "default_bimanual",
            "emit_grasp_frame": "true",
        },
    )
    .with_default_joint_acceleration_limit(1.0)
    .with_fixed_joints(
        "openarm_left_finger_joint1",
        "openarm_left_finger_joint2",
        "openarm_right_finger_joint1",
        "openarm_right_finger_joint2",
    )
)

OPENARM_DOF = 7
OPENARM_SIDES = ("left", "right")
# Connection order is all left arm joints, all right arm joints, then grippers.
OPENARM_LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_ARM_JOINTS = [*OPENARM_LEFT_ARM_JOINTS, *OPENARM_RIGHT_ARM_JOINTS]
OPENARM_GRIPPER_JOINTS = ["left_gripper", "right_gripper"]
OPENARM_JOINTS = [*OPENARM_ARM_JOINTS, *OPENARM_GRIPPER_JOINTS]
OPENARM_HOME_JOINTS = [0.0] * len(OPENARM_ARM_JOINTS)
OPENARM_GRIPPER_COLLISION_EXCLUSIONS = [
    (f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2") for side in OPENARM_SIDES
]


def validate_side(side: str) -> None:
    if side not in OPENARM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_urdf_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_bimanual_model_config() -> RobotModelConfig:
    """Build the single fourteen-joint planning model with one group per arm.

    Collision exclusions cannot span robots, so both arms plan as one robot.
    """
    canonical_joint_names = [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]
    return RobotModelConfig(
        model=OPENARM_BIMANUAL_MODEL,
        joint_names=canonical_joint_names,
        base_link="openarm_body_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=tuple(openarm_urdf_joints("left")),
                base_link="openarm_body_link0",
                tip_link="openarm_left_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=tuple(openarm_urdf_joints("right")),
                base_link="openarm_body_link0",
                tip_link="openarm_right_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="both_arms",
                joint_names=tuple(canonical_joint_names),
                base_link="openarm_body_link0",
            ),
        ],
        collision_exclusion_pairs=OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        home_joints=list(OPENARM_HOME_JOINTS),
    )
