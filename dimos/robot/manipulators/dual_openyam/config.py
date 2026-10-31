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

"""Arm-only Dual OpenYAM canonical planning model configuration."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.model import (
    DUAL_OPENYAM_MODEL,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS

DUAL_OPENYAM_SIDES = ("left", "right")
DUAL_OPENYAM_HOME_PER_ARM = list(OPENYAM_HOME_JOINTS)
DUAL_OPENYAM_HOME_JOINTS = [*DUAL_OPENYAM_HOME_PER_ARM, *DUAL_OPENYAM_HOME_PER_ARM]


def dual_openyam_arm_joints(side: str) -> list[str]:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return [f"{side}_joint{index}" for index in range(1, 7)]


def dual_openyam_model_config() -> RobotModelConfig:
    """Build the combined arm-only planning model."""
    return RobotModelConfig(
        model=DUAL_OPENYAM_MODEL.with_default_joint_acceleration_limit(1.0),
        joint_names=list(DUAL_OPENYAM_ARM_JOINTS),
        base_link="dual_openyam_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_manipulator",
                joint_names=tuple(dual_openyam_arm_joints("left")),
                base_link="dual_openyam_base",
                tip_link="left_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="right_manipulator",
                joint_names=tuple(dual_openyam_arm_joints("right")),
                base_link="dual_openyam_base",
                tip_link="right_grasp_frame",
            ),
        ],
        auto_convert_meshes=True,
        home_joints=list(DUAL_OPENYAM_HOME_JOINTS),
        tf_extra_links=[],
    )
