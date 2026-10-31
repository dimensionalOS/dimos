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

"""OpenYAM canonical planning model configuration."""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import (
    joint_names,
)
from dimos.robot.manipulators.openyam.joints import (
    OPENYAM_ARM_JOINTS as OPENYAM_ARM_JOINTS,
    OPENYAM_DOF as OPENYAM_DOF,
    OPENYAM_GRIPPER_JOINT as OPENYAM_GRIPPER_JOINT,
    OPENYAM_JOINTS as OPENYAM_JOINTS,
)
from dimos.utils.data import LfsPath

OPENYAM_HARDWARE_ID = "openyam"
OPENYAM_HOME_JOINTS = [0.0, 1.047, 1.047, 0.0, 0.0, 0.0]
OPENYAM_PACKAGE = LfsPath("yam_description")
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "i2rt/yam.urdf"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"yam_description": OPENYAM_PACKAGE}


def make_openyam_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build the canonical visualization and planning config for OpenYAM."""
    model_joint_names = joint_names(OPENYAM_DOF, prefix="yam_joint")
    model = (
        RobotModel.from_file(
            OPENYAM_MODEL_PATH,
            package_paths=OPENYAM_PACKAGE_PATHS,
        )
        .with_renamed_joints(dict(zip(joint_names(OPENYAM_DOF), model_joint_names, strict=True)))
        .with_fixed_joints("joint7", "joint8")
    )
    return RobotModelConfig(
        model=model.with_default_joint_acceleration_limit(2.0),
        joint_names=model_joint_names,
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="base",
                tip_link="gripper_tip",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=[],
        gripper_hardware_id=OPENYAM_HARDWARE_ID,
        home_joints=list(OPENYAM_HOME_JOINTS if home_joints is None else home_joints),
    )
