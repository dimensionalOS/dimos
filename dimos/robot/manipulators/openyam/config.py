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

"""OpenYAM hardware and planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import (
    base_pose,
    joint_names,
)
from dimos.utils.data import LfsPath

OPENYAM_DOF = 6
OPENYAM_HARDWARE_ID = "arm"
OPENYAM_ARM_JOINTS = joint_names(OPENYAM_DOF, prefix="yam_joint")
OPENYAM_GRIPPER_JOINT = f"{OPENYAM_HARDWARE_ID}/gripper"
OPENYAM_JOINTS = [*OPENYAM_ARM_JOINTS, OPENYAM_GRIPPER_JOINT]
OPENYAM_PACKAGE = LfsPath("yam_description")
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "urdf/yam_gripper.urdf.xacro"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"yam_description": OPENYAM_PACKAGE}


def make_openyam_hardware(
    hw_id: str = "arm",
    *,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Create OpenYAM hardware, defaulting to the generic mock adapter."""
    adapter_kwargs: dict[str, object] = {}
    if home_joints is not None:
        adapter_kwargs["initial_positions"] = [*home_joints, 0.0]
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*OPENYAM_ARM_JOINTS, f"{hw_id}/gripper"],
        adapter_type="mock",
        address=None,
        auto_enable=auto_enable,
        limits=JointLimits(
            position_lower=[*([None] * OPENYAM_DOF), 0.0],
            position_upper=[*([None] * OPENYAM_DOF), 1.0],
            velocity_max=[None] * len(OPENYAM_JOINTS),
        ),
        adapter_kwargs=adapter_kwargs,
    )


def openyam_hardware(
    hw_id: str = "arm",
    *,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    """Create mock OpenYAM hardware for simulation and configuration checks."""
    return make_openyam_hardware(hw_id, home_joints=home_joints)


def make_openyam_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build a planning config for the gripper-equipped OpenYAM."""
    model_joint_names = joint_names(OPENYAM_DOF, prefix="yam_joint")
    return RobotModelConfig(
        model=RobotModel.from_file(OPENYAM_MODEL_PATH, package_paths=OPENYAM_PACKAGE_PATHS),
        base_pose=base_pose(),
        joint_names=model_joint_names,
        base_link="yam_base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="yam_base_link",
                tip_link="yam_hand_tcp",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=[],
        gripper_hardware_id="arm",
        home_joints=home_joints or [0.0] * OPENYAM_DOF,
    )
