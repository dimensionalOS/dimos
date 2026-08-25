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

"""Piper planning model configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
)
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]

PIPER_DESCRIPTION_REPO = "https://github.com/agilexrobotics/agx_arm_urdf"
PIPER_DESCRIPTION_REF = "f6642ce0d7872c686f29c99e9e10cd23d1d49313"
_PIPER_REPO = RobotDescriptionSource(url=PIPER_DESCRIPTION_REPO, ref=PIPER_DESCRIPTION_REF)
PIPER_MODEL_PATH = _PIPER_REPO / "piper" / "urdf" / "piper_with_gripper_description.xacro"
PIPER_PACKAGE_PATHS: dict[str, Path] = {
    # Upstream URIs are package://agx_arm_description/agx_arm_urdf/..., so the
    # package root is the parent of the preserved agx_arm_urdf checkout folder.
    "agx_arm_description": _PIPER_REPO.parent,
}
PIPER_FK_MODEL = _PIPER_REPO / "piper" / "urdf" / "piper_description.urdf"
PIPER_SIM_PATH = LfsPath("piper/scene.xml")
PIPER_HOME_JOINTS = [
    0.793,
    1.568186214614724,
    -1.0290351975897356,
    0.0008456548489068756,
    0.9771515619106422,
    -0.13286819850920156,
]


def _adapter_kwargs(home_joints: list[float] | None = None) -> dict[str, object]:
    if home_joints is None:
        return {}
    return {"initial_positions": home_joints}


def make_piper_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    gripper: bool = True,
    auto_enable: bool = True,
    adapter_kwargs: dict[str, object] | None = None,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    kwargs = _adapter_kwargs(home_joints)
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    gripper_joints = [f"{hw_id}/gripper"] if gripper else []
    initial_positions = kwargs.get("initial_positions")
    if gripper and isinstance(initial_positions, list):
        kwargs["initial_positions"] = [*initial_positions, 0.0]
    if adapter_type == "mock":
        kwargs["limits"] = JointLimits(
            position_lower=[*([-math.pi] * 6), *([0.0] * len(gripper_joints))],
            position_upper=[*([math.pi] * 6), *([0.08] * len(gripper_joints))],
            velocity_max=[*([math.pi] * 6), *([0.0] * len(gripper_joints))],
        )
    elif adapter_type == "module":
        kwargs["limits"] = JointLimits(
            position_lower=[*([-math.pi] * 6), *([0.0] * len(gripper_joints))],
            position_upper=[*([math.pi] * 6), *([0.035] * len(gripper_joints))],
            velocity_max=[*([math.pi] * 6), *([0.0] * len(gripper_joints))],
        )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*make_joints(hw_id, 6), *gripper_joints],
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        adapter_kwargs=kwargs,
    )


def piper_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = True,
    mock_without_address: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    address = global_config.can_port or "can0"
    if mock_without_address and not global_config.can_port:
        return make_piper_hardware(
            hw_id,
            gripper=gripper,
            home_joints=home_joints,
        )
    return make_piper_hardware(
        hw_id,
        adapter_type="piper",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
    )


def make_piper_model_config(
    name: str = "arm",
    *,
    joint_prefix: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    dof = 6
    local_joint_names = joint_names(dof)
    model_home_joints = list(home_joints) if home_joints is not None else list(PIPER_HOME_JOINTS)
    return RobotModelConfig(
        name=name,
        model=RobotModel.from_file(PIPER_MODEL_PATH, package_paths=PIPER_PACKAGE_PATHS),
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="base_link",
                tip_link="gripper_base",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            dof,
            joint_prefix=joint_prefix,
        ),
        gripper_hardware_id=name,
        home_joints=model_home_joints,
    )
