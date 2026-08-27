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

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import (
    base_pose,
    joint_names,
)
from dimos.utils.data import LfsPath

PIPER_GRIPPER_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("gripper_base", "link7"),
    ("gripper_base", "link8"),
    ("link7", "link8"),
    ("link6", "gripper_base"),
]

PIPER_MODEL_PATH = LfsPath("piper_description") / "urdf/piper_description.xacro"
PIPER_PACKAGE_PATHS: dict[str, Path] = {
    "piper_description": LfsPath("piper_description"),
    "piper_gazebo": LfsPath("piper_description"),
}
PIPER_FK_MODEL = LfsPath("piper_description") / "urdf/piper_description.urdf"
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
    canonical_joint_names: list[str] | None = None,
) -> HardwareComponent:
    kwargs = _adapter_kwargs(home_joints)
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    gripper_joints = [f"{hw_id}/gripper"] if gripper else []
    initial_positions = kwargs.get("initial_positions")
    if gripper and isinstance(initial_positions, list):
        kwargs["initial_positions"] = [*initial_positions, 0.0]
    limits: JointLimits | None = None
    if adapter_type == "mock":
        limits = JointLimits(
            position_lower=[*([-math.pi] * 6), *([0.0] * len(gripper_joints))],
            position_upper=[*([math.pi] * 6), *([0.08] * len(gripper_joints))],
            velocity_max=[*([math.pi] * 6), *([0.0] * len(gripper_joints))],
        )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*(canonical_joint_names or joint_names(6)), *gripper_joints],
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        limits=limits,
        adapter_kwargs=kwargs,
    )


def piper_hardware(
    hw_id: str = "arm",
    *,
    gripper: bool = True,
    mock_without_address: bool = True,
    home_joints: list[float] | None = None,
    canonical_joint_names: list[str] | None = None,
) -> HardwareComponent:
    if global_config.simulation:
        return make_piper_hardware(
            hw_id,
            adapter_type="sim_mujoco",
            address=str(PIPER_SIM_PATH),
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    address = global_config.can_port or "can0"
    if mock_without_address and not global_config.can_port:
        return make_piper_hardware(
            hw_id,
            gripper=gripper,
            home_joints=home_joints,
            canonical_joint_names=canonical_joint_names,
        )
    return make_piper_hardware(
        hw_id,
        adapter_type="piper",
        address=address,
        gripper=gripper,
        home_joints=home_joints,
        canonical_joint_names=canonical_joint_names,
    )


def make_piper_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    dof = 6
    model_joint_names = joint_names(dof)
    model_home_joints = list(home_joints) if home_joints is not None else list(PIPER_HOME_JOINTS)
    return RobotModelConfig(
        model=RobotModel.from_file(PIPER_MODEL_PATH, package_paths=PIPER_PACKAGE_PATHS),
        base_pose=base_pose(),
        joint_names=model_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="base_link",
                tip_link="gripper_base",
            )
        ],
        auto_convert_meshes=True,
        collision_exclusion_pairs=PIPER_GRIPPER_COLLISION_EXCLUSIONS,
        gripper_hardware_id="arm",
        home_joints=model_home_joints,
    )
