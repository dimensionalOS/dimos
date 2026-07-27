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

"""Galaxea A1Z planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

A1Z_DOF = 6

A1Z_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("arm_link2", "arm_link5"),
    ("arm_link4", "arm_link6"),
]

A1Z_G1Z_MODEL_PATH = LfsPath("a1z_description") / "A1Z_G1Z/urdf/A1Z_G1Z.urdf"
A1Z_FLANGE_MODEL_PATH = LfsPath("a1z_description") / "A1Z_Flange/urdf/A1Z_Flange.urdf"
A1Z_FK_MODEL = A1Z_FLANGE_MODEL_PATH
A1Z_PACKAGE_PATHS: dict[str, Path] = {
    "A1Z_G1Z": LfsPath("a1z_description") / "A1Z_G1Z",
    "A1Z_Flange": LfsPath("a1z_description") / "A1Z_Flange",
}


def make_a1z_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    has_gripper: bool = True,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    adapter_kwargs: dict[str, object] = {}
    if home_joints is not None:
        adapter_kwargs["initial_positions"] = home_joints
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=make_joints(hw_id, A1Z_DOF),
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/gripper"] if has_gripper else [],
        adapter_kwargs=adapter_kwargs,
    )


def make_a1z_model_config(
    name: str = "arm",
    *,
    has_gripper: bool = True,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    local_joint_names = joint_names(A1Z_DOF, prefix="arm_joint")
    return RobotModelConfig(
        name=name,
        model_path=A1Z_G1Z_MODEL_PATH if has_gripper else A1Z_FLANGE_MODEL_PATH,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="base_link",
                tip_link=("gripper_eef_link" if has_gripper else "arm_link6"),
            )
        ],
        package_paths=A1Z_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A1Z_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            A1Z_DOF,
            joint_prefix=joint_prefix,
            urdf_joint_prefix="arm_",
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name if has_gripper else None,
        home_joints=home_joints or [0.0] * A1Z_DOF,
    )
