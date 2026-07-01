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

"""A-750 planning model configuration helpers."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.core.global_config import global_config
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import (
    base_pose,
    coordinator_joint_mapping,
    joint_names,
)
from dimos.utils.data import LfsPath

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

A750_HOME_JOINTS = [0.0, 0.0, -math.radians(90), 0.0, 0.0, 0.0]
A750_MODEL_PATH = LfsPath("a750_description") / "urdf/a750_rev1.urdf"
A750_FK_MODEL = LfsPath("a750_description/urdf/a750_rev1_no_gripper.urdf")
A750_PACKAGE_PATHS: dict[str, Path] = {
    "a750_description": LfsPath("a750_description"),
    "a750_gazebo": LfsPath("a750_description"),
}


def make_a750_hardware(
    hw_id: str = "arm",
    *,
    adapter_type: str = "a750",
    address: str | None = None,
    gripper: bool = True,
    auto_enable: bool = True,
    home_joints: list[float] | None = None,
) -> HardwareComponent:
    joints = make_joints(hw_id, 6)
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=joints,
        adapter_type=adapter_type,
        address=address,
        auto_enable=auto_enable,
        gripper_joints=[f"{hw_id}/finger"] if gripper else [],
        adapter_kwargs={"initial_positions": home_joints or A750_HOME_JOINTS},
    )


def a750_hardware(hw_id: str = "arm", *, mock_without_address: bool = False) -> HardwareComponent:
    if mock_without_address and not global_config.device_path:
        return make_a750_hardware(
            hw_id,
            adapter_type="mock",
            address=None,
        )
    return make_a750_hardware(
        hw_id,
        address=global_config.device_path or "/dev/ttyACM0",
    )


def make_a750_model_config(
    name: str = "arm",
    *,
    joint_prefix: str | None = None,
    coordinator_task_name: str | None = None,
) -> RobotModelConfig:
    dof = 6
    local_joint_names = joint_names(dof)
    return RobotModelConfig(
        name=name,
        model_path=A750_MODEL_PATH,
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
        package_paths=A750_PACKAGE_PATHS,
        auto_convert_meshes=True,
        collision_exclusion_pairs=A750_GRIPPER_COLLISION_EXCLUSIONS,
        joint_name_mapping=coordinator_joint_mapping(
            name,
            dof,
            joint_prefix=joint_prefix,
        ),
        coordinator_task_name=coordinator_task_name or f"traj_{name}",
        gripper_hardware_id=name,
        home_joints=A750_HOME_JOINTS,
    )
