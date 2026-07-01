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

"""OpenArm hardware and planning model configuration helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose
from dimos.utils.data import LfsPath

OPENARM_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("openarm_left_link5", "openarm_left_link7"),
    ("openarm_right_link5", "openarm_right_link7"),
]

OPENARM_PKG = LfsPath("openarm_description")
OPENARM_LEFT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_left.urdf"
OPENARM_RIGHT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_right.urdf"
OPENARM_V10_FK_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_single.urdf"
OPENARM_PACKAGE_PATHS: dict[str, Path] = {"openarm_description": OPENARM_PKG}

# Linux assigns can0/can1 in USB enumeration order, which is not guaranteed stable.
# Flip these if physical arms come up swapped.
LEFT_CAN = "can1"
RIGHT_CAN = "can0"

# Leave true for normal operation; it is idempotent and ensures motors are in
# the expected CTRL_MODE=MIT mode at connect time.
AUTO_SET_MIT_MODE = True
OPENARM_ADAPTER_KWARGS = {"auto_set_mit_mode": AUTO_SET_MIT_MODE}


def validate_side(side: str) -> None:
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, 8)]


def openarm_hardware(
    side: str,
    name: str | None = None,
    *,
    adapter_type: str = "mock",
    address: str | None = None,
    adapter_kwargs: dict[str, Any] | None = None,
) -> HardwareComponent:
    validate_side(side)
    kwargs = {"side": side}
    if adapter_kwargs:
        kwargs.update(adapter_kwargs)
    return HardwareComponent(
        hardware_id=name or f"{side}_arm",
        hardware_type=HardwareType.MANIPULATOR,
        joints=openarm_joints(side),
        adapter_type=adapter_type,
        address=address,
        adapter_kwargs=kwargs,
    )


def openarm_model_config(side: str, name: str | None = None) -> RobotModelConfig:
    validate_side(side)
    resolved_name = name or f"{side}_arm"
    local_joint_names = openarm_joints(side)
    return RobotModelConfig(
        name=resolved_name,
        model_path=OPENARM_LEFT_MODEL if side == "left" else OPENARM_RIGHT_MODEL,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="openarm_body_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="openarm_body_link0",
                tip_link=f"openarm_{side}_link7",
            )
        ],
        package_paths=OPENARM_PACKAGE_PATHS,
        collision_exclusion_pairs=OPENARM_COLLISION_EXCLUSIONS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        coordinator_task_name=f"traj_{resolved_name}",
        home_joints=[0.0] * 7,
    )


def openarm_single_hardware(
    *,
    adapter_type: str = "mock",
    address: str | None = None,
) -> HardwareComponent:
    return openarm_hardware(
        "left",
        name="arm",
        adapter_type=adapter_type,
        address=address,
    )


def openarm_single_model_config() -> RobotModelConfig:
    local_joint_names = openarm_joints("left")
    return RobotModelConfig(
        name="arm",
        model_path=OPENARM_V10_FK_MODEL,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="openarm_body_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(local_joint_names),
                base_link="openarm_body_link0",
                tip_link="openarm_left_link7",
            )
        ],
        package_paths=OPENARM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        coordinator_task_name="traj_arm",
        home_joints=[0.0] * 7,
    )
