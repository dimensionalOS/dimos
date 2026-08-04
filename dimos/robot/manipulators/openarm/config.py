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

"""OpenArm hardware and planning model configuration."""

from __future__ import annotations

from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose
from dimos.utils.data import LfsPath

OPENARM_DOF = 7
OPENARM_HARDWARE_ID = "openarm"
OPENARM_SIDES = ("left", "right")
# Order must match OpenArmDamiaoAdapter.joint_names: all arm groups in
# declaration order (left then right), then all grippers.
OPENARM_LEFT_ARM_JOINTS = [f"left_arm/joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_RIGHT_ARM_JOINTS = [f"right_arm/joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_ARM_JOINTS = [*OPENARM_LEFT_ARM_JOINTS, *OPENARM_RIGHT_ARM_JOINTS]
OPENARM_GRIPPER_JOINTS = ["left_arm/gripper", "right_arm/gripper"]
OPENARM_JOINTS = [*OPENARM_ARM_JOINTS, *OPENARM_GRIPPER_JOINTS]

OPENARM_COLLISION_EXCLUSIONS: list[tuple[str, str]] = [
    ("openarm_left_link5", "openarm_left_link7"),
    ("openarm_right_link5", "openarm_right_link7"),
]

OPENARM_PKG = LfsPath("openarm_description")
OPENARM_LEFT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_left.urdf"
OPENARM_RIGHT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_right.urdf"
OPENARM_V10_FK_MODEL = OPENARM_PKG / "urdf/robot/openarm_v10_single.urdf"
OPENARM_GRAVITY_MODEL_PATH = OPENARM_PKG / "urdf/robot/openarm_v10_bimanual.urdf"
OPENARM_PACKAGE_PATHS: dict[str, Path] = {"openarm_description": OPENARM_PKG}

# MIT gains measured on v10 hardware (legacy adapter): with gravity
# compensation active the PD terms only handle transient tracking, and high kd
# excites gearbox buzz. Gripper slots bypass MIT control, so their gains are 0.
_ARM_KP = (100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0)
_ARM_KD = (1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8)


def validate_side(side: str) -> None:
    if side not in OPENARM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_arm_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"{side}_arm/joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_hardware() -> HardwareComponent:
    """Select the physical or in-memory whole-body adapter for OpenArm."""
    adapter_type = "mock_whole_body" if global_config.simulation else "openarm_damiao"
    adapter_kwargs: dict[str, object] = {}
    if not global_config.simulation:
        adapter_kwargs["runtime_config"] = DamiaoRuntimeConfig(gravity_comp=True)
    return HardwareComponent(
        hardware_id=OPENARM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(OPENARM_JOINTS),
        adapter_type=adapter_type,
        auto_enable=True,
        adapter_kwargs=adapter_kwargs,
        wb_config=WholeBodyConfig(
            kp=(*_ARM_KP, *_ARM_KP, 0.0, 0.0),
            kd=(*_ARM_KD, *_ARM_KD, 0.0, 0.0),
        ),
    )


def openarm_model_config(side: str, name: str | None = None) -> RobotModelConfig:
    """Build one side's seven-joint planning model."""
    validate_side(side)
    resolved_name = name or f"{side}_arm"
    local_joint_names = [f"openarm_{side}_joint{i}" for i in range(1, OPENARM_DOF + 1)]
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
        joint_name_mapping={
            coordinator_name: urdf_name
            for coordinator_name, urdf_name in zip(
                openarm_arm_joints(side), local_joint_names, strict=True
            )
        },
        home_joints=[0.0] * OPENARM_DOF,
    )
