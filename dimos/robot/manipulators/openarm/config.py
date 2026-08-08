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

OPENARM_PKG = LfsPath("openarm_description")
OPENARM_LEFT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v20_left.urdf"
OPENARM_RIGHT_MODEL = OPENARM_PKG / "urdf/robot/openarm_v20_right.urdf"
OPENARM_BIMANUAL_MODEL = OPENARM_PKG / "urdf/robot/openarm_v20_bimanual.urdf"
OPENARM_PACKAGE_PATHS: dict[str, Path] = {"openarm_description": OPENARM_PKG}

# MIT gains measured on v1.0 hardware, carried over as the v2.0 starting
# point: with gravity compensation active the PD terms only handle transient
# tracking, and high kd excites gearbox buzz. Gripper slots bypass MIT
# control, so their gains are 0.
_ARM_KP = (100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0)
_ARM_KD = (1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8)


def validate_side(side: str) -> None:
    if side not in OPENARM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_arm_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"{side}_arm/joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_urdf_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, OPENARM_DOF + 1)]


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


def openarm_bimanual_model_config(name: str = OPENARM_HARDWARE_ID) -> RobotModelConfig:
    """Build the single fourteen-joint planning model with one group per arm.

    Collision exclusions cannot span robots, so both arms plan as one robot.
    """
    local_joint_names = [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]
    return RobotModelConfig(
        name=name,
        model_path=OPENARM_BIMANUAL_MODEL,
        base_pose=base_pose(),
        joint_names=local_joint_names,
        base_link="openarm_body_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_manipulator",
                joint_names=tuple(openarm_urdf_joints("left")),
                base_link="openarm_body_link0",
                tip_link="openarm_left_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="right_manipulator",
                joint_names=tuple(openarm_urdf_joints("right")),
                base_link="openarm_body_link0",
                tip_link="openarm_right_grasp_frame",
            ),
        ],
        package_paths=OPENARM_PACKAGE_PATHS,
        auto_convert_meshes=True,
        max_velocity=0.5,
        max_acceleration=1.0,
        joint_name_mapping={
            coordinator_name: urdf_name
            for side in OPENARM_SIDES
            for coordinator_name, urdf_name in zip(
                openarm_arm_joints(side), openarm_urdf_joints(side), strict=True
            )
        },
        home_joints=[0.0] * (2 * OPENARM_DOF),
    )
