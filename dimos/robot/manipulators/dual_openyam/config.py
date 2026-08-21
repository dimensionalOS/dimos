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

"""Arm-only Dual OpenYAM hardware and model configuration."""

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose
from dimos.robot.manipulators.dual_openyam.model import (
    DUAL_OPENYAM_MODEL,
)

DUAL_OPENYAM_DOF_PER_ARM = 6
DUAL_OPENYAM_HARDWARE_ID = "dual_openyam"
DUAL_OPENYAM_ADAPTER_TYPE = "dual_openyam_damiao"
DUAL_OPENYAM_SIDES = ("left", "right")
DUAL_OPENYAM_LEFT_ARM_JOINTS = [
    f"left_arm/joint{index}" for index in range(1, DUAL_OPENYAM_DOF_PER_ARM + 1)
]
DUAL_OPENYAM_RIGHT_ARM_JOINTS = [
    f"right_arm/joint{index}" for index in range(1, DUAL_OPENYAM_DOF_PER_ARM + 1)
]
DUAL_OPENYAM_ARM_JOINTS = [
    *DUAL_OPENYAM_LEFT_ARM_JOINTS,
    *DUAL_OPENYAM_RIGHT_ARM_JOINTS,
]
DUAL_OPENYAM_GRIPPER_JOINTS = ["left_arm/gripper", "right_arm/gripper"]
DUAL_OPENYAM_JOINTS = [*DUAL_OPENYAM_ARM_JOINTS, *DUAL_OPENYAM_GRIPPER_JOINTS]
DUAL_OPENYAM_HOME_PER_ARM = [0.0, 1.047, 1.047, 0.0, 0.0, 0.0]
DUAL_OPENYAM_HOME_JOINTS = [*DUAL_OPENYAM_HOME_PER_ARM, *DUAL_OPENYAM_HOME_PER_ARM]
DUAL_OPENYAM_URDF_ARM_JOINTS = [
    f"{side}_joint{index}"
    for side in DUAL_OPENYAM_SIDES
    for index in range(1, DUAL_OPENYAM_DOF_PER_ARM + 1)
]
_ARM_KP = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0)
_ARM_KD = (5.0, 5.0, 5.0, 1.5, 1.5, 1.5)


def dual_openyam_arm_joints(side: str) -> list[str]:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return [f"{side}_arm/joint{index}" for index in range(1, 7)]


def dual_openyam_urdf_joints(side: str) -> list[str]:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return [f"{side}_joint{index}" for index in range(1, 7)]


def dual_openyam_hardware(
    *,
    left_can_port: str | None = None,
    right_can_port: str | None = None,
) -> HardwareComponent:
    """Select mock hardware or an explicitly configured dual-CAN adapter."""
    if left_can_port is None and right_can_port is None:
        return dual_openyam_mock_hardware()
    if left_can_port is None or right_can_port is None:
        raise ValueError("Dual OpenYAM hardware requires both left and right CAN ports")
    return _hardware_component(
        DUAL_OPENYAM_ADAPTER_TYPE,
        {
            "runtime_config": DamiaoRuntimeConfig(
                bus_devices={"left": left_can_port, "right": right_can_port},
                gravity_comp=True,
            )
        },
    )


def dual_openyam_mock_hardware() -> HardwareComponent:
    """Build an in-memory complete dual-arm actuator component."""
    return _hardware_component(
        "mock_whole_body",
        {"initial_positions": [*DUAL_OPENYAM_HOME_JOINTS, 0.0, 0.0]},
    )


def _hardware_component(
    adapter_type: str,
    adapter_kwargs: dict[str, object],
) -> HardwareComponent:
    return HardwareComponent(
        hardware_id=DUAL_OPENYAM_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(DUAL_OPENYAM_JOINTS),
        adapter_type=adapter_type,
        auto_enable=True,
        adapter_kwargs=adapter_kwargs,
        wb_config=WholeBodyConfig(
            kp=(*_ARM_KP, *_ARM_KP, 0.0, 0.0),
            kd=(*_ARM_KD, *_ARM_KD, 0.0, 0.0),
        ),
    )


def dual_openyam_model_config() -> RobotModelConfig:
    """Build the combined arm-only planning model."""
    mapping = dict(
        zip(
            DUAL_OPENYAM_ARM_JOINTS,
            DUAL_OPENYAM_URDF_ARM_JOINTS,
            strict=True,
        )
    )
    return RobotModelConfig(
        name=DUAL_OPENYAM_HARDWARE_ID,
        model=DUAL_OPENYAM_MODEL,
        base_pose=base_pose(),
        joint_names=list(DUAL_OPENYAM_URDF_ARM_JOINTS),
        base_link="dual_openyam_base",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_manipulator",
                joint_names=tuple(dual_openyam_urdf_joints("left")),
                base_link="dual_openyam_base",
                tip_link="left_grasp_frame",
            ),
            PlanningGroupDefinition(
                name="right_manipulator",
                joint_names=tuple(dual_openyam_urdf_joints("right")),
                base_link="dual_openyam_base",
                tip_link="right_grasp_frame",
            ),
        ],
        auto_convert_meshes=True,
        joint_name_mapping=mapping,
        home_joints=list(DUAL_OPENYAM_HOME_JOINTS),
        max_velocity=2.0,
        max_acceleration=1.0,
        tf_extra_links=[],
    )
