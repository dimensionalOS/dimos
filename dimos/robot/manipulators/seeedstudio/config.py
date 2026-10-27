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

"""Seeed Studio reBot B601-DM hardware and planning model configuration helpers."""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.core.global_config import global_config
from dimos.hardware.manipulators.seeedstudio.adapter import (
    ARM_DOF,
    ARM_LOWER,
    ARM_UPPER,
    ARM_VELOCITY_MAX,
    GRIPPER_MAX_OPENING_M,
    JOINT2_REST_MARGIN,
)
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import joint_names

SEEEDSTUDIO_DESCRIPTION_REPO = "https://github.com/Seeed-Projects/reBotArm_control_py.git"
SEEEDSTUDIO_DESCRIPTION_REF = "6415d43130d1e143c70dc106096a857ac5556f81"
_SEEEDSTUDIO_REPO = RobotDescriptionSource(
    url=SEEEDSTUDIO_DESCRIPTION_REPO,
    ref=SEEEDSTUDIO_DESCRIPTION_REF,
)
SEEEDSTUDIO_MODEL_PATH = _SEEEDSTUDIO_REPO / "urdf" / "DM" / "urdf" / "ReBot_Arm_DM.urdf"

# The vendor URDF lists 50-200 rad/s joint velocities; the motors top out at
# 10-30 rad/s. Scale planned trajectories to 0.2 rad/s (joints 1-3) and
# 0.8 rad/s (joints 4-6), under the adapter's POS_VEL ceiling.
SEEEDSTUDIO_VELOCITY_SCALE = 0.004
# One control tick sweeps seven serial feedback requests and seven POS_VEL
# writes over the USB bridge, roughly 30 ms.
SEEEDSTUDIO_TICK_RATE_HZ = 25.0
SEEEDSTUDIO_ACCELERATION_LIMIT = 0.5


def seeedstudio_hardware(hw_id: str = "arm") -> HardwareComponent:
    """Configure mock B601-DM hardware unless an explicit CAN port selects the real adapter."""
    adapter_type = "mock"
    address = None
    limits = None
    if not global_config.simulation and global_config.can_port:
        adapter_type = "seeedstudio_b601_dm"
        address = global_config.can_port
    else:
        limits = JointLimits(
            position_lower=[*ARM_LOWER, 0.0],
            position_upper=[*ARM_UPPER, GRIPPER_MAX_OPENING_M],
            velocity_max=[ARM_VELOCITY_MAX] * ARM_DOF + [0.0],
        )
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.MANIPULATOR,
        joints=[*joint_names(ARM_DOF), f"{hw_id}/gripper"],
        adapter_type=adapter_type,
        address=address,
        auto_enable=True,
        limits=limits,
    )


def seeedstudio_gripper_task(hw_id: str = "arm") -> TaskConfig:
    """Normalized open/close control of the gripper joint (0.0 closed, 1.0 open)."""
    return TaskConfig(
        name=f"{hw_id}_gripper",
        type="gripper",
        joint_names=[f"{hw_id}/gripper"],
        priority=20,
    )


def make_seeedstudio_model_config(
    *,
    gripper_hardware_id: str | None = "arm",
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Vendor model with fingers fixed closed; the gripper is driven, not planned."""
    model_joint_names = joint_names(ARM_DOF)
    model = (
        RobotModel.from_file(SEEEDSTUDIO_MODEL_PATH)
        .with_fixed_joints("finger_left", "finger_right")
        # Match the adapter: joint2 rests just past the URDF's 0.0 upper bound.
        .with_joint_position_limits("joint2", lower=ARM_LOWER[1], upper=JOINT2_REST_MARGIN)
        .with_default_joint_acceleration_limit(SEEEDSTUDIO_ACCELERATION_LIMIT)
    )
    return RobotModelConfig(
        model=model,
        joint_names=model_joint_names,
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(model_joint_names),
                base_link="base_link",
                tip_link="end_link",
            )
        ],
        auto_convert_meshes=True,
        gripper_hardware_id=gripper_hardware_id,
        home_joints=home_joints or [0.0] * ARM_DOF,
    )
