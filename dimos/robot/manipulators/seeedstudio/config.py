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
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators.seeedstudio.joints import (
    SEEEDSTUDIO_ARM_JOINTS,
    SEEEDSTUDIO_DOF,
    SEEEDSTUDIO_GRIPPER_JOINT,
    SEEEDSTUDIO_HARDWARE_ID,
    SEEEDSTUDIO_JOINTS,
)

SEEEDSTUDIO_DESCRIPTION_REPO = "https://github.com/Seeed-Projects/reBotArm_control_py.git"
SEEEDSTUDIO_DESCRIPTION_REF = "6415d43130d1e143c70dc106096a857ac5556f81"
_SEEEDSTUDIO_REPO = RobotDescriptionSource(
    url=SEEEDSTUDIO_DESCRIPTION_REPO,
    ref=SEEEDSTUDIO_DESCRIPTION_REF,
)
SEEEDSTUDIO_MODEL_PATH = _SEEEDSTUDIO_REPO / "urdf" / "DM" / "urdf" / "ReBot_Arm_DM.urdf"

# At its folded mechanical rest this arm's joint2 encoder reads about
# +0.0055 rad, just past the vendor URDF's 0.0 upper limit.
SEEEDSTUDIO_JOINT2_LIMITS = (-3.14, 0.02)
# The vendor URDF lists 50-200 rad/s joint velocities; the motors top out at
# 10-30 rad/s. Scale planned trajectories to 0.2 rad/s (joints 1-3) and
# 0.8 rad/s (joints 4-6).
SEEEDSTUDIO_VELOCITY_SCALE = 0.004
SEEEDSTUDIO_ACCELERATION_LIMIT = 0.5
# MIT gains for the DM4340P shoulder and elbow and the DM4310 wrist; the
# gripper runs its own current-limited position control.
SEEEDSTUDIO_KP = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0, 0.0)
SEEEDSTUDIO_KD = (5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0)


def seeedstudio_model() -> RobotModel:
    """Vendor model with the fingers fixed closed; the gripper is driven, not planned."""
    lower, upper = SEEEDSTUDIO_JOINT2_LIMITS
    return (
        RobotModel.from_file(SEEEDSTUDIO_MODEL_PATH)
        .with_fixed_joints("finger_left", "finger_right")
        .with_joint_position_limits("joint2", lower=lower, upper=upper)
        .with_default_joint_acceleration_limit(SEEEDSTUDIO_ACCELERATION_LIMIT)
    )


def seeedstudio_hardware(
    *,
    address: str | None = None,
    gravity_comp: bool = False,
) -> HardwareComponent:
    """Configure the B601-DM, running mock hardware while no address reaches it.

    The address is the serial port of the Damiao USB-CAN bridge shipped with
    the arm (``/dev/ttyACM0``) or a SocketCAN interface (``can0``), given here
    or through the coordinator's ``addresses`` under ``"arm"``.
    """
    return HardwareComponent(
        hardware_id=SEEEDSTUDIO_HARDWARE_ID,
        hardware_type=HardwareType.WHOLE_BODY,
        joints=list(SEEEDSTUDIO_JOINTS),
        adapter_type="seeedstudio_b601_dm",
        address=address,
        auto_enable=True,
        adapter_kwargs={"runtime_config": DamiaoRuntimeConfig(gravity_comp=gravity_comp)},
        wb_config=WholeBodyConfig(kp=SEEEDSTUDIO_KP, kd=SEEEDSTUDIO_KD),
        mock_limits=JointLimits(
            position_lower=[*([None] * SEEEDSTUDIO_DOF), 0.0],
            position_upper=[*([None] * SEEEDSTUDIO_DOF), 1.0],
            velocity_max=[None] * len(SEEEDSTUDIO_JOINTS),
        ),
    )


def seeedstudio_gripper_task() -> TaskConfig:
    """Normalized open/close control of the gripper joint (0.0 closed, 1.0 open)."""
    return TaskConfig(
        name=f"{SEEEDSTUDIO_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[SEEEDSTUDIO_GRIPPER_JOINT],
        priority=20,
    )


def make_seeedstudio_model_config(
    *,
    home_joints: list[float] | None = None,
) -> RobotModelConfig:
    """Build the planning config for the arm joints of the vendor model."""
    return RobotModelConfig(
        model=seeedstudio_model(),
        joint_names=list(SEEEDSTUDIO_ARM_JOINTS),
        base_link="base_link",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=SEEEDSTUDIO_ARM_JOINTS,
                base_link="base_link",
                tip_link="end_link",
            )
        ],
        auto_convert_meshes=True,
        gripper_hardware_id=SEEEDSTUDIO_HARDWARE_ID,
        home_joints=home_joints or [0.0] * SEEEDSTUDIO_DOF,
    )
