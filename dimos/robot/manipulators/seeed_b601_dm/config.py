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

"""Seeed B601-DM model and planning configuration."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoGripper,
    DamiaoMotor,
    DamiaoRobot,
)
from dimos.robot.manipulators.seeed_b601_dm.joints import (
    B601_DM_ARM_JOINTS,
    B601_DM_DOF,
    B601_DM_GRIPPER_JOINT,
)

B601_DM_HARDWARE_ID = "b601_dm"
B601_DM_GRIPPER_NAME = "gripper"
# Measured on the arm: the gripper motor travels 6.39 rad between its stops,
# and opening moves it toward more negative positions. Calibration drives
# past the full stroke so it reaches both stops from any start.
GRIPPER_CALIBRATION_TRAVEL_RAD = 7.0
GRIPPER_CURRENT = 0.15
B601_DM_SOURCE = RobotDescriptionSource(
    url="https://github.com/Seeed-Projects/reBot-DevArm",
    ref="8494bdfa91f4e3712bb7e7743e7bbcc63b01b9af",
)
# At its folded mechanical rest this arm's joint2 encoder reads up to about
# +0.008 rad, just past the vendor URDF's 0.0 upper limit, which would put
# the planner's start state out of bounds.
B601_DM_JOINT2_LIMITS = (-3.14, 0.02)
B601_DM_ACCELERATION_LIMIT = 0.5
# Replace the vendor's 50-200 rad/s limits for both joint and Cartesian planning.
B601_DM_VELOCITY_LIMITS = dict(zip(B601_DM_ARM_JOINTS, (0.2, 0.2, 0.2, 0.8, 0.8, 0.8), strict=True))
# Lazy: the description repository is fetched on first load, not on import.
B601_DM_MODEL = (
    RobotModel.from_file(B601_DM_SOURCE / "Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf")
    .with_fixed_joints("finger_left", "finger_right")
    .with_joint_position_limits(
        "joint2", lower=B601_DM_JOINT2_LIMITS[0], upper=B601_DM_JOINT2_LIMITS[1]
    )
    .with_joint_velocity_limits(B601_DM_VELOCITY_LIMITS)
    .with_default_joint_acceleration_limit(B601_DM_ACCELERATION_LIMIT)
)


def b601_dm_gripper_task() -> TaskConfig:
    """Normalized open/close control of the gripper joint (0.0 closed, 1.0 open)."""
    return TaskConfig(
        name=f"{B601_DM_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[B601_DM_GRIPPER_JOINT],
        priority=20,
    )


def b601_dm_model_config(
    *,
    home_joints: list[float] | None = None,
    robot: str | None = None,
) -> RobotModelConfig:
    """Build the planning config for the arm joints of the vendor model.

    Args:
        home_joints: The arm's home posture; defaults to all zeros.
        robot: Name the moving joints under this robot, e.g. "arm".
    """
    return named_for_robot(
        RobotModelConfig(
            model=B601_DM_MODEL,
            joint_names=list(B601_DM_ARM_JOINTS),
            base_link="base_link",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator",
                    joint_names=B601_DM_ARM_JOINTS,
                    base_link="base_link",
                    tip_link="end_link",
                )
            ],
            auto_convert_meshes=True,
            home_joints=list(home_joints or [0.0] * B601_DM_DOF),
        ),
        robot,
    )


def _arm_motors() -> list[DamiaoMotor]:
    # Seeed's motor table. DM4340P uses the DM4340 driver profile.
    types = ["DM4340"] * 3 + ["DM4310"] * 3
    return [
        DamiaoMotor(f"joint{index}", motor_type, index, index + 0x10)
        for index, motor_type in enumerate(types, start=1)
    ]


#: One B601-DM arm and its calibrated jaw on one Damiao CAN bus.
B601_DM = DamiaoRobot(
    buses=[
        DamiaoBus(
            arm=_arm_motors(),
            # Seeed's SDK MIT gains; the shoulder kd is capped at the shared
            # codec's 5.0 maximum. The gripper runs its own current-limited
            # control.
            kp=[120.0, 120.0, 120.0, 18.0, 18.0, 18.0],
            kd=[5.0, 5.0, 5.0, 2.0, 2.0, 2.0],
            gripper=DamiaoGripper(
                name=B601_DM_GRIPPER_NAME,
                motor=DamiaoMotor("gripper_motor", "DM4310", 0x07, 0x17),
                opening="decreasing_position",
                current=GRIPPER_CURRENT,
                calibration_travel=GRIPPER_CALIBRATION_TRAVEL_RAD,
            ),
            name="arm",
        )
    ],
    model=b601_dm_model_config().model,
    gravity_model=B601_DM_MODEL,
    base_link="base_link",
    joint_names=[*B601_DM_ARM_JOINTS, B601_DM_GRIPPER_NAME],
    gravity=True,
    home_joints=[0.0] * B601_DM_DOF,
)
