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

"""Arm-only Dual OpenYAM canonical planning model configuration and robot data."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import JointDescription, RobotModel
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoGripper,
    DamiaoMotor,
    DamiaoRobot,
)
from dimos.robot.manipulators.dual_openyam.joints import (
    DUAL_OPENYAM_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS,
)
from dimos.robot.manipulators.dual_openyam.model import (
    DUAL_OPENYAM_MODEL,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS

DUAL_OPENYAM_SIDES = ("left", "right")
DUAL_OPENYAM_HOME_PER_ARM = list(OPENYAM_HOME_JOINTS)
DUAL_OPENYAM_HOME_JOINTS = [*DUAL_OPENYAM_HOME_PER_ARM, *DUAL_OPENYAM_HOME_PER_ARM]
DUAL_OPENYAM_KP = [80.0, 80.0, 80.0, 10.0, 10.0, 10.0]
DUAL_OPENYAM_KD = [5.0, 5.0, 5.0, 1.5, 1.5, 1.5]


def dual_openyam_arm_joints(side: str) -> list[str]:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return [f"{side}_joint{index}" for index in range(1, 7)]


def dual_openyam_model_config(*, robot: str | None = None) -> RobotModelConfig:
    """Build the combined arm-only planning model."""
    return named_for_robot(
        RobotModelConfig(
            model=DUAL_OPENYAM_MODEL.with_default_joint_acceleration_limit(1.0),
            joint_names=list(DUAL_OPENYAM_ARM_JOINTS),
            base_link="dual_openyam_base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="left_manipulator",
                    joint_names=tuple(dual_openyam_arm_joints("left")),
                    base_link="dual_openyam_base",
                    tip_link="left_grasp_frame",
                ),
                PlanningGroupDefinition(
                    name="right_manipulator",
                    joint_names=tuple(dual_openyam_arm_joints("right")),
                    base_link="dual_openyam_base",
                    tip_link="right_grasp_frame",
                ),
            ],
            auto_convert_meshes=True,
            home_joints=list(DUAL_OPENYAM_HOME_JOINTS),
            tf_extra_links=[],
        ),
        robot,
    )


def dual_openyam_control_model(*, robot: str | None = None) -> RobotModel:
    """The model the connection and coordinator use: the arms plus one opening joint per jaw.

    The dual URDF has its finger joints fixed, so each jaw is a made-up joint,
    "left_gripper" or "right_gripper", from 0 (closed) to 1 (open).

    Args:
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/left_joint1" and "arm/left_gripper".
    """
    model = dual_openyam_model_config().model.with_added_joints(
        *[
            JointDescription(
                name=name,
                type="revolute",
                parent_link="dual_openyam_base",
                child_link=f"{name}_control_link",
                lower=0.0,
                upper=1.0,
                velocity=0.0,
                effort=0.0,
            )
            for name in DUAL_OPENYAM_GRIPPER_JOINTS
        ]
    )
    if robot is None:
        return model
    return model.with_renamed_joints(
        {
            name: f"{robot}/{name}"
            for name in [*DUAL_OPENYAM_ARM_JOINTS, *DUAL_OPENYAM_GRIPPER_JOINTS]
        }
    )


def _bus(side: str) -> DamiaoBus:
    joints = dual_openyam_arm_joints(side)
    types = ["DM4340"] * 3 + ["DM4310"] * 3
    return DamiaoBus(
        arm=[
            DamiaoMotor(name, motor_type, index, index + 0x10)
            for index, (name, motor_type) in enumerate(zip(joints, types, strict=True), start=1)
        ],
        kp=list(DUAL_OPENYAM_KP),
        kd=list(DUAL_OPENYAM_KD),
        gripper=DamiaoGripper(
            name=f"{side}_gripper",
            joint=f"{side}_gripper",
            open_at="upper",
            motor=DamiaoMotor(f"{side}_gripper", "DM4310", 0x08, 0x18),
            opening="decreasing_position",
            current=0.15,
        ),
        name=f"{side}_arm",
    )


#: Two OpenYAM arms, each with its own jaw, on two Damiao buses.
DUAL_OPENYAM = DamiaoRobot(
    buses=[_bus(side) for side in DUAL_OPENYAM_SIDES],
    model=dual_openyam_control_model(),
    gravity_model=DUAL_OPENYAM_MODEL,
    base_link="dual_openyam_base",
    joint_names=[*DUAL_OPENYAM_ARM_JOINTS, *DUAL_OPENYAM_GRIPPER_JOINTS],
    gravity=True,
    home_joints=list(DUAL_OPENYAM_HOME_JOINTS),
)
