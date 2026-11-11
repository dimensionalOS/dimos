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
from dimos.manipulation.planning.spec.gripper import GripperConfig
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
# MIT gains from the step sweeps with the fitted gravity model
# (dual_openyam_calibration_2026-10-15.json, control_gains); stock was 80/5 arm, 10/1.5 wrist.
# The shoulder runs at the elbow's 240 rather than the swept 120: with the fitted
# masses both arms held 1.4 to 1.7 deg high at joint 2. Wrist kd stays near 1.5:
# 3 and above buzzes at 100 Hz.
DUAL_OPENYAM_KP = [120.0, 240.0, 240.0, 80.0, 80.0, 80.0]
DUAL_OPENYAM_KD = [5.0, 5.0, 5.0, 1.5, 1.5, 1.5]

# These pairs overlap by a few mm at the folded rest pose.
DUAL_OPENYAM_COLLISION_EXCLUSIONS = [
    (f"{side}_link{a}", f"{side}_link{b}")
    for side in DUAL_OPENYAM_SIDES
    for a, b in ((1, 3), (2, 4))
]

# Finger joint position (metres) at fully open; zero is closed with the pads touching.
# Calipers on the rig's printed fingers: 98 mm between the nose faces at full open, so
# 49 mm of travel per finger (the i2rt travel of 46.95 mm gave 93.9 mm).
DUAL_OPENYAM_FINGER_OPEN = -0.049


def dual_openyam_arm_joints(side: str) -> list[str]:
    if side not in DUAL_OPENYAM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    return [f"{side}_joint{index}" for index in range(1, 7)]


def dual_openyam_gripper_config() -> GripperConfig:
    """Which OpenYAM links may touch a grasped object and how the fingers close.

    For the pick's grasp contact check: only the fingertip links may touch the
    object, and the check slides the four finger joints from open to closed to
    drop grasps that hit the table or a neighbour, or close on nothing.
    """
    return GripperConfig(
        pad_links=tuple(
            f"{side}_tip_{jaw}" for side in DUAL_OPENYAM_SIDES for jaw in ("left", "right")
        ),
        finger_joints={
            f"{side}_finger_{jaw}_joint": (DUAL_OPENYAM_FINGER_OPEN, 0.0)
            for side in DUAL_OPENYAM_SIDES
            for jaw in ("left", "right")
        },
    )


def dual_openyam_model_config(*, robot: str | None = None) -> RobotModelConfig:
    """Build the combined arm-only planning model."""
    return named_for_robot(
        RobotModelConfig(
            model=DUAL_OPENYAM_MODEL.with_default_joint_acceleration_limit(1.0),
            joint_names=list(DUAL_OPENYAM_ARM_JOINTS),
            base_link="dual_openyam_base",
            collision_exclusion_pairs=list(DUAL_OPENYAM_COLLISION_EXCLUSIONS),
            gripper=dual_openyam_gripper_config(),
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

    Each jaw is a made-up joint, "left_gripper" or "right_gripper", from 0
    (closed) to 1 (open); the URDF's finger joints are the planner's, for the
    grasp contact check.

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
