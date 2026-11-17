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

from dataclasses import replace
import math

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.gripper import GripperConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
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
from dimos.robot.manipulators.feetech.robot import (
    FeetechBus,
    FeetechMotor,
    FeetechRobot,
    FeetechTrigger,
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

# The table under both arms, starting 10 cm out so the bases stay clear:
# (top_z, center_x, center_y, size_x, size_y), metres. Closed fingertips touched it at
# (0.25, +-0.20) on 2026-11-16 at -0.039 (left) and -0.044 (right) commanded; the top
# sits between them, about 3 mm off for each arm. The overhead depth sees the table
# about 2 mm lower on the right; the rest is the two arms' calibrations differing.
DUAL_OPENYAM_TABLE_BOXES = ((-0.042, 0.40, 0.0, 0.60, 1.20),)
# Home with each arm's fingers pointing down, its fingertip centre over its own half of
# the table at (0.25, +-0.20, 0.15); solved by IK on the tcp model, left arm first. A
# position-only move_to_pose keeps the gripper's angle, so from here it stays fingers-down.
DUAL_OPENYAM_FINGERS_DOWN_HOME_JOINTS = [
    *(-0.0857, 1.5307, 1.4718, -1.4627, 0.0, -0.0857),
    *(-0.0214, 1.4166, 1.3760, -1.4335, -0.0655, 0.0013),
]
# Where the gripper's parts sit along the approach from the tcp, metres, for the
# heuristic grasp: the closed pads' centre behind the tcp (the printed finger's extra
# 9 mm), the fingertips' reach past it and the gripper body behind it, the last two
# read off the collision meshes (0.8 mm and 82.5 mm).
DUAL_OPENYAM_HEURISTIC_AIM = {
    "pad_behind_tcp": 0.009,
    "fingertip_reach": 0.001,
    "palm_behind_tcp": 0.0825,
}
# Grasp frame to the closed pads' centre, metres: the pad centre sits 3.7 cm beyond the
# grasp frame and the rig's printed fingers reach 9 mm further.
DUAL_OPENYAM_GRASP_FRAME_TO_PADS = {side: 0.037 + 0.009 for side in DUAL_OPENYAM_SIDES}
# The same gripper in GraspGenX's convention: origin on the gripper link, approach
# along +Z, jaws closing along X. The open jaws span 98 mm (calipers between the nose
# faces); the pads' centre is 0.10 m past the gripper link plus the grasp-frame offset.
_GRASPGENX_PAD_DEPTH = 0.10 + 0.037
DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.098, 0.028, 0.020),
    "offset_open": (0.0, 0.0, _GRASPGENX_PAD_DEPTH),
    "extents_half_open": (0.049, 0.028, 0.020),
    "offset_half_open": (0.0, 0.0, _GRASPGENX_PAD_DEPTH),
    "fingertip_depth": 0.10 + DUAL_OPENYAM_GRASP_FRAME_TO_PADS["left"],
}
# GraspGenX frame -> {side}_tcp: the quarter turn that takes GraspGenX's X closing axis
# onto the gripper's Y, then out along the approach to the fingertip the tcp sits on.
DUAL_OPENYAM_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, DUAL_OPENYAM_GRIPPER_SWEEP_VOLUME["fingertip_depth"]),
    (0.0, 0.0, 0.0, 1.0),
)
# Overhead D405 colour optical pose from the 2026-10-15 calibration, expressed as camera_link.
DUAL_OPENYAM_OVERHEAD_CAMERA_TRANSFORM = Transform(
    translation=Vector3(x=-0.01684, y=0.01262, z=0.41508),
    rotation=Quaternion(-0.02168, 0.47733, -0.00152, 0.87845),
    frame_id="world",
    child_frame_id="camera_link",
)

# These pairs overlap by a few mm at the folded rest pose. The pads meet at zero and the
# wrist nests in the gripper body, so those hulls always touch too.
DUAL_OPENYAM_COLLISION_EXCLUSIONS = [
    *(
        (f"{side}_link{a}", f"{side}_link{b}")
        for side in DUAL_OPENYAM_SIDES
        for a, b in ((1, 3), (2, 4))
    ),
    *((f"{side}_tip_left", f"{side}_tip_right") for side in DUAL_OPENYAM_SIDES),
    *((f"{side}_link4", f"{side}_gripper") for side in DUAL_OPENYAM_SIDES),
]

# The workcell model's finger joints: the planner's grasp contact check slides them;
# the connection and the map's self filter hold them fixed.
DUAL_OPENYAM_FINGER_JOINTS = [
    f"{side}_finger_{jaw}_joint" for side in DUAL_OPENYAM_SIDES for jaw in ("left", "right")
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
            joint: (DUAL_OPENYAM_FINGER_OPEN, 0.0) for joint in DUAL_OPENYAM_FINGER_JOINTS
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


def dual_openyam_tcp_model_config(*, robot: str | None = None) -> RobotModelConfig:
    """The planning model with each arm's tip at its fingertip centre, ``{side}_tcp``.

    A pose sent to an arm then places the closed pads' centre, and fingers-down is
    roll pi: the tcp's +Z points out of the fingers. Home is fingers-down.
    """
    config = dual_openyam_model_config(robot=robot)
    model = config.model
    for side in DUAL_OPENYAM_SIDES:
        model = model.with_fixed_frame(
            f"{side}_tcp",
            f"{side}_grasp_frame",
            xyz=(0.0, 0.0, -DUAL_OPENYAM_GRASP_FRAME_TO_PADS[side]),
            rpy=(0.0, math.pi, 0.0),
        )
    config.model = model
    config.planning_groups = [
        replace(group, tip_link=f"{group.name.split('_')[0]}_tcp")
        for group in config.planning_groups
    ]
    config.home_joints = list(DUAL_OPENYAM_FINGERS_DOWN_HOME_JOINTS)
    return config


def dual_openyam_control_model(*, robot: str | None = None) -> RobotModel:
    """The model the connection and coordinator use: the arms plus one opening joint per jaw.

    Each jaw is a made-up joint, "left_gripper" or "right_gripper", from 0
    (closed) to 1 (open); the URDF's finger joints are the planner's, for the
    grasp contact check.

    Args:
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/left_joint1" and "arm/left_gripper".
    """
    model = (
        dual_openyam_model_config()
        .model.with_fixed_joints(*DUAL_OPENYAM_FINGER_JOINTS)
        .with_added_joints(
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

# dual_openyam.urdf position limits for joint1 to joint6; both arms share them.
DUAL_OPENYAM_JOINT_LIMITS = (
    (-2.618, 3.1416),
    (0.0, 3.6652),
    (0.0, 3.1416),
    (-1.693, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
)
# Trigger angle from released to fully squeezed on the Dimensional leaders.
DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS = 1.5


def _leader_bus(side: str, *, trigger: bool = True) -> FeetechBus:
    # Leader servos joint_1 to joint_6 drive the arm joints in order; joint_7
    # is the gripper trigger.
    return FeetechBus(
        side,
        tuple(
            FeetechMotor(f"joint_{index}", joint, lower=lower, upper=upper)
            for index, (joint, (lower, upper)) in enumerate(
                zip(dual_openyam_arm_joints(side), DUAL_OPENYAM_JOINT_LIMITS, strict=True), start=1
            )
        ),
        trigger=(
            FeetechTrigger("joint_7", DUAL_OPENYAM_LEADER_TRIGGER_TRAVEL_RADIANS)
            if trigger
            else None
        ),
    )


#: The two YAM leader arms, one Feetech bus each.
DUAL_OPENYAM_LEADERS = FeetechRobot(buses=tuple(_leader_bus(side) for side in DUAL_OPENYAM_SIDES))
#: The two YAM leader arms without their trigger servos.
DUAL_OPENYAM_ARM_LEADERS = FeetechRobot(
    buses=tuple(_leader_bus(side, trigger=False) for side in DUAL_OPENYAM_SIDES)
)
