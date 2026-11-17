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

"""Pinned OpenArm geometry and canonical planning configuration."""

from __future__ import annotations

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
from dimos.robot.manipulators.feetech.robot import (
    FeetechBus,
    FeetechMotor,
    FeetechRobot,
    FeetechTrigger,
)

OPENARM_DESCRIPTION_URL = "https://github.com/enactic/openarm_description"
OPENARM_DESCRIPTION_REF = "1fba2cbc05001f05b4514120b70130b4ac06f409"

OPENARM_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url=OPENARM_DESCRIPTION_URL,
    ref=OPENARM_DESCRIPTION_REF,
)
OPENARM_DESCRIPTION_ROOT = OPENARM_DESCRIPTION_SOURCE / "."
OPENARM_BIMANUAL_XACRO = (
    OPENARM_DESCRIPTION_SOURCE
    / "assets"
    / "robot"
    / "openarm_v2.0"
    / "urdf"
    / "openarm_v20.urdf.xacro"
)

OPENARM_BIMANUAL_MODEL = RobotModel.from_file(
    OPENARM_BIMANUAL_XACRO,
    package_paths={"openarm_description": OPENARM_DESCRIPTION_ROOT},
    xacro_args={
        "robot_preset": "default_bimanual",
        "emit_grasp_frame": "true",
    },
).with_default_joint_acceleration_limit(1.0)

OPENARM_DOF = 7
OPENARM_SIDES = ("left", "right")
# Connection order is all left arm joints, all right arm joints, then grippers.
OPENARM_LEFT_ARM_JOINTS = [f"openarm_left_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_RIGHT_ARM_JOINTS = [f"openarm_right_joint{i}" for i in range(1, OPENARM_DOF + 1)]
OPENARM_ARM_JOINTS = [*OPENARM_LEFT_ARM_JOINTS, *OPENARM_RIGHT_ARM_JOINTS]
# Each jaw turns on finger_joint1, in rad; the URDF has finger_joint2 copy it.
# The left jaw is open at its joint's upper end, the right at its lower end.
OPENARM_GRIPPER_JOINTS = [f"openarm_{side}_finger_joint1" for side in OPENARM_SIDES]
OPENARM_FINGER_JOINTS = [
    f"openarm_{side}_finger_joint{index}" for side in OPENARM_SIDES for index in (1, 2)
]
OPENARM_JOINTS = [*OPENARM_ARM_JOINTS, *OPENARM_GRIPPER_JOINTS]
OPENARM_HOME_JOINTS = [0.0] * len(OPENARM_ARM_JOINTS)
OPENARM_GRIPPER_COLLISION_EXCLUSIONS = [
    (f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2") for side in OPENARM_SIDES
]


def validate_side(side: str) -> None:
    if side not in OPENARM_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


def openarm_urdf_joints(side: str) -> list[str]:
    validate_side(side)
    return [f"openarm_{side}_joint{i}" for i in range(1, OPENARM_DOF + 1)]


def openarm_bimanual_model_config(*, robot: str | None = None) -> RobotModelConfig:
    """Build the single fourteen-joint planning model with one group per arm.

    Collision exclusions cannot span robots, so both arms plan as one robot.

    Args:
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/openarm_left_joint1" and "arm/openarm_left_finger_joint1".
    """
    canonical_joint_names = [*openarm_urdf_joints("left"), *openarm_urdf_joints("right")]
    return named_for_robot(
        RobotModelConfig(
            model=OPENARM_BIMANUAL_MODEL,
            joint_names=canonical_joint_names,
            base_link="openarm_body_link0",
            planning_groups=[
                PlanningGroupDefinition(
                    name="left_arm",
                    joint_names=tuple(openarm_urdf_joints("left")),
                    base_link="openarm_body_link0",
                    tip_link="openarm_left_grasp_frame",
                ),
                PlanningGroupDefinition(
                    name="right_arm",
                    joint_names=tuple(openarm_urdf_joints("right")),
                    base_link="openarm_body_link0",
                    tip_link="openarm_right_grasp_frame",
                ),
                PlanningGroupDefinition(
                    name="both_arms",
                    joint_names=tuple(canonical_joint_names),
                    base_link="openarm_body_link0",
                ),
            ],
            collision_exclusion_pairs=OPENARM_GRIPPER_COLLISION_EXCLUSIONS,
            auto_convert_meshes=True,
            home_joints=list(OPENARM_HOME_JOINTS),
        ),
        robot,
        fingers=OPENARM_FINGER_JOINTS,
    )


OPENARM_KP = [100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0]
OPENARM_KD = [1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8]


def _bus(side: str) -> DamiaoBus:
    types = ["DM8009"] * 2 + ["DM4340"] * 2 + ["DM4310"] * 3
    joints = openarm_urdf_joints(side)
    return DamiaoBus(
        arm=[
            DamiaoMotor(name, motor_type, index, index + 0x10)
            for index, (name, motor_type) in enumerate(zip(joints, types, strict=True), start=1)
        ],
        kp=list(OPENARM_KP),
        kd=list(OPENARM_KD),
        gripper=DamiaoGripper(
            name=f"{side}_gripper",
            joint=f"openarm_{side}_finger_joint1",
            open_at="upper" if side == "left" else "lower",
            motor=DamiaoMotor(f"openarm_{side}_gripper", "DM4310", 0x08, 0x18),
            opening="decreasing_position",
            current=0.15,
        ),
        name=f"{side}_arm",
    )


#: Both OpenArm arms and jaws, each side on its own Damiao bus.
OPENARM = DamiaoRobot(
    buses=[_bus(side) for side in OPENARM_SIDES],
    model=openarm_bimanual_model_config().model,
    gravity_model=OPENARM_BIMANUAL_MODEL,
    base_link="openarm_body_link0",
    joint_names=[*OPENARM_ARM_JOINTS, *OPENARM_GRIPPER_JOINTS],
    gravity=True,
    home_joints=list(OPENARM_HOME_JOINTS),
)

# Inside the OpenArm URDF limits, which the connection enforces by refusing
# the whole command; clamping keeps a leader resting on a stop from stalling
# every joint. Joint 4 (elbow) has no negative range.
OPENARM_MINI_JOINT_LIMITS: dict[str, tuple[tuple[float, float], ...]] = {
    "left": (
        (-3.45, 1.35),
        (-3.30, 0.15),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
    "right": (
        (-1.35, 3.45),
        (-0.15, 3.30),
        (-1.50, 1.50),
        (0.0, 2.40),
        (-1.50, 1.50),
        (-0.75, 0.75),
        (-1.50, 1.50),
    ),
}
# Squeezing the trigger this far closes the jaw fully.
OPENARM_MINI_TRIGGER_TRAVEL_RADIANS = 1.0


def _mini_bus(side: str, *, trigger: bool = True) -> FeetechBus:
    # Leader servos joint_1 to joint_7 (IDs 1 to 7) drive the arm joints in
    # order; the gripper trigger is at ID 8.
    return FeetechBus(
        side,
        tuple(
            FeetechMotor(f"joint_{index}", joint, lower=lower, upper=upper)
            for index, (joint, (lower, upper)) in enumerate(
                zip(openarm_urdf_joints(side), OPENARM_MINI_JOINT_LIMITS[side], strict=True),
                start=1,
            )
        ),
        trigger=FeetechTrigger("gripper", OPENARM_MINI_TRIGGER_TRAVEL_RADIANS) if trigger else None,
    )


#: The two OpenArm Mini leader arms, one Feetech bus each.
OPENARM_MINI_LEADERS = FeetechRobot(buses=tuple(_mini_bus(side) for side in OPENARM_SIDES))
#: The same leaders with the trigger servo removed, e.g. for a Joy-Con in its place.
OPENARM_MINI_ARM_LEADERS = FeetechRobot(
    buses=tuple(_mini_bus(side, trigger=False) for side in OPENARM_SIDES)
)
