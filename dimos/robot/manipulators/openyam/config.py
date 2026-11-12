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

"""OpenYAM canonical planning model configuration and robot data."""

from __future__ import annotations

from pathlib import Path

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.assets.source import RobotDescriptionSource
from dimos.robot.manipulators._modeling import (
    joint_names,
    named_for_robot,
)
from dimos.robot.manipulators.damiao.robot import (
    DamiaoBus,
    DamiaoGripper,
    DamiaoMotor,
    DamiaoRobot,
)
from dimos.robot.manipulators.openyam.joints import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_DOF,
)
from dimos.utils.data import LfsPath

OPENYAM_HOME_JOINTS = [0.0, 1.047, 1.047, 0.0, 0.0, 0.0]
OPENYAM_DESCRIPTION_SOURCE = RobotDescriptionSource(
    url="https://github.com/dimensionalOS/dim_description",
    ref="f9853344bbc9fa00aade6ff3ffb7db6532d185e1",
)
OPENYAM_PACKAGE = OPENYAM_DESCRIPTION_SOURCE / "openyam_single"
OPENYAM_MODEL_PATH = OPENYAM_PACKAGE / "openyam_single.urdf"
OPENYAM_PACKAGE_PATHS: dict[str, Path] = {"openyam_single": OPENYAM_PACKAGE}
#: Joint damping while an operator guides the arm by hand. Stiffness is zero
#: and gravity is compensated, so the arm floats instead of holding a pose.
OPENYAM_TEACH_DAMPING = [2.0, 2.0, 2.0, 0.5, 0.5, 0.5]
OPENYAM_GRIPPER_NAME = "gripper"
#: The jaw's canonical finger joints, in m: joint7 is driven and joint8 copies
#: it. The managed model calls them hand_finger_joint1 and hand_finger_joint2;
#: both are fully open at their lower end and closed at their upper end.
OPENYAM_FINGER_JOINTS = ("joint7", "joint8")


def make_openyam_model_config(
    *,
    home_joints: list[float] | None = None,
    robot: str | None = None,
) -> RobotModelConfig:
    """Build the canonical visualization and planning config for OpenYAM.

    Args:
        home_joints: The arm's home posture; defaults to ``OPENYAM_HOME_JOINTS``.
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/yam_joint1" and "arm/joint7".
    """
    model_joint_names = joint_names(OPENYAM_DOF, prefix="yam_joint")
    model = (
        RobotModel.from_file(
            OPENYAM_MODEL_PATH,
            package_paths=OPENYAM_PACKAGE_PATHS,
        )
        .with_renamed_joints(
            {
                **dict(
                    zip(
                        joint_names(OPENYAM_DOF, prefix="arm_joint_"),
                        model_joint_names,
                        strict=True,
                    )
                ),
                # The managed model's jaw keeps its URDF mimic; only its name
                # is canonicalized to the robot's joint7/joint8.
                "hand_finger_joint1": OPENYAM_FINGER_JOINTS[0],
                "hand_finger_joint2": OPENYAM_FINGER_JOINTS[1],
            }
        )
        .with_fixed_frame("gripper_tip", "arm_tcp")
    )
    return named_for_robot(
        RobotModelConfig(
            model=model.with_default_joint_acceleration_limit(2.0),
            joint_names=model_joint_names,
            base_link="base",
            planning_groups=[
                PlanningGroupDefinition(
                    name="manipulator",
                    joint_names=tuple(model_joint_names),
                    base_link="base",
                    tip_link="gripper_tip",
                )
            ],
            auto_convert_meshes=True,
            collision_exclusion_pairs=[],
            home_joints=list(OPENYAM_HOME_JOINTS if home_joints is None else home_joints),
        ),
        robot,
        fingers=OPENYAM_FINGER_JOINTS,
    )


def _arm_motors() -> list[DamiaoMotor]:
    types = ["DM4340"] * 3 + ["DM4310"] * 3
    return [
        DamiaoMotor(f"yam_joint{index}", motor_type, index, index + 0x10)
        for index, motor_type in enumerate(types, start=1)
    ]


#: One OpenYAM arm and its gripper on a single Damiao bus.
OPENYAM = DamiaoRobot(
    buses=[
        DamiaoBus(
            arm=_arm_motors(),
            kp=[80.0, 80.0, 80.0, 10.0, 10.0, 10.0],
            kd=[5.0, 5.0, 5.0, 1.5, 1.5, 1.5],
            gripper=DamiaoGripper(
                name=OPENYAM_GRIPPER_NAME,
                joint=OPENYAM_FINGER_JOINTS[0],
                open_at="lower",
                motor=DamiaoMotor("yam_gripper", "DM4310", 0x08, 0x18),
                opening="decreasing_position",
                current=0.15,
            ),
            name="arm",
        )
    ],
    model=make_openyam_model_config().model,
    gravity_model=RobotModel.from_file(
        LfsPath("yam_description") / "urdf/yam_gripper_gravity.urdf"
    ),
    base_link="base",
    joint_names=[*OPENYAM_ARM_JOINTS, OPENYAM_FINGER_JOINTS[0]],
    gravity=True,
    teach_kd=list(OPENYAM_TEACH_DAMPING),
    passive_grippers=frozenset({OPENYAM_GRIPPER_NAME}),
    home_joints=list(OPENYAM_HOME_JOINTS),
)
