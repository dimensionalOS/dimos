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

"""Two Metal arms, their canonical names, planning model and robot data."""

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.damiao.robot import DamiaoBus, DamiaoRobot
from dimos.robot.manipulators.makermods_metal.config import (
    METAL_ARM_JOINTS,
    METAL_FINGER_JOINTS,
    make_metal_model_config,
    metal_gripper,
    metal_motors,
)

SIDES = ("left", "right")
ARM_JOINTS = {side: tuple(f"{side}_{joint}" for joint in METAL_ARM_JOINTS) for side in SIDES}
ALL_ARM_JOINTS = (*ARM_JOINTS["left"], *ARM_JOINTS["right"])
#: Each jaw's finger joints, e.g. left_joint7 driven and left_joint8 copying it.
FINGER_JOINTS = {side: tuple(f"{side}_{joint}" for joint in METAL_FINGER_JOINTS) for side in SIDES}

DUAL_METAL_BASE = RobotModel.from_root_link("dual_metal_base", name="dual_makermods_metal")

Vector3 = tuple[float, float, float]
# Example bench layout only; no published official bimanual mount transform was found.
DEFAULT_LEFT_XYZ: Vector3 = (0.0, 0.3, 0.0)
DEFAULT_RIGHT_XYZ: Vector3 = (0.0, -0.3, 0.0)
ZERO_RPY: Vector3 = (0.0, 0.0, 0.0)


def dual_metal_model_config(
    *,
    left_xyz: Vector3 = DEFAULT_LEFT_XYZ,
    right_xyz: Vector3 = DEFAULT_RIGHT_XYZ,
    left_rpy: Vector3 = ZERO_RPY,
    right_rpy: Vector3 = ZERO_RPY,
    robot: str | None = None,
) -> RobotModelConfig:
    """A single collision world and independently selectable pose groups for both arms.

    Args:
        left_xyz: Position of the left arm's base in the shared frame.
        right_xyz: Position of the right arm's base in the shared frame.
        left_rpy: Orientation of the left arm's base.
        right_rpy: Orientation of the right arm's base.
        robot: Name the moving joints under this robot, e.g. "arm" gives
            "arm/left_joint1" and "arm/left_joint7".
    """
    if left_xyz == right_xyz:
        raise ValueError("Metal arm bases must have distinct positions")
    single = make_metal_model_config().model
    return named_for_robot(
        RobotModelConfig(
            model=DUAL_METAL_BASE.with_attached_model(
                single,
                parent_link="dual_metal_base",
                joint_name="left_mount",
                prefix="left_",
                xyz=left_xyz,
                rpy=left_rpy,
            ).with_attached_model(
                single,
                parent_link="dual_metal_base",
                joint_name="right_mount",
                prefix="right_",
                xyz=right_xyz,
                rpy=right_rpy,
            ),
            joint_names=list(ALL_ARM_JOINTS),
            base_link="dual_metal_base",
            planning_groups=[
                PlanningGroupDefinition(
                    name=f"{side}_arm",
                    joint_names=ARM_JOINTS[side],
                    base_link="dual_metal_base",
                    tip_link=f"{side}_gripper_base",
                )
                for side in SIDES
            ],
            auto_convert_meshes=True,
        ),
        robot,
        fingers=[*FINGER_JOINTS["left"], *FINGER_JOINTS["right"]],
    )


def _bus(side: str) -> DamiaoBus:
    return DamiaoBus(
        arm=metal_motors(prefix=f"{side}_"),
        kp=[100.0, 100.0, 100.0, 50.0, 20.0, 20.0],
        kd=[5.0, 5.0, 5.0, 5.0, 0.6, 0.6],
        gripper=metal_gripper(f"{side}_gripper", FINGER_JOINTS[side][0]),
        name=f"{side}_arm",
    )


#: Two Metal arms, each with its own jaw, on two Damiao buses.
_DUAL_METAL_MODEL = dual_metal_model_config().model
DUAL_METAL = DamiaoRobot(
    buses=[_bus(side) for side in SIDES],
    model=_DUAL_METAL_MODEL,
    gravity_model=_DUAL_METAL_MODEL,
    base_link="dual_metal_base",
    joint_names=[*ALL_ARM_JOINTS, *(FINGER_JOINTS[side][0] for side in SIDES)],
    gravity=False,
)
