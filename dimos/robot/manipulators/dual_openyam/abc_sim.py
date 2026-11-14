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

"""Both OpenYAM arms in ABC Sim's bimanual scenes (``data/abc_<task>``, see
``data/abc_yam/SOURCE.md``).

The scenes keep ABC's 14 position actuators in their own order, which is the order of
``DUAL_OPENYAM_SIM_JOINTS``: each arm's six joints, then its gripper. A gripper value is
the driven finger's slide in metres, 0.0 closed to 0.0475 open. The planning model,
``abc_yam/dual_yam.urdf``, is generated from the simulator's MJCF, so planned and
simulated poses agree.
"""

from __future__ import annotations

import math
from pathlib import Path

from dimos.control.components import HardwareComponent, HardwareType
from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators._modeling import named_for_robot
from dimos.robot.manipulators.openyam.config import OPENYAM_HOME_JOINTS
from dimos.utils.data import LfsPath, get_data

DUAL_OPENYAM_SIM_ROBOT = "arm"
DUAL_OPENYAM_SIM_SIDES = ("left", "right")
DUAL_OPENYAM_SIM_MODEL_PATH = LfsPath("abc_yam") / "dual_yam.urdf"
DUAL_OPENYAM_SIM_SCENE = LfsPath("abc_put_relative/scene.xml")
#: World position of the model's ``dual_base``, midway between ABC's two arm bases. The
#: table top, and so the bases, is at 0.75 m in some ABC scenes and 0.76 m in others.
DUAL_OPENYAM_SIM_BASE_XY = (0.2525, 0.0)
DUAL_OPENYAM_SIM_GRIPPER_OPEN_M = 0.0475
# ABC's yam.xml joint ranges, joint1..joint6.
_ARM_LOWER = [-2.61799, 0.0, 0.0, -1.5708, -1.5708, -2.0944]
_ARM_UPPER = [3.05433, 3.66519, 3.66519, 1.5708, 1.5708, 2.0944]


class AbcScenePath(LfsPath):
    """``data/<scene>/scene.xml`` of an ABC scene, pulled together with ``abc_yam``: the
    scenes load their arm meshes from ``abc_yam/meshes``. Resolve it in the launching
    process (it is not meant to cross into a module's config)."""

    def _ensure_downloaded(self) -> Path:
        get_data("abc_yam")
        return LfsPath._ensure_downloaded(self)


def abc_scene(name: str) -> AbcScenePath:
    """The scene file of ABC scene archive ``name``, e.g. ``abc_pour``."""
    return AbcScenePath(f"{name}/scene.xml")


def dual_openyam_sim_arm_joints(side: str) -> list[str]:
    """``arm/left_joint1``..``arm/left_joint6``, or the right arm's."""
    return [f"{DUAL_OPENYAM_SIM_ROBOT}/{side}_joint{i}" for i in range(1, 7)]


def dual_openyam_sim_gripper_joint(side: str) -> str:
    return f"{DUAL_OPENYAM_SIM_ROBOT}/{side}_gripper"


def dual_openyam_sim_gripper_task(side: str) -> str:
    return f"{side}_arm_gripper"


DUAL_OPENYAM_SIM_ARM_JOINTS = [
    joint for side in DUAL_OPENYAM_SIM_SIDES for joint in dual_openyam_sim_arm_joints(side)
]
DUAL_OPENYAM_SIM_JOINTS = [
    joint
    for side in DUAL_OPENYAM_SIM_SIDES
    for joint in (*dual_openyam_sim_arm_joints(side), dual_openyam_sim_gripper_joint(side))
]
#: ABC's home keyframe: both arms folded, both grippers closed.
DUAL_OPENYAM_SIM_RESET = [*OPENYAM_HOME_JOINTS, 0.0, *OPENYAM_HOME_JOINTS, 0.0]
DUAL_OPENYAM_SIM_GROUPS = {side: f"{side}_manipulator" for side in DUAL_OPENYAM_SIM_SIDES}


def dual_openyam_sim_base_pose(table_z: float = 0.75) -> PoseStamped:
    return PoseStamped(frame_id="world", position=Vector3(*DUAL_OPENYAM_SIM_BASE_XY, table_z))


def make_dual_openyam_sim_model_config(table_z: float = 0.75) -> RobotModelConfig:
    """Planning model for both simulated arms, one planning group per arm."""
    urdf_joints = [f"{side}_joint{i}" for side in DUAL_OPENYAM_SIM_SIDES for i in range(1, 7)]
    return named_for_robot(
        RobotModelConfig(
            model=RobotModel.from_file(
                DUAL_OPENYAM_SIM_MODEL_PATH
            ).with_default_joint_acceleration_limit(2.0),
            base_pose=dual_openyam_sim_base_pose(table_z),
            joint_names=urdf_joints,
            base_link="dual_base",
            planning_groups=[
                PlanningGroupDefinition(
                    name=DUAL_OPENYAM_SIM_GROUPS[side],
                    joint_names=tuple(f"{side}_joint{i}" for i in range(1, 7)),
                    base_link=f"{side}_base",
                    tip_link=f"{side}_grasp_frame",
                )
                for side in DUAL_OPENYAM_SIM_SIDES
            ],
            auto_convert_meshes=True,
            home_joints=[*OPENYAM_HOME_JOINTS, *OPENYAM_HOME_JOINTS],
            pre_grasp_offset=0.05,
        ),
        DUAL_OPENYAM_SIM_ROBOT,
    )


def dual_openyam_sim_hardware() -> HardwareComponent:
    """Both simulated arms as one device; it attaches to the blueprint's MujocoSimModule.

    The sim adapter reports every joint as +-pi, so the limits are given here: ABC's
    joint ranges, and each gripper's finger travel.
    """
    lower = [*_ARM_LOWER, 0.0, *_ARM_LOWER, 0.0]
    upper = [
        *_ARM_UPPER,
        DUAL_OPENYAM_SIM_GRIPPER_OPEN_M,
        *_ARM_UPPER,
        DUAL_OPENYAM_SIM_GRIPPER_OPEN_M,
    ]
    velocity = [*([math.pi] * 6), 0.0, *([math.pi] * 6), 0.0]
    return HardwareComponent(
        hardware_id=DUAL_OPENYAM_SIM_ROBOT,
        hardware_type=HardwareType.MANIPULATOR,
        joints=list(DUAL_OPENYAM_SIM_JOINTS),
        adapter_type="sim_mujoco",
        auto_enable=True,
        limits=JointLimits(position_lower=lower, position_upper=upper, velocity_max=velocity),
    )
