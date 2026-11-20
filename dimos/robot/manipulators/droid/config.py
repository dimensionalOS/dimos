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

"""RoboLab's DROID robot: a Franka Panda with a Robotiq 2F-85, as NVIDIA's RoboLab simulates it."""

from __future__ import annotations

import math

from dimos.control.components import HardwareComponent, HardwareType
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.utils.data import LfsPath

DROID_ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
# RoboLab's driven Robotiq joint: 0 open, pi/4 closed.
DROID_GRIPPER_JOINT = "finger_joint"
DROID_JOINTS = [*DROID_ARM_JOINTS, DROID_GRIPPER_JOINT]
# RoboLab's start pose for every task.
DROID_HOME_JOINTS = [0.0, -math.pi / 5, 0.0, -4 * math.pi / 5, 0.0, 3 * math.pi / 5, 0.0]
# data/robolab_droid: built from upstream Franka and Robotiq descriptions, matched to
# RoboLab's USD (see its SOURCE.md). The fingers are fixed open; grasp_frame is midway
# between the open pads, z along the approach and y along which the pads close.
DROID_MODEL_PATH = LfsPath("robolab_droid") / "droid.urdf"
DROID_HARDWARE_ID = "arm"
DROID_GRIPPER_TASK = "arm_gripper"

# The Robotiq's pads in GraspGenX's convention (approach +Z, jaws closing along X), the
# origin at RoboLab's gripper base_link: pads 0.1111-0.1491 m out, 85 mm apart open,
# 22 mm wide, 38 mm deep.
DROID_GRIPPER_SWEEP_VOLUME = {
    "extents_open": (0.085, 0.022, 0.038),
    "offset_open": (0.0, 0.0, 0.1301),
    "extents_half_open": (0.0425, 0.022, 0.038),
    "offset_half_open": (0.0, 0.0, 0.1301),
    "fingertip_depth": 0.1491,
}
# That origin -> grasp_frame, the planning tip: out the 0.1301 m to the pad midpoint, and
# the quarter turn from GraspGenX's X onto grasp_frame's y.
DROID_GRASP_FRAME_TO_TCP = (
    (0.0, 1.0, 0.0, 0.0),
    (-1.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.1301),
    (0.0, 0.0, 0.0, 1.0),
)


def make_droid_model_config() -> RobotModelConfig:
    """Planning model for the DROID robot, its base at the RoboLab world's origin."""
    return RobotModelConfig(
        model=RobotModel.from_file(DROID_MODEL_PATH).with_default_joint_acceleration_limit(2.0),
        base_pose=PoseStamped(frame_id="world", position=Vector3(0.0, 0.0, 0.0)),
        joint_names=list(DROID_ARM_JOINTS),
        base_link="panda_link0",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=tuple(DROID_ARM_JOINTS),
                base_link="panda_link0",
                tip_link="grasp_frame",
            )
        ],
        auto_convert_meshes=True,
        home_joints=list(DROID_HOME_JOINTS),
        pre_grasp_offset=0.05,
    )


def droid_robolab_hardware() -> HardwareComponent:
    """The DROID robot RoboLab simulates; its runner is reached at the sim_robolab default."""
    return HardwareComponent(
        hardware_id=DROID_HARDWARE_ID,
        hardware_type=HardwareType.MANIPULATOR,
        joints=list(DROID_JOINTS),
        adapter_type="sim_robolab",
        auto_enable=True,
    )
