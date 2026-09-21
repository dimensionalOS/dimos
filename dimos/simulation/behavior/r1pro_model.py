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

"""R1 Pro planning description matching the installed BEHAVIOR robot assets."""

import json
from pathlib import Path

from dimos.hardware.spec import JointLimits
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE, make_r1pro_model_config
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.simulation.behavior.setup import MARKER, behavior_project

GRIPPER_JOINTS = tuple(
    f"{side}_gripper_finger_joint{i}" for side in ("left", "right") for i in (1, 2)
)
MODEL_JOINTS = (*UPPER_BODY_JOINTS, *GRIPPER_JOINTS)
# The exclusions ship with the pinned OmniGibson r1pro robot definition.
COLLISION_EXCLUSIONS = (
    ("left_arm_link1", "torso_link4"),
    ("left_arm_link2", "torso_link4"),
    ("right_arm_link1", "torso_link4"),
    ("right_arm_link2", "torso_link4"),
    ("left_arm_link5", "left_arm_link7"),
    ("right_arm_link5", "right_arm_link7"),
    ("left_gripper_finger_link1", "left_realsense_link"),
    ("right_gripper_finger_link1", "right_realsense_link"),
    ("left_gripper_finger_link1", "left_gripper_finger_link2"),
    ("right_gripper_finger_link1", "right_gripper_finger_link2"),
    *(("base_link", f"wheel_motor_link{i}") for i in (1, 2, 3)),
    ("torso_link2", "torso_link4"),
)


def simulation_model(assets: Path | None = None) -> RobotModel:
    if assets is None:
        project = behavior_project()
        marker = project / MARKER
        assets = (
            Path(json.loads(marker.read_text())["data_path"])
            if marker.exists()
            else project / ".assets"
        )
    return (
        RobotModel.from_file(
            assets / "omnigibson-robot-assets/models/r1pro/urdf/r1pro_original.urdf"
        )
        .with_default_joint_acceleration_limit(2.0)
        .with_fixed_joints(
            *(f"{kind}_motor_joint{i}" for kind in ("steer", "wheel") for i in (1, 2, 3))
        )
        .with_renamed_joints({name: coordinator_name(name) for name in MODEL_JOINTS})
        .with_planar_base(R1PRO_PLANAR_BASE)
    )


def simulation_model_config() -> RobotModelConfig:
    config = make_r1pro_model_config(simulation_model(), COLLISION_EXCLUSIONS)
    config.base_link = R1PRO_PLANAR_BASE.root_link
    config.joint_names = [
        *R1PRO_PLANAR_BASE.joint_names,
        *(coordinator_name(n) for n in MODEL_JOINTS),
    ]
    # Only torso/arm groups are executable. Base and grippers are measured context.
    config.home_joints = None
    return config


def upper_body_limits() -> JointLimits:
    joints = {joint.name: joint for joint in simulation_model().load().joints}
    selected = [joints[coordinator_name(name)] for name in UPPER_BODY_JOINTS]
    return JointLimits(
        position_lower=[j.lower for j in selected],
        position_upper=[j.upper for j in selected],
        velocity_max=[j.velocity for j in selected],
    )
