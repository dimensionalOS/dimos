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

"""Upper-body planning model for the Unitree G1."""

from __future__ import annotations

from pathlib import Path

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_arms, g1_legs_waist
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.assets.model import RobotModel
from dimos.utils.data import LfsPath

G1_URDF_PATH = Path(__file__).resolve().parent / "g1.urdf"
G1_UPPER_BODY_NAME = "g1_upper_body"

G1_WAIST_JOINTS = tuple(g1_legs_waist[-3:])
G1_LEFT_ARM_JOINTS = tuple(g1_arms[:7])
G1_RIGHT_ARM_JOINTS = tuple(g1_arms[7:])


def _urdf_joint_name(coordinator_name: str) -> str:
    return f"{coordinator_name.partition('/')[2]}_joint"


G1_UPPER_BODY_JOINTS = (*G1_WAIST_JOINTS, *g1_arms)
G1_UPPER_BODY_JOINT_NAME_MAPPING = {
    joint_name: _urdf_joint_name(joint_name) for joint_name in G1_UPPER_BODY_JOINTS
}
G1_UPPER_BODY_MODEL = (
    RobotModel.from_file(
        G1_URDF_PATH,
        package_paths={"g1_description": LfsPath("g1_urdf")},
    )
    .with_subtree_rooted_at("pelvis")
    .without_joint_subtrees("left_hip_pitch_joint", "right_hip_pitch_joint")
)
G1_TELEOP_ARM_MODEL = G1_UPPER_BODY_MODEL.with_fixed_joints(
    *(_urdf_joint_name(name) for name in G1_WAIST_JOINTS)
)

G1_READY_JOINTS = {
    "left_arm": (-0.4, 0.2, 0.0, 1.2, 0.0, 0.0, 0.0),
    "right_arm": (-0.4, -0.2, 0.0, 1.2, 0.0, 0.0, 0.0),
}
G1_READY_SPEED_SCALE = 0.25


def g1_upper_body_model_config() -> RobotModelConfig:
    """Build the stationary G1 upper-body collision and kinematics model.

    Waist joints remain in the model so measured torso motion is reflected in
    collision checks, but only the two arm groups are eligible for planning.
    The removed leg branches are therefore outside the collision world; this
    model must only be used while the robot is stationary.
    """
    local_waist = tuple(_urdf_joint_name(name) for name in G1_WAIST_JOINTS)
    local_left = tuple(_urdf_joint_name(name) for name in G1_LEFT_ARM_JOINTS)
    local_right = tuple(_urdf_joint_name(name) for name in G1_RIGHT_ARM_JOINTS)
    return RobotModelConfig(
        name=G1_UPPER_BODY_NAME,
        model=G1_UPPER_BODY_MODEL,
        joint_names=[*local_waist, *local_left, *local_right],
        base_link="pelvis",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=local_left,
                base_link="pelvis",
                tip_link="left_rubber_hand",
            ),
            PlanningGroupDefinition(
                name="right_arm",
                joint_names=local_right,
                base_link="pelvis",
                tip_link="right_rubber_hand",
            ),
        ],
        collision_exclusion_pairs=[
            ("torso_link", "left_shoulder_yaw_link"),
            ("torso_link", "left_shoulder_roll_link"),
            ("torso_link", "right_shoulder_yaw_link"),
            ("torso_link", "right_shoulder_roll_link"),
        ],
        max_velocity=1.0,
        max_acceleration=2.0,
        joint_name_mapping=G1_UPPER_BODY_JOINT_NAME_MAPPING,
    )
