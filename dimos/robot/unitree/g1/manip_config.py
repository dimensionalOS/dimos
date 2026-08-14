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

"""Unitree G1 planning model configuration.

Full-body model: all 29 body joints are registered so the planning world
tracks the live legs/waist stance for collision checking, while only the
7-DOF arm planning groups are ever planned. The LFS URDF variant is used
because its root link is ``pelvis`` (PinkIK requires ``base_link`` to be
the model root); the in-repo ``g1.urdf`` roots at ``world`` via a floating
joint, which neither PinkIK nor the RoboPlan composite accepts.
"""

from __future__ import annotations

from dimos.control.components import make_humanoid_joints
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.manipulators._modeling import base_pose
from dimos.utils.data import LfsPath

_G1_URDF = LfsPath("g1_urdf/g1.urdf")
_G1_PACKAGE_DIR = LfsPath("g1_urdf")
# Nominal standing pelvis height; matches G1GrootWBCTask's height_cmd, so
# world-frame targets line up with the ground frame at spawn. This is only the
# startup placement: once the robot walks, ManipulationModule.latch_base_pose()
# replaces it with the measured pelvis pose.
G1_NOMINAL_PELVIS_Z = 0.74

_ARM_JOINT_STEMS = (
    "shoulder_pitch",
    "shoulder_roll",
    "shoulder_yaw",
    "elbow",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
)


def _arm_urdf_joints(side: str) -> tuple[str, ...]:
    return tuple(f"{side}_{stem}_joint" for stem in _ARM_JOINT_STEMS)


def _arm_group(side: str) -> PlanningGroupDefinition:
    return PlanningGroupDefinition(
        name=f"{side}_arm",
        joint_names=_arm_urdf_joints(side),
        base_link="torso_link",
        tip_link=f"{side}_hand_palm_link",
    )


def make_g1_model_config(name: str = "g1") -> RobotModelConfig:
    coordinator_joints = make_humanoid_joints("g1")
    urdf_joints = [f"{joint.partition('/')[2]}_joint" for joint in coordinator_joints]
    return RobotModelConfig(
        name=name,
        model_path=_G1_URDF,
        base_pose=base_pose(z=G1_NOMINAL_PELVIS_Z),
        joint_names=urdf_joints,
        base_link="pelvis",
        planning_groups=[_arm_group("left"), _arm_group("right")],
        package_paths={"unitree_g1": _G1_PACKAGE_DIR},
        auto_convert_meshes=True,
        # The shoulder_yaw/roll link meshes extend back into the torso; at the
        # default pose they overlap by a few mm and every plan would be
        # rejected with a collision at start.
        collision_exclusion_pairs=[
            ("torso_link", "left_shoulder_yaw_link"),
            ("torso_link", "left_shoulder_roll_link"),
            ("torso_link", "right_shoulder_yaw_link"),
            ("torso_link", "right_shoulder_roll_link"),
        ],
        joint_name_mapping=dict(zip(coordinator_joints, urdf_joints, strict=True)),
        max_velocity=1.0,
        max_acceleration=2.5,
        # Elbow zero is a ~90 deg forward bend (positive straightens); seed
        # pose-target IK with a partially extended reach so solutions extend
        # the arm instead of parking at the bend.
        ik_posture={
            "left_shoulder_pitch_joint": -0.4,
            "left_elbow_joint": 1.2,
            "right_shoulder_pitch_joint": -0.4,
            "right_elbow_joint": 1.2,
        },
    )
