# Copyright 2025-2026 Dimensional Inc.
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

"""Unitree G1 catalog entries for the manipulation stack.

Treats one G1 arm as a stationary 7-DOF manipulator rooted at
``torso_link``.  The G1 ``WHOLE_BODY`` HardwareComponent already
publishes joint state under dimos canonical names
(``g1_LeftShoulderPitch``, …) but the G1 URDF uses the upstream
Unitree names (``left_shoulder_pitch_joint``, …) — we expose
``joint_name_mapping`` so the manipulation module can translate
between the two.

Caveats:
- Base motion: IK assumes the torso is static.  The robot must not
  be walking while a manipulation trajectory executes.
- Gripper: the G1 hand is articulated (14 finger joints), not a
  binary gripper.  No gripper config attached.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.control.coordinator import TaskConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.data import LfsPath

# URDF + meshes shipped under data/g1_urdf/.  Mirrors the structure
# of the cached newton-assets package; references like
# ``package://unitree_g1/meshes/...`` resolve via ``package_paths``.
_G1_URDF = LfsPath("g1_urdf/g1.urdf")
_G1_PACKAGE_DIR = LfsPath("g1_urdf")

# (URDF joint, dimos canonical joint) per arm.  The dimos names are
# what ``make_humanoid_joints("g1")`` emits — we hand-mirror them
# rather than slicing g1_arms to keep this file standalone.
_LEFT_ARM_JOINT_PAIRS = [
    ("left_shoulder_pitch_joint", "g1_LeftShoulderPitch"),
    ("left_shoulder_roll_joint", "g1_LeftShoulderRoll"),
    ("left_shoulder_yaw_joint", "g1_LeftShoulderYaw"),
    ("left_elbow_joint", "g1_LeftElbow"),
    ("left_wrist_roll_joint", "g1_LeftWristRoll"),
    ("left_wrist_pitch_joint", "g1_LeftWristPitch"),
    ("left_wrist_yaw_joint", "g1_LeftWristYaw"),
]
_RIGHT_ARM_JOINT_PAIRS = [
    ("right_shoulder_pitch_joint", "g1_RightShoulderPitch"),
    ("right_shoulder_roll_joint", "g1_RightShoulderRoll"),
    ("right_shoulder_yaw_joint", "g1_RightShoulderYaw"),
    ("right_elbow_joint", "g1_RightElbow"),
    ("right_wrist_roll_joint", "g1_RightWristRoll"),
    ("right_wrist_pitch_joint", "g1_RightWristPitch"),
    ("right_wrist_yaw_joint", "g1_RightWristYaw"),
]


@dataclass(frozen=True)
class G1ArmCatalogEntry:
    """Pre-configured pair the blueprint composes into the sim.

    ``robot_model_config`` is the manipulation-module side (URDF joint
    names + the coord<->urdf mapping for state translation).
    ``task_config`` is the coordinator side (dimos canonical joint
    names so the trajectory task claims the right joints).
    """

    name: str
    robot_model_config: RobotModelConfig
    task_config: TaskConfig


def _g1_arm(
    name: str,
    pairs: list[tuple[str, str]],
    end_effector_link: str,
    *,
    task_priority: int = 20,
) -> G1ArmCatalogEntry:
    urdf_joints = [u for u, _ in pairs]
    coord_joints = [c for _, c in pairs]
    coord_to_urdf = {c: u for u, c in pairs}

    rmc = RobotModelConfig(
        name=name,
        model_path=_G1_URDF,
        joint_names=urdf_joints,
        end_effector_link=end_effector_link,
        base_link="torso_link",
        package_paths={"unitree_g1": _G1_PACKAGE_DIR},
        joint_name_mapping=coord_to_urdf,
        coordinator_task_name=f"traj_{name}",
        # Stationary base; relative to the robot's torso. If the WBC
        # walks the robot, IK targets need to be re-expressed from
        # world to torso first — out of scope for this first pass.
        base_pose=PoseStamped(
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        max_velocity=1.0,
        max_acceleration=2.5,
        # Home pose: zero everywhere (matches ARM_DEFAULT_POSE).
        home_joints=[0.0] * len(urdf_joints),
    )

    task = TaskConfig(
        name=f"traj_{name}",
        type="trajectory",
        joint_names=coord_joints,
        priority=task_priority,
    )

    return G1ArmCatalogEntry(name=name, robot_model_config=rmc, task_config=task)


def g1_left_arm(name: str = "g1_left_arm") -> G1ArmCatalogEntry:
    return _g1_arm(name, _LEFT_ARM_JOINT_PAIRS, "left_wrist_yaw_link")


def g1_right_arm(name: str = "g1_right_arm") -> G1ArmCatalogEntry:
    return _g1_arm(name, _RIGHT_ARM_JOINT_PAIRS, "right_wrist_yaw_link")


__all__ = ["G1ArmCatalogEntry", "g1_left_arm", "g1_right_arm"]
