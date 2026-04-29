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
``torso_link``.  Joint names match the ``g1_29dof_with_hand_rev_1_0``
URDF + the names ``ControlCoordinator`` already uses (no prefixing
because the existing G1 ``WHOLE_BODY`` HardwareComponent registers
arm joints under bare URDF names).

Caveats:
- Base motion: IK assumes the torso is static.  The robot must not
  be walking while a manipulation trajectory executes.
- Gripper: the G1 hand is articulated (14 finger joints), not a
  binary gripper.  This catalog entry omits ``gripper`` for now —
  pick/place via attach-to-palm constraint is a follow-up.
"""

from __future__ import annotations

from typing import Any

from dimos.robot.config import RobotConfig
from dimos.utils.data import LfsPath

# URDF + meshes shipped under data/g1_urdf/.  Mirrors the structure
# of the cached newton-assets package; references like
# ``package://unitree_g1/meshes/...`` resolve via ``package_paths``.
_G1_URDF = LfsPath("g1_urdf/g1.urdf")
_G1_PACKAGE_DIR = LfsPath("g1_urdf")

_LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

_RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def _g1_arm(
    name: str,
    joint_names: list[str],
    end_effector_link: str,
    **overrides: Any,
) -> RobotConfig:
    defaults: dict[str, Any] = {
        "name": name,
        "model_path": _G1_URDF,
        "joint_prefix": "",  # bare URDF names — match ControlCoordinator
        "joint_names": list(joint_names),
        "end_effector_link": end_effector_link,
        "base_link": "torso_link",
        "package_paths": {"unitree_g1": _G1_PACKAGE_DIR},
        # Mock adapter — joint commands flow through the existing
        # WHOLE_BODY G1 HardwareComponent in the sim blueprint, not a
        # separate manipulator adapter.  This config is only here to
        # describe the kinematic chain to the planner.
        "adapter_type": "mock",
        "auto_enable": True,
        "auto_convert_meshes": True,
        # Higher than the JointServoTask priority (10) so the
        # manipulation trajectory wins arbitration on the arm joints.
        "task_priority": 20,
        # Conservative pick — G1 arm policies in DDS run at similar
        # caps; we can raise once Drake-planned trajectories look
        # well-behaved.
        "max_velocity": 1.0,
        "max_acceleration": 2.5,
        # Home pose: zero everywhere (matches ARM_DEFAULT_POSE in
        # _groot_wbc_common.py).  Drake will pick this when the
        # planner asks for a "rest" state.
        "home_joints": [0.0] * len(joint_names),
    }
    defaults.update(overrides)
    return RobotConfig(**defaults)


def g1_left_arm(name: str = "g1_left_arm", **overrides: Any) -> RobotConfig:
    return _g1_arm(name, _LEFT_ARM_JOINTS, "left_wrist_yaw_link", **overrides)


def g1_right_arm(name: str = "g1_right_arm", **overrides: Any) -> RobotConfig:
    return _g1_arm(name, _RIGHT_ARM_JOINTS, "right_wrist_yaw_link", **overrides)


__all__ = ["g1_left_arm", "g1_right_arm"]
