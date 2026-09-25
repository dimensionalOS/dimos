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


"""R1 Pro joint layout, shared by the hardware bridge and the planning model."""

from __future__ import annotations

NAMESPACE = "r1pro"

TORSO_JOINTS = tuple(f"torso_joint{i}" for i in range(1, 5))
LEFT_ARM_JOINTS = tuple(f"left_arm_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_arm_joint{i}" for i in range(1, 8))

# Order matches the flat 18-element MotorCommandArray the vendor stack speaks.
UPPER_BODY_JOINTS = (*TORSO_JOINTS, *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS)

# Driven by the chassis velocity task or the gripper adapter, never planned, so
# the planning model welds them at their URDF zero.
PASSIVE_JOINTS = (
    *(f"steer_motor_joint{i}" for i in range(1, 4)),
    *(f"wheel_motor_joint{i}" for i in range(1, 4)),
    *(f"{side}_gripper_finger_joint{i}" for side in ("left", "right") for i in (1, 2)),
)


def coordinator_name(urdf_joint: str) -> str:
    return f"{NAMESPACE}/{urdf_joint}"
