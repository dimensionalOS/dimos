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

"""Distinct Dual OpenYAM collection and released-ABC rollout profiles."""

from dimos.imitation.collection.native_recorder import declare_recorder
from dimos.imitation.dataprep.core import QualityConfig, SyncConfig
from dimos.imitation.profile import (
    ImageSource,
    JointPositionAction,
    JointPositionSource,
    PolicyIOProfile,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM_GRIPPER_JOINTS,
    DUAL_OPENYAM_JOINTS,
    DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_RIGHT_ARM_JOINTS,
)

DUAL_OPENYAM_CAMERA_SHAPE = (480, 640, 3)
DUAL_OPENYAM_FPS = 30.0
ABC_JOINTS = (
    *DUAL_OPENYAM_LEFT_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS[0],
    *DUAL_OPENYAM_RIGHT_ARM_JOINTS,
    DUAL_OPENYAM_GRIPPER_JOINTS[1],
)

_quality = QualityConfig(
    mode="strict",
    min_source_rate_ratio=0.95,
    max_camera_gap_ms=100.0,
    max_alignment_error_ms=20.0,
)

DUAL_OPENYAM_TWO_WRIST_IO = PolicyIOProfile(
    name="dual-openyam-quest",
    robot_type="dual_openyam",
    observations={
        "observation.images.left_wrist": ImageSource(
            stream="left_wrist_image",
            shape=DUAL_OPENYAM_CAMERA_SHAPE,
        ),
        "observation.images.right_wrist": ImageSource(
            stream="right_wrist_image",
            shape=DUAL_OPENYAM_CAMERA_SHAPE,
        ),
        "observation.state": JointPositionSource(
            stream="coordinator_joint_state",
            joints=tuple(DUAL_OPENYAM_JOINTS),
        ),
    },
    action=JointPositionAction(
        key="action",
        demonstration=JointPositionSource(
            stream="applied_joint_position_command",
            joints=tuple(DUAL_OPENYAM_JOINTS),
        ),
    ),
    sync=SyncConfig(
        anchor="observation.images.left_wrist",
        rate_hz=DUAL_OPENYAM_FPS,
        tolerance_ms=20.0,
    ),
    quality=_quality,
)

DUAL_OPENYAM_ABC_IO = PolicyIOProfile(
    name="dual-openyam-abc",
    robot_type="dual_openyam",
    observations={
        "top": ImageSource(stream="top_image", shape=DUAL_OPENYAM_CAMERA_SHAPE),
        "left": ImageSource(stream="left_wrist_image", shape=DUAL_OPENYAM_CAMERA_SHAPE),
        "right": ImageSource(stream="right_wrist_image", shape=DUAL_OPENYAM_CAMERA_SHAPE),
        "state": JointPositionSource(
            stream="coordinator_joint_state",
            joints=ABC_JOINTS,
        ),
    },
    action=JointPositionAction(
        key="actions",
        demonstration=JointPositionSource(
            stream="applied_joint_position_command",
            joints=ABC_JOINTS,
        ),
    ),
    sync=SyncConfig(anchor="top", rate_hz=DUAL_OPENYAM_FPS, tolerance_ms=20.0),
    quality=_quality,
)

DualOpenYamQuestRecorder = declare_recorder(
    "DualOpenYamQuestRecorder",
    __name__,
    DUAL_OPENYAM_TWO_WRIST_IO,
)
