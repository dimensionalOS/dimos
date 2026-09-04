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

"""OpenYAM collection and rollout policy profiles."""

from dimos.imitation.collection.native_recorder import declare_recorder
from dimos.imitation.dataprep.core import QualityConfig, SyncConfig
from dimos.imitation.profile import (
    ImageSource,
    JointPositionAction,
    JointPositionSource,
    PolicyIOProfile,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS

OPENYAM_CAMERA_SHAPE = (480, 640, 3)
OPENYAM_FPS = 30.0


def _profile(name: str, action_stream: str) -> PolicyIOProfile:
    joints = tuple(OPENYAM_JOINTS)
    return PolicyIOProfile(
        name=name,
        robot_type="openyam",
        observations={
            "observation.images.wrist": ImageSource(
                stream="wrist_image",
                shape=OPENYAM_CAMERA_SHAPE,
            ),
            "observation.state": JointPositionSource(
                stream="coordinator_joint_state",
                joints=joints,
            ),
        },
        action=JointPositionAction(
            key="action",
            demonstration=JointPositionSource(stream=action_stream, joints=joints),
        ),
        sync=SyncConfig(
            anchor="observation.images.wrist",
            rate_hz=OPENYAM_FPS,
            tolerance_ms=20.0,
        ),
        quality=QualityConfig(
            mode="strict",
            min_source_rate_ratio=0.95,
            max_camera_gap_ms=100.0,
            max_alignment_error_ms=20.0,
        ),
    )


OPENYAM_QUEST_IO = _profile("openyam-quest", "applied_joint_position_command")
OPENYAM_TEACH_IO = _profile("openyam-teach", "coordinator_joint_state")

OpenYamQuestRecorder = declare_recorder(
    "OpenYamQuestRecorder",
    __name__,
    OPENYAM_QUEST_IO,
)
OpenYamTeachRecorder = declare_recorder(
    "OpenYamTeachRecorder",
    __name__,
    OPENYAM_TEACH_IO,
)
