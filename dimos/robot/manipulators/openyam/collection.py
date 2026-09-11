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

"""OpenYAM collection presets, safe to import without robot hardware."""

from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.dataprep.core import QualityConfig, SyncConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.joints import OPENYAM_JOINTS


def _profile(name: str, action_stream: str) -> CollectionProfile:
    return CollectionProfile(
        name=name,
        robot_type="openyam",
        observations={
            "observation.images.wrist": CollectionFeature(
                stream="wrist_image",
                message_type=Image,
                field="data",
                dtype="video",
                shape=(480, 640, 3),
                names=["height", "width", "channels"],
            ),
            "observation.state": CollectionFeature(
                stream="coordinator_joint_state",
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(OPENYAM_JOINTS),),
                names=list(OPENYAM_JOINTS),
            ),
        },
        actions={
            "action": CollectionFeature(
                stream=action_stream,
                message_type=JointState,
                field="position",
                dtype="float32",
                shape=(len(OPENYAM_JOINTS),),
                names=list(OPENYAM_JOINTS),
            ),
        },
        sync=SyncConfig(anchor="observation.images.wrist", rate_hz=30, tolerance_ms=20),
        quality=QualityConfig(mode="strict"),
    )


OPENYAM_QUEST_COLLECTION = _profile("openyam-quest", "applied_joint_position_command")
