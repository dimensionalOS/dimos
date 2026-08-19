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

"""CollectionRecorder — captures teleop collection streams to a memory2 DB.

A `Recorder` (memory2) subscribes each declared `In` port and appends every
message to a SQLite store, flushing durably on stop(). Only *connected*
streams are recorded, so the same recorder works for any arm once the
collection blueprint remaps the joint port onto that coordinator's per-robot
joint output.

The recorded stream names are the port names, and match what DataPrep reads:
`color_image` and `coordinator_joint_state` (observation), `status` (episode
segmentation).
"""

from __future__ import annotations

from pydantic import Field

from dimos.core.stream import In
from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


class CollectionRecorderConfig(RecorderConfig):
    # None of the collection streams carry a world pose this stack can
    # resolve; skipping the tf lookup avoids per-message warnings at the
    # control and camera rates.
    poseless_streams: list[str] = Field(
        default_factory=lambda: [
            "coordinator_joint_state",
            "coordinator_joint_target",
            "status",
            "color_image",
            "chest_image",
            "left_hand_image",
            "right_hand_image",
            "waist_image",
        ]
    )


class CollectionRecorder(Recorder):
    """Records the streams DataPrep consumes from a teleop session."""

    config: CollectionRecorderConfig

    color_image: In[Image]  # observation (single camera)
    # Additional observation cameras for multi-view rigs; each stays silent
    # unless a collection blueprint wires a camera onto it.
    chest_image: In[Image]
    left_hand_image: In[Image]
    right_hand_image: In[Image]
    waist_image: In[Image]
    coordinator_joint_state: In[JointState]  # observation + action (measured/next state)
    # action (commanded targets); recorded when the coordinator runs with
    # publish_joint_targets on, silent otherwise
    coordinator_joint_target: In[JointState]
    status: In[EpisodeStatus]  # episode start/save/discard segmentation
