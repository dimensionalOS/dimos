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
streams are recorded, so the same recorder works for any arm whose
coordinator publishes `coordinator_joint_state`.

The recorded stream names match what DataPrep reads: `color_image`
and `coordinator_joint_state` (observation), `status` (episode segmentation).
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState


class CollectionRecorderConfig(RecorderConfig):
    pass


class CollectionRecorder(Recorder):
    """Records the streams DataPrep consumes from a teleop session."""

    config: CollectionRecorderConfig

    color_image: In[Image]  # observation (camera)
    coordinator_joint_state: In[JointState]  # observation + action (measured/next state)
    status: In[EpisodeStatus]  # episode start/save/discard segmentation
