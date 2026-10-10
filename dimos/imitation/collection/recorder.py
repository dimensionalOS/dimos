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

"""CollectionRecorder — captures teleop collection streams to a memory DB.

A `Recorder` (memory) subscribes each declared `In` port and appends every
message to a SQLite store, flushing durably on stop(). Only *connected*
streams are recorded, so the same recorder works for any arm whose
coordinator publishes `coordinator_joint_state`.

The recorded stream names match what DataPrep reads: `color_image`
and `coordinator_joint_state` (observation), `status` (episode segmentation).
"""

from __future__ import annotations

from typing import Any

from dimos_generated.geometry_msgs.msg import Pose
from dimos_generated.sensor_msgs.msg import Image, JointState

from dimos.core.stream import In
from dimos.imitation.collection.episode import EpisodeStatus
from dimos.memory.module import Recorder, RecorderConfig
from dimos.msgs.std_msgs.String import String


class CollectionRecorderConfig(RecorderConfig):
    record_tf: bool = False
    stream_codecs: dict[str, str] = {"status": "json"}


class CollectionRecorder(Recorder):
    """Records the streams DataPrep consumes from a teleop session."""

    config: CollectionRecorderConfig

    color_image: In[Image]  # observation (camera)
    coordinator_joint_state: In[JointState]  # observation + action (measured/next state)
    status: In[String]  # episode start/save/discard segmentation

    def _resolve_ts(self, name: str, msg: Any) -> float:
        if name == "status":
            return EpisodeStatus.from_json(msg.data).ts
        return super()._resolve_ts(name, msg)

    async def _resolve_pose(self, name: str, msg: object, ts: float) -> Pose | None:
        if name in self.config.poseless_streams:
            return None
        return await super()._resolve_pose(name, msg, ts)
