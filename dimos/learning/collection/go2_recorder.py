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

"""Go2 collection recorder for VLA training data.

A memory2 `Recorder` that captures the Go2 streams useful for training:
camera (`color_image`), pose (`odom`), the action channel (`cmd_vel`), and the
episode boundaries (`status`, produced by `EpisodeMonitorModule`). Distinct from
`Go2Memory` (spatial memory) — this one adds the action + episode-status streams
`DataPrep` needs, and writes its own session DB. Lidar is intentionally omitted
for a smaller camera+proprioception+action dataset (add an `In[PointCloud2]`
port if training needs it).
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.stream import In
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image


class Go2CollectionRecorderConfig(RecorderConfig):
    db_path: str | Path = "recording_go2_collection.db"


class Go2CollectionRecorder(Recorder):
    """Records Go2 camera + pose + action + episode-boundary streams for training."""

    config: Go2CollectionRecorderConfig

    color_image: In[Image]  # observation (camera)
    odom: In[PoseStamped]  # observation (pose)
    cmd_vel: In[Twist]  # action channel
    status: In[EpisodeStatus]  # episode start/save/discard segmentation

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        # Tag recorded frames with the robot's odom pose (avoids per-message tf
        # lookups). Mirrors Go2Memory's odom pose setter.
        self._last_odom_pose = msg
        return self._last_odom_pose
