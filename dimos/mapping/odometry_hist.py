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

"""Accumulate an odometry stream into the path it has travelled."""

from __future__ import annotations

from collections import deque
import math
from typing import Any

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path


class OdometryHistConfig(ModuleConfig):
    # Empty follows the odometry's own frame_id.
    frame_id: str = ""
    min_step_meters: float = 0.02
    max_poses: int = 20000
    min_publish_interval_seconds: float = 0.1


class OdometryHist(Module):
    """``odometry`` in, the trail it has drawn out, as a ``nav_msgs/Path``."""

    config: OdometryHistConfig

    odometry: In[Odometry]

    odom_hist: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._poses: deque[PoseStamped] = deque(maxlen=self.config.max_poses)
        self._last_publish_ts = 0.0

    async def handle_odometry(self, msg: Odometry) -> None:
        position = msg.pose.position
        point = (position.x, position.y, position.z)
        if self._poses:
            previous = self._poses[-1]
            if math.dist((previous.x, previous.y, previous.z), point) < self.config.min_step_meters:
                return

        orientation = msg.pose.orientation
        frame_id = self.config.frame_id or msg.frame_id
        self._poses.append(
            PoseStamped(
                ts=msg.ts,
                frame_id=frame_id,
                position=list(point),
                orientation=[orientation.x, orientation.y, orientation.z, orientation.w],
            )
        )
        if msg.ts - self._last_publish_ts < self.config.min_publish_interval_seconds:
            return
        self._last_publish_ts = msg.ts
        # A copy: Path holds the list by reference.
        self.odom_hist.publish(Path(ts=msg.ts, frame_id=frame_id, poses=list(self._poses)))
