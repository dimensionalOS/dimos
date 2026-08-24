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

from __future__ import annotations

from collections import deque
import math
from typing import Any

from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path


class OdometryPathConfig(ModuleConfig):
    # Empty follows the odometry's own frame_id.
    frame_id: str = ""
    min_step_meters: float = 0.02
    max_poses: int = Field(20000, ge=1)
    min_publish_interval_seconds: float = 0.1


class OdometryPath(Module):
    config: OdometryPathConfig

    odometry: In[Odometry]

    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._poses: deque[PoseStamped] = deque(maxlen=self.config.max_poses)
        # None, not 0.0: a recording stamped from zero would suppress the first path.
        self._last_publish_ts: float | None = None
        self._unpublished = False

    async def handle_odometry(self, msg: Odometry) -> None:
        position = msg.pose.position
        point = (position.x, position.y, position.z)
        frame_id = self.config.frame_id or msg.frame_id
        previous = self._poses[-1] if self._poses else None
        if (
            previous is None
            or math.dist((previous.x, previous.y, previous.z), point) >= self.config.min_step_meters
        ):
            orientation = msg.pose.orientation
            self._poses.append(
                PoseStamped(
                    ts=msg.ts,
                    frame_id=frame_id,
                    position=list(point),
                    orientation=[orientation.x, orientation.y, orientation.z, orientation.w],
                )
            )
            self._unpublished = True

        if not self._unpublished:
            return
        if self._last_publish_ts is not None:
            elapsed = msg.ts - self._last_publish_ts
            # A replay restart moves the stamp backwards; publish rather than wait it out.
            if 0.0 <= elapsed < self.config.min_publish_interval_seconds:
                return
        self._last_publish_ts = msg.ts
        self._unpublished = False
        self.path.publish(Path(ts=msg.ts, frame_id=frame_id, poses=list(self._poses)))
