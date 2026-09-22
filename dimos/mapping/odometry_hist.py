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
from copy import deepcopy
import math
from typing import Any

from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.nav_msgs.msg import Odometry, Path
from pydantic import Field
import rerun as rr

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.time import to_nanoseconds


def path_at_true_height(path: Path) -> Any:
    """The default z lift clears a costmap a bare odometry demo has none of."""
    points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses]
    return rr.LineStrips3D([points] if points else [], colors=[(0, 255, 128)], radii=0.02)


class OdometryHistConfig(ModuleConfig):
    # Empty follows the odometry's frame_id.
    frame_id: str = ""
    min_step_meters: float = 0.02
    max_poses: int = Field(20000, ge=1)
    min_publish_interval_seconds: float = 0.1


class OdometryHist(Module):
    config: OdometryHistConfig

    odometry: In[Odometry]

    odom_hist: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._poses: deque[PoseStamped] = deque(maxlen=self.config.max_poses)
        # None, not 0.0: a recording stamped from zero would suppress the first path.
        self._last_publish_ns: int | None = None
        self._unpublished = False

    async def handle_odometry(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        point = (position.x, position.y, position.z)
        frame_id = self.config.frame_id or msg.header.frame_id
        previous = self._poses[-1] if self._poses else None
        if (
            previous is None
            or math.dist(
                (previous.pose.position.x, previous.pose.position.y, previous.pose.position.z),
                point,
            )
            >= self.config.min_step_meters
        ):
            header = deepcopy(msg.header)
            header.frame_id = frame_id
            self._poses.append(PoseStamped(header=header, pose=msg.pose.pose))
            self._unpublished = True

        if not self._unpublished:
            return
        stamp_ns = to_nanoseconds(msg.header.stamp)
        if self._last_publish_ns is not None:
            elapsed = stamp_ns - self._last_publish_ns
            # A replay restart moves the stamp backwards; publish rather than wait it out.
            if 0 <= elapsed < round(self.config.min_publish_interval_seconds * 1_000_000_000):
                return
        self._last_publish_ns = stamp_ns
        self._unpublished = False
        header = deepcopy(msg.header)
        header.frame_id = frame_id
        self.odom_hist.publish(Path(header=header, poses=list(self._poses)))
