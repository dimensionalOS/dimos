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

"""Sanity check modules for the eval framework.

Just outputs a path from start to goal ignoring map.
"""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class StraightLinePlannerConfig(ModuleConfig):
    world_frame: str = "map"
    num_waypoints: int = 20


class StraightLinePlanner(Module):
    """Emits a straight-line Path from start to goal. Ignores the map."""

    config: StraightLinePlannerConfig

    global_map: In[PointCloud2]
    start_pose: In[PoseStamped]
    goal_pose: In[PoseStamped]
    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_start: PoseStamped | None = None

    async def handle_global_map(self, _msg: PointCloud2) -> None:
        return

    async def handle_start_pose(self, msg: PoseStamped) -> None:
        self._latest_start = msg

    async def handle_goal_pose(self, msg: PoseStamped) -> None:
        start = self._latest_start
        if start is None:
            logger.warning("StraightLinePlanner received goal before start; skipping")
            return
        path = self._straight_line(start, msg)
        self.path.publish(path)

    def _straight_line(self, start: PoseStamped, goal: PoseStamped) -> Path:
        n = max(2, self.config.num_waypoints)
        sx, sy, sz = start.x, start.y, start.z
        gx, gy, gz = goal.x, goal.y, goal.z
        xs = np.linspace(sx, gx, n)
        ys = np.linspace(sy, gy, n)
        zs = np.linspace(sz, gz, n)
        orient = [
            start.orientation.x,
            start.orientation.y,
            start.orientation.z,
            start.orientation.w,
        ]
        now = time.time()
        poses = [
            PoseStamped(
                ts=now,
                frame_id=self.config.world_frame,
                position=[float(x), float(y), float(z)],
                orientation=orient,
            )
            for x, y, z in zip(xs, ys, zs, strict=True)
        ]
        return Path(ts=now, frame_id=self.config.world_frame, poses=poses)
