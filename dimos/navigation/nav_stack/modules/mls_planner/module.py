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

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class MlsPlannerConfig(ModuleConfig):
    world_frame: str = "world"
    voxel_size: float = 0.1


class MlsPlanner(Module):
    """3D multi-level surface planner.

    Stub: emits a 2-pose straight-line path from latest odometry to the goal.
    The real surface-graph A* will replace _plan() in a follow-up.
    """

    config: MlsPlannerConfig

    global_map: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PoseStamped]
    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latest_odom: Odometry | None = None
        self._latest_map: PointCloud2 | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_map)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def _on_map(self, msg: PointCloud2) -> None:
        with self._lock:
            self._latest_map = msg

    def _on_goal(self, goal: PoseStamped) -> None:
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return
        path = self._plan(odom, goal)
        self.path.publish(path)

    def _plan(self, odom: Odometry, goal: PoseStamped) -> Path:
        start_pose = PoseStamped(
            ts=time.time(),
            frame_id=self.config.world_frame,
            position=[odom.x, odom.y, odom.z],
            orientation=[
                odom.orientation.x,
                odom.orientation.y,
                odom.orientation.z,
                odom.orientation.w,
            ],
        )
        return Path(
            ts=time.time(),
            frame_id=self.config.world_frame,
            poses=[start_pose, goal],
        )
