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

"""In-blueprint test probe for nav-stack E2E tests.

Drops into a blueprint, autoconnects to ``/odometry`` (In) and
``/clicked_point`` (Out), and exposes RPCs the test harness calls.
Lets tests stay transport-agnostic — the framework wires whichever
transport the rest of the blueprint uses (LCM, SHM, etc).
"""

from __future__ import annotations

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


class NavTestProbe(Module):
    odometry: In[Odometry]
    clicked_point: Out[PointStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._count = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._count += 1
            self._x = float(msg.x)
            self._y = float(msg.y)
            self._z = float(msg.pose.position.z)

    @rpc
    def latest_pose(self) -> tuple[float, float, float, int]:
        with self._lock:
            return self._x, self._y, self._z, self._count

    @rpc
    def publish_goal(self, x: float, y: float, z: float) -> None:
        msg = PointStamped(x=x, y=y, z=z, ts=time.time(), frame_id="map")
        self.clicked_point.publish(msg)
