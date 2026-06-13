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

import math
from threading import Event, RLock, Thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalRelayConfig(ModuleConfig):
    replan_hz: float = 10.0


class GoalRelay(Module):
    """Drive the planner from the active goal and the latest odometry.

    The MLS planner replans once per goal_pose using the most recent
    start_pose. This holds the active goal and republishes start_pose plus
    goal_pose at replan_hz, so the planner keeps replanning from the robot's
    current pose while it follows the path. Replanning stops on goal_reached
    or a non-finite (cancel) goal.
    """

    config: GoalRelayConfig

    odometry: In[Odometry]
    goal: In[PointStamped]
    goal_reached: In[Bool]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._latest_odom: Odometry | None = None
        self._goal: PointStamped | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        if self.goal_reached.transport is not None:
            self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))
        self._thread = Thread(target=self._replan, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_odometry(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg

    def _on_goal(self, point: PointStamped) -> None:
        finite = math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)
        with self._lock:
            self._goal = point if finite else None

    def _on_goal_reached(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._goal = None

    def _replan(self) -> None:
        period = 1.0 / self.config.replan_hz
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            with self._lock:
                goal = self._goal
                odom = self._latest_odom
            if goal is not None and odom is not None:
                self.start_pose.publish(odom.to_pose_stamped())
                # Let start_pose land before the goal triggers planning.
                self._stop_event.wait(0.05)
                self.goal_pose.publish(goal.to_pose_stamped())
            elapsed = time.perf_counter() - start_time
            self._stop_event.wait(max(0.0, period - elapsed))
