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
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalRelay(Module):
    """Pair each goal with the latest odometry into start and goal poses for the planner.

    The MLS planner plans once per goal_pose using the most recent start_pose,
    so both are sent together at goal time rather than streaming odometry.
    """

    odometry: In[Odometry]
    goal: In[PointStamped]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_odom: Odometry | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))

    def _on_odometry(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_goal(self, point: PointStamped) -> None:
        if not (math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)):
            return
        odom = self._latest_odom
        if odom is None:
            logger.warning("GoalRelay received a goal before any odometry, dropping it")
            return
        self.start_pose.publish(odom.to_pose_stamped())
        # Let start_pose land before the goal triggers planning.
        time.sleep(0.05)
        self.goal_pose.publish(point.to_pose_stamped())
