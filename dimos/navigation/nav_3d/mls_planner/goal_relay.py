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

from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.constants import TF_LOOKUP_TOLERANCE_S
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalRelayConfig(ModuleConfig):
    base_frame: str = "base_link"
    sensor_frame: str = "mid360_link"
    # Lidar height above the ground while standing. Leave as None to skip the
    # ground correction and pass odometry through unshifted.
    lidar_height: float | None = None


class GoalRelay(Module):
    """Adapt odometry and goal points to the planner's PoseStamped inputs.

    Odometry is corrected to the robot base frame via tf.
    """

    config: GoalRelayConfig

    odometry: In[Odometry]
    goal: In[PointStamped]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._base_height: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))

    def _on_odometry(self, msg: Odometry) -> None:
        base = self.tf.get(msg.frame_id, self.config.base_frame, msg.ts, TF_LOOKUP_TOLERANCE_S)
        if base is None:
            logger.warning(
                "No %s -> %s transform for odometry stamp %.3f, dropping frame.",
                msg.frame_id,
                self.config.base_frame,
                msg.ts,
            )
            return
        start = base.to_pose(ts=msg.ts)
        if self.config.lidar_height is not None:
            base_height = self._base_from_ground()
            if base_height is None:
                return
            start.position.z -= base_height
        self.start_pose.publish(start)

    def _base_from_ground(self) -> float | None:
        # The base -> sensor mount is static rig geometry, so resolve it once.
        if self._base_height is None and self.config.lidar_height is not None:
            mount = self.tf.get(self.config.base_frame, self.config.sensor_frame)
            if mount is None:
                logger.warning(
                    "No %s -> %s mount transform on tf, cannot ground-project the start pose.",
                    self.config.base_frame,
                    self.config.sensor_frame,
                )
            else:
                self._base_height = self.config.lidar_height - mount.translation.z
        return self._base_height

    def _on_goal(self, point: PointStamped) -> None:
        self.goal_pose.publish(point.to_pose_stamped())
