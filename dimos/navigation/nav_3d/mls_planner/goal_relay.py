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
from dimos.navigation.tf_pose import OdomBasePose, base_height_above_ground
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalRelayConfig(ModuleConfig):
    base_frame: str = "base_link"
    # Lidar height above the ground while standing. None skips the ground
    # correction.
    lidar_height: float | None = None


class GoalRelay(Module):
    """Adapt odometry and goal points to the planner's PoseStamped inputs."""

    config: GoalRelayConfig

    odometry: In[Odometry]
    goal: In[PointStamped]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._base_pose: OdomBasePose | None = None
        self._base_height: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))

    def _on_odometry(self, msg: Odometry) -> None:
        if self._base_pose is None:
            self._base_pose = OdomBasePose(self.tf, self.config.base_frame)
        start = self._base_pose.resolve(msg)
        if start is None:
            return
        if self.config.lidar_height is not None:
            base_height = self._resolve_base_height(msg.child_frame_id, self.config.lidar_height)
            if base_height is None:
                return
            start.position.z -= base_height
        self.start_pose.publish(start)

    def _resolve_base_height(self, sensor_frame: str, lidar_height: float) -> float | None:
        if self._base_height is None:
            assert self._base_pose is not None
            leg = self._base_pose.sensor_to_base(sensor_frame)
            if leg is None:
                return None
            self._base_height = base_height_above_ground(lidar_height, -leg)
        return self._base_height

    def _on_goal(self, point: PointStamped) -> None:
        self.goal_pose.publish(point.to_pose_stamped())
