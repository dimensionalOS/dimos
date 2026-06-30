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

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry


class GoalRelayConfig(ModuleConfig):
    pass


class GoalRelay(Module):
    """Adapt odometry and goal points to the planner's PoseStamped inputs."""

    config: GoalRelayConfig

    odometry: In[Odometry]
    goal: In[PointStamped]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))

    def _on_odometry(self, msg: Odometry) -> None:
        self.start_pose.publish(msg.to_pose_stamped())

    def _on_goal(self, point: PointStamped) -> None:
        self.goal_pose.publish(point.to_pose_stamped())
