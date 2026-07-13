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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# limitations under the License.

from __future__ import annotations

import math

from dimos_lcm.std_msgs import Bool
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalRelayConfig(ModuleConfig):
    # Distance (m) within which the robot is considered "arrived" at its goal.
    arrival_tolerance: float = 0.3


class GoalRelay(Module):
    """Adapt odometry + goal points to the planner's PoseStamped inputs, and hold.

    While pursuing a clicked goal it forwards that goal. On teleop cancel
    (``stop_movement``) OR once the robot arrives at the goal, it enters a HOLD
    mode where it continuously republishes ``goal_pose = current pose`` on every
    odometry update. That keeps the global planner "at goal" (empty path) so the
    robot stays put -- and tracks the robot if it is physically moved -- until a
    new goal is clicked.
    """

    config: GoalRelayConfig

    odometry: In[Odometry]
    goal: In[PointStamped]
    stop_movement: In[Bool]

    start_pose: Out[PoseStamped]
    goal_pose: Out[PoseStamped]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._latest_pose: PoseStamped | None = None
        self._active_goal: PoseStamped | None = None
        self._holding: bool = True  # no goal yet -> hold at current pose

    @rpc
    def start(self) -> None:
        super().start()
        self._odom_count = 0
        logger.warning("[CANCELDBG] GoalRelay STARTED (holding=%s)", self._holding)
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop_movement)))

    def _on_odometry(self, msg: Odometry) -> None:
        pose = msg.to_pose_stamped()
        self._latest_pose = pose
        self.start_pose.publish(pose)
        # Arrival: once we reach the active goal, switch to hold.
        if not self._holding and self._active_goal is not None:
            if self._dist(pose, self._active_goal) < self.config.arrival_tolerance:
                self._holding = True
                logger.warning("[CANCELDBG] GoalRelay ARRIVED at goal -> holding")
        # Hold mode: keep the goal pinned to the current pose (tracks physical moves).
        if self._holding:
            self.goal_pose.publish(pose)
        # Heartbeat every ~2s so it's obvious in the log whether we're holding.
        self._odom_count += 1
        if self._odom_count % 60 == 0:
            logger.warning(
                "[CANCELDBG] GoalRelay tick holding=%s active_goal=%s pose=(%.2f,%.2f)",
                self._holding,
                None if self._active_goal is None else "set",
                pose.position.x,
                pose.position.y,
            )

    def _on_goal(self, point: PointStamped) -> None:
        # MovementManager cancels navigation by publishing a NaN goal (see its
        # _cancel_goal). Treat that as "hold here", NOT a fresh destination --
        # otherwise it immediately un-does the stop_movement hold and shoves NaN
        # at the planner, which then falls back to its last real goal and the
        # robot walks back to it. Only a finite goal resumes pursuit.
        finite = all(math.isfinite(v) for v in (point.x, point.y, point.z))
        logger.warning(
            "[CANCELDBG] GoalRelay GOT GOAL finite=%s xyz=(%s,%s,%s)",
            finite,
            point.x,
            point.y,
            point.z,
        )
        if not finite:
            self._active_goal = None
            self._holding = True
            logger.warning("[CANCELDBG] GoalRelay NaN goal -> CANCEL/hold")
            return
        goal = point.to_pose_stamped()
        self._active_goal = goal
        self._holding = False
        self.goal_pose.publish(goal)

    def _on_stop_movement(self, msg: Bool) -> None:
        logger.warning("[CANCELDBG] GoalRelay GOT STOP_MOVEMENT data=%s -> hold", msg.data)
        self._active_goal = None
        self._holding = True

    @staticmethod
    def _dist(a: PoseStamped, b: PoseStamped) -> float:
        return math.hypot(a.position.x - b.position.x, a.position.y - b.position.y)
