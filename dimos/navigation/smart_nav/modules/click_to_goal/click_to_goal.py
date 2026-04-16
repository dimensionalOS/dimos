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

"""ClickToGoal: forwards clicked_point to LocalPlanner's way_point + FarPlanner's goal."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ClickToGoal(Module):
    """Relay clicked_point → way_point + goal for click-to-navigate.

    Publishes only in response to user actions — never on odometry updates.

    Ports:
        clicked_point (In[PointStamped]): Click from viewer.
        odometry (In[Odometry]): Vehicle pose (cached, used only on stop_movement).
        stop_movement (In[Bool]): Cancel active goal by anchoring at robot pose.
        way_point (Out[PointStamped]): Navigation waypoint for LocalPlanner.
        goal (Out[PointStamped]): Navigation goal for FarPlanner.
    """

    config: ModuleConfig

    clicked_point: In[PointStamped]
    odometry: In[Odometry]
    stop_movement: In[Bool]
    way_point: Out[PointStamped]
    goal: Out[PointStamped]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        state.pop("_lock", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.odometry.subscribe(self._on_odom)
        self.clicked_point.subscribe(self._on_click)
        self.stop_movement.subscribe(self._on_stop_movement)

    def _on_odom(self, msg: Odometry) -> None:
        # Cache the robot pose so stop_movement can anchor at it.
        # No publishing happens here — publishes are driven only by user input.
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y
            self._robot_z = msg.pose.position.z

    def _on_click(self, msg: PointStamped) -> None:
        # Reject invalid clicks (sky/background gives inf or huge coords)
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            logger.warning("Ignored invalid click", x=msg.x, y=msg.y, z=msg.z)
            return
        if abs(msg.x) > 500 or abs(msg.y) > 500 or abs(msg.z) > 50:
            logger.warning("Ignored out-of-range click", x=msg.x, y=msg.y, z=msg.z)
            return

        logger.info("Goal", x=round(msg.x, 1), y=round(msg.y, 1), z=round(msg.z, 1))
        self.way_point.publish(msg)
        self.goal.publish(msg)

    def _on_stop_movement(self, msg: Bool) -> None:
        """Cancel navigation by publishing a NaN goal (sentinel for 'no goal').

        Downstream planners treat NaN as "clear goal and stop planning".
        Navigation stays idle until a new clicked_point arrives.
        """
        if not msg.data:
            return

        cancel = PointStamped(
            ts=time.time(), frame_id="map", x=float("nan"), y=float("nan"), z=float("nan")
        )
        self.way_point.publish(cancel)
        self.goal.publish(cancel)
        logger.info("Navigation cancelled — waiting for new goal")
