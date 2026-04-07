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


class ClickToGoal(Module[ModuleConfig]):
    """Relay clicked_point → way_point + goal for click-to-navigate.

    Publishes only in response to user actions — never on odometry updates.

    Ports:
        clicked_point (In[PointStamped]): Click from viewer.
        odometry (In[Odometry]): Vehicle pose (cached, used only on stop_movement).
        stop_movement (In[Bool]): Cancel active goal by anchoring at robot pose.
        way_point (Out[PointStamped]): Navigation waypoint for LocalPlanner.
        goal (Out[PointStamped]): Navigation goal for FarPlanner.
    """

    default_config = ModuleConfig

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
        state = super().__getstate__()
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
            print(f"[click_to_goal] Ignored invalid click: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
            return
        if abs(msg.x) > 500 or abs(msg.y) > 500 or abs(msg.z) > 50:
            print(
                f"[click_to_goal] Ignored out-of-range click: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})"
            )
            return

        print(f"[click_to_goal] Goal: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
        self.way_point.publish(msg)
        self.goal.publish(msg)

    def _on_stop_movement(self, msg: Bool) -> None:
        """Cancel navigation by setting the goal to the robot's current position."""
        if not msg.data:
            return

        with self._lock:
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z

        here = PointStamped(ts=time.time(), frame_id="map", x=rx, y=ry, z=rz)
        self.way_point.publish(here)
        self.goal.publish(here)
