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

"""ClickToGoal: forwards clicked_point to LocalPlanner's way_point.

Receives clicked_point from RerunWebSocketServer (or any module that
publishes PointStamped clicks) and re-publishes as way_point / goal
for the navigation stack. Also publishes goal_path (straight line from
robot to goal) for Rerun visualization.
"""

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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path


class ClickToGoal(Module):
    """Relay clicked_point → way_point + goal for click-to-navigate.

    Ports:
        clicked_point (In[PointStamped]): Click from viewer.
        odometry (In[Odometry]): Vehicle pose for goal line rendering.
        stop_movement (In[Bool]): Cancel active goal by setting goal to current position.
        way_point (Out[PointStamped]): Navigation waypoint for LocalPlanner.
        goal (Out[PointStamped]): Navigation goal for FarPlanner.
        goal_path (Out[Path]): Straight line from robot to goal for Rerun.
    """

    config: ModuleConfig
    default_config = ModuleConfig

    clicked_point: In[PointStamped]
    odometry: In[Odometry]
    stop_movement: In[Bool]
    way_point: Out[PointStamped]
    goal: Out[PointStamped]
    goal_path: Out[Path]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._initial_goal_sent = False

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        state.pop("_lock", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()

    @rpc
    def start(self) -> None:
        self.odometry._transport.subscribe(self._on_odom)
        self.clicked_point._transport.subscribe(self._on_click)
        self.stop_movement._transport.subscribe(self._on_stop_movement)

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y
            self._robot_z = msg.pose.position.z
            if not self._initial_goal_sent:
                self._initial_goal_sent = True
                # Set the initial goal to the robot's current position so the
                # local planner doesn't chase its default (0,0) goal on startup.
                here = PointStamped(
                    ts=time.time(),
                    frame_id="map",
                    x=self._robot_x,
                    y=self._robot_y,
                    z=self._robot_z,
                )
                # Only anchor the local planner — don't send to FAR planner's
                # goal input, as it causes FAR to enter "goal reached" idle state
                # and stop processing new goals promptly.
                self.way_point.publish(here)

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

        with self._lock:
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z

        print(f"[click_to_goal] Goal: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
        self.way_point.publish(msg)
        self.goal.publish(msg)

        # Publish a straight-line path from robot to goal for visualization
        now = time.time()
        poses = [
            PoseStamped(
                ts=now, frame_id="map", position=[rx, ry, rz + 0.3], orientation=[0, 0, 0, 1]
            ),
            PoseStamped(
                ts=now,
                frame_id="map",
                position=[msg.x, msg.y, msg.z + 0.3],
                orientation=[0, 0, 0, 1],
            ),
        ]
        self.goal_path.publish(Path(ts=now, frame_id="map", poses=poses))

    def _on_stop_movement(self, msg: Bool) -> None:
        """Cancel navigation by setting the goal to the robot's current position."""
        if not msg.data:
            return

        with self._lock:
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z

        here = PointStamped(ts=time.time(), frame_id="map", x=rx, y=ry, z=rz)
        self.way_point.publish(here)
        self.goal.publish(here)
