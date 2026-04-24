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

"""MovementManager: click-to-goal + teleop/nav velocity mux in one module.

Combines the responsibilities of ClickToGoal and CmdVelMux:
- Validates and forwards clicked_point → goal (+ way_point)
- Multiplexes nav_cmd_vel and tele_cmd_vel → cmd_vel
- When teleop starts: cancels the active nav goal and publishes stop_movement
- When teleop ends: nav resumes but stays idle until a new click

This avoids the round-trip where CmdVelMux had to publish stop_movement
over a stream to ClickToGoal, which then had to publish a NaN goal to the
planner. Now goal cancellation is immediate and internal.
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
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MovementManagerConfig(ModuleConfig):
    """Config for MovementManager."""

    # Seconds after the last teleop message before nav_cmd_vel is re-enabled.
    tele_cooldown_sec: float = 1.0
    body_frame: str = "body"
    # Element-wise multiplier for incoming teleop twists.
    # Default is identity (all 1.0). Set a component to 0.0 to lock it out.
    tele_cmd_vel_scaling: Twist = Twist(Vector3(1, 1, 1), Vector3(1, 1, 1))


class MovementManager(Module):
    """Click-to-goal relay + teleop/nav velocity mux.

    Ports:
        clicked_point (In[PointStamped]): Click from viewer → publishes goal.
        nav_cmd_vel (In[Twist]): Velocity from the autonomous planner.
        tele_cmd_vel (In[Twist]): Velocity from keyboard/joystick teleop.
        goal (Out[PointStamped]): Navigation goal for the global planner.
        way_point (Out[PointStamped]): Immediate waypoint (disconnected in smart_nav).
        cmd_vel (Out[Twist]): Merged velocity — teleop wins when active.
        stop_movement (Out[Bool]): Fired once when teleop takes over, for
            modules that listen directly (e.g. FarPlanner C++ binary).

    Robot pose is obtained via the TF tree (``map → body``) rather than
    an Odometry stream.
    """

    config: MovementManagerConfig

    clicked_point: In[PointStamped]
    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]

    goal: Out[PointStamped]
    way_point: Out[PointStamped]
    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._teleop_active = False
        self._last_teleop_time = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.clicked_point.subscribe(self._on_click)
        self.nav_cmd_vel.subscribe(self._on_nav)
        self.tele_cmd_vel.subscribe(self._on_teleop)

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._teleop_active = False
        super().stop()

    # TODO: when/if we change transform frame stuff (especially naming) we should change how this is done.
    # This is in the "it works" category of code changes
    def _query_pose(self) -> tuple[float, float, float]:
        """Return (x, y, z) from the TF tree, falling back to cached values.

        Tries ``map → body_frame`` first (corrected pose), then
        ``odom → body_frame`` (startup fallback).  Caches the last
        successful parent frame to avoid repeated BFS misses.
        """
        child = self.config.body_frame
        # Always try map first (corrected pose), fall back to odom (startup).
        for parent in ("map", "odom"):
            tf = self.tf.get(parent, child)
            if tf is not None:
                with self._lock:
                    self._robot_x = float(tf.translation.x)
                    self._robot_y = float(tf.translation.y)
                    self._robot_z = float(tf.translation.z)
                break
        with self._lock:
            return self._robot_x, self._robot_y, self._robot_z

    # ── Click-to-goal ─────────────────────────────────────────────────────

    def _on_click(self, msg: PointStamped) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            logger.warning("Ignored invalid click", x=msg.x, y=msg.y, z=msg.z)
            return
        if abs(msg.x) > 500 or abs(msg.y) > 500 or abs(msg.z) > 50:
            logger.warning("Ignored out-of-range click", x=msg.x, y=msg.y, z=msg.z)
            return

        logger.info("Goal", x=round(msg.x, 1), y=round(msg.y, 1), z=round(msg.z, 1))
        self.way_point.publish(msg)
        self.goal.publish(msg)

    def _cancel_goal(self) -> None:
        self.stop_movement.publish(Bool(data=True))
        # NOTE: this NaN goal is more of a safety fallback.
        # It can be REALLY bad if a robot is supposed to stop moving but wont
        # we should probably think a more robust/strict requirement on planners
        cancel = PointStamped(
            ts=time.time(), frame_id="map", x=float("nan"), y=float("nan"), z=float("nan")
        )
        self.way_point.publish(cancel)
        self.goal.publish(cancel)
        logger.info("Navigation cancelled — waiting for new goal")

    # ── Velocity mux ─────────────────────────────────────────────────────

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            if self._teleop_active:
                # Check if cooldown has expired.
                elapsed = time.monotonic() - self._last_teleop_time
                if elapsed < self.config.tele_cooldown_sec:
                    return
                self._teleop_active = False
        self.cmd_vel.publish(msg)

    def _on_teleop(self, msg: Twist) -> None:
        with self._lock:
            was_active = self._teleop_active
            self._teleop_active = True
            self._last_teleop_time = time.monotonic()

        if not was_active:
            self._cancel_goal()
            logger.info("Teleop active")

        scale = self.config.tele_cmd_vel_scaling
        scaled = Twist(
            linear=Vector3(
                msg.linear.x * scale.linear.x,
                msg.linear.y * scale.linear.y,
                msg.linear.z * scale.linear.z,
            ),
            angular=Vector3(
                msg.angular.x * scale.angular.x,
                msg.angular.y * scale.angular.y,
                msg.angular.z * scale.angular.z,
            ),
        )
        self.cmd_vel.publish(scaled)
