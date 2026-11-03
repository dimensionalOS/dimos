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

"""RouteFollower: owns the multi-waypoint route between the viewer and the planner.

Ports and streams follow specs/001-multi-waypoint-route/contracts/route-streams.md.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool, String
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.route_follower.messages import CommandOp, build_state_document, parse_command
from dimos.navigation.route_follower.route import (
    MSG_STOP_ROUTE_FIRST,
    Route,
    RouteStatus,
    Waypoint,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

IDENTITY_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
GOAL_MATCH_RADIUS_M = 0.05
GOAL_MATCH_WINDOW_S = 0.1
ZERO_TWIST_WINDOW_S = 0.1
OWN_STOP_WINDOW_S = 1.0


class RouteFollowerConfig(ModuleConfig):
    max_waypoints: int = 20
    pass_radius_m: float = 0.6
    heartbeat_timeout_s: float = 3.0
    state_publish_hz: float = 2.0


class RouteFollower(Module):
    """Follow a planned route one goal at a time and report its state."""

    config: RouteFollowerConfig

    odom: In[PoseStamped]
    viewer_click: In[PointStamped]
    goal_reached: In[Bool]
    stop_seen: In[Bool]
    goal_seen: In[PoseStamped]
    tele_cmd_seen: In[Twist]
    route_cmd: In[String]

    clicked_point: Out[PointStamped]
    route_goal: Out[PoseStamped]
    route_stop: Out[Bool]
    route_state: Out[String]
    route_path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._route = Route(max_waypoints=self.config.max_waypoints)
        self._robot_position_known = False
        self._click_frame: str | None = None
        self._last_odom: PoseStamped | None = None
        self._last_goal: PoseStamped | None = None
        self._state_thread: threading.Thread | None = None
        self._clock = time.monotonic
        self._last_command_time = 0.0
        self._stop_pending_at: float | None = None
        self._own_stop_at: float | None = None
        self._twist_events: list[tuple[float, bool]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.viewer_click.subscribe(self._on_viewer_click)))
        self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_goal_reached)))
        self.register_disposable(Disposable(self.stop_seen.subscribe(self._on_stop_seen)))
        self.register_disposable(Disposable(self.goal_seen.subscribe(self._on_goal_seen)))
        self.register_disposable(Disposable(self.tele_cmd_seen.subscribe(self._on_tele_cmd_seen)))
        self.register_disposable(Disposable(self.route_cmd.subscribe(self._on_route_cmd)))
        self._stop_event.clear()
        self._last_command_time = self._clock()
        self._state_thread = threading.Thread(target=self._state_publish_loop, daemon=True)
        self._state_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._state_thread is not None:
            self._state_thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            revision = self._route.revision
            self._last_odom = msg
            was_known = self._robot_position_known
            self._robot_position_known = True
            self._check_pass_radius_locked()
            if self._route.revision != revision or not was_known:
                self._publish_route_outputs_locked()

    def _on_viewer_click(self, msg: PointStamped) -> None:
        with self._lock:
            revision = self._route.revision
            if self._route.status is RouteStatus.RUNNING:
                self._route.set_message(MSG_STOP_ROUTE_FIRST)
            elif not self._route.add_mode:
                self.clicked_point.publish(msg)
                return
            else:
                self._click_frame = msg.frame_id
                self._route.add_waypoint(msg.x, msg.y, msg.z)
            if self._route.revision != revision:
                self._publish_route_outputs_locked()

    def _on_goal_reached(self, msg: Bool) -> None:
        with self._lock:
            revision = self._route.revision
            if not msg.data:
                if self._stop_pending_at is not None:
                    self._stop_pending_at = None
                    return
                self._on_planner_failure_locked()
                return
            if self._route.status in (RouteStatus.RUNNING, RouteStatus.HELD):
                index = self._route.current_index
                if index is not None and index == len(self._route.waypoints):
                    self._route.complete()
            if self._route.revision != revision:
                self._publish_route_outputs_locked()

    def _on_planner_failure_locked(self) -> None:
        if self._route.status is RouteStatus.RUNNING:
            self._route.fail_current()
            self._publish_route_outputs_locked()

    def _on_stop_seen(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            now = self._clock()
            if self._own_stop_at is not None and now - self._own_stop_at < OWN_STOP_WINDOW_S:
                self._own_stop_at = None
                return
            if self._idle_twist_recent_locked(now):
                return
            revision = self._route.revision
            if self._route.status in (RouteStatus.RUNNING, RouteStatus.HELD):
                self._route.interrupt()
            if self._route.revision != revision:
                self._publish_route_outputs_locked()

    def _on_goal_seen(self, msg: PoseStamped) -> None:
        with self._lock:
            if self._matches_last_goal_locked(msg):
                return
            revision = self._route.revision
            if self._route.status in (RouteStatus.RUNNING, RouteStatus.HELD):
                self._route.interrupt()
            if self._route.revision != revision:
                self._publish_route_outputs_locked()

    def _on_tele_cmd_seen(self, msg: Twist) -> None:
        with self._lock:
            now = self._clock()
            zero = all(
                v == 0.0
                for v in (
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z,
                )
            )
            self._twist_events.append((now, zero))
            self._twist_events = [
                (t, z) for t, z in self._twist_events if now - t <= 2 * ZERO_TWIST_WINDOW_S
            ]

    def _idle_twist_recent_locked(self, now: float) -> bool:
        window = [z for t, z in self._twist_events if now - t <= ZERO_TWIST_WINDOW_S]
        return bool(window) and all(window)

    def _matches_last_goal_locked(self, msg: PoseStamped) -> bool:
        goal = self._last_goal
        if goal is None:
            return False
        if abs(msg.ts - goal.ts) > GOAL_MATCH_WINDOW_S:
            return False
        return math.dist((msg.x, msg.y, msg.z), (goal.x, goal.y, goal.z)) <= GOAL_MATCH_RADIUS_M

    def _on_route_cmd(self, msg: String) -> None:
        command = parse_command(msg.data)
        if command is None:
            logger.warning("Ignored invalid route command", raw=msg.data)
            return
        with self._lock:
            self._last_command_time = self._clock()
            revision = self._route.revision
            if command.op is CommandOp.KEEPALIVE:
                return
            if command.op is CommandOp.SET_MODE:
                self._route.set_add_mode(bool(command.enabled))
            elif command.op is CommandOp.START:
                if self._route.start():
                    self._issue_current_goal_locked()
            elif command.op is CommandOp.STOP:
                now = self._clock()
                self._own_stop_at = now
                self._stop_pending_at = now
                self.route_stop.publish(Bool(data=True))
                self._route.stop()
            elif command.op is CommandOp.REMOVE:
                if command.index is not None:
                    self._route.remove_waypoint(command.index)
            elif command.op is CommandOp.CLEAR:
                self._route.clear()
            if self._route.revision != revision:
                self._publish_route_outputs_locked()

    def _state_publish_loop(self) -> None:
        interval = 1.0 / self.config.state_publish_hz
        while not self._stop_event.wait(interval):
            self._maintenance_tick()

    def _maintenance_tick(self) -> None:
        with self._lock:
            if self._clock() - self._last_command_time >= self.config.heartbeat_timeout_s:
                if self._route.add_mode:
                    self._route.set_add_mode(False)
                if self._route.status is RouteStatus.RUNNING:
                    self._route.hold()
            self._publish_route_outputs_locked()

    def _check_pass_radius_locked(self) -> None:
        status = self._route.status
        if status not in (RouteStatus.RUNNING, RouteStatus.HELD):
            return
        index = self._route.current_index
        waypoints = self._route.waypoints
        if index is None or index >= len(waypoints):
            return
        if not self._within_pass_radius(waypoints[index - 1]):
            return
        self._route.mark_current_reached()
        if status is RouteStatus.RUNNING:
            self._issue_current_goal_locked()

    def _issue_current_goal_locked(self) -> None:
        self._stop_pending_at = None
        index = self._route.current_index
        if index is None:
            return
        waypoints = self._route.waypoints
        current = waypoints[index - 1]
        if index < len(waypoints) and self._within_pass_radius(current):
            self._route.mark_current_reached()
            self._issue_current_goal_locked()
            return
        goal = self._waypoint_pose(current)
        self._last_goal = goal
        self.route_goal.publish(goal)

    def _within_pass_radius(self, waypoint: Waypoint) -> bool:
        odom = self._last_odom
        if odom is None:
            return False
        return (
            math.dist((odom.x, odom.y, odom.z), (waypoint.x, waypoint.y, waypoint.z))
            <= self.config.pass_radius_m
        )

    def _waypoint_pose(self, waypoint: Waypoint) -> PoseStamped:
        return PoseStamped(
            ts=time.time(),
            frame_id=self._click_frame or "world",
            position=[waypoint.x, waypoint.y, waypoint.z],
            orientation=list(IDENTITY_ORIENTATION),
        )

    def _publish_route_outputs_locked(self) -> None:
        frame = self._click_frame or "world"
        now = time.time()
        self.route_state.publish(
            String(data=json.dumps(build_state_document(self._route, self._robot_position_known)))
        )
        self.route_path.publish(
            Path(
                ts=now,
                frame_id=frame,
                poses=[
                    PoseStamped(
                        ts=now,
                        frame_id=frame,
                        position=[w.x, w.y, w.z],
                        orientation=list(IDENTITY_ORIENTATION),
                    )
                    for w in self._route.waypoints
                ],
            )
        )


route_follower = RouteFollower.blueprint
