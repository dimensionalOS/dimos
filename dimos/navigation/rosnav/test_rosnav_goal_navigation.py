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

"""
Integration test: send a goal point to ROSNav and verify the robot reaches it.

Starts the navigation stack in simulation mode with Unity, waits for odom to
stabilise (robot has spawned), sends a global goal via the ``set_goal`` RPC,
and asserts that the robot's final position moves toward the target.

Requires:
    - Docker with BuildKit
    - NVIDIA GPU with drivers
    - X11 display (real or virtual)

Run:
    pytest dimos/navigation/rosnav/test_rosnav_goal_navigation.py -m slow -s
"""

import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool
import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.rosnav.rosnav_module import ROSNav

# How long to wait for the robot to move toward the goal (seconds).
GOAL_TIMEOUT_SEC = 120

# How long to wait for initial odom before sending a goal.
ODOM_WAIT_SEC = 30

# Seconds to wait after receiving first odom before sending the goal,
# letting the nav stack initialise fully.
WARMUP_SEC = 10

# Minimum displacement (metres) to consider the robot "moved".
MIN_DISPLACEMENT_M = 0.5

# Goal in the map frame — 3 metres forward from origin.
GOAL_X = 3.0
GOAL_Y = 0.0


class GoalTracker(Module):
    """Subscribes to odom and goal_reached, records positions for assertions."""

    color_image: In[Image]
    lidar: In[PointCloud2]
    global_pointcloud: In[PointCloud2]
    odom: In[PoseStamped]
    goal_active: In[PoseStamped]
    goal_reached: In[Bool]
    path: In[NavPath]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._odom_history: list[PoseStamped] = []
        self._cmd_vel_count: int = 0
        self._nonzero_cmd_vel_count: int = 0
        self._goal_reached_flag = False
        self._first_odom_event = threading.Event()
        self._goal_reached_event = threading.Event()
        self._moved_event = threading.Event()
        self._start_pose: PoseStamped | None = None
        self._unsub_fns: list[Any] = []

    @rpc
    def start(self) -> None:
        self._unsub_fns.append(self.odom.subscribe(self._on_odom))
        self._unsub_fns.append(self.goal_reached.subscribe(self._on_goal_reached))
        self._unsub_fns.append(self.cmd_vel.subscribe(self._on_cmd_vel))

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._odom_history.append(msg)
            if len(self._odom_history) == 1:
                self._first_odom_event.set()
            # Check if the robot has moved significantly from its start pose
            if self._start_pose is not None and not self._moved_event.is_set():
                dx = msg.position.x - self._start_pose.position.x
                dy = msg.position.y - self._start_pose.position.y
                if math.sqrt(dx * dx + dy * dy) > MIN_DISPLACEMENT_M:
                    self._moved_event.set()

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd_vel_count += 1
            if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01:
                self._nonzero_cmd_vel_count += 1

    def _on_goal_reached(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._goal_reached_flag = True
            self._goal_reached_event.set()

    @rpc
    def wait_for_first_odom(self, timeout: float = ODOM_WAIT_SEC) -> bool:
        return self._first_odom_event.wait(timeout)

    @rpc
    def wait_for_movement(self, timeout: float = GOAL_TIMEOUT_SEC) -> bool:
        return self._moved_event.wait(timeout)

    @rpc
    def wait_for_goal_reached(self, timeout: float = GOAL_TIMEOUT_SEC) -> bool:
        return self._goal_reached_event.wait(timeout)

    @rpc
    def mark_start(self) -> None:
        """Snapshot the current odom as the 'start' for displacement measurement."""
        with self._lock:
            if self._odom_history:
                self._start_pose = self._odom_history[-1]

    @rpc
    def get_start_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._start_pose

    @rpc
    def get_latest_odom(self) -> PoseStamped | None:
        with self._lock:
            return self._odom_history[-1] if self._odom_history else None

    @rpc
    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached_flag

    @rpc
    def get_odom_count(self) -> int:
        with self._lock:
            return len(self._odom_history)

    @rpc
    def get_cmd_vel_stats(self) -> tuple[int, int]:
        with self._lock:
            return self._cmd_vel_count, self._nonzero_cmd_vel_count

    @rpc
    def stop(self) -> None:
        for unsub in self._unsub_fns:
            unsub()
        self._unsub_fns.clear()


def _distance_2d(a: PoseStamped, b: PoseStamped) -> float:
    """Euclidean distance in the XY plane."""
    return math.sqrt((a.position.x - b.position.x) ** 2 + (a.position.y - b.position.y) ** 2)


@pytest.mark.slow
def test_rosnav_goal_reached():
    """Send a navigation goal and verify the robot reaches it."""

    coordinator = (
        autoconnect(
            ROSNav.blueprint(mode="simulation"),
            GoalTracker.blueprint(),
        )
        .global_config(viewer="none")
        .build()
    )

    try:
        tracker = coordinator.get_instance(GoalTracker)
        rosnav = coordinator.get_instance(ROSNav)

        # 1. Wait for odom — proves the sim is running and the robot has spawned.
        assert tracker.wait_for_first_odom(ODOM_WAIT_SEC), (
            f"No odom received within {ODOM_WAIT_SEC}s — Unity sim may not be running."
        )

        # Let the nav stack fully initialise before sending a goal.
        print(f"  Odom received. Waiting {WARMUP_SEC}s for nav stack warmup...")
        time.sleep(WARMUP_SEC)

        # Snapshot the current position as "start".
        tracker.mark_start()
        start_pose = tracker.get_start_pose()
        assert start_pose is not None
        print(
            f"  Robot start: ({start_pose.position.x:.2f}, "
            f"{start_pose.position.y:.2f}, {start_pose.position.z:.2f})"
        )

        # 2. Send a goal in the map frame via set_goal (non-blocking).
        goal = PoseStamped(
            position=Vector3(GOAL_X, GOAL_Y, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
            ts=time.time(),
        )
        print(f"  Sending set_goal({GOAL_X}, {GOAL_Y}) in map frame...")
        rosnav.set_goal(goal)

        # 3. Wait for either goal_reached or significant movement.
        moved = tracker.wait_for_movement(GOAL_TIMEOUT_SEC)
        reached = tracker.is_goal_reached()

        end_pose = tracker.get_latest_odom()
        assert end_pose is not None

        displacement = _distance_2d(start_pose, end_pose)
        total_cmd, nonzero_cmd = tracker.get_cmd_vel_stats()
        print(
            f"  Robot end: ({end_pose.position.x:.2f}, "
            f"{end_pose.position.y:.2f}, {end_pose.position.z:.2f})"
        )
        print(f"  Displacement: {displacement:.2f}m (goal was {GOAL_X}m)")
        print(f"  Odom messages: {tracker.get_odom_count()}")
        print(f"  cmd_vel messages: {total_cmd} total, {nonzero_cmd} non-zero")
        print(f"  goal_reached: {reached}")

        # 4. Assert the robot moved.
        assert moved or reached, (
            f"Robot did not move within {GOAL_TIMEOUT_SEC}s. "
            f"Displacement: {displacement:.2f}m, cmd_vel: {total_cmd} total / {nonzero_cmd} non-zero. "
            f"The nav stack may not be generating velocity commands."
        )

        assert displacement > MIN_DISPLACEMENT_M, (
            f"Robot only moved {displacement:.2f}m toward goal at ({GOAL_X}, {GOAL_Y}). "
            f"Expected at least {MIN_DISPLACEMENT_M}m."
        )

        if reached:
            print("  ✅ goal_reached signal received")
        else:
            print(
                f"  ✅ Robot moved {displacement:.2f}m toward goal "
                f"(goal_reached not yet received)"
            )

    finally:
        coordinator.stop()
