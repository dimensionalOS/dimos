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

"""TrajectoryFollower: the motion controller as a dimos module.

A transport shell around the pure pose+path -> twist ``TrajectoryController``; it owns subscriptions, the
control clock and arrival. No map: the room the planner priced arrives in the path's stamps (``control/profile.py``).
"""

from __future__ import annotations

from collections.abc import Callable
import math
from threading import Event, RLock, Thread
import time
from typing import Any

from pydantic import ImportString
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.tf_pose import TfPose
from dimos.navigation.trajectory_follower.fancy.controller import TrajectoryController
from dimos.navigation.trajectory_follower.fancy.laws import hinted
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GoalLatch:
    """Arrival edge detector: fires once per goal, then holds until it moves."""

    def __init__(self, tolerance: float) -> None:
        self.tolerance = tolerance
        self._goal: tuple[float, float] | None = None
        self._reached = False

    @property
    def reached(self) -> bool:
        return self._reached

    def set_goal(self, xy: tuple[float, float]) -> None:
        # moves under the tolerance are the same goal: replans snap the path end to the grid
        if self._goal is None or math.dist(xy, self._goal) > self.tolerance:
            self._goal = xy
            self._reached = False

    def arrive(self, xy: tuple[float, float]) -> bool:
        """True exactly once: the tick this position first reaches the goal."""
        if self._goal is None or self._reached:
            return False
        if math.dist(xy, self._goal) < self.tolerance:
            self._reached = True
            return True
        return False


class TrajectoryFollowerConfig(ModuleConfig):
    # "module:factory" of the law; None runs `laws/hinted.py` (the native twin's law), `laws/seed.py:make` is the A/B baseline.
    controller: ImportString[Callable[..., Any]] | None = None
    control_frequency: float = 10.0
    goal_tolerance: float = 0.20  # planar distance that counts as arrival (m)
    # The planner's own body: the law decodes the path stamps with its governor band.
    embodiment: Embodiment = GO2
    # The pose is the `path.frame_id -> base_frame` edge on tf, read each tick.
    base_frame: str = "base_link"
    # Deadman: zero the twist once the held path is this old, measured from arrival; must clear the ~1 Hz replan cadence.
    max_path_age_s: float = 2.5


class TrajectoryFollower(Module, spec.TrajectoryFollower):
    """Track the planned path; stop and latch goal_reached on arrival."""

    config: TrajectoryFollowerConfig

    path: In[Path]
    tf: IO[TFMessage]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        # Built in start(): the tf buffer needs the port's transport.
        self._pose_src: TfPose | None = None
        self._path: Path | None = None
        self._path_at: float | None = None
        self._latch = GoalLatch(self.config.goal_tolerance)
        self._controller: TrajectoryController = (self.config.controller or hinted.make)(
            self.config.embodiment
        )
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._controller.reset()
        self._pose_src = TfPose(self.tfbuffer, self.config.base_frame, self.config.max_path_age_s)
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        self._thread = Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_path(self, msg: Path) -> None:
        """A plan to follow. An empty one is a stop: forget the plan and halt now."""
        with self._lock:
            if not msg.poses:
                self._path = None
                self._path_at = None
                self._controller.reset()
            else:
                self._path = msg
                self._path_at = time.monotonic()
                if len(msg.poses) >= 2:
                    # a single-pose stub is a refusal, not an arrival target
                    self._latch.set_goal((msg.poses[-1].position.x, msg.poses[-1].position.y))
        if not msg.poses:
            # now, not a control period later
            self.nav_cmd_vel.publish(Twist())

    def _control_loop(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            started = time.perf_counter()
            self.tick()
            elapsed = time.perf_counter() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def tick(self) -> None:
        """One control tick: step, or zero the twist and say what is missing."""
        now = time.monotonic()
        with self._lock:
            path = self._path
            age = None if self._path_at is None else now - self._path_at
        pose = None
        if path is not None and self._pose_src is not None:
            pose = self._pose_src.get(path.frame_id)
        if path is not None and pose is not None:
            assert age is not None
            self.step(pose, path, age)
        elif path is not None:
            # a plan with no live pose under it: the deadman on the pose
            self.nav_cmd_vel.publish(Twist())

    def step(self, pose: PoseStamped, path: Path, age: float) -> None:
        """One control tick against a path that arrived `age` seconds ago."""
        # the deadman outranks arrival: a goal reached against an unrefreshed plan is a coincidence
        if age > self.config.max_path_age_s:
            self.nav_cmd_vel.publish(Twist())
            return
        xy = (pose.position.x, pose.position.y)
        with self._lock:
            arrived, reached = self._latch.arrive(xy), self._latch.reached
        if arrived:
            self.nav_cmd_vel.publish(Twist())
            self.goal_reached.publish(Bool(True))
            logger.info("goal reached", x=round(xy[0], 2), y=round(xy[1], 2))
            return
        if reached:
            self.nav_cmd_vel.publish(Twist())
            return
        self.nav_cmd_vel.publish(self._controller.update(pose, path, time.monotonic()))
