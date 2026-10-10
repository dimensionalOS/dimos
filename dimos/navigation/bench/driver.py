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

"""Runs one benchmark case against the navigation stack in the simulated world.

Resets the robot to the case's start, premaps by seeding the stack with the scene's cloud or
by walking the reference route on teleop, sends the goal until the stack echoes it, then
watches for a terminal condition and writes it to terminal.json. Collisions never end an
episode. The runner stops the process.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import json
import math
from pathlib import Path
from threading import Event, Thread
import time

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Bool import Bool
from dimos.navigation.bench.ground_truth import GroundTruth
from dimos.navigation.bench.oracle import RouteTracker
from dimos.navigation.bench.scorer import GOAL_ECHO_M
from dimos.navigation.bench.suite import Manifest
from dimos.simulation.go2_sim.world_spec import SimWorldSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TERMINAL_FILE = "terminal.json"
ODOM_FRAME_ID = "odom"
WATCH_DT = 0.05
FIRST_POSE_WAIT_S = 60.0


class EpisodeDriverConfig(ModuleConfig):
    manifest: Path = Path()
    case_id: str = ""
    out_dir: Path = Path()
    settle_s: float = 2.0
    seed_settle_s: float = 4.0
    teleop_hz: float = 20.0
    goal_resend_s: float = 1.0
    goal_wait_s: float = 8.0


class EpisodeDriver(Module):
    """Drives one case: premap, reset, goal, and the terminal condition."""

    config: EpisodeDriverConfig

    ground_truth: In[PoseStamped]
    goal: In[PointStamped]
    goal_reached: In[Bool]
    cmd_vel: In[Twist]

    tele_cmd_vel: Out[Twist]
    clicked_point: Out[PointStamped]
    loaded_map: Out[PointCloud2]

    _world: SimWorldSpec
    _thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        manifest = Manifest.load(self.config.manifest)
        self._rules = manifest.rules
        self._case = next(c for c in manifest.cases if c.id == self.config.case_id)
        self._truth = GroundTruth(self._case.scene())
        self._pose: tuple[float, NDArray[np.float64], NDArray[np.float64]] | None = None
        self._recent: deque[tuple[float, NDArray[np.float64]]] = deque()
        self._moving: deque[tuple[float, bool]] = deque()
        self._last_moving: float | None = None
        self._echo: float | None = None
        self._arrival: float | None = None
        self._stop_event = Event()
        self.register_disposable(Disposable(self.ground_truth.subscribe(self._on_pose)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        self.register_disposable(Disposable(self.goal_reached.subscribe(self._on_reached)))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd)))
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_pose(self, msg: PoseStamped) -> None:
        xyz = np.array(tuple(msg.position), dtype=np.float64)
        rpy = np.array(tuple(msg.orientation.to_euler()), dtype=np.float64)
        self._pose = (msg.ts, xyz, rpy)
        self._recent.append((msg.ts, xyz))
        while self._recent and self._recent[0][0] < msg.ts - self._rules.stuck_s:
            self._recent.popleft()

    def _on_goal(self, msg: PointStamped) -> None:
        if np.linalg.norm(np.subtract((msg.x, msg.y, msg.z), self._case.goal)) <= GOAL_ECHO_M:
            self._echo = self._echo if self._echo is not None else msg.ts

    def _on_reached(self, msg: Bool) -> None:
        if msg.data and self._echo is not None and self._arrival is None:
            self._arrival = time.time()

    def _on_cmd(self, msg: Twist) -> None:
        moving = (
            math.hypot(msg.linear.x, msg.linear.y) >= self._rules.moving_cmd
            or abs(msg.angular.z) >= self._rules.moving_cmd
        )
        now = time.time()
        self._moving.append((now, moving))
        if moving:
            self._last_moving = now
        while self._moving and self._moving[0][0] < now - self._rules.stuck_s:
            self._moving.popleft()

    def _run(self) -> None:
        try:
            self._write(self._episode())
        except Exception:
            logger.exception("Episode driver failed")
            self._write({"reason": "driver_error"})

    def _episode(self) -> dict[str, object]:
        case, rules = self._case, self._rules
        if not self._wait(lambda: self._pose is not None, FIRST_POSE_WAIT_S):
            return {"reason": "no_ground_truth"}
        self._reset_to_start()
        record: dict[str, object] = {"case_id": case.id, "premap": rules.premap}
        started = time.time()
        route = self._truth.route(case.start, case.goal, centered=True)
        if route is None:
            return {**record, "reason": "no_reference_route"}
        if rules.premap == "seed":
            cloud = self._truth.premap_cloud(route.points)
            self.loaded_map.publish(
                PointCloud2.from_numpy(cloud, frame_id=ODOM_FRAME_ID, timestamp=time.time())
            )
            record["premap_points"] = len(cloud)
            self._stop_event.wait(self.config.seed_settle_s)
        else:
            walked = self._walk(route.points, rules.timeout_s(route.length) or FIRST_POSE_WAIT_S)
            record["premap_walked"] = walked
            self._reset_to_start()
        record["premap_s"] = round(time.time() - started, 2)
        t0 = self._send_goal()
        if t0 is None:
            return {**record, "reason": "goal_lost"}
        record["t0"] = t0
        reason = self._watch(t0, rules.timeout_s(case.route_length))
        return {**record, "reason": reason, "t_end": time.time()}

    def _reset_to_start(self) -> None:
        x, y, yaw = self._case.start
        z = float(self._truth.height[self._truth.index((x, y))])
        self._world.reset_pose(x, y, z, yaw)
        self._stop_event.wait(self.config.settle_s)

    def _walk(self, route: NDArray[np.float64], timeout_s: float) -> bool:
        """Teleop along the route from the true pose until the tracker arrives."""
        tracker = RouteTracker(route)
        deadline = time.time() + timeout_s
        period = 1.0 / self.config.teleop_hz
        arrived = False
        while not arrived and time.time() < deadline and not self._stop_event.is_set():
            assert self._pose is not None
            _, xyz, rpy = self._pose
            command, arrived = tracker.step(float(xyz[0]), float(xyz[1]), float(rpy[2]))
            self.tele_cmd_vel.publish(
                Twist(Vector3(command[0], command[1], 0.0), Vector3(0.0, 0.0, command[2]))
            )
            self._stop_event.wait(period)
        self.tele_cmd_vel.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        return arrived

    def _send_goal(self) -> float | None:
        """Send the goal until the stack echoes it. The echo's stamp starts the episode clock."""
        deadline = time.time() + self.config.goal_wait_s
        while self._echo is None and time.time() < deadline and not self._stop_event.is_set():
            self.clicked_point.publish(
                PointStamped(*self._case.goal, ts=time.time(), frame_id=ODOM_FRAME_ID)
            )
            self._wait(lambda: self._echo is not None, self.config.goal_resend_s)
        return self._echo

    def _watch(self, t0: float, timeout_s: float | None) -> str:
        rules = self._rules
        if self._last_moving is None or self._last_moving < t0:
            self._last_moving = t0
        while not self._stop_event.is_set():
            now = time.time()
            if self._arrival is not None:
                return "arrived"
            assert self._pose is not None
            if np.any(np.abs(self._pose[2][:2]) > rules.fall_rad):
                return "fall"
            if timeout_s is not None and now - t0 >= timeout_s:
                return "timeout"
            if now - t0 >= rules.stuck_s and (verdict := self._stuck_or_stalled(now)) is not None:
                return verdict
            self._stop_event.wait(WATCH_DT)
        return "stopped"

    def _stuck_or_stalled(self, now: float) -> str | None:
        # the callbacks keep appending, so work on copies
        commands, recent = list(self._moving), list(self._recent)
        moving = [m for t, m in commands if t >= now - self._rules.stuck_s]
        stalled = self._last_moving is not None and now - self._last_moving >= self._rules.stalled_s
        if not moving or not recent:
            return "stalled" if stalled else None
        if sum(moving) / len(moving) >= 0.8:
            xy = np.array([xyz[:2] for _, xyz in recent])
            progress = float(np.linalg.norm(xy - xy[0], axis=1).max())
            return "stuck" if progress < self._rules.stuck_progress_m else None
        return "stalled" if stalled else None

    def _wait(self, done: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline and not self._stop_event.is_set():
            if done():
                return True
            self._stop_event.wait(WATCH_DT)
        return done()

    def _write(self, record: dict[str, object]) -> None:
        self.config.out_dir.mkdir(parents=True, exist_ok=True)
        path = self.config.out_dir / TERMINAL_FILE
        path.write_text(json.dumps(record, indent=2) + "\n")
        logger.info("Episode terminal", **{k: v for k, v in record.items() if k != "t0"})
