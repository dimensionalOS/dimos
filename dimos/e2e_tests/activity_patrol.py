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

"""Activity-aware patrol: drive the robot to observe the human NPCs.

A stationary robot only ever sees whatever its camera happens to face. This
controller instead round-robins the humans, driving the robot (over ``/cmd_vel``,
which ``GO2Connection`` consumes to move the DimSim agent) to a vantage point a
couple of metres from the current target, then rotating to face it and dwelling
briefly — so the robot's camera frames each human in turn while its 360° lidar
sweeps the surrounding room. The pursuit control law mirrors
``DirectCmdVelExplorer``: heading error drives the turn, and forward motion is
gated on roughly facing the target.
"""

from __future__ import annotations

import math
import threading
import time
from typing import TYPE_CHECKING

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle

if TYPE_CHECKING:
    from dimos.e2e_tests.human_activity import HumanActivityDriver

logger = setup_logger()


class ActivityPatrol:
    """Drive the robot to observe each human in turn while their tasks play out."""

    def __init__(
        self,
        driver: HumanActivityDriver,
        *,
        linear_speed: float = 0.7,
        rotation_speed: float = 1.5,
        vantage_m: float = 2.3,
        arrive_m: float = 0.7,
        observe_s: float = 3.0,
        drive_timeout_s: float = 18.0,
        publish_rate: float = 10.0,
    ) -> None:
        self._driver = driver
        self._linear_speed = linear_speed
        self._rotation_speed = rotation_speed
        self._vantage_m = vantage_m
        self._arrive_m = arrive_m
        self._observe_s = observe_s
        self._drive_timeout_s = drive_timeout_s
        self._dt = 1.0 / publish_rate

        self._cmd_vel: LCMTransport[Twist] | None = None
        self._odom: LCMTransport[PoseStamped] | None = None
        self._pose: PoseStamped | None = None
        self._unsub = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._spawn_xy: tuple[float, float] | None = None
        self._observed: set[str] = set()  # humans the robot got a good look at
        self._max_displacement = 0.0
        self._t0: float | None = None
        self._traj: list[tuple[float, float, float]] = []  # (elapsed_s, x, y)

    # ------------------------------------------------------------------ #

    def start(self) -> None:
        self._cmd_vel = LCMTransport("/cmd_vel", Twist)
        self._odom = LCMTransport("/odom", PoseStamped)
        self._t0 = time.monotonic()
        self._unsub = self._odom.subscribe(self._on_odom)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._publish(0.0, 0.0)
        if self._unsub:
            self._unsub()
        if self._cmd_vel:
            self._cmd_vel.stop()
        if self._odom:
            self._odom.stop()

    @property
    def observed_humans(self) -> set[str]:
        """Humans the robot drove up to and faced (a good camera look)."""
        return set(self._observed)

    @property
    def max_displacement_m(self) -> float:
        """Farthest the robot got from its spawn — proof it actually patrolled."""
        return self._max_displacement

    # ------------------------------------------------------------------ #

    def _on_odom(self, msg: PoseStamped) -> None:
        self._pose = msg
        if self._spawn_xy is None:
            self._spawn_xy = (msg.x, msg.y)
        else:
            d = math.hypot(msg.x - self._spawn_xy[0], msg.y - self._spawn_xy[1])
            self._max_displacement = max(self._max_displacement, d)
        # sample the trajectory at ~10 Hz for the activity view
        if self._t0 is not None:
            t = time.monotonic() - self._t0
            if not self._traj or t - self._traj[-1][0] >= 0.1:
                self._traj.append((t, msg.x, msg.y))

    @property
    def trajectory(self) -> list[tuple[float, float, float]]:
        """(elapsed_s, x, y) samples of the robot's real patrol path."""
        return list(self._traj)

    def _publish(self, linear: float, angular: float) -> None:
        if self._cmd_vel is not None:
            self._cmd_vel.broadcast(
                None, Twist(linear=Vector3(linear, 0, 0), angular=Vector3(0, 0, angular))
            )

    def _vantage_for(self, hx: float, hy: float, rx: float, ry: float) -> tuple[float, float]:
        """A point ``vantage_m`` from the human toward the robot (stand off, don't collide)."""
        dx, dy = rx - hx, ry - hy
        d = math.hypot(dx, dy)
        if d < 1e-3:
            dx, dy, d = 1.0, 0.0, 1.0
        return hx + dx / d * self._vantage_m, hy + dy / d * self._vantage_m

    def _loop(self) -> None:
        # wait for the first odom so we know where the robot is
        deadline = time.monotonic() + 15.0
        while self._pose is None and time.monotonic() < deadline and not self._stop.is_set():
            time.sleep(0.1)
        if self._pose is None:
            logger.warning("ActivityPatrol: no /odom received — robot will not patrol")
            return

        names = self._driver.human_names()
        rr_idx = 0
        while not self._stop.is_set():
            target = names[rr_idx % len(names)]
            rr_idx += 1
            self._observe_target(target)

    def _observe_target(self, name: str) -> None:
        """Drive to a vantage near ``name``, then face it and dwell."""
        drive_deadline = time.monotonic() + self._drive_timeout_s
        # --- approach ---
        while not self._stop.is_set() and time.monotonic() < drive_deadline:
            pose = self._pose
            hpos = self._driver.live_position(name)
            if pose is None or hpos is None:
                time.sleep(self._dt)
                continue
            vx, vy = self._vantage_for(hpos[0], hpos[1], pose.x, pose.y)
            dist = math.hypot(vx - pose.x, vy - pose.y)
            if dist < self._arrive_m:
                break
            heading = math.atan2(vy - pose.y, vx - pose.x)
            err = normalize_angle(heading - pose.yaw)
            linear = self._linear_speed if abs(err) < 0.35 else 0.0
            angular = max(-self._rotation_speed, min(self._rotation_speed, err * 2.0))
            self._publish(linear, angular)
            time.sleep(self._dt)

        # --- observe: rotate to face the human and hold ---
        observe_until = time.monotonic() + self._observe_s
        faced = False
        while not self._stop.is_set() and time.monotonic() < observe_until:
            pose = self._pose
            hpos = self._driver.live_position(name)
            if pose is None or hpos is None:
                time.sleep(self._dt)
                continue
            err = normalize_angle(math.atan2(hpos[1] - pose.y, hpos[0] - pose.x) - pose.yaw)
            if abs(err) < 0.25:
                faced = True
            angular = max(-self._rotation_speed, min(self._rotation_speed, err * 2.0))
            self._publish(0.0, angular)
            time.sleep(self._dt)
        self._publish(0.0, 0.0)
        if faced:
            self._observed.add(name)
