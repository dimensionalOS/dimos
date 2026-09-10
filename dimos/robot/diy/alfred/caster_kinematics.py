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

"""Body twist to powered-caster steer and drive joint targets, for display only.

The FlowBase controller does this on the Pi and reports no caster state, so this
reproduces the geometry from the commanded cmd_vel for a URDF with caster joints.
Per caster with kingpin (hx, hy) in base_link: v = (vx - wz*hy, vy + wz*hx),
steer = atan2(vy, vx), drive += |v| / r * dt. Steer zero is heading +X.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState

CASTER_HARDWARE_ID = "casters"
CASTER_CORNERS = ("front_left", "front_right", "rear_left", "rear_right")
# kingpins in base_link (m): i2rt hips +/-0.2 in x, +/-0.1985 in y from the Flow Base CAD
CASTER_HIPS = {
    "front_left": (0.200, 0.1985),
    "front_right": (0.200, -0.1985),
    "rear_left": (-0.200, 0.1985),
    "rear_right": (-0.200, -0.1985),
}
WHEEL_RADIUS_M = 0.050


def caster_coordinator_joints() -> list[str]:
    """Coordinator joint names: casters/<corner>_steer, casters/<corner>_drive."""
    return [f"{CASTER_HARDWARE_ID}/{c}_{k}" for c in CASTER_CORNERS for k in ("steer", "drive")]


def caster_urdf_joints() -> list[str]:
    """Matching alfred_v2.urdf joint names."""
    return [f"{c}_{k}_joint" for c in CASTER_CORNERS for k in ("caster", "wheel")]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class CasterKinematics(Module):
    """Subscribe ``cmd_vel``; publish caster steer/drive positions as ``joint_command``."""

    cmd_vel: In[Twist]
    joint_command: Out[JointState]

    def __init__(
        self,
        rate_hz: float = 30.0,
        wheel_radius: float = WHEEL_RADIUS_M,
        min_speed: float = 1e-3,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._rate_hz = rate_hz
        self._r = wheel_radius
        self._min_speed = min_speed
        self._twist = (0.0, 0.0, 0.0)
        self._steer = dict.fromkeys(CASTER_CORNERS, 0.0)
        self._drive = dict.fromkeys(CASTER_CORNERS, 0.0)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    async def handle_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._twist = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _step(self, dt: float) -> JointState:
        with self._lock:
            vx, vy, wz = self._twist
        names: list[str] = []
        pos: list[float] = []
        vel: list[float] = []
        for corner in CASTER_CORNERS:
            hx, hy = CASTER_HIPS[corner]
            vix, viy = vx - wz * hy, vy + wz * hx
            speed = math.hypot(vix, viy)
            if speed > self._min_speed:
                self._steer[corner] = math.atan2(viy, vix)
                omega = speed / self._r
            else:
                omega = 0.0
            self._drive[corner] = _wrap(self._drive[corner] + omega * dt)
            names += [
                f"{CASTER_HARDWARE_ID}/{corner}_steer",
                f"{CASTER_HARDWARE_ID}/{corner}_drive",
            ]
            pos += [self._steer[corner], self._drive[corner]]
            vel += [0.0, omega]
        return JointState(name=names, position=pos, velocity=vel)

    def _run_loop(self) -> None:
        period = 1.0 / self._rate_hz
        last = time.perf_counter()
        while not self._stop_event.is_set():
            now = time.perf_counter()
            dt, last = now - last, now
            self.joint_command.publish(self._step(dt))
            time.sleep(period)

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run_loop, daemon=True, name="caster-kinematics"
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()
