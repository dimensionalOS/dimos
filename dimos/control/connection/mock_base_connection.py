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

"""A mock joint connection that also turns its base speeds into odometry."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry

_BASE_AXES = ("base_forward", "base_lateral", "base_turn")


class MockBaseConnection(MockJointConnection):
    """``MockJointConnection`` that also publishes where its commanded base speeds took it.

    The base is the velocity joints ``<source>/base_forward`` and ``base_lateral``
    (m/s) and ``base_turn`` (rad/s). Each reading period their speeds move a pose that
    starts at 0 in the ``odom`` frame, published on ``odometry``; nothing slips or lags.
    """

    odometry: Out[Odometry]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._base: list[str] = []
        self._stop_odometry = threading.Event()
        self._odometer: threading.Thread | None = None

    def connect(self) -> None:
        super().connect()
        commands = self.config.joint_commands
        self._base = [self._full_name(axis) for axis in _BASE_AXES]
        missing = [
            joint
            for joint, axis in zip(self._base, _BASE_AXES, strict=True)
            if commands.get(joint, commands.get(axis)) != (Interface.VELOCITY,)
        ]
        if missing:
            raise ValueError(f"MockBaseConnection needs velocity base joints {missing}")

    @rpc
    def start(self) -> None:
        super().start()
        self._odometer = threading.Thread(target=self._odometry_loop, daemon=True)
        self._odometer.start()

    @rpc
    def stop(self) -> None:
        self._stop_odometry.set()
        if self._odometer is not None and self._odometer is not threading.current_thread():
            self._odometer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _odometry_loop(self) -> None:
        x = y = yaw = 0.0
        last = time.monotonic()
        while not self._stop_odometry.wait(1.0 / self.config.state_rate_hz):
            with self._hardware_lock:
                vx, vy, wz = (self._speeds[joint] for joint in self._base)
            now = time.monotonic()
            dt, last = now - last, now
            # Body-frame speeds rotated into the odom frame at the current heading.
            x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
            y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
            yaw += wz * dt
            position = Vector3(x, y, 0.0)
            orientation = Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            self.odometry.publish(
                Odometry(
                    ts=time.time(),
                    frame_id=self.tf_frames.get("odom", "odom"),
                    child_frame_id=self._frame,
                    pose=Pose(position, orientation),
                    twist=Twist(Vector3(vx, vy, 0.0), Vector3(0.0, 0.0, wz)),
                )
            )
