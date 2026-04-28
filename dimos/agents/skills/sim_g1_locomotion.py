# Copyright 2025-2026 Dimensional Inc.
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

"""Sim-friendly G1 locomotion skill.

The DDS-based ``UnitreeG1SkillContainer.move()`` requires a
``G1ConnectionSpec`` provider. The in-process ``MujocoSimModule`` +
SHM whole-body adapter pair don't implement that spec, so the regular
container can't be composed in sim blueprints.

This skill talks to the same ``/cmd_vel`` topic the WASD dashboard
publishes on — ``ControlCoordinator`` + ``GrootWBCTask`` already
consume that twist. ``move(duration=0)`` leaves the twist applied
until a new call (matches the DDS skill's semantics).
"""

from __future__ import annotations

import threading
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


class G1SimLocomotion(Module):
    cmd_vel: Out[Twist]

    _lock: threading.Lock
    _stop_timer: threading.Timer | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._stop_timer = None

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None
            self.cmd_vel.publish(_zero_twist())
        super().stop()

    @skill
    def move(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            move(**args)

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). 0 keeps the velocity until the next move() call.
        """

        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, yaw))

        with self._lock:
            if self._stop_timer is not None:
                self._stop_timer.cancel()
                self._stop_timer = None
            self.cmd_vel.publish(twist)
            if duration > 0:
                self._stop_timer = threading.Timer(duration, self._publish_zero)
                self._stop_timer.daemon = True
                self._stop_timer.start()

        return f"Moving at velocity=({x}, {y}, {yaw}) for {duration} seconds"

    def _publish_zero(self) -> None:
        with self._lock:
            self._stop_timer = None
            self.cmd_vel.publish(_zero_twist())


def _zero_twist() -> Twist:
    return Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
