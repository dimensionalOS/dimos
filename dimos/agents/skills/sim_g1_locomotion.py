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
publishes on. ``GrootWBCTask`` zeros the velocity command if no twist
arrives within ``timeout=1.0`` seconds (`groot_wbc_task.py:152`), so
``move(duration>0)`` republishes the twist in a tight loop until the
duration elapses, mirroring ``unitree.connection.move`` (which
republishes every 10ms on real WebRTC).
"""

from __future__ import annotations

import threading
import time

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# 20Hz republish — well below the 1s WBC timeout, low overhead. Real
# Unitree WebRTC connection uses 100Hz; LCM through autoconnect has
# more per-publish overhead so we slow it down.
_REPUBLISH_INTERVAL_S = 0.05


class G1SimLocomotion(Module):
    cmd_vel: Out[Twist]

    _continuous_thread: threading.Thread | None
    _continuous_stop: threading.Event | None
    _control_lock: threading.Lock

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._continuous_thread = None
        self._continuous_stop = None
        self._control_lock = threading.Lock()

    @rpc
    def stop(self) -> None:
        self._stop_continuous()
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
            duration: How long to move (seconds). 0 means run continuously until the next move call.
        """

        twist = Twist(linear=Vector3(x, y, 0.0), angular=Vector3(0.0, 0.0, yaw))

        # Always cancel any prior continuous-mode publisher first.
        self._stop_continuous()

        if duration > 0:
            # Blocking republish loop; the LLM tool call returns when the
            # motion completes, then we send a zero twist to halt cleanly.
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.cmd_vel.publish(twist)
                time.sleep(_REPUBLISH_INTERVAL_S)
            self.cmd_vel.publish(_zero_twist())
            return f"Moved at velocity=({x}, {y}, {yaw}) for {duration} seconds"

        # duration == 0: continuous mode. Spawn a background thread that
        # keeps republishing this twist until cancelled (next move() call
        # or stop()).
        with self._control_lock:
            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._republish_until_cancelled,
                args=(twist, stop_event),
                name="G1SimLocomotion-continuous",
                daemon=True,
            )
            self._continuous_stop = stop_event
            self._continuous_thread = thread
            thread.start()

        return f"Moving continuously at velocity=({x}, {y}, {yaw}). Call move with non-zero values or duration to change."

    def _republish_until_cancelled(self, twist: Twist, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            self.cmd_vel.publish(twist)
            stop_event.wait(_REPUBLISH_INTERVAL_S)

    def _stop_continuous(self) -> None:
        with self._control_lock:
            stop_event = self._continuous_stop
            thread = self._continuous_thread
            self._continuous_stop = None
            self._continuous_thread = None
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)


def _zero_twist() -> Twist:
    return Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
