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

"""PACK MIND — direct velocity teleop for the live demo.

``relative_move`` is goal-based: it runs the A* planner and then waits on an
inter-module RPC (``ReplanningAStarPlanner/is_goal_reached``). On a CPU/macOS host
that RPC over LCM is flaky and can hang for 120s — so the dog reaches the goal but
the call never returns, freezing the driver.

This bypasses all of that: publish a ``Twist`` straight to ``MovementManager``'s
``tele_cmd_vel`` (its keyboard-teleop lane, which takes priority and forwards to the
dog). The dog moves the instant the Twist lands — no goal, no confirmation round
trip. Drive in short bursts; ``drive`` auto-stops after ``duration``.
"""

from __future__ import annotations

import time

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_MAX_DURATION = 3.0  # safety cap on a single burst


class VelocityTeleop(Module):
    """Publishes velocity bursts to MovementManager's teleop lane (no planner)."""

    tele_cmd_vel: Out[Twist]

    @rpc
    def start(self) -> None:
        super().start()

    def _publish(self, forward: float, turn: float) -> None:
        self.tele_cmd_vel.publish(
            Twist(linear=Vector3(forward, 0.0, 0.0), angular=Vector3(0.0, 0.0, turn))
        )

    @skill
    def drive(self, forward: float = 0.0, turn: float = 0.0, duration: float = 0.6) -> str:
        """Drive the robot by velocity for a short burst, then stop. Instant — no
        path planner. Use this for keyboard teleop instead of relative_move.

        Args:
            forward: Linear speed in m/s. Positive = forward, negative = back.
            turn: Angular speed in rad/s. Positive = turn left, negative = right.
            duration: Seconds to move before auto-stopping (capped at 3s).
        """
        duration = max(0.0, min(duration, _MAX_DURATION))
        self._publish(forward, turn)
        time.sleep(duration)
        self._publish(0.0, 0.0)  # stop
        logger.info("teleop drive", forward=forward, turn=turn, duration=round(duration, 2))
        return f"drove forward={forward} turn={turn} for {duration:.1f}s"

    @skill
    def halt(self) -> str:
        """Stop the robot immediately."""
        self._publish(0.0, 0.0)
        return "stopped"


velocity_teleop = VelocityTeleop.blueprint
