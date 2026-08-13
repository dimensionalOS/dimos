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

"""MovementManager: the click-to-goal relay, and the teleop cancel that goes with it.

The velocity mux this module used to also be is now its own module — `CmdVelMux`
in python, `CmdVelMuxNative` in rust — because it belongs on the robot with the
follower while the click relay belongs on the laptop with rerun. A stack needs
both: `MovementManager.blueprint()` alone turns clicks into goals but drives
nothing. One teleop keystroke has to land on both halves, so both subscribe
`tele_cmd_vel`: the mux preempts nav and stops the follower, this cancels the
goal. Nothing routes the cancel back the other way — teleop originates here.
"""

from __future__ import annotations

import math
import time

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# without this you can (basically) click into infinity in rerun (not good for the planner)
MAX_CLICK_HORIZONTAL_M = 500.0
MAX_CLICK_VERTICAL_M = 50.0


class MovementManagerConfig(ModuleConfig):
    pass


class MovementManager(Module):
    """Relay clicks to goals, and cancel the goal when the operator takes over."""

    config: MovementManagerConfig

    clicked_point: In[PointStamped]
    tele_cmd_vel: In[Twist]

    goal: Out[PointStamped]
    way_point: Out[PointStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.clicked_point.subscribe(self._on_click)))
        self.register_disposable(Disposable(self.tele_cmd_vel.subscribe(self._on_teleop)))

    def _on_click(self, msg: PointStamped) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            logger.warning("Ignored invalid click", x=msg.x, y=msg.y, z=msg.z)
            return
        if (
            abs(msg.x) > MAX_CLICK_HORIZONTAL_M
            or abs(msg.y) > MAX_CLICK_HORIZONTAL_M
            or abs(msg.z) > MAX_CLICK_VERTICAL_M
        ):
            logger.warning("Ignored out-of-range click", x=msg.x, y=msg.y, z=msg.z)
            return

        logger.debug("Goal", x=round(msg.x, 1), y=round(msg.y, 1), z=round(msg.z, 1))
        self.way_point.publish(msg)
        self.goal.publish(msg)

    def _cancel_goal(self) -> None:
        # NOTE: this NaN goal is more of a safety fallback.
        # It can be REALLY bad if a robot is supposed to stop moving but wont
        # we should probably think a more robust/strict requirement on planners
        cancel = PointStamped(
            ts=time.time(), frame_id="map", x=float("nan"), y=float("nan"), z=float("nan")
        )
        self.way_point.publish(cancel)
        self.goal.publish(cancel)
        logger.debug("Navigation cancelled — waiting for new goal")

    def _on_teleop(self, _msg: Twist) -> None:
        self._cancel_goal()
