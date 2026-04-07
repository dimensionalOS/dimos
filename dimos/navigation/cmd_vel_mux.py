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

"""CmdVelMux: merges nav and teleop velocity commands.

Teleop (tele_cmd_vel) takes priority over autonomous navigation
(nav_cmd_vel). When teleop is active, nav commands are suppressed
and a stop_movement signal is published. After a cooldown period
with no teleop input, nav commands resume.
"""

from __future__ import annotations

import threading
from typing import Any

from dimos_lcm.std_msgs import Bool

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CmdVelMuxConfig(ModuleConfig):
    teleop_cooldown_sec: float = 1.0
    teleop_linear_scale: float = 1.0


class CmdVelMux(Module[CmdVelMuxConfig]):
    """Multiplexes nav_cmd_vel and tele_cmd_vel into a single cmd_vel output.

    When teleop input arrives, stop_movement is published so downstream
    modules (planner, explorer) can cancel their active goals.

    Ports:
        nav_cmd_vel (In[Twist]): Velocity from the autonomous planner.
        tele_cmd_vel (In[Twist]): Velocity from keyboard/joystick teleop.
        cmd_vel (Out[Twist]): Merged output — teleop wins when active.
        stop_movement (Out[Bool]): Published when teleop begins.
    """

    default_config = CmdVelMuxConfig

    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]
    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._teleop_active = False
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        state.pop("_lock", None)
        state.pop("_timer", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._timer = None

    @rpc
    def start(self) -> None:
        self.nav_cmd_vel.subscribe(self._on_nav)
        self.tele_cmd_vel.subscribe(self._on_teleop)

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
        super().stop()

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            if self._teleop_active:
                return
        self.cmd_vel.publish(msg)

    def _on_teleop(self, msg: Twist) -> None:
        was_active: bool
        with self._lock:
            was_active = self._teleop_active
            self._teleop_active = True
            if self._timer is not None:
                self._timer.cancel()
            self._timer = threading.Timer(
                self.config.teleop_cooldown_sec,
                self._end_teleop,
            )
            self._timer.daemon = True
            self._timer.start()

        if not was_active:
            self.stop_movement.publish(Bool(data=True))
            logger.info("Teleop active — published stop_movement")

        s = self.config.teleop_linear_scale
        if s != 1.0:
            msg = Twist(
                linear=[msg.linear.x * s, msg.linear.y * s, msg.linear.z],
                angular=[msg.angular.x, msg.angular.y, msg.angular.z],
            )
        self.cmd_vel.publish(msg)

    def _end_teleop(self) -> None:
        with self._lock:
            self._teleop_active = False
            self._timer = None
