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

"""CmdVelMux: teleop preempts nav on cmd_vel, and a watchdog zeros it when nav dies.

The python twin of the `cmd_vel_mux` native module (`cmd_vel_mux_native.py`),
which is the one that goes on the robot. Same ports, same config names, same
decisions — a stack swaps one for the other without touching a remapping.

Pairs with `MovementManager`, which kept the click relay. Both subscribe
`tele_cmd_vel` so one keystroke reaches both halves: this one preempts nav and
stops the follower, that one cancels the goal.
"""

from __future__ import annotations

from threading import Event, Lock, Thread
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How finely the deadman is sampled. `nav_stale_s` says when it trips; this only
# says how promptly. Fixed for the same reason it is fixed in the rust.
WATCHDOG_HZ = 10.0


class CmdVelMuxConfig(ModuleConfig):
    # How long a teleop command keeps nav preempted.
    tele_cooldown_sec: float = 1.0
    # Per-axis multipliers applied to teleop twists only. Nav is forwarded raw.
    tele_scale_linear: list[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])
    tele_scale_angular: list[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])
    # The deadman: nav_cmd_vel unheard for this long holds cmd_vel at zero.
    # 0.5 s is five missed ticks of a 10 Hz follower.
    nav_stale_s: float = 0.5


class CmdVelMux(Module):
    """Mux teleop over nav onto cmd_vel, and zero it when nav goes quiet."""

    config: CmdVelMuxConfig

    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]

    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = Lock()
        # Arrival times, never msg.ts: what these decide is how long since a
        # producer was last heard from, and the producer's clock is not ours.
        self._last_teleop: float | None = None
        self._last_nav: float | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.nav_cmd_vel.subscribe(self._on_nav)))
        self.register_disposable(Disposable(self.tele_cmd_vel.subscribe(self._on_teleop)))
        self._thread = Thread(target=self._watchdog_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.cmd_vel.publish(Twist())
        super().stop()

    def _teleop_holds(self, now: float) -> bool:
        """True while teleop owns cmd_vel."""
        return (
            self._last_teleop is not None
            and now - self._last_teleop < self.config.tele_cooldown_sec
        )

    def _nav_is_stale(self, now: float) -> bool:
        """True when the watchdog should publish a zero twist.

        Nav has been heard from once, has since gone quiet, and teleop is not
        driving — no zeros over an operator's own commands.
        """
        return (
            self._last_nav is not None
            and now - self._last_nav > self.config.nav_stale_s
            and not self._teleop_holds(now)
        )

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            now = time.monotonic()
            # recorded even when dropped: the deadman guards the producer's
            # liveness, not what got forwarded
            self._last_nav = now
            if self._teleop_holds(now):
                return
        self.cmd_vel.publish(msg)

    def _on_teleop(self, msg: Twist) -> None:
        with self._lock:
            self._last_teleop = time.monotonic()
        self.stop_movement.publish(Bool(data=True))
        self.cmd_vel.publish(_scale(msg, self.config))

    def _watchdog_loop(self) -> None:
        """The deadman the network link used to provide for free.

        With the follower co-located there is no dropped link to stop cmd_vel,
        so a dead or wedged follower would leave the last twist standing.
        """
        period = 1.0 / WATCHDOG_HZ
        zeroing = False
        while not self._stop_event.is_set():
            with self._lock:
                stale = self._nav_is_stale(time.monotonic())
            if stale:
                if not zeroing:
                    zeroing = True
                    logger.warning(
                        "nav_cmd_vel went stale, holding cmd_vel at zero",
                        nav_stale_s=self.config.nav_stale_s,
                    )
                # every tick, not just the edge: a single zero can be lost
                self.cmd_vel.publish(Twist())
            elif zeroing:
                zeroing = False
                logger.info("nav_cmd_vel recovered, releasing cmd_vel")
            self._stop_event.wait(period)


def _scale(msg: Twist, config: CmdVelMuxConfig) -> Twist:
    lin, ang = config.tele_scale_linear, config.tele_scale_angular
    return Twist(
        linear=Vector3(msg.linear.x * lin[0], msg.linear.y * lin[1], msg.linear.z * lin[2]),
        angular=Vector3(msg.angular.x * ang[0], msg.angular.y * ang[1], msg.angular.z * ang[2]),
    )
