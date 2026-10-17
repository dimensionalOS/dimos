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

"""Holonomic trajectory controller.

``DanHolonomicTC`` follows a planned ``Path`` with the holonomic tracking law of
``HolonomicPathController``. It owns trajectory control only: the planner owns route
safety and sends an empty ``Path`` when nothing ahead is traversable.
"""

from __future__ import annotations

from threading import Event, RLock, Thread
from typing import Any

from dimos_lcm.std_msgs import Bool
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.experimental.dannav.holonomic_tc.holonomic_path_controller import (
    HolonomicPathController,
)
from dimos.navigation.experimental.dannav.holonomic_tc.run_profiles import (
    RunProfileError,
    get_run_profile,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DanHolonomicTCConfig(ModuleConfig):
    control_frequency: float = 10.0
    run_profile: str = "walk"
    # Overrides the profile's cruise speed only, not its acceleration or yaw caps.
    speed_m_s: float | None = Field(default=None, gt=0.0, allow_inf_nan=False)
    goal_tolerance: float = 0.05
    k_position_per_s: float = Field(default=5.0, ge=0.0, allow_inf_nan=False)
    k_yaw_per_s: float = Field(default=1.0, ge=0.0, allow_inf_nan=False)


class DanHolonomicTC(Module):
    """Follow a planned ``Path`` with the holonomic tracking law.

    Consumes the robot's ``PoseStamped`` odom directly. An empty path stops the
    follow, a non-empty path replaces the active one. Publishes ``nav_cmd_vel``
    until the goal is within tolerance, then ``goal_reached``. ``stop_movement``
    cancels the current path.
    """

    config: DanHolonomicTCConfig

    path: In[Path]
    odom: In[PoseStamped]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._odom: PoseStamped | None = None
        profile = get_run_profile(self.config.run_profile)
        self._controller = HolonomicPathController(
            profile,
            self.config.speed_m_s or profile.requested_planner_speed_m_s,
            self.config.k_position_per_s,
            self.config.k_yaw_per_s,
            self.config.goal_tolerance,
        )
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._stop_event.clear()
        self._thread = Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._odom = msg

    def _on_path(self, path: Path) -> None:
        self._follow(path if path.poses else None)

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._follow(None)

    def _follow(self, path: Path | None) -> None:
        with self._lock:
            self._controller.set_path(path)
            if not self._controller.active:
                self.nav_cmd_vel.publish(Twist())

    def _control_loop(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.wait(period):
            with self._lock:
                if self._odom is None or not self._controller.active:
                    continue
                try:
                    cmd = self._controller.step(self._odom)
                except Exception:
                    logger.exception("Path following failed, stopping.")
                    self._follow(None)
                    continue
                if cmd is not None:
                    self.nav_cmd_vel.publish(cmd)
                if not self._controller.active:
                    self.goal_reached.publish(Bool(True))
                    logger.info("Goal reached")

    @rpc
    def set_run_profile(self, profile: str) -> bool:
        """Switch the movement envelope, applied live."""
        try:
            run_profile = get_run_profile(profile)
        except RunProfileError as exc:
            logger.warning("Rejected run profile.", profile=profile, reason=str(exc))
            return False
        speed = self.config.speed_m_s or run_profile.requested_planner_speed_m_s
        with self._lock:
            self._controller.configure(run_profile, speed)
        return True

    @rpc
    def get_state(self) -> NavigationState:
        with self._lock:
            active = self._controller.active
        return NavigationState.FOLLOWING_PATH if active else NavigationState.IDLE
