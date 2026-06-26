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

"""Continuous holonomic path follower for the dannav loop planner.

Control ticks run on the odom subscription callback (not a background thread).
``_on_path`` only swaps the ``PathDistancer`` under ``_lock``; each throttled
odom update computes and publishes ``nav_cmd_vel``. When the dannav
``GlobalPlanner`` publishes a new path, the next tick tracks it without
stopping the robot.
"""

from __future__ import annotations

import math
from threading import RLock
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.core.global_config import GlobalConfig
from dimos.navigation.dannav.controllers import (
    HolonomicPathController,
    command_envelope_overrides_for_profile,
)
from dimos.navigation.dannav.path_distancer import PathDistancer
from dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles import GO2_RUN_PROFILES, RunProfileError
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _session_speed_and_envelope(
    global_config: GlobalConfig,
) -> tuple[float, CommandEnvelopeOverrides | None, str]:
    """Match ``LocalPlanner`` session envelope: ``go2_run_profile`` sets speed and caps."""
    profile_name = global_config.go2_run_profile
    if profile_name == GO2_RUN_PROFILES.default_profile_name:
        speed = (
            float(global_config.planner_robot_speed)
            if global_config.planner_robot_speed is not None
            else GO2_RUN_PROFILES.get("walk").requested_planner_speed_m_s
        )
        return speed, None, profile_name

    profile = GO2_RUN_PROFILES.get(profile_name)
    return profile.requested_planner_speed_m_s, command_envelope_overrides_for_profile(profile), profile_name


class HolonomicControllerConfig(ModuleConfig):
    goal_tolerance: float = 0.2


class HolonomicController(Module):
    """Track the latest ``Path`` with the holonomic law, never stopping to replan."""

    config: HolonomicControllerConfig

    path: In[Path]
    odom: In[PoseStamped]

    nav_cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._path_distancer: PathDistancer | None = None
        self._current_odom: PoseStamped | None = None
        self._controller: HolonomicPathController | None = None
        self._speed: float = 0.0
        self._control_frequency: float = 10.0
        self._last_tick_monotonic = 0.0

    @rpc
    def start(self) -> None:
        super().start()

        global_config = self.config.g
        self._control_frequency = float(global_config.local_planner_control_rate_hz)
        try:
            speed, envelope_overrides, _ = _session_speed_and_envelope(global_config)
        except RunProfileError as exc:
            logger.warning(
                "Unknown run profile; falling back to walk speed.",
                profile=global_config.go2_run_profile,
                reason=str(exc),
            )
            profile_name = GO2_RUN_PROFILES.default_profile_name
            speed = (
                float(global_config.planner_robot_speed)
                if global_config.planner_robot_speed is not None
                else GO2_RUN_PROFILES.get(profile_name).requested_planner_speed_m_s
            )
            envelope_overrides = None
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"planner speed must be a positive finite float, got {speed!r}")
        if global_config.nerf_speed < 1.0:
            speed *= global_config.nerf_speed
        self._speed = speed
        self._controller = HolonomicPathController(
            global_config,
            speed,
            self._control_frequency,
            k_position_per_s=global_config.local_planner_holonomic_kp,
            k_yaw_per_s=global_config.local_planner_holonomic_ky,
            k_velocity_per_s=global_config.local_planner_holonomic_kv,
            k_yaw_rate_per_s=global_config.local_planner_holonomic_kw,
        )
        self._controller.set_command_envelope(envelope_overrides)

        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))

    @rpc
    def stop(self) -> None:
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        pose = self._as_pose_stamped(msg)
        with self._lock:
            self._current_odom = pose
        self._tick()

    def _on_path(self, path: Path) -> None:
        if not path.poses:
            return
        path_distancer = PathDistancer(path)
        with self._lock:
            self._path_distancer = path_distancer
        self._tick()

    def _tick(self) -> None:
        now = time.monotonic()
        period = 1.0 / self._control_frequency
        if now - self._last_tick_monotonic < period:
            return
        self._last_tick_monotonic = now

        with self._lock:
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        if path_distancer is None or current_odom is None:
            return

        cmd = self._compute_cmd(path_distancer, current_odom)
        self.nav_cmd_vel.publish(cmd)

    def _compute_cmd(
        self, path_distancer: PathDistancer, current_odom: PoseStamped
    ) -> Twist:
        assert self._controller is not None
        current_pos = np.array(
            [float(current_odom.position.x), float(current_odom.position.y)],
            dtype=np.float64,
        )

        if path_distancer.distance_to_goal(current_pos) < self.config.goal_tolerance:
            return Twist()

        closest_index = path_distancer.find_closest_point_index(current_pos)
        lookahead_point = path_distancer.find_lookahead_point(closest_index)
        return self._controller.advance(lookahead_point, current_odom)

    @staticmethod
    def _as_pose_stamped(msg: PoseStamped) -> PoseStamped:
        to_pose = getattr(msg, "to_pose_stamped", None)
        if callable(to_pose):
            return to_pose()
        return msg
