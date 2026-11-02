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

"""The path layer of the holonomic tracking law.

A path is reduced once to arc length, tangent heading, curvature and a speed
profile. Each tick the reference is the robot's foot on the path, moving along the
tangent at the profiled speed, so the feedback only ever pulls across the path.
"""

from __future__ import annotations

import math

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.experimental.dannav.geometry.path_distancer import project_to_polyline
from dimos.navigation.experimental.dannav.geometry.path_speed_profile import profile_speed
from dimos.navigation.experimental.dannav.holonomic_tc.holonomic_tracking_controller import (
    PlanarState,
    track,
)
from dimos.navigation.experimental.dannav.holonomic_tc.run_profiles import RunProfile

# Long segments are split to this spacing, so a corner's turn stays within one step.
STEP_M = 0.1


class HolonomicPathController:
    """Follow a ``Path`` with :func:`track`, inside a run profile's envelope."""

    def __init__(
        self,
        profile: RunProfile,
        speed_m_s: float,
        k_position_per_s: float,
        k_yaw_per_s: float,
        goal_tolerance: float,
    ) -> None:
        self._k_position = k_position_per_s
        self._k_yaw = k_yaw_per_s
        self._goal_tolerance = goal_tolerance
        self._path: Path | None = None
        self.configure(profile, speed_m_s)

    @property
    def active(self) -> bool:
        return self._path is not None

    def configure(self, profile: RunProfile, speed_m_s: float) -> None:
        self._profile = profile
        self._limits = profile.path_speed_profile_limits_at(speed_m_s)
        self.set_path(self._path)

    def set_path(self, path: Path | None) -> None:
        self._path = None
        if path is None or len(path.poses) < 2:
            return
        xy = np.array([[p.position.x, p.position.y] for p in path.poses])
        xy = xy[np.concatenate([[True], np.hypot(*np.diff(xy, axis=0).T) > 1e-6])]
        if len(xy) < 2:
            return
        pieces = np.ceil(np.hypot(*np.diff(xy, axis=0).T) / STEP_M).astype(int)
        xy = np.concatenate(
            [
                np.linspace(a, b, n, endpoint=False)
                for a, b, n in zip(xy[:-1], xy[1:], pieces, strict=True)
            ]
            + [xy[-1:]]
        )
        d = np.diff(xy, axis=0)
        s = np.concatenate([[0.0], np.cumsum(np.hypot(d[:, 0], d[:, 1]))])
        tangent = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
        heading = np.concatenate([tangent[:1], (tangent[:-1] + tangent[1:]) / 2, tangent[-1:]])
        curvature = np.gradient(heading, s)
        self._speed = profile_speed(s, curvature, self._limits, self._profile.goal_decel_m_s2)
        self._xy, self._s, self._heading = xy, s, heading
        self._path = path

    def step(self, odom: PoseStamped) -> Twist | None:
        """The next command; a zero twist on arrival, which also drops the path."""
        if self._path is None:
            return None
        x, y = float(odom.position.x), float(odom.position.y)
        yaw = float(odom.orientation.euler[2])
        foot = project_to_polyline(x, y, self._xy)
        s = foot.s_along_path_m
        end = self._xy[-1]
        if math.hypot(end[0] - x, end[1] - y) < self._goal_tolerance:
            self._path = None
            return Twist()
        v = float(np.interp(s, self._s, self._speed))
        heading = float(np.interp(s, self._s, self._heading))
        reference = PlanarState(
            x=foot.foot_xy[0],
            y=foot.foot_xy[1],
            yaw=heading,
            vx=v * math.cos(heading),
            vy=v * math.sin(heading),
        )
        return track(
            reference,
            x,
            y,
            yaw,
            self._k_position,
            self._k_yaw,
            self._limits.max_speed_m_s,
            self._limits.max_yaw_rate_rad_s,
        )
