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

"""Progress-indexed full-pose reference for a poses-only Path.

The path carries no timing, so the reference is parameterized by arc length
``s`` instead of a clock: each tick the robot's position is projected onto the
path to get its progress ``s_robot``, and the reference pose is the path
interpolated at ``s_robot + lookahead``. The reference therefore advances with
the robot — if the robot lags the reference waits, and a replan just
re-projects onto the new path (no clock to reset, no re-ramp from rest).

The per-waypoint yaw is the planner's COMMANDED orientation, interpolated along
arc length exactly like position. It is deliberately NOT the path tangent — a
full-pose planner commands orientation independently of travel direction (e.g.
strafe through a tunnel while facing along it).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from dimos.msgs.nav_msgs.Path import Path

_EPS = 1e-9


@dataclass(frozen=True)
class PoseSample:
    """Reference pose + spatial rates at one arc-length station."""

    s: float
    x: float
    y: float
    yaw: float  # commanded yaw (wrapped to [-pi, pi])
    tangent_x: float  # unit d(x)/ds — world frame
    tangent_y: float  # unit d(y)/ds — world frame
    dyaw_ds: float  # d(commanded yaw)/ds (rad/m)


class ProgressPathReference:
    """Arc-length parameterization + stateful projection of a full-pose Path.

    Projection is windowed and (nearly) monotonic: each ``advance()`` searches
    only segments within ``[s_prev - back_m, s_prev + window_m]``, so closed
    paths (goal == start) never match the far end on tick 1, while a bounded
    backslide lets the reference wait for a robot pushed backwards.
    """

    def __init__(self, path: Path, *, window_m: float = 1.0, back_m: float = 0.5) -> None:
        if path is None or len(path.poses) < 2:
            raise ValueError("ProgressPathReference needs a path with >= 2 poses")
        xs: list[float] = []
        ys: list[float] = []
        yaws: list[float] = []
        for p in path.poses:
            x, y = float(p.position.x), float(p.position.y)
            yaw = float(p.orientation.euler[2])
            if xs and math.hypot(x - xs[-1], y - ys[-1]) < _EPS:
                # Coincident waypoint: keep the position once but adopt the
                # later pose's yaw (a stop-and-rotate hint survives as a yaw
                # target at that station).
                yaws[-1] = yaw
                continue
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
        if len(xs) < 2:
            raise ValueError("ProgressPathReference: path has no positive arc length")

        self._x = np.asarray(xs)
        self._y = np.asarray(ys)
        self._yaw_unwrapped = np.unwrap(np.asarray(yaws))
        dx = np.diff(self._x)
        dy = np.diff(self._y)
        ds = np.hypot(dx, dy)  # all > _EPS by construction
        self._s = np.concatenate(([0.0], np.cumsum(ds)))
        self._tx = dx / ds
        self._ty = dy / ds
        self._dyaw_ds = np.diff(self._yaw_unwrapped) / ds
        # Position-curvature per interior waypoint: turn of the tangent between
        # adjacent segments over the local arc length. Used by the speed
        # regulator (centripetal budget); independent of the commanded yaw.
        seg_yaw = np.arctan2(dy, dx)
        if len(seg_yaw) > 1:
            turn = np.abs(np.diff(np.unwrap(seg_yaw)))
            local_ds = 0.5 * (ds[:-1] + ds[1:])
            kappa_interior = turn / np.maximum(local_ds, _EPS)
        else:
            kappa_interior = np.zeros(0)
        # Per-segment curvature = max of its endpoint waypoint curvatures.
        self._kappa = np.zeros(len(ds))
        if len(kappa_interior):
            self._kappa[:-1] = np.maximum(self._kappa[:-1], kappa_interior)
            self._kappa[1:] = np.maximum(self._kappa[1:], kappa_interior)

        self._window_m = float(window_m)
        self._back_m = float(back_m)
        self._s_progress = 0.0

    @property
    def length(self) -> float:
        return float(self._s[-1])

    @property
    def progress(self) -> float:
        return self._s_progress

    def advance(self, x: float, y: float) -> float:
        """Project (x, y) onto the path within the progress window; update and
        return the progress arc length ``s_robot``."""
        s_lo = max(0.0, self._s_progress - self._back_m)
        s_hi = min(self.length, self._s_progress + self._window_m)
        i_lo = int(np.searchsorted(self._s, s_lo, side="right") - 1)
        i_hi = int(np.searchsorted(self._s, s_hi, side="left"))
        i_hi = max(i_hi, i_lo + 1)  # at least one segment

        starts_x = self._x[i_lo:i_hi]
        starts_y = self._y[i_lo:i_hi]
        seg_dx = self._x[i_lo + 1 : i_hi + 1] - starts_x
        seg_dy = self._y[i_lo + 1 : i_hi + 1] - starts_y
        seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
        t = ((x - starts_x) * seg_dx + (y - starts_y) * seg_dy) / np.maximum(seg_len_sq, _EPS)
        t = np.clip(t, 0.0, 1.0)
        foot_x = starts_x + t * seg_dx
        foot_y = starts_y + t * seg_dy
        d_sq = (x - foot_x) ** 2 + (y - foot_y) ** 2
        best = int(np.argmin(d_sq))
        s_found = float(self._s[i_lo + best] + t[best] * math.sqrt(max(seg_len_sq[best], 0.0)))
        self._s_progress = min(max(s_found, s_lo), s_hi)
        return self._s_progress

    def sample(self, s: float) -> PoseSample:
        """Interpolate the full pose (and its spatial rates) at arc length ``s``
        (clamped to the path)."""
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self._s, s, side="right") - 1)
        i = min(max(i, 0), len(self._s) - 2)
        ds = float(self._s[i + 1] - self._s[i])
        a = (s - float(self._s[i])) / ds if ds > _EPS else 0.0
        yaw_u = self._yaw_unwrapped
        yaw = float(yaw_u[i] + a * (yaw_u[i + 1] - yaw_u[i]))
        return PoseSample(
            s=s,
            x=float(self._x[i] + a * (self._x[i + 1] - self._x[i])),
            y=float(self._y[i] + a * (self._y[i + 1] - self._y[i])),
            yaw=(yaw + math.pi) % (2.0 * math.pi) - math.pi,
            tangent_x=float(self._tx[i]),
            tangent_y=float(self._ty[i]),
            dyaw_ds=float(self._dyaw_ds[i]),
        )

    def max_rates_ahead(self, s: float, horizon_m: float) -> tuple[float, float]:
        """(max |d yaw/ds|, max position curvature) over ``[s, s + horizon_m]``.

        The speed regulator uses these to slow down BEFORE a fast-yaw stretch
        or a tight curve rather than at it.
        """
        s = min(max(s, 0.0), self.length)
        i_lo = int(np.searchsorted(self._s, s, side="right") - 1)
        i_lo = min(max(i_lo, 0), len(self._s) - 2)
        i_hi = int(np.searchsorted(self._s, min(s + horizon_m, self.length), side="left"))
        i_hi = max(i_hi, i_lo + 1)
        return (
            float(np.max(np.abs(self._dyaw_ds[i_lo:i_hi]))),
            float(np.max(self._kappa[i_lo:i_hi])),
        )

    def end_pose(self) -> tuple[float, float, float]:
        return (
            float(self._x[-1]),
            float(self._y[-1]),
            float((self._yaw_unwrapped[-1] + math.pi) % (2.0 * math.pi) - math.pi),
        )
