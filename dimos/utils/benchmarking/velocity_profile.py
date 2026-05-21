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

"""Curvature velocity profiling as a controller-agnostic benchmark wrapper.

Option 2 ("B"). Unlike the command smoothers (LPF / rate limiter, which
only shape the command and cannot beat the plant floor), this attacks
the floor itself: the FOPDT spatial lag is ~(tau+L)*v, so slowing down
where path curvature is high *lowers the lag exactly where tracking is
worst*. It is the architecturally correct tracking lever.

Wraps the pre-existing
:class:`dimos.control.tasks.velocity_profiler.VelocityProfiler`
(curvature -> centripetal-accel speed limit -> fwd/bwd accel passes) and
applies it as a per-tick cap on the controller's commanded ``(vx, wz)``:
at the robot's current path index it caps ``|vx|`` to the profile speed
and scales ``wz`` by the same factor so the commanded turn radius
(vx/wz) — i.e. the path geometry — is preserved; the robot just
traverses the corner slower.

``max_angular_speed`` defaults to the Go2 Rung-1 saturation envelope
(``WZ_MAX = 1.5 rad/s``); ``max_linear_speed`` is the cohort target
speed. No control-law change — a pure output wrapper, same seam as the
rate limiter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.msgs.nav_msgs.Path import Path

# Go2 Rung-1 saturation envelope (mirrors runner.VX_MAX / WZ_MAX).
GO2_VX_MAX = 1.0  # m/s
GO2_WZ_MAX = 1.5  # rad/s


@dataclass
class VelocityProfileConfig:
    """Curvature-profile knobs. Defaults come from the Go2 saturation
    envelope; ``max_linear_speed`` should be set to the cohort speed.

    ``lookahead_pts`` makes the cap use the *minimum* profile speed over
    the next N path points so the robot starts slowing *before* the
    corner (a pure at-index cap brakes too late).
    """

    max_linear_speed: float = 0.55
    max_angular_speed: float = GO2_WZ_MAX
    max_centripetal_accel: float = 1.0
    max_linear_accel: float = 1.0
    max_linear_decel: float = 2.0
    min_speed: float = 0.05
    lookahead_pts: int = 8


class PathSpeedCap:
    """Per-tick curvature speed cap for one path.

    Build once per run (``for_path``); call :meth:`cap` each tick with
    the robot xy and the controller's commanded ``(vx, wz)``.
    """

    def __init__(self, cfg: VelocityProfileConfig | None = None) -> None:
        self.cfg = cfg or VelocityProfileConfig()
        self._profiler = VelocityProfiler(
            max_linear_speed=self.cfg.max_linear_speed,
            max_angular_speed=self.cfg.max_angular_speed,
            max_linear_accel=self.cfg.max_linear_accel,
            max_linear_decel=self.cfg.max_linear_decel,
            max_centripetal_accel=self.cfg.max_centripetal_accel,
            min_speed=self.cfg.min_speed,
        )
        self._pts: np.ndarray | None = None
        self._profile: np.ndarray | None = None

    def for_path(self, path: Path) -> None:
        """(Re)compute the speed profile for ``path``. Call on path start."""
        self._profile = np.asarray(self._profiler.compute_profile(path), dtype=float)
        self._pts = np.array([[p.position.x, p.position.y] for p in path.poses], dtype=float)

    def speed_limit_at(self, x: float, y: float) -> float:
        """Profile speed at the nearest path index, min over the lookahead
        window (so braking starts before the corner)."""
        if self._pts is None or self._profile is None or len(self._profile) == 0:
            return self.cfg.max_linear_speed
        i = int(np.argmin(np.sum((self._pts - np.array([x, y])) ** 2, axis=1)))
        j = min(len(self._profile), i + max(1, self.cfg.lookahead_pts))
        return float(np.min(self._profile[i:j]))

    def cap(
        self, x: float, y: float, vx: float, vy: float, wz: float
    ) -> tuple[float, float, float]:
        """Cap |vx| to the profile speed; scale vy/wz by the same factor
        so the commanded path geometry (turn radius) is preserved."""
        vlim = self.speed_limit_at(x, y)
        s = abs(vx)
        if s <= vlim or s < 1e-9:
            return vx, vy, wz
        k = vlim / s
        return vx * k, vy * k, wz * k


__all__ = ["GO2_VX_MAX", "GO2_WZ_MAX", "PathSpeedCap", "VelocityProfileConfig"]
