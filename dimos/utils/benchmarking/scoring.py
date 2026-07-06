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

"""Offline scoring for path-following benchmark runs.

Source-agnostic: an :class:`ExecutedTrajectory` from sim and from hardware
score identically. Scoring is purely geometric against the reference
:class:`~dimos.msgs.nav_msgs.Path.Path` (cross-track error, heading error,
arrival) — no time-parameterized reference is needed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.trigonometry import angle_diff


@dataclass
class TrajectoryTick:
    """One control period worth of recorded state."""

    t: float  # seconds since trajectory start
    pose: PoseStamped
    cmd_twist: Twist
    actual_twist: Twist  # plant output (sim) or measured (hw)


@dataclass
class ExecutedTrajectory:
    ticks: list[TrajectoryTick] = field(default_factory=list)
    arrived: bool = False


@dataclass
class ScoreResult:
    # Path-following (spatial CTE — measured against a Path).
    cte_rms: float = 0.0  # m
    cte_max: float = 0.0  # m
    heading_err_rms: float = 0.0  # rad
    heading_err_max: float = 0.0  # rad
    time_to_complete: float = 0.0  # s
    linear_speed_rms: float = 0.0  # m/s, |cmd linear|
    angular_speed_rms: float = 0.0  # rad/s, |cmd wz|
    cmd_rate_integral: float = 0.0  # Sum |dcmd| (smoothness; lower is smoother)
    arrived: bool = False
    n_ticks: int = 0


# Geometry helpers


def _path_xy(path: Path) -> NDArray[np.float64]:
    return np.array([[p.position.x, p.position.y] for p in path.poses], dtype=np.float64)


def nearest_segment(
    pt: NDArray[np.float64], path_xy: NDArray[np.float64]
) -> tuple[int, float, float]:
    """Find nearest path segment to ``pt``.

    Returns ``(seg_idx, perp_dist, t_along_seg)`` where ``seg_idx`` indexes
    the segment from ``path_xy[seg_idx]`` to ``path_xy[seg_idx+1]`` and
    ``t_along_seg`` is the parameter (clamped to [0, 1]) of the foot of
    the perpendicular.
    """
    if len(path_xy) < 2:
        d = float(np.linalg.norm(pt - path_xy[0]))
        return 0, d, 0.0

    starts = path_xy[:-1]
    ends = path_xy[1:]
    segs = ends - starts
    seg_len_sq = np.sum(segs * segs, axis=1)
    seg_len_sq = np.where(seg_len_sq < 1e-12, 1.0, seg_len_sq)

    rel = pt[None, :] - starts
    t = np.clip(np.sum(rel * segs, axis=1) / seg_len_sq, 0.0, 1.0)
    foot = starts + t[:, None] * segs
    dists = np.linalg.norm(pt[None, :] - foot, axis=1)

    idx = int(np.argmin(dists))
    return idx, float(dists[idx]), float(t[idx])


def _segment_yaw(path_xy: NDArray[np.float64], seg_idx: int) -> float:
    if len(path_xy) < 2:
        return 0.0
    seg_idx = max(0, min(seg_idx, len(path_xy) - 2))
    dx = path_xy[seg_idx + 1, 0] - path_xy[seg_idx, 0]
    dy = path_xy[seg_idx + 1, 1] - path_xy[seg_idx, 1]
    return float(math.atan2(dy, dx))


# Scoring


def _twist_linear_speed(t: Twist) -> float:
    return float(math.hypot(t.linear.x, t.linear.y))


def _twist_angular_speed(t: Twist) -> float:
    return float(abs(t.angular.z))


def _cmd_delta(a: Twist, b: Twist) -> float:
    """L2 norm of (a - b) over the (vx, vy, wz) channels."""
    dvx = a.linear.x - b.linear.x
    dvy = a.linear.y - b.linear.y
    dwz = a.angular.z - b.angular.z
    return float(math.sqrt(dvx * dvx + dvy * dvy + dwz * dwz))


def score_run(reference_path: Path, executed: ExecutedTrajectory) -> ScoreResult:
    """Score an executed trajectory against its reference path."""
    if not executed.ticks:
        return ScoreResult(arrived=executed.arrived, n_ticks=0)

    path_xy = _path_xy(reference_path)
    if len(path_xy) == 0:
        return ScoreResult(arrived=executed.arrived, n_ticks=len(executed.ticks))

    cte_sq: list[float] = []
    cte_abs: list[float] = []
    he_abs: list[float] = []
    he_sq: list[float] = []
    lin_sq: list[float] = []
    ang_sq: list[float] = []

    for tick in executed.ticks:
        pt = np.array([tick.pose.position.x, tick.pose.position.y], dtype=np.float64)
        seg_idx, d, _ = nearest_segment(pt, path_xy)
        cte_abs.append(d)
        cte_sq.append(d * d)

        path_yaw = _segment_yaw(path_xy, seg_idx)
        he = abs(angle_diff(tick.pose.orientation.euler[2], path_yaw))
        he_abs.append(he)
        he_sq.append(he * he)

        lin_sq.append(_twist_linear_speed(tick.cmd_twist) ** 2)
        ang_sq.append(_twist_angular_speed(tick.cmd_twist) ** 2)

    cmd_rate_integral = sum(
        _cmd_delta(executed.ticks[i].cmd_twist, executed.ticks[i - 1].cmd_twist)
        for i in range(1, len(executed.ticks))
    )

    return ScoreResult(
        cte_rms=math.sqrt(sum(cte_sq) / len(cte_sq)),
        cte_max=max(cte_abs),
        heading_err_rms=math.sqrt(sum(he_sq) / len(he_sq)),
        heading_err_max=max(he_abs),
        time_to_complete=executed.ticks[-1].t - executed.ticks[0].t,
        linear_speed_rms=math.sqrt(sum(lin_sq) / len(lin_sq)),
        angular_speed_rms=math.sqrt(sum(ang_sq) / len(ang_sq)),
        cmd_rate_integral=cmd_rate_integral,
        arrived=executed.arrived,
        n_ticks=len(executed.ticks),
    )
