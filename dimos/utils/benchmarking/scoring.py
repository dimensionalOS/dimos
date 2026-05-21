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
score identically.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.characterization.trajectories import TrajRefState
from dimos.utils.trigonometry import angle_diff

RefFn = Callable[[float], TrajRefState]


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
    # Trajectory tracking (time-indexed — measured against r(t)). Decomposed
    # in the reference yaw frame at each tick. Positive along-track lag
    # means the robot is BEHIND the reference along the reference's
    # heading. ``traj_completed_on_time_pct`` is the fraction of the
    # expected duration spanned by the run (1.0 if duration unknown).
    along_track_lag_rms: float = 0.0  # m
    along_track_lag_max: float = 0.0  # m
    cross_track_traj_rms: float = 0.0  # m
    cross_track_traj_max: float = 0.0  # m
    heading_err_traj_rms: float = 0.0  # rad
    heading_err_traj_max: float = 0.0  # rad
    traj_completed_on_time_pct: float = 0.0  # 0..1


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _path_xy(path: Path) -> NDArray[np.float64]:
    return np.array([[p.position.x, p.position.y] for p in path.poses], dtype=np.float64)


def _nearest_segment(
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


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------


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
        seg_idx, d, _ = _nearest_segment(pt, path_xy)
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


# ---------------------------------------------------------------------------
# Trajectory-tracking scoring (time-indexed, decomposed in ref-yaw frame)
# ---------------------------------------------------------------------------


def score_run_with_trajectory(
    executed: ExecutedTrajectory,
    ref_fn: RefFn,
    *,
    duration_s: float | None = None,
) -> ScoreResult:
    """Score against a time-indexed reference ``r(t)``.

    The error vector ``(pose.x - ref.x, pose.y - ref.y)`` is rotated into
    the **reference yaw frame** at each tick:

      - ``along_track`` (+ if robot is ahead of reference along its heading)
      - ``cross_track``  (+ if robot is to the LEFT of the reference direction)

    Heading error uses :func:`angle_diff` so wrap-around is handled.

    ``traj_completed_on_time_pct`` reports the fraction of ``duration_s``
    spanned by the run. When ``duration_s`` is ``None``, defaults to 1.0
    if any ticks were recorded (analysis is responsible for supplying
    the expected duration).

    Path-following fields on the returned :class:`ScoreResult` are zero —
    call :func:`score_run` separately if both are needed.
    """
    if not executed.ticks:
        return ScoreResult(arrived=executed.arrived, n_ticks=0)

    along_lag_sq: list[float] = []
    along_lag_abs: list[float] = []
    cross_sq: list[float] = []
    cross_abs: list[float] = []
    he_sq: list[float] = []
    he_abs: list[float] = []
    lin_sq: list[float] = []
    ang_sq: list[float] = []

    for tick in executed.ticks:
        ref = ref_fn(tick.t)
        ex = tick.pose.position.x - ref.x
        ey = tick.pose.position.y - ref.y

        cos_y = math.cos(ref.yaw)
        sin_y = math.sin(ref.yaw)
        # Project world error into ref-yaw frame. Along-track is the
        # ref-x component (positive = robot ahead). Lag (the diagnostic
        # quantity) is the negative of along-track signed offset:
        along_signed = cos_y * ex + sin_y * ey  # + = ahead
        lag = -along_signed  # + = behind, matches "robot is X behind ref"
        cross = -sin_y * ex + cos_y * ey  # + = left of ref direction

        along_lag_sq.append(lag * lag)
        along_lag_abs.append(abs(lag))
        cross_sq.append(cross * cross)
        cross_abs.append(abs(cross))

        he = angle_diff(tick.pose.orientation.euler[2], ref.yaw)
        he_sq.append(he * he)
        he_abs.append(abs(he))

        lin_sq.append(_twist_linear_speed(tick.cmd_twist) ** 2)
        ang_sq.append(_twist_angular_speed(tick.cmd_twist) ** 2)

    n = len(executed.ticks)
    cmd_rate_integral = sum(
        _cmd_delta(executed.ticks[i].cmd_twist, executed.ticks[i - 1].cmd_twist)
        for i in range(1, n)
    )

    span = executed.ticks[-1].t - executed.ticks[0].t
    if duration_s is not None and duration_s > 0.0:
        completed_pct = min(1.0, span / duration_s)
    else:
        completed_pct = 1.0

    return ScoreResult(
        time_to_complete=span,
        linear_speed_rms=math.sqrt(sum(lin_sq) / n),
        angular_speed_rms=math.sqrt(sum(ang_sq) / n),
        cmd_rate_integral=cmd_rate_integral,
        arrived=executed.arrived,
        n_ticks=n,
        along_track_lag_rms=math.sqrt(sum(along_lag_sq) / n),
        along_track_lag_max=max(along_lag_abs),
        cross_track_traj_rms=math.sqrt(sum(cross_sq) / n),
        cross_track_traj_max=max(cross_abs),
        heading_err_traj_rms=math.sqrt(sum(he_sq) / n),
        heading_err_traj_max=max(he_abs),
        traj_completed_on_time_pct=completed_pct,
    )
