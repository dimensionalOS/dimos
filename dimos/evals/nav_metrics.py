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

"""Navigation quality over a recorded run: arrival, time, facing, bumps, path shape.

Pure functions over pose and command series, so a grader is one call and every
number can be recomputed from the recording.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
from itertools import pairwise
import json
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

from dimos.evals.constants import (
    NAV_BUMP_MIN_S,
    NAV_BUMP_RATIO,
    NAV_BUMPS_FOR_ZERO_CREDIT,
    NAV_CMD_HOLD_S,
    NAV_FACING_TOL_DEG,
    NAV_JITTER_M,
    NAV_MIN_CMD_MPS,
    NAV_SUCCESS_RADIUS_M,
    NAV_TURN_HYSTERESIS_DEG,
    NAV_WEIGHTS,
)

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

Box2D = tuple[float, float, float, float]  # minx, miny, maxx, maxy
PoseSample = tuple[float, float, float, float]  # ts, x, y, yaw
CmdSample = tuple[float, float, float, float]  # ts, vx, vy, wz


@dataclass(frozen=True, kw_only=True)
class NavParams:
    success_radius_m: float = NAV_SUCCESS_RADIUS_M  # around the end point, or the object's box edge
    facing_tol_deg: float = NAV_FACING_TOL_DEG  # toward the object's centre
    min_cmd_mps: float = NAV_MIN_CMD_MPS
    bump_ratio: float = NAV_BUMP_RATIO
    bump_min_s: float = NAV_BUMP_MIN_S
    cmd_hold_s: float = NAV_CMD_HOLD_S
    jitter_m: float = NAV_JITTER_M
    turn_hysteresis_deg: float = NAV_TURN_HYSTERESIS_DEG


@dataclass(frozen=True, kw_only=True)
class NavMetrics:
    reached: bool
    final_distance_m: float
    min_distance_m: float
    time_to_object_s: float  # first entry into the radius; the run's duration if never
    duration_s: float
    facing: bool
    facing_error_deg: float
    bumps: int
    path_length_m: float
    straight_line_m: float  # start to the end point
    straightness: float  # straight_line / path_length, capped at 1
    total_turning_rad: float
    turn_reversals: int
    turn_reversals_per_m: float
    finished_declared: bool
    declared_at_s: float | None  # seconds after the first pose

    def score(self) -> float:
        """0 unless reached; then arrival plus facing, straightness and bump credit."""
        if not self.reached:
            return 0.0
        w = NAV_WEIGHTS
        return (
            w["reached"]
            + w["facing"] * float(self.facing)
            + w["straightness"] * self.straightness
            + w["bumps"] * max(0.0, 1.0 - self.bumps / NAV_BUMPS_FOR_ZERO_CREDIT)
        )


def box_of(center: Sequence[float], size: Sequence[float]) -> Box2D:
    return (
        center[0] - size[0] / 2,
        center[1] - size[1] / 2,
        center[0] + size[0] / 2,
        center[1] + size[1] / 2,
    )


def distance_to_box(x: float, y: float, box: Box2D) -> float:
    """Euclidean distance from a point to an axis-aligned box; 0 inside."""
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return math.hypot(dx, dy)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def path_length(poses: Sequence[PoseSample], jitter_m: float = NAV_JITTER_M) -> float:
    """Sum of xy steps above the odometry jitter floor."""
    return sum(
        d
        for (_, x0, y0, _), (_, x1, y1, _) in pairwise(poses)
        if (d := math.hypot(x1 - x0, y1 - y0)) >= jitter_m
    )


def turn_reversals(poses: Sequence[PoseSample], hysteresis_deg: float) -> tuple[int, float]:
    """(direction reversals, total |yaw change|). A reversal counts once the yaw has
    swung ``hysteresis_deg`` against the current turning direction."""
    hyst = math.radians(hysteresis_deg)
    direction, swing, reversals, total = 0, 0.0, 0, 0.0
    for (_, _, _, y0), (_, _, _, y1) in pairwise(poses):
        d = _wrap(y1 - y0)
        total += abs(d)
        if direction == 0 or d * direction < 0:
            swing += d
            if abs(swing) >= hyst:
                reversals += direction != 0
                direction = 1 if swing > 0 else -1
                swing = 0.0
        else:
            swing = 0.0
    return reversals, total


def bumps(
    poses: Sequence[PoseSample], cmds: Sequence[CmdSample], params: NavParams = NavParams()
) -> int:
    """Episodes where a linear command was held but the robot barely moved.

    Habitat and DimSim publish the achieved pose after sliding along geometry, so a
    commanded speed with no displacement is contact.
    """
    count, blocked_since, counted, ci = 0, None, False, 0
    for (t0, x0, y0, _), (t1, x1, y1, _) in pairwise(poses):
        while ci + 1 < len(cmds) and cmds[ci + 1][0] <= t1:
            ci += 1
        commanded = 0.0
        if cmds and cmds[ci][0] <= t1 <= cmds[ci][0] + params.cmd_hold_s:
            commanded = math.hypot(cmds[ci][1], cmds[ci][2])
        achieved = math.hypot(x1 - x0, y1 - y0) / (t1 - t0) if t1 > t0 else 0.0
        if commanded >= params.min_cmd_mps and achieved < params.bump_ratio * commanded:
            blocked_since = t0 if blocked_since is None else blocked_since
            if not counted and t1 - blocked_since >= params.bump_min_s:
                count, counted = count + 1, True
        else:
            blocked_since, counted = None, False
    return count


def score_navigation(
    poses: Sequence[PoseSample],
    cmds: Sequence[CmdSample],
    end_xy: tuple[float, float],
    target: Box2D,
    params: NavParams = NavParams(),
    *,
    declared_at: float | None = None,
) -> NavMetrics:
    if not poses:
        raise LookupError("no poses recorded")
    t_start = poses[0][0]
    if distance_to_box(end_xy[0], end_xy[1], target) == 0.0:  # the point is on the object
        dist = [distance_to_box(x, y, target) for _, x, y, _ in poses]
    else:
        dist = [math.hypot(x - end_xy[0], y - end_xy[1]) for _, x, y, _ in poses]
    entered = next(
        (p[0] for p, d in zip(poses, dist, strict=True) if d <= params.success_radius_m), None
    )
    duration = poses[-1][0] - t_start
    _, fx, fy, fyaw = poses[-1]
    cx, cy = (target[0] + target[2]) / 2, (target[1] + target[3]) / 2
    facing_err = math.degrees(abs(_wrap(math.atan2(cy - fy, cx - fx) - fyaw)))
    length = path_length(poses, params.jitter_m)
    reversals, turning = turn_reversals(poses, params.turn_hysteresis_deg)
    return NavMetrics(
        reached=dist[-1] <= params.success_radius_m,
        final_distance_m=dist[-1],
        min_distance_m=min(dist),
        time_to_object_s=(entered - t_start) if entered is not None else duration,
        duration_s=duration,
        facing=facing_err <= params.facing_tol_deg,
        facing_error_deg=facing_err,
        bumps=bumps(poses, cmds, params),
        path_length_m=length,
        straight_line_m=dist[0],
        straightness=min(1.0, dist[0] / length) if length >= params.jitter_m else 0.0,
        total_turning_rad=turning,
        turn_reversals=reversals,
        turn_reversals_per_m=reversals / length if length >= 0.5 else 0.0,
        finished_declared=declared_at is not None,
        declared_at_s=(declared_at - t_start) if declared_at is not None else None,
    )


# -- reading a recording ----------------------------------------------------------


def read_poses(store: Store) -> list[PoseSample]:
    """``odom`` (PoseStamped) or ``odometry`` (nav_msgs), whichever was recorded."""
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    name = next((n for n in ("odom", "odometry") if n in store.streams), None)
    if name is None:
        raise LookupError("no odometry recorded")
    out: list[PoseSample] = []
    for obs in getattr(store.streams, name):
        m = obs.data
        pose = (
            m
            if isinstance(m, PoseStamped)
            else PoseStamped(position=m.position, orientation=m.orientation)
        )
        out.append((obs.ts, pose.x, pose.y, pose.yaw))
    return out


def read_cmds(store: Store) -> list[CmdSample]:
    if "cmd_vel" not in store.streams:
        return []
    return [
        (o.ts, o.data.linear.x, o.data.linear.y, o.data.angular.z) for o in store.streams.cmd_vel
    ]


def read_declared(store: Store) -> float | None:
    """When ``finished`` was first published True, if ever."""
    if "finished" not in store.streams:
        return None
    return next((o.ts for o in store.streams.finished if o.data.data), None)


def write_metrics(metrics: NavMetrics, path: Path, **extra: Any) -> None:
    path.write_text(json.dumps({**asdict(metrics), **extra}, indent=2))
