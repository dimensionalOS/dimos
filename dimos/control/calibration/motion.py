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

"""How a calibration procedure moves an arm: ramps, paths and the checks on every tick.

Everything here is pure: joint vectors in the spec's ``joint_names`` order,
radians, and an :class:`~dimos.control.calibration.gravity_poses.
ArmModel` for the workspace. A ramp is a minimum-jerk profile whose peak joint
speed is the speed cap, so a 10 deg/s ramp never moves any joint faster than
10 deg/s. A path goes straight when the straight ramp keeps the transition
clearance, else through home, else nowhere. The tick checks stop the arm when
a target would leave the workspace (:class:`SegmentGuard`), would pass a joint
limit (:func:`limit_error`), or the arm stops following its targets
(:func:`tracking_error`): by more than ``MAX_POSE_ERROR_DEG`` while it holds,
and while it ramps (:func:`ramp_lag_limits_deg`) by more than
``RAMP_LAG_LIMIT_DEG`` beyond the lag it started with and the lag a PD
follower has at that speed, so a stall shows sooner.
"""

from __future__ import annotations

from collections.abc import Iterator, Sequence
import math

import numpy as np
from numpy.typing import NDArray

from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.spec import ArmCalibrationSpec, urdf_limits

# The safety limits of the bench-proven step test.
MAX_STEP_DEG = 5.0
# How far a joint may lag its target while it holds (a step of MAX_STEP_DEG
# included), and how much more than it is expected to lag while it ramps.
MAX_POSE_ERROR_DEG = 10.0
RAMP_LAG_LIMIT_DEG = 5.0
MAX_SPEED_DEG_S = 30.0
DEFAULT_SPEED_DEG_S = 10.0
# Peak speed of a minimum-jerk ramp over its average speed.
MIN_JERK_PEAK_RATIO = 1.875
# A measured point may sag this share of the transition clearance below the
# floor its targets keep before the arm is stopped.
MEASURED_SAG_FRACTION = 0.5

Vector = NDArray[np.float64]


def speed_rad_s(speed_deg_s: float) -> float:
    """``speed_deg_s`` in rad/s, refused unless finite and in (0, ``MAX_SPEED_DEG_S``]."""
    if not math.isfinite(speed_deg_s) or not 0.0 < speed_deg_s <= MAX_SPEED_DEG_S:
        raise ValueError(f"speed must be in (0, {MAX_SPEED_DEG_S:g}] deg/s, got {speed_deg_s}")
    return math.radians(speed_deg_s)


def check_step(delta_deg: float, max_step_deg: float = MAX_STEP_DEG) -> None:
    """Refuse a step that is not finite, is zero or exceeds ``max_step_deg``."""
    if not math.isfinite(delta_deg) or delta_deg == 0.0:
        raise ValueError(f"a step must be a finite non-zero angle, got {delta_deg}")
    if abs(delta_deg) > max_step_deg:
        raise ValueError(f"a step of {delta_deg:+g} deg exceeds the {max_step_deg:g} deg limit")


def min_jerk(s: float) -> float:
    """The minimum-jerk blend at ``s`` in [0, 1]: 0 at 0, 1 at 1, zero speed at both ends."""
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)


def ramp_duration_s(start: Vector, end: Vector, speed: float) -> float:
    """How long a minimum-jerk ramp takes for its fastest joint to peak at ``speed`` rad/s."""
    travel = float(np.max(np.abs(np.asarray(end) - np.asarray(start)), initial=0.0))
    return MIN_JERK_PEAK_RATIO * travel / speed


def ramp(start: Vector, end: Vector, speed: float, period_s: float) -> Iterator[Vector]:
    """Targets one ``period_s`` apart from ``start`` (excluded) to ``end`` (included).

    Every joint follows the same minimum-jerk blend, so the arm moves along
    the straight joint-space line, and no joint exceeds ``speed`` rad/s.
    """
    a = np.asarray(start, dtype=np.float64)
    b = np.asarray(end, dtype=np.float64)
    duration = ramp_duration_s(a, b, speed)
    ticks = max(1, math.ceil(duration / period_s))
    for tick in range(1, ticks + 1):
        yield a + min_jerk(tick / ticks) * (b - a)


def joint_limits(spec: ArmCalibrationSpec) -> tuple[Vector, Vector]:
    """The URDF's lower and upper limit of every arm joint, ``joint_names`` order."""
    limits = {joint.name: urdf_limits(joint) for joint in spec.model.load().joints}
    lower = np.asarray([limits[name][0] for name in spec.joint_names], dtype=np.float64)
    upper = np.asarray([limits[name][1] for name in spec.joint_names], dtype=np.float64)
    return lower, upper


def limit_error(
    joint_names: Sequence[str], q: Vector, lower: Vector, upper: Vector, margin_rad: float = 0.0
) -> str | None:
    """Which joints of ``q`` are past (or within ``margin_rad`` of) a limit, or ``None``."""
    values = np.asarray(q, dtype=np.float64)
    out = [
        f"{name} at {math.degrees(float(v)):.1f} deg "
        f"(limits {math.degrees(float(lo)):.1f}..{math.degrees(float(hi)):.1f})"
        for name, v, lo, hi in zip(joint_names, values, lower, upper, strict=True)
        if not lo + margin_rad <= v <= hi - margin_rad
    ]
    return ", ".join(out) or None


def limit_margin_rad(q: Vector, lower: Vector, upper: Vector) -> float:
    """How far the joint of ``q`` nearest a limit is from it, rad (negative past it)."""
    values = np.asarray(q, dtype=np.float64)
    return float(np.min(np.minimum(values - lower, upper - values)))


def plan_path(arm: ArmModel, current: Vector, target: Vector) -> list[Vector] | None:
    """Waypoints to ``target``: straight when that ramp clears the table, else via home, else None."""
    if arm.transition_ok(current, target):
        return [np.asarray(target, dtype=np.float64)]
    home = np.asarray(arm.home, dtype=np.float64)
    if arm.transition_ok(current, home) and arm.transition_ok(home, target):
        return [home, np.asarray(target, dtype=np.float64)]
    return None


def ramp_lag_limits_deg(
    start_lag_deg: Vector, target_speed_deg_s: Vector, follow_s: Vector
) -> NDArray[np.float64]:
    """How far each joint may lag its target while ramping, deg.

    ``RAMP_LAG_LIMIT_DEG`` beyond the lag it had when the ramp began (a
    joint resting on its sag) plus the lag a PD follower has at the target's
    speed (``follow_s`` = kd / kp), never more than ``MAX_POSE_ERROR_DEG``. A
    joint that stops following grows past it within half a second at 10 deg/s.
    """
    expected = np.abs(start_lag_deg) + np.abs(target_speed_deg_s) * follow_s
    limits: NDArray[np.float64] = np.minimum(RAMP_LAG_LIMIT_DEG + expected, MAX_POSE_ERROR_DEG)
    return limits


def tracking_error(
    joint_names: Sequence[str],
    commanded: Vector,
    measured: Vector,
    limit_deg: float | Vector = MAX_POSE_ERROR_DEG,
) -> str | None:
    """Why the arm is not following its targets, or ``None`` while every joint is within its limit.

    ``limit_deg`` is one limit for every joint or one per joint.
    """
    error = np.degrees(np.asarray(measured) - np.asarray(commanded))
    limits = np.broadcast_to(np.asarray(limit_deg, dtype=np.float64), error.shape)
    far = [
        (name, value, limit)
        for name, value, limit in zip(joint_names, error, limits, strict=True)
        if abs(value) > limit
    ]
    if not far:
        return None
    shown = {round(float(limit), 1) for _, _, limit in far}
    if len(shown) == 1:
        lags = ", ".join(f"{name} {value:+.1f} deg" for name, value, _ in far)
        what = f"more than {shown.pop():g} deg from its target ({lags})"
    else:
        lags = ", ".join(
            f"{name} {value:+.1f} deg (limit {limit:.1f})" for name, value, limit in far
        )
        what = f"too far from its target ({lags})"
    return f"the arm is {what}: stalled, blocked or sagging"


class SegmentGuard:
    """The workspace check for every target between two arm postures, and the measured pose.

    The floor of each checked point is that of ``ArmModel.transition_ok``: the
    transition clearance, or lower where an end of the segment already has the
    point lower. Keep-out regions hold everywhere. A measured pose gets
    ``MEASURED_SAG_FRACTION`` of the transition clearance more room, since the
    arm lags and sags a little behind its targets.
    """

    def __init__(self, arm: ArmModel, start: Vector, end: Vector) -> None:
        self._arm = arm
        heights = [self._heights(arm.q(q)) for q in (start, end)]
        self._floor = np.minimum(
            arm.workspace.min_transition_z_m, np.minimum(*heights) - 1e-9
        ).astype(np.float64)
        self._sag = MEASURED_SAG_FRACTION * arm.workspace.transition_clearance_m
        self._names = ("fingertip", *arm.clearance_frames)

    def _heights(self, q: Vector) -> Vector:
        tip, others = self._arm.points(q)
        return np.concatenate([[tip[2]], others[:, 2]]).astype(np.float64)

    def check_target(self, q: Vector) -> str | None:
        """Why the target ``q`` leaves the workspace, or ``None``."""
        return self._check(q, 0.0, "target")

    def check_measured(self, q: Vector) -> str | None:
        """Why the measured pose ``q`` has left the workspace, or ``None``."""
        return self._check(q, self._sag, "measured pose")

    def _check(self, q: Vector, slack_m: float, what: str) -> str | None:
        tip, others = self._arm.points(self._arm.q(q))
        heights = np.concatenate([[tip[2]], others[:, 2]])
        low = [
            f"{name} at z={height:+.3f} m (floor {limit:+.3f} m)"
            for name, height, limit in zip(self._names, heights, self._floor - slack_m, strict=True)
            if height <= limit
        ]
        if low:
            return f"the {what} is too close to the table: {', '.join(low)}"
        points = np.vstack([tip, others])
        inside = [
            space.name
            for space in self._arm.workspace.keep_out
            if bool((points @ np.asarray(space.normal) > space.offset_m + slack_m).any())
        ]
        if inside:
            return f"the {what} enters the keep-out region {', '.join(inside)}"
        return None
