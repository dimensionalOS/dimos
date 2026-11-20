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

"""One motion of a calibrated arm, fed one ``joint_state`` sample at a time.

Each sample gives the next target: a minimum-jerk path whose fastest joint peaks at the speed,
by the sample's measured time, then a hold at the path's end. A hold ends one window after it
starts (``fixed``), once the arm is settled (``settle``), or also once it stops converging
(``converge``: the latest window's spread is no smaller than the one before, and more than an
encoder step over the noise band). Settled means every joint's spread over the latest window
is inside its noise band, or within one encoder step (the smallest change it reads). A joint farther from
its target than its ``stall_rad`` (its motor's torque limit over kp: the motor cannot push
through) or a point of the arm at the table or in a keep-out region stops the motion with an
error. Joint vectors are radians in
the spec's joint order.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
from typing import Literal

import numpy as np

from dimos.control.calibration.gravity_poses import ArmModel

# Peak over mean speed of the minimum-jerk blend, 15/8.
MIN_JERK_PEAK_RATIO = 1.875

HoldMode = Literal["fixed", "settle", "converge"]


@dataclass(frozen=True)
class Sample:
    t: float
    q: np.ndarray
    dq: np.ndarray


def plan_path(arm: ArmModel, start: np.ndarray, end: np.ndarray) -> list[np.ndarray]:
    """Waypoints from ``start`` to ``end``: straight when that keeps the clearance, else via home."""
    if arm.transition_ok(start, end):
        return [start, end]
    if arm.transition_ok(start, arm.home) and arm.transition_ok(arm.home, end):
        return [start, arm.home, end]
    raise ValueError(f"no clear path to [{_deg(end)}] deg, straight or through home")


def _deg(q: np.ndarray) -> str:
    return ", ".join(f"{math.degrees(v):.1f}" for v in q)


@dataclass
class Motion:
    """Targets along ``path`` at ``speed_rad_s``, then a hold at its last point.

    ``tolerance_rad`` is each joint's noise band as a spread.
    A ``floating`` joint follows its measured position, and its hold also ends ("drifted")
    once the fingertip moves ``drift_limit_m`` from where the hold began. Without
    ``publish`` it only watches: the connection holds the arm.
    """

    arm: ArmModel
    path: list[np.ndarray]
    speed_rad_s: float
    mode: HoldMode
    window_s: float
    tolerance_rad: np.ndarray
    stall_rad: np.ndarray
    floating: int | None = None
    drift_limit_m: float = math.inf
    publish: bool = True
    done: threading.Event = field(default_factory=threading.Event)
    ended: str = ""
    error: str | None = None
    t: list[float] = field(default_factory=list)
    q: list[np.ndarray] = field(default_factory=list)
    dq: list[np.ndarray] = field(default_factory=list)
    _t0: float | None = None
    _tip0: np.ndarray | None = None

    def __post_init__(self) -> None:
        self._durations = [
            MIN_JERK_PEAK_RATIO * float(np.abs(b - a).max()) / self.speed_rad_s
            for a, b in zip(self.path, self.path[1:], strict=False)
        ]
        low, high = zip(*(self.arm.spec.joint_ranges[n] for n in self.arm.joint_names), strict=True)
        self._low, self._high = np.asarray(low), np.asarray(high)

    def feed(self, sample: Sample) -> np.ndarray | None:
        """The target for this sample, or None once the motion has ended."""
        if self.done.is_set():
            return None
        if self._t0 is None:
            self._t0 = sample.t
        target = self._path_target(sample.t - self._t0)
        if target is None:
            target = self.path[-1].copy()
            self.t.append(sample.t)
            self.q.append(sample.q)
            self.dq.append(sample.dq)
            self.ended = self._hold_ended() or ""
        if self.floating is not None:
            target[self.floating] = sample.q[self.floating]
        target = np.clip(target, self._low, self._high)
        self.error = self._guard(sample.q, target)
        if self.error or self.ended:
            self.done.set()
            return None
        return target if self.publish else None

    def _path_target(self, elapsed: float) -> np.ndarray | None:
        for a, b, duration in zip(self.path, self.path[1:], self._durations, strict=False):
            if elapsed < duration:
                s = elapsed / duration
                result: np.ndarray = a + s**3 * (10.0 - 15.0 * s + 6.0 * s * s) * (b - a)
                return result
            elapsed -= duration
        return None

    def _hold_ended(self) -> str | None:
        times, q = np.asarray(self.t), np.asarray(self.q)
        elapsed = times[-1] - times[0]
        if self.mode == "fixed":
            return "rested" if elapsed >= self.window_s else None
        if self.floating is not None:
            if self._tip0 is None:
                self._tip0 = self.arm.fingertip(q[0])
            if np.linalg.norm(self.arm.fingertip(q[-1]) - self._tip0) > self.drift_limit_m:
                return "drifted"
        if elapsed <= self.window_s:
            return None
        steps = np.abs(np.diff(q, axis=0))
        quantum = np.where(steps > 0.0, steps, np.inf).min(axis=0)
        quantum = np.where(np.isfinite(quantum), quantum, 0.0)
        tolerance = np.maximum(self.tolerance_rad, quantum)
        spread = np.ptp(q[times > times[-1] - self.window_s], axis=0)
        if (spread <= tolerance).all():
            return "settled"
        if self.mode == "converge" and elapsed > 2.0 * self.window_s:
            earlier = (times > times[-1] - 2.0 * self.window_s) & (
                times <= times[-1] - self.window_s
            )
            unsettled = spread > tolerance + quantum
            if (spread[unsettled] >= np.ptp(q[earlier], axis=0)[unsettled]).any():
                return "not converging"
        return None

    def _guard(self, q: np.ndarray, target: np.ndarray) -> str | None:
        lag = np.abs(q - target)
        stalled = [
            f"{name} {math.degrees(e):.1f} deg"
            for name, e, limit in zip(self.arm.joint_names, lag, self.stall_rad, strict=True)
            if e > limit
        ]
        if stalled:
            return (
                f"stalled or blocked, past what the motor can push: {', '.join(stalled)} off target"
            )
        tip, others = self.arm.points(q)
        points = np.vstack([tip, others])
        if (points[:, 2] <= self.arm.workspace.floor_z_m).any():
            return f"the arm reached the table (z {self.arm.workspace.floor_z_m:+.3f} m)"
        inside = [space.name for space in self.arm.workspace.keep_out if space.violated_by(points)]
        if inside:
            return f"the arm entered the keep-out region {', '.join(inside)}"
        return None
