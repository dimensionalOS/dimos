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

"""Drift of a floating arm: how far the fingertip wanders once the hand lets go.

Shared by the live driver (``demo_float_test``) and the offline report
(``float_report``). A window is the ``float_events`` marker plus the joint
samples recorded during it; the drift is the fingertip motion between the
start and the end of the window, computed through the arm model so a joint
error far from the base counts for what it does at the fingers.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json
import math
from typing import Any

import numpy as np

from dimos.robot.manipulators.calibration.gravity_poses import ArmModel, check_side
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_DOF_PER_ARM

# Milestone: a floated arm, let go, stays within this at the fingertip over 10 s.
DRIFT_LIMIT_MM = 2.0
# The pose at either end of a window is the median over this much of it, so one
# noisy encoder sample does not decide the drift.
EDGE_WINDOW_S = 0.25


@dataclass(frozen=True)
class FloatEvent:
    """The ``float_events`` marker ``mark_float`` publishes when a window starts."""

    side: str
    index: int
    t0: float
    duration_s: float
    kd: float = float("nan")
    floating: bool = True
    q0_deg: dict[str, float] | None = None
    data: dict[str, Any] | None = None

    @classmethod
    def from_json(cls, text: str) -> FloatEvent:
        raw = json.loads(text)
        return cls(
            side=str(raw["side"]),
            index=int(raw["index"]),
            t0=float(raw["t0"]),
            duration_s=float(raw["duration_s"]),
            kd=float(raw.get("kd", float("nan"))),
            floating=bool(raw.get("floating", True)),
            q0_deg=raw.get("q0_deg"),
            data=raw.get("data"),
        )

    def to_json(self) -> str:
        return json.dumps(asdict(self))


@dataclass(frozen=True)
class DriftMetrics:
    """What one float window says about the arm."""

    side: str
    index: int
    duration_s: float  # the window actually covered by samples
    samples: int
    fingertip_drift_mm: float  # start pose to end pose, through the model
    fingertip_peak_mm: float  # farthest the fingertip got from the start pose
    max_joint_drift_deg: float
    max_joint: str
    joint_drift_deg: dict[str, float]

    @property
    def passed(self) -> bool:
        return self.fingertip_drift_mm < DRIFT_LIMIT_MM

    def as_row(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "index": self.index,
            "duration_s": round(self.duration_s, 3),
            "samples": self.samples,
            "fingertip_drift_mm": round(self.fingertip_drift_mm, 3),
            "fingertip_peak_mm": round(self.fingertip_peak_mm, 3),
            "max_joint_drift_deg": round(self.max_joint_drift_deg, 4),
            "max_joint": self.max_joint,
            "passed": self.passed,
        }


class FingertipModel:
    """Fingertip position of one arm from its six joint angles, base frame, metres.

    The fingertip is ``{side}_grasp_frame`` moved 46 mm along the gripper's -z
    (``gravity_poses.FINGERTIP_OFFSET_M``), the same point the pose filter uses.
    """

    def __init__(self, side: str) -> None:
        self.side = check_side(side)
        self._arm = ArmModel(side=side)
        self.joint_names = list(self._arm.joint_names)

    def fingertip_m(self, joints_rad: Sequence[float]) -> np.ndarray:
        if len(joints_rad) != DUAL_OPENYAM_DOF_PER_ARM:
            raise ValueError(f"expected {DUAL_OPENYAM_DOF_PER_ARM} angles, got {len(joints_rad)}")
        tip, _ = self._arm.points(self._arm.full_q(np.asarray(joints_rad, dtype=np.float64)))
        return np.asarray(tip, dtype=np.float64)


def edge_pose(t: np.ndarray, q: np.ndarray, *, start: bool) -> np.ndarray:
    """Median joint vector over the first (or last) ``EDGE_WINDOW_S`` of the samples."""
    if start:
        mask = t <= t[0] + EDGE_WINDOW_S
    else:
        mask = t >= t[-1] - EDGE_WINDOW_S
    return np.median(q[mask], axis=0)


def analyze_window(
    model: FingertipModel,
    event: FloatEvent,
    t: np.ndarray,
    q_rad: np.ndarray,
) -> DriftMetrics:
    """Drift over one window from its samples (``t`` seconds, ``q_rad`` rows in joint order 1..6).

    ``t`` must be sorted. Raises ``ValueError`` on fewer than two samples.
    """
    t = np.asarray(t, dtype=np.float64)
    q_rad = np.asarray(q_rad, dtype=np.float64)
    if t.ndim != 1 or q_rad.shape != (t.size, DUAL_OPENYAM_DOF_PER_ARM):
        raise ValueError(f"need (n,) times and (n, 6) angles, got {t.shape} and {q_rad.shape}")
    if t.size < 2:
        raise ValueError(f"window {event.index} on {event.side} has {t.size} sample(s)")
    q_start = edge_pose(t, q_rad, start=True)
    q_end = edge_pose(t, q_rad, start=False)
    tip0 = model.fingertip_m(q_start)
    drift_m = float(np.linalg.norm(model.fingertip_m(q_end) - tip0))
    peak_m = max(float(np.linalg.norm(model.fingertip_m(row) - tip0)) for row in q_rad)
    joint_drift = {
        name: math.degrees(float(end - start))
        for name, start, end in zip(model.joint_names, q_start, q_end, strict=True)
    }
    max_joint = max(joint_drift, key=lambda name: abs(joint_drift[name]))
    return DriftMetrics(
        side=event.side,
        index=event.index,
        duration_s=float(t[-1] - t[0]),
        samples=int(t.size),
        fingertip_drift_mm=drift_m * 1000.0,
        fingertip_peak_mm=peak_m * 1000.0,
        max_joint_drift_deg=abs(joint_drift[max_joint]),
        max_joint=max_joint,
        joint_drift_deg=joint_drift,
    )


def fingertip_drift_mm_series(
    model: FingertipModel, t: np.ndarray, q_rad: np.ndarray
) -> np.ndarray:
    """Fingertip distance from the window's start pose at every sample, millimetres."""
    tip0 = model.fingertip_m(edge_pose(np.asarray(t), np.asarray(q_rad), start=True))
    return np.asarray(
        [np.linalg.norm(model.fingertip_m(row) - tip0) * 1000.0 for row in np.asarray(q_rad)]
    )


def segment_windows(
    events: list[FloatEvent], t_end: float
) -> list[tuple[FloatEvent, float, float]]:
    """``(event, start, stop)`` per marker: its duration, cut short by the next marker."""
    ordered = sorted(events, key=lambda event: event.t0)
    windows: list[tuple[FloatEvent, float, float]] = []
    for position, event in enumerate(ordered):
        stop = event.t0 + event.duration_s
        if position + 1 < len(ordered):
            stop = min(stop, ordered[position + 1].t0)
        windows.append((event, event.t0, min(stop, t_end)))
    return windows


def format_metrics(metrics: DriftMetrics) -> str:
    verdict = "ok" if metrics.passed else "FAIL"
    return (
        f"{metrics.side} pose {metrics.index}: fingertip drift {metrics.fingertip_drift_mm:.2f} mm "
        f"over {metrics.duration_s:.1f} s / {metrics.samples} samples "
        f"(peak {metrics.fingertip_peak_mm:.2f} mm), "
        f"max joint drift {metrics.max_joint_drift_deg:.3f} deg on {metrics.max_joint} "
        f"[{verdict}, limit {DRIFT_LIMIT_MM:g} mm]"
    )


def format_table(rows: list[DriftMetrics]) -> str:
    header = (
        "| side | pose | window (s) | samples | fingertip drift (mm) | peak (mm) "
        "| max joint drift (deg) | joint | pass |"
    )
    lines = [header, "|" + "---|" * 9]
    for m in sorted(rows, key=lambda m: (m.side, m.index)):
        lines.append(
            f"| {m.side} | {m.index} | {m.duration_s:.1f} | {m.samples} "
            f"| {m.fingertip_drift_mm:.2f} | {m.fingertip_peak_mm:.2f} "
            f"| {m.max_joint_drift_deg:.3f} | {m.max_joint} | {'ok' if m.passed else 'FAIL'} |"
        )
    lines.append("")
    return "\n".join(lines)
