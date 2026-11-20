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

Shared by a live driver and the offline report (``float_report``). A window is
the ``float_events`` marker plus the joint samples recorded during it; the drift
is the fingertip motion from the first limp sample to the end of the window, computed
through the arm's model (its spec's tip frame and fingertip offset) so a joint
error far from the base counts for what it does at the fingers. A window whose
samples do not span the duration its marker asked for is incomplete and fails.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json
import math
from typing import Any

import numpy as np

from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.spec import ArmCalibrationSpec

# Milestone: a floated arm, let go, stays within this at the fingertip over 10 s.
DRIFT_LIMIT_MM = 2.0
# The pose at the end of a window is the median over this much of it, so one
# noisy encoder sample does not decide the drift.
EDGE_WINDOW_S = 0.25
WINDOW_COMPLETE_TOLERANCE_S = 0.1


@dataclass(frozen=True)
class FloatEvent:
    """The ``float_events`` marker a driver publishes when a window starts.

    ``side`` names the arm that floats (e.g. "left"), as the robot's arms are
    named in ``robots.arm_spec``.
    """

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
    """What one float window says about the arm.

    ``requested_s`` is the window its marker asked for; one whose samples span
    less, within ``WINDOW_COMPLETE_TOLERANCE_S``, is incomplete and fails.
    """

    side: str
    index: int
    duration_s: float  # the window actually covered by samples
    requested_s: float
    samples: int
    fingertip_drift_mm: float  # first limp sample to end pose, through the model
    fingertip_peak_mm: float  # farthest the fingertip got from the first limp sample
    max_joint_drift_deg: float
    max_joint: str
    joint_drift_deg: dict[str, float]

    @property
    def complete(self) -> bool:
        return self.duration_s >= self.requested_s - WINDOW_COMPLETE_TOLERANCE_S

    @property
    def passed(self) -> bool:
        return self.complete and self.fingertip_drift_mm < DRIFT_LIMIT_MM

    @property
    def verdict(self) -> str:
        if not self.complete:
            return "cut short"
        return "ok" if self.passed else "FAIL"

    def as_row(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "index": self.index,
            "duration_s": round(self.duration_s, 3),
            "requested_s": round(self.requested_s, 3),
            "complete": self.complete,
            "samples": self.samples,
            "fingertip_drift_mm": round(self.fingertip_drift_mm, 3),
            "fingertip_peak_mm": round(self.fingertip_peak_mm, 3),
            "max_joint_drift_deg": round(self.max_joint_drift_deg, 4),
            "max_joint": self.max_joint,
            "passed": self.passed,
        }


class FingertipModel:
    """Fingertip position of one arm from its joint angles, workspace frame, metres.

    The fingertip is the spec's tip frame moved by the workspace's
    ``point_offset_m``, the same point the pose filter checks.
    """

    def __init__(self, spec: ArmCalibrationSpec | ArmModel) -> None:
        self._arm = spec if isinstance(spec, ArmModel) else ArmModel(spec)
        self.arm_id = self._arm.arm_id
        self.joint_names = list(self._arm.joint_names)

    def fingertip_m(self, joints_rad: Sequence[float] | np.ndarray) -> np.ndarray:
        if len(joints_rad) != len(self.joint_names):
            raise ValueError(f"expected {len(self.joint_names)} angles, got {len(joints_rad)}")
        return self._arm.fingertip(np.asarray(joints_rad, dtype=np.float64))


def edge_pose(t: np.ndarray, q: np.ndarray, *, start: bool) -> np.ndarray:
    """The first joint vector, or the median over the last ``EDGE_WINDOW_S`` of the samples."""
    if start:
        return np.asarray(q[0], dtype=np.float64)
    mask = t >= t[-1] - EDGE_WINDOW_S
    return np.asarray(np.median(q[mask], axis=0), dtype=np.float64)


def analyze_window(
    model: FingertipModel,
    event: FloatEvent,
    t: np.ndarray,
    q_rad: np.ndarray,
) -> DriftMetrics:
    """Drift over one window from its samples (``t`` seconds, ``q_rad`` rows in arm joint order).

    ``t`` must be sorted. Raises ``ValueError`` on fewer than two samples.
    """
    t = np.asarray(t, dtype=np.float64)
    q_rad = np.asarray(q_rad, dtype=np.float64)
    dof = len(model.joint_names)
    if t.ndim != 1 or q_rad.shape != (t.size, dof):
        raise ValueError(f"need (n,) times and (n, {dof}) angles, got {t.shape} and {q_rad.shape}")
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
        requested_s=event.duration_s,
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
    """Fingertip distance from the window's first sample at every sample, millimetres."""
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
    return (
        f"{metrics.side} pose {metrics.index}: fingertip drift {metrics.fingertip_drift_mm:.2f} mm "
        f"over {metrics.duration_s:.1f} of {metrics.requested_s:g} s / {metrics.samples} samples "
        f"(peak {metrics.fingertip_peak_mm:.2f} mm), "
        f"max joint drift {metrics.max_joint_drift_deg:.3f} deg on {metrics.max_joint} "
        f"[{metrics.verdict}, limit {DRIFT_LIMIT_MM:g} mm]"
    )


def format_table(rows: list[DriftMetrics]) -> str:
    header = (
        "| side | pose | window (s) | samples | fingertip drift (mm) | peak (mm) "
        "| max joint drift (deg) | joint | pass |"
    )
    lines = [header, "|" + "---|" * 9]
    for m in sorted(rows, key=lambda m: (m.side, m.index)):
        lines.append(
            f"| {m.side} | {m.index} | {m.duration_s:.1f} of {m.requested_s:g} | {m.samples} "
            f"| {m.fingertip_drift_mm:.2f} | {m.fingertip_peak_mm:.2f} "
            f"| {m.max_joint_drift_deg:.3f} | {m.max_joint} | {m.verdict} |"
        )
    lines.append("")
    return "\n".join(lines)
