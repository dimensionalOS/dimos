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

"""Drift of a floating arm: how far the fingertip wanders, through the arm's model, once let go."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np

from dimos.control.calibration.gravity_poses import ArmModel


@dataclass(frozen=True)
class DriftMetrics:
    """One float window: fingertip motion from the first sample to the last, and the farthest."""

    duration_s: float
    samples: int
    fingertip_drift_mm: float
    fingertip_peak_mm: float
    joint_drift_deg: dict[str, float]
    drift_limit_mm: float

    @property
    def passed(self) -> bool:
        return self.fingertip_drift_mm < self.drift_limit_mm

    @property
    def max_joint(self) -> str:
        return max(self.joint_drift_deg, key=lambda name: abs(self.joint_drift_deg[name]))


def analyze_float(
    arm: ArmModel,
    t: Sequence[float] | np.ndarray,
    q_rad: Sequence[Sequence[float]] | np.ndarray,
    *,
    duration_s: float,
    drift_limit_mm: float,
) -> DriftMetrics:
    """Drift over one window: ``t`` seconds (sorted), ``q_rad`` one row per sample in arm joint order.

    ``drift_limit_mm`` is the operator's pass limit. Raises ``ValueError`` for non-finite or
    misshapen samples, or ones that cover less of ``duration_s`` than their largest gap allows.
    """
    t_ = np.asarray(t, dtype=np.float64)
    q = np.asarray(q_rad, dtype=np.float64)
    if t_.ndim != 1 or t_.size < 2 or q.shape != (t_.size, arm.dof):
        raise ValueError(f"need (n,) times and (n, {arm.dof}) angles, got {t_.shape} and {q.shape}")
    if not (np.isfinite(t_).all() and np.isfinite(q).all()):
        raise ValueError("float samples hold NaN or infinite values")
    covered = float(t_[-1] - t_[0])
    if covered < duration_s - float(np.diff(t_).max()):
        raise ValueError(f"the float samples cover {covered:.2f} of {duration_s:g} s")
    tips = np.asarray([arm.fingertip(row) for row in q])
    distance = np.linalg.norm(tips - tips[0], axis=1) * 1000.0
    return DriftMetrics(
        duration_s=covered,
        samples=int(t_.size),
        fingertip_drift_mm=float(distance[-1]),
        fingertip_peak_mm=float(distance.max()),
        joint_drift_deg={
            name: math.degrees(float(q[-1, i] - q[0, i])) for i, name in enumerate(arm.joint_names)
        },
        drift_limit_mm=drift_limit_mm,
    )
