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

"""Feedback-relative joint command bounds shared by streaming solvers."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def bound_joint_command(
    candidate: NDArray[np.float64],
    previous: NDArray[np.float64],
    measured: NDArray[np.float64],
    lower: NDArray[np.float64],
    upper: NDArray[np.float64],
    velocity: NDArray[np.float64],
    dt: float,
    tracking_error: float,
) -> NDArray[np.float64]:
    arrays = [
        np.asarray(x, dtype=float) for x in (candidate, previous, measured, lower, upper, velocity)
    ]
    if not arrays[0].ndim == 1 or any(x.shape != arrays[0].shape for x in arrays):
        raise ValueError("Joint command shapes differ")
    if (
        not all(np.isfinite(x).all() for x in arrays[:3])
        or np.isnan(lower).any()
        or np.isnan(upper).any()
        or not np.isfinite(velocity).all()
        or np.any(velocity <= 0)
    ):
        raise ValueError("Invalid joint command values")
    if not np.isfinite([dt, tracking_error]).all() or dt <= 0 or tracking_error <= 0:
        raise ValueError("Invalid control timestep or tracking bound")
    lo = np.maximum(np.maximum(previous - velocity * dt, measured - tracking_error), lower)
    hi = np.minimum(np.minimum(previous + velocity * dt, measured + tracking_error), upper)
    if np.any(lo > hi):
        raise ValueError("Empty joint command envelope")
    return np.clip(candidate, lo, hi)
