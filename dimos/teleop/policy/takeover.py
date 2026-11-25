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

"""Policy weights and pose-to-progress examples, independent of robot control."""

from collections.abc import Sequence
import math
from typing import TypeAlias

import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform

Weights6: TypeAlias = tuple[float, float, float, float, float, float]
Point3: TypeAlias = tuple[float, float, float]


def linear(progress: float) -> float:
    """Validate progress in [0, 1] and return it unchanged."""
    if not math.isfinite(progress) or not 0 <= progress <= 1:
        raise ValueError("progress must be finite and between zero and one")
    return progress


def quadratic(progress: float) -> float:
    return linear(progress) ** 2


def power(progress: float, q: float = 3.0) -> float:
    if not math.isfinite(q) or q <= 0:
        raise ValueError("q must be finite and positive")
    return float(linear(progress) ** q)


def logarithmic(progress: float, k: float = 9.0) -> float:
    if not math.isfinite(k) or k <= 0:
        raise ValueError("k must be finite and positive")
    return math.log1p(k * linear(progress)) / math.log1p(k)


def distance_to_target(
    pose: Transform, target: Point3, outer_radius: float, inner_radius: float = 0.0
) -> float:
    """Increase progress from 0 to 1 while approaching a target, distances in metres."""
    if not (math.isfinite(outer_radius) and 0 <= inner_radius < outer_radius):
        raise ValueError("radii must be finite with 0 <= inner_radius < outer_radius")
    position = pose.translation.to_numpy()
    target_array = np.asarray(target, dtype=float)
    if (
        target_array.shape != (3,)
        or not np.isfinite(target_array).all()
        or not np.isfinite(position).all()
    ):
        raise ValueError("position and target must be finite 3D points")
    distance = float(np.linalg.norm(position - target_array))
    return float(np.clip((outer_radius - distance) / (outer_radius - inner_radius), 0, 1))


def reference_path_progress(pose: Transform, path: Sequence[Point3]) -> float:
    """Project onto the nearest polyline segment and return fractional arc length.

    Repeated points are ignored. At self-intersections the first closest segment
    wins; use a non-self-intersecting path when progress must be unambiguous.
    """
    points = np.asarray(path, dtype=float)
    position = pose.translation.to_numpy()
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or len(points) < 2
        or not np.isfinite(points).all()
        or not np.isfinite(position).all()
    ):
        raise ValueError("path needs at least two finite 3D points and a finite pose")
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = lengths > 0
    if not valid.any():
        raise ValueError("path must have nonzero length")
    starts, segments, lengths = points[:-1][valid], segments[valid], lengths[valid]
    fractions = np.clip(np.sum((position - starts) * segments, axis=1) / lengths**2, 0, 1)
    projections = starts + fractions[:, None] * segments
    closest = int(np.argmin(np.linalg.norm(position - projections, axis=1)))
    return float(
        (np.sum(lengths[:closest]) + fractions[closest] * lengths[closest]) / np.sum(lengths)
    )
