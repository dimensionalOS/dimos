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

"""Prediction and braking envelopes in metres and simulation seconds."""

import math

import numpy as np
from numpy.typing import NDArray

BRAKING_DECELERATION = 0.75


def predict_position(
    position: NDArray[np.float64],
    velocity: NDArray[np.float64],
    latency_s: float,
    route: NDArray[np.float64] | None,
    maximum_lead: float,
) -> NDArray[np.float64]:
    speed = float(np.linalg.norm(velocity[:2]))
    stopping_distance = speed * speed / (2 * BRAKING_DECELERATION)
    distance = min(
        maximum_lead,
        max(speed * (latency_s + 0.5) + stopping_distance, 2.0 if route is not None else 0.0),
    )
    if route is None or len(route) == 0:
        return position + velocity * (distance / speed if speed > 1e-6 else 0.0)
    if len(route) == 1:
        return np.asarray(route[0], dtype=np.float64).copy()
    segments = np.diff(route, axis=0)
    lengths2 = np.sum(segments * segments, axis=1)
    fractions = np.clip(
        np.sum((position - route[:-1]) * segments, axis=1) / np.maximum(lengths2, 1e-12), 0, 1
    )
    projections = route[:-1] + segments * fractions[:, None]
    nearest = int(np.argmin(np.linalg.norm(projections - position, axis=1)))
    previous = projections[nearest]
    for waypoint in route[nearest + 1 :]:
        delta = waypoint - previous
        length = float(np.linalg.norm(delta))
        if length >= distance and length > 1e-9:
            return np.asarray(previous + delta * distance / length, dtype=np.float64)
        distance -= length
        previous = waypoint
    return np.asarray(previous, dtype=np.float64).copy()


def speed_for_distance(
    distance: float, latency_s: float, deceleration: float = BRAKING_DECELERATION
) -> float:
    """Solve v*T + v²/(2*a) <= distance with a conservative deceleration budget."""
    if distance <= 0:
        return 0.0
    return max(
        0.0,
        math.sqrt((deceleration * latency_s) ** 2 + 2 * deceleration * distance)
        - deceleration * latency_s,
    )


def govern_velocity(
    command_xy: NDArray[np.float64],
    position_xy: NDArray[np.float64],
    low: NDArray[np.float64],
    high: NDArray[np.float64],
    latency_s: float,
) -> NDArray[np.float64]:
    scale = 1.0
    for axis, speed in enumerate(command_xy):
        if abs(speed) < 1e-9:
            continue
        distance = high[axis] - position_xy[axis] if speed > 0 else position_xy[axis] - low[axis]
        scale = min(scale, speed_for_distance(float(distance), latency_s) / abs(speed))
    return command_xy * scale
