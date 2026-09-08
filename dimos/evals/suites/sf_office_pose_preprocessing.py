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

"""Frozen preprocessing for agent-encoded PoseStamped trajectories."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from typing import Any, TypedDict

import numpy as np
from numpy.typing import NDArray
from scipy.signal import savgol_filter

DT_S = 0.1


class ProcessedPoseTrajectory(TypedDict):
    """Canonical arrays available to trajectory-analysis agents."""

    time_s: NDArray[np.float64]
    position_m: NDArray[np.float64]
    yaw_rad: NDArray[np.float64]
    velocity_xy_m_s: NDArray[np.float64]
    speed_m_s: NDArray[np.float64]


def _hampel(values: NDArray[np.float64], minimum_threshold: float) -> NDArray[np.float64]:
    filtered = values.copy()
    for index in range(len(values)):
        window = values[max(0, index - 5) : min(len(values), index + 6)]
        median = float(np.median(window))
        mad = float(np.median(np.abs(window - median)))
        if abs(values[index] - median) > max(minimum_threshold, 3.0 * 1.4826 * mad):
            filtered[index] = median
    return filtered


def preprocess_encoded_poses(
    encoded_poses: Sequence[Mapping[str, Any]],
) -> ProcessedPoseTrajectory:
    """Apply the benchmark's frozen trajectory preprocessing to encoded poses."""
    by_timestamp: dict[float, tuple[NDArray[np.float64], NDArray[np.float64], float | None]] = {}
    for encoded in encoded_poses:
        timestamp = float(encoded["timestamp_s"])
        position = np.asarray(encoded["position_m"], dtype=float)
        planar_position = encoded.get("planar_position_m")
        if planar_position is not None:
            planar_position_array = np.asarray(planar_position, dtype=float)
            if planar_position_array.shape != (2,):
                raise ValueError("encoded planar position must contain XY")
            position[:2] = planar_position_array
        quaternion = np.asarray(encoded["quaternion_xyzw"], dtype=float)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError("encoded pose must contain 3D position and xyzw quaternion")
        if not np.isfinite(timestamp) or not np.all(np.isfinite(position)):
            raise ValueError("encoded pose timestamp and position must be finite")
        quaternion_norm = float(np.linalg.norm(quaternion))
        if not np.all(np.isfinite(quaternion)) or quaternion_norm == 0.0:
            raise ValueError("encoded pose quaternion must be finite and nonzero")
        yaw_deg = encoded.get("yaw_deg")
        by_timestamp[timestamp] = (
            position,
            quaternion / quaternion_norm,
            float(yaw_deg) if yaw_deg is not None else None,
        )

    if len(by_timestamp) < 11:
        raise ValueError("at least 11 unique encoded poses are required")
    timestamps = np.asarray(sorted(by_timestamp), dtype=float)
    positions = np.asarray([by_timestamp[timestamp][0] for timestamp in timestamps])
    quaternions = np.asarray([by_timestamp[timestamp][1] for timestamp in timestamps])
    encoded_yaw = [by_timestamp[timestamp][2] for timestamp in timestamps]
    if all(yaw is not None for yaw in encoded_yaw):
        raw_yaw = np.unwrap(np.radians(np.asarray(encoded_yaw, dtype=float)))
    else:
        qx, qy, qz, qw = quaternions.T
        raw_yaw = np.unwrap(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    duration_s = float(timestamps[-1] - timestamps[0])
    if duration_s <= 0.0:
        raise ValueError("encoded pose timestamps must span positive time")
    time_s = np.arange(math.floor(duration_s * 10.0) + 1, dtype=float) * DT_S
    raw_time_s = timestamps - timestamps[0]
    processed_positions = np.column_stack(
        [
            np.interp(time_s, raw_time_s, positions[:, component])
            for component in range(positions.shape[1])
        ]
    )
    processed_yaw = np.interp(time_s, raw_time_s, raw_yaw)
    for component in range(processed_positions.shape[1]):
        processed_positions[:, component] = savgol_filter(
            _hampel(processed_positions[:, component], 0.005),
            11,
            2,
            mode="interp",
        )
    processed_yaw = savgol_filter(_hampel(processed_yaw, math.radians(1.0)), 11, 2, mode="interp")
    velocity = np.gradient(processed_positions[:, :2], DT_S, axis=0)
    return {
        "time_s": time_s,
        "position_m": processed_positions,
        "yaw_rad": processed_yaw,
        "velocity_xy_m_s": velocity,
        "speed_m_s": np.linalg.norm(velocity, axis=1),
    }
