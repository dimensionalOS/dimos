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

"""Execution state and interpolation for generated ROS2 trajectories."""

from enum import IntEnum
import math

from dimos_generated.dimos_msgs.msg import TrajectoryStatus
from dimos_generated.trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from dimos.msgs.time import to_seconds


class TrajectoryState(IntEnum):
    """Internal state-machine names matching generated wire constants."""

    IDLE = TrajectoryStatus.IDLE
    EXECUTING = TrajectoryStatus.EXECUTING
    COMPLETED = TrajectoryStatus.COMPLETED
    ABORTED = TrajectoryStatus.ABORTED
    FAULT = TrajectoryStatus.FAULT


def trajectory_duration(trajectory: JointTrajectory) -> float:
    """Return the last waypoint time in seconds; validated paths have increasing times."""
    return to_seconds(trajectory.points[-1].time_from_start) if trajectory.points else 0.0


def _sample_point(point: JointTrajectoryPoint) -> tuple[list[float], list[float]]:
    return list(point.positions), list(point.velocities) or [0.0] * len(point.positions)


def sample_trajectory(
    trajectory: JointTrajectory, seconds: float
) -> tuple[list[float], list[float]]:
    """Linearly sample a validated path, clamping outside its waypoint time range.

    ROS permits absent velocities; this position controller's feed-forward then
    defaults to zero. Inputs remain unchanged and returned arrays are independent.
    """
    if not math.isfinite(seconds):
        raise ValueError("sample time must be finite")
    if not trajectory.points:
        return [], []
    first, last = trajectory.points[0], trajectory.points[-1]
    if seconds <= to_seconds(first.time_from_start):
        return _sample_point(first)
    if seconds >= to_seconds(last.time_from_start):
        return _sample_point(last)
    for index in range(len(trajectory.points) - 1):
        before, after = trajectory.points[index], trajectory.points[index + 1]
        start, end = to_seconds(before.time_from_start), to_seconds(after.time_from_start)
        if start <= seconds <= end:
            alpha = (seconds - start) / (end - start)
            positions0, velocities0 = _sample_point(before)
            positions1, velocities1 = _sample_point(after)
            return (
                [
                    left + alpha * (right - left)
                    for left, right in zip(positions0, positions1, strict=True)
                ],
                [
                    left + alpha * (right - left)
                    for left, right in zip(velocities0, velocities1, strict=True)
                ],
            )
    raise ValueError("trajectory times must be increasing")
