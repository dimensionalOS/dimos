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

"""Operations on generated joint trajectories."""

from __future__ import annotations

from enum import IntEnum

from dimos.message_utils.time import to_seconds
from dimos.messages.trajectory_msgs.msg import JointTrajectory


class TrajectoryState(IntEnum):
    IDLE = 0
    EXECUTING = 1
    COMPLETED = 2
    ABORTED = 3
    FAULT = 4


def duration(message: JointTrajectory) -> float:
    return max((to_seconds(point.time_from_start) for point in message.points), default=0.0)


def sample(message: JointTrajectory, t: float) -> tuple[list[float], list[float]]:
    """
    Sample the trajectory at time t using linear interpolation.

    Args:
        t: Time from trajectory start (seconds)

    Returns:
        Tuple of (positions, velocities) at time t
    """
    if not message.points:
        return [], []

    # Clamp t to valid range
    t = max(0.0, min(t, duration(message)))

    # Find bracketing points
    if t <= to_seconds(message.points[0].time_from_start):
        return list(message.points[0].positions), list(message.points[0].velocities)

    if t >= to_seconds(message.points[-1].time_from_start):
        return list(message.points[-1].positions), list(message.points[-1].velocities)

    # Find interval
    for i in range(len(message.points) - 1):
        t0 = to_seconds(message.points[i].time_from_start)
        t1 = to_seconds(message.points[i + 1].time_from_start)

        if t0 <= t <= t1:
            # Linear interpolation
            alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            p0 = message.points[i]
            p1 = message.points[i + 1]

            positions = [
                p0.positions[j] + alpha * (p1.positions[j] - p0.positions[j])
                for j in range(len(p0.positions))
            ]
            velocities = [
                p0.velocities[j] + alpha * (p1.velocities[j] - p0.velocities[j])
                for j in range(len(p0.velocities))
            ]
            return positions, velocities

    # Fallback
    return list(message.points[-1].positions), list(message.points[-1].velocities)
