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

"""Timed absolute tool poses and normalized gripper openings.

This RPC value is ordinary-pickle compatible. It is not an LCM wire message.
"""

from __future__ import annotations

import math
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class EefTrajectoryPoint:
    """One synchronized waypoint, in meters, unit quaternions and [0, 1] openings.

    ``poses`` keys identify model tool frames. Every pose is expressed in
    one common reference frame. Gripper keys identify joints whose native command is normalized.
    """

    def __init__(
        self,
        time_from_start: float,
        poses: dict[str, PoseStamped],
        grippers: dict[str, float] | None = None,
    ) -> None:
        self.time_from_start = time_from_start
        self.poses = poses
        self.grippers = grippers if grippers is not None else {}


class EefTrajectory:
    """An absolute tool trajectory with one common clock for every frame."""

    def __init__(self, points: list[EefTrajectoryPoint], timestamp: float | None = None) -> None:
        self.points = list(points)
        self.timestamp = time.time() if timestamp is None else timestamp
        if len(self.points) < 2:
            raise ValueError("EEF trajectory requires an anchor and at least one target")
        frames = set(self.points[0].poses)
        grippers = set(self.points[0].grippers)
        if not frames or any(not name for name in frames | grippers):
            raise ValueError("EEF trajectory requires nonempty frame and joint names")
        reference_frame = self.points[0].poses[next(iter(frames))].frame_id
        if not reference_frame:
            raise ValueError("EEF poses require an explicit reference frame")
        previous = -1.0
        for i, point in enumerate(self.points):
            t = point.time_from_start
            if not math.isfinite(t) or t < 0 or t <= previous or (i == 0 and t != 0):
                raise ValueError("EEF waypoint times must start at zero and strictly increase")
            previous = t
            if set(point.poses) != frames or set(point.grippers) != grippers:
                raise ValueError("Every EEF waypoint must target the same frames and grippers")
            for pose in point.poses.values():
                q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                if pose.frame_id != reference_frame:
                    raise ValueError("EEF poses must use one common reference frame")
                if not np.isfinite([pose.x, pose.y, pose.z, *q]).all():
                    raise ValueError("EEF pose must be finite")
                if not math.isclose(float(np.linalg.norm(q)), 1.0, abs_tol=1e-6):
                    raise ValueError("EEF orientation must be a unit quaternion")
            if any(not math.isfinite(g) or not 0 <= g <= 1 for g in point.grippers.values()):
                raise ValueError("EEF gripper openings must be finite and normalized")

    @property
    def duration(self) -> float:
        return self.points[-1].time_from_start

    def sample(self, elapsed: float) -> EefTrajectoryPoint:
        """Linearly interpolate translation/opening and shortest-arc SLERP rotation."""
        t = max(0.0, min(elapsed, self.duration))
        for first, last in zip(self.points, self.points[1:], strict=False):
            if t <= last.time_from_start:
                alpha = (t - first.time_from_start) / (last.time_from_start - first.time_from_start)
                poses = {}
                for frame, a in first.poses.items():
                    b = last.poses[frame]
                    qa, qb = a.orientation, b.orientation
                    rotations = Rotation.from_quat(
                        [[qa.x, qa.y, qa.z, qa.w], [qb.x, qb.y, qb.z, qb.w]]
                    )
                    q = Slerp([0.0, 1.0], rotations)(alpha).as_quat()
                    poses[frame] = PoseStamped(
                        ts=0.0,
                        frame_id=a.frame_id,
                        position=[
                            a.x + alpha * (b.x - a.x),
                            a.y + alpha * (b.y - a.y),
                            a.z + alpha * (b.z - a.z),
                        ],
                        orientation=q,
                    )
                return EefTrajectoryPoint(
                    t,
                    poses,
                    {
                        name: value + alpha * (last.grippers[name] - value)
                        for name, value in first.grippers.items()
                    },
                )
        raise AssertionError("Validated EEF trajectory has no sampling interval")
