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

"""Bounded pose-history recording for trajectory visualization."""

from __future__ import annotations

import threading
from typing import Sequence

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class TrajectoryRecorder:
    """Records every pose; decimates in place instead of dropping old history
    once max_poses is hit, so the published trail never grows unbounded but
    still spans the whole session. Returns a publish snapshot every
    publish_every calls to record().
    """

    def __init__(self, frame_id: str, max_poses: int, publish_every: int) -> None:
        self._frame_id = frame_id
        self._max_poses = max_poses
        self._publish_every = publish_every
        self._lock = threading.Lock()
        self._poses: list[PoseStamped] = []

    def record(
        self,
        ts: float,
        position: Sequence[float],
        orientation: Sequence[float],
        frame_count: int,
    ) -> list[PoseStamped] | None:
        with self._lock:
            self._poses.append(
                PoseStamped(
                    ts=ts,
                    frame_id=self._frame_id,
                    position=[float(position[0]), float(position[1]), float(position[2])],
                    orientation=[
                        float(orientation[0]),
                        float(orientation[1]),
                        float(orientation[2]),
                        float(orientation[3]),
                    ],
                )
            )
            if len(self._poses) >= self._max_poses:
                self._poses = self._poses[::2]
            if frame_count % self._publish_every == 0:
                return list(self._poses)
            return None
