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

"""Latch a fiducial marker's pose and hold it after the tag leaves view.

A manipulation target has to outlive the sighting: the robot looks at the tag
from across the room, then walks up to it, and by the time the arm moves the
tag is out of frame or occluded by the robot itself. So this consumes marker
detections, waits for several agreeing sightings, then republishes that one
pose forever.

Publishes the same ``PoseStamped`` contract ``SimBodyPose`` publishes in
simulation, so downstream consumers cannot tell perception from ground truth.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MarkerLatchConfig(ModuleConfig):
    # Empty accepts any marker; the first tag to reach consensus wins.
    marker_ids: list[int] = Field(default_factory=list)
    frame_id: str = "world"
    # Sightings that must agree before latching. Detection is throttled to
    # ~2 Hz upstream, so keep this small enough to latch in a couple seconds.
    required_sightings: int = Field(3, ge=1)
    # Agreement radius between consecutive sightings.
    position_tolerance_m: float = Field(0.05, gt=0.0)
    publish_hz: float = Field(5.0, gt=0.0)


class MarkerLatchModule(Module):
    """Hold the first consistently-seen marker pose as a manipulation target."""

    config: MarkerLatchConfig

    detections: In[Detection3DArray]
    object_pose: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._latched: PoseStamped | None = None
        self._latched_id: int | None = None
        self._candidate_id: int | None = None
        self._candidate: list[tuple[float, float, float]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.detections.subscribe(self._on_detections)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    @rpc
    def get_latched_pose(self) -> PoseStamped | None:
        """The held pose, or None while still waiting for agreeing sightings."""
        with self._lock:
            return None if self._latched is None else self._copy(self._latched)

    @rpc
    def get_latched_marker_id(self) -> int | None:
        with self._lock:
            return self._latched_id

    @rpc
    def reset(self) -> None:
        """Drop the latch so the next tag seen becomes the new target."""
        with self._lock:
            self._latched = None
            self._latched_id = None
            self._candidate_id = None
            self._candidate = []
        logger.info("MarkerLatchModule: latch cleared, accepting new sightings")

    def _wanted(self, marker_id: int) -> bool:
        return not self.config.marker_ids or marker_id in self.config.marker_ids

    def _on_detections(self, msg: Detection3DArray) -> None:
        with self._lock:
            if self._latched is not None:
                return

        for detection in msg.detections[: msg.detections_length]:
            try:
                marker_id = int(str(detection.id).strip())
            except (TypeError, ValueError):
                continue
            if not self._wanted(marker_id):
                continue
            position = detection.bbox.center.position
            orientation = detection.bbox.center.orientation
            self._accumulate(marker_id, position, orientation)
            return

    def _accumulate(self, marker_id: int, position: Any, orientation: Any) -> None:
        xyz = (float(position.x), float(position.y), float(position.z))

        with self._lock:
            if marker_id != self._candidate_id:
                # A different tag: start its streak from this sighting.
                self._candidate_id = marker_id
                self._candidate = [xyz]
            elif (
                float(np.linalg.norm(np.subtract(xyz, self._candidate[-1])))
                > self.config.position_tolerance_m
            ):
                # Moved too far to be the same pot; restart the streak.
                self._candidate = [xyz]
            else:
                self._candidate.append(xyz)

            if len(self._candidate) < self.config.required_sightings:
                return

            mean = np.mean(self._candidate, axis=0)
            # Build the pose explicitly: PoseStamped(other) drops frame_id.
            self._latched = PoseStamped(
                position=[float(mean[0]), float(mean[1]), float(mean[2])],
                orientation=[
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ],
                frame_id=self.config.frame_id,
                ts=time.time(),
            )
            self._latched_id = marker_id
            sightings = len(self._candidate)

        logger.info(
            "MarkerLatchModule latched marker",
            marker_id=marker_id,
            sightings=sightings,
            frame_id=self.config.frame_id,
            position=[round(float(v), 3) for v in mean],
        )

    def _copy(self, pose: PoseStamped) -> PoseStamped:
        return PoseStamped(
            position=[pose.position.x, pose.position.y, pose.position.z],
            orientation=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            frame_id=pose.frame_id,
            ts=time.time(),
        )

    def _publish_loop(self) -> None:
        period = 1.0 / self.config.publish_hz
        while not self._stop.wait(period):
            with self._lock:
                latched = self._latched
            if latched is not None:
                self.object_pose.publish(self._copy(latched))
