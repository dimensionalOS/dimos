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

from __future__ import annotations

import time
from threading import RLock
from typing import TYPE_CHECKING, Any

import cv2
from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, sharpness_window
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
    from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

logger = setup_logger()

# COCO classes treated as seats, and the fraction of a seat's box that must be
# covered by a person box before we call it occupied.
SEAT_CLASSES = ("chair", "couch", "bench")
OCCUPANCY_OVERLAP = 0.2

# In-place scan: rotate slowly while the continuous detector watches, until a
# target is found or we've turned roughly all the way around.
SCAN_YAW_RATE = 0.5  # rad/s (slow, to keep motion blur low) [rad/s]
# Publish cmd_vel at ~10 Hz; the Go2 stops between commands if they arrive too
# slowly, so a low rate means it never actually turns.
SCAN_TICK = 0.1  # seconds between cmd_vel publishes [s]
SCAN_DURATION = 14.0  # ~one full revolution at SCAN_YAW_RATE [s]
SCAN_LOG_EVERY = 2.0  # seconds between progress logs [s]


class Config(ModuleConfig):
    camera_info: CameraInfo
    # Sharpest-frame target frequency (Hz). The detector only runs on the
    # crispest frame in each window, which suppresses motion blur.
    detect_freq: float = 5.0


class SeatFinderSkill(Module):
    """Self-contained seat/object finder: scan in place, then navigate to one.

    Detection runs continuously on a sharpness-filtered, backpressured stream
    (motion-robust). When a skill is called the module first stops frontier
    exploration (so it cannot clobber our goal), rotates in place until a target
    is seen, projects it to a 3D pose via the pointcloud, and publishes that pose
    on ``goal_request`` for the A* planner. Owning the whole flow keeps a single
    navigation-goal source, avoiding the explorer/seat-goal conflict.
    """

    config: Config

    color_image: In[Image]
    pointcloud: In[PointCloud2]
    goal_request: Out[PoseStamped]
    detections_image: Out[Image]
    stop_explore_cmd: Out[Bool]
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._detector = Yolo2DDetector()
        self._latest: ImageDetections2D | None = None
        self._lock = RLock()
        self._subscription: DisposableBase | None = None

    @rpc
    def start(self) -> None:
        super().start()
        sharp = backpressure(
            sharpness_window(self.config.detect_freq, self.color_image.pure_observable())
        )
        self._subscription = sharp.subscribe(
            on_next=self._on_frame,
            on_error=lambda e: logger.exception("Error in seat detection loop", exc_info=e),
        )

    @rpc
    def stop(self) -> None:
        if self._subscription is not None:
            self._subscription.dispose()
            self._subscription = None
        self.cmd_vel.publish(Twist.zero())
        super().stop()

    def _on_frame(self, image: Image) -> None:
        detections = self._detector.process_image(image)
        with self._lock:
            self._latest = detections
        self.detections_image.publish(self._annotate(image, detections.detections))

    @skill
    def find_empty_seat(self) -> str:
        """Look around for an empty seat, chair, or sofa and navigate to it.

        Use this when asked to guide someone to a free seat. The robot turns in
        place to search, then heads to the seat. Do NOT also call exploration or
        other movement tools; this skill owns the search and the motion.
        """
        return self._scan_and_navigate(self._select_empty_seats, "empty seat")

    @skill
    def find_object(self, query: str) -> str:
        """Look around for an object named `query` and navigate next to it.

        Use this to locate and approach a specific item (e.g. "bottle",
        "backpack", "chair"). Matches the YOLO (COCO) class name. The robot turns
        in place to search; do NOT also call exploration or movement tools.
        """
        return self._scan_and_navigate(lambda dets: self._select_by_name(dets, query), query)

    def _scan_and_navigate(self, selector: Any, label: str) -> str:
        # Single goal source: stop the frontier explorer so it can't overwrite
        # the goal we are about to publish.
        self.stop_explore_cmd.publish(Bool(data=True))

        detections, candidates = self._scan_in_place(selector)
        logger.info(
            f"SeatFinder: label={label!r} matched={len(candidates)} "
            f"names={[d.name for d in detections.detections] if detections else []}"
        )
        if detections is None or not candidates:
            return f"No {label} found after looking around."

        pointcloud = self.pointcloud.get_next()
        transform = self.tf.get("camera_optical", pointcloud.frame_id, detections.image.ts, 5.0)
        if not transform:
            return f"Could not resolve the camera transform, cannot locate the {label}."

        best = max(candidates, key=lambda d: d.bbox_2d_volume())
        target3d = Detection3DPC.from_2d(
            best,
            world_pointcloud=pointcloud,
            camera_info=self.config.camera_info,
            world_to_optical_transform=transform,
        )
        if target3d is None:
            return f"Found a {label} but could not compute its 3D position."

        pose = target3d.pose
        self.goal_request.publish(pose)
        return (
            f"Found a {label} at ({pose.position.x:.2f}, {pose.position.y:.2f}). "
            "Navigating there now."
        )

    def _scan_in_place(self, selector: Any) -> tuple[ImageDetections2D | None, list[Detection2DBBox]]:
        """Rotate slowly in place until the continuous detector yields a match
        (or we've turned all the way around). Always stops the robot on exit."""
        deadline = time.time() + SCAN_DURATION
        next_log = 0.0
        yaw = Twist(
            linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, SCAN_YAW_RATE)
        )
        try:
            while True:
                with self._lock:
                    detections = self._latest
                candidates = selector(detections.detections) if detections is not None else []
                if candidates or time.time() >= deadline:
                    return detections, candidates
                now = time.time()
                if now >= next_log:
                    seen = [d.name for d in detections.detections] if detections else []
                    logger.info(f"SeatFinder scan: rotating, currently seeing {seen}")
                    next_log = now + SCAN_LOG_EVERY
                self.cmd_vel.publish(yaw)
                time.sleep(SCAN_TICK)
        finally:
            self.cmd_vel.publish(Twist.zero())

    def _select_empty_seats(self, detections: list[Detection2DBBox]) -> list[Detection2DBBox]:
        seats = [d for d in detections if d.name in SEAT_CLASSES]
        persons = [d for d in detections if d.name == "person"]
        return [s for s in seats if not self._is_occupied(s, persons)]

    def _select_by_name(
        self, detections: list[Detection2DBBox], query: str
    ) -> list[Detection2DBBox]:
        q = query.lower()
        return [d for d in detections if d.name.lower() in q or q in d.name.lower()]

    def _is_occupied(self, seat: Detection2DBBox, persons: list[Detection2DBBox]) -> bool:
        sx1, sy1, sx2, sy2 = seat.bbox
        seat_area = max(1.0, (sx2 - sx1) * (sy2 - sy1))
        for p in persons:
            px1, py1, px2, py2 = p.bbox
            iw = max(0.0, min(sx2, px2) - max(sx1, px1))
            ih = max(0.0, min(sy2, py2) - max(sy1, py1))
            if (iw * ih) / seat_area > OCCUPANCY_OVERLAP:
                return True
        return False

    def _annotate(self, image: Image, detections: list[Detection2DBBox]) -> Image:
        img = image.to_opencv().copy()
        persons = [d for d in detections if d.name == "person"]
        for d in detections:
            x1, y1, x2, y2 = (int(v) for v in d.bbox)
            if d.name in SEAT_CLASSES:
                occupied = self._is_occupied(d, persons)
                color = (0, 0, 255) if occupied else (0, 255, 0)
                text = f"{d.name} {'occupied' if occupied else 'EMPTY'}"
            elif d.name == "person":
                color = (255, 0, 0)
                text = "person"
            else:
                color = (150, 150, 150)
                text = d.name
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                img, text, (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )
        return Image.from_opencv(img, ts=image.ts)


__all__ = ["SeatFinderSkill"]
