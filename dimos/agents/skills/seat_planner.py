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

"""On-demand YOLO empty-seat picker for the manual-map → auto-find demo.

Flow:
  1. Operator drives the Go2 manually (Rerun click-to-goal or teleop) to build
     the voxel map and bring the seats into view.
  2. Operator runs `dimos mcp call find_empty_seat_now` from another terminal.
  3. This skill grabs the latest YOLO detections, picks an unoccupied seat,
     projects it to a 3D pose via the world voxel cloud, and publishes the pose
     to `goal_request`. A* draws the path; if MovementManager is enabled the
     robot walks there.

YOLO runs continuously so the annotated image is always available in the
viewer. The skill itself never publishes cmd_vel and never rotates.
"""

from __future__ import annotations

import math
import threading
from threading import RLock
from typing import TYPE_CHECKING, Any

import cv2
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
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

SEAT_CLASSES = ("chair", "couch", "bench")
OCCUPANCY_OVERLAP = 0.2

# Spoken / typed phrases that trigger find_empty_seat_now via /human_input.
# Matched case-insensitively as substrings, so partial Whisper transcriptions
# still fire (e.g. "椅子まで行って" matches "椅子").
SEAT_TRIGGER_KEYWORDS = ("椅子", "空席", "席まで", "chair", "seat", "vacant")


class Config(ModuleConfig):
    camera_info: CameraInfo
    detect_freq: float = 5.0          # YOLO inference rate on sharpest frame [Hz]


class SeatPlanner(Module):
    config: Config

    color_image: In[Image]
    pointcloud: In[PointCloud2]
    human_input: In[str]
    goal_request: Out[PoseStamped]
    detections_image: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._detector = Yolo2DDetector()
        self._latest: ImageDetections2D | None = None
        self._lock = RLock()
        self._subscription: DisposableBase | None = None
        self._voice_busy = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        sharp = backpressure(
            sharpness_window(self.config.detect_freq, self.color_image.pure_observable())
        )
        self._subscription = sharp.subscribe(
            on_next=self._on_frame,
            on_error=lambda e: logger.exception("SeatPlanner detection error", exc_info=e),
        )
        self.register_disposable(Disposable(self.human_input.subscribe(self._on_human_input)))

    @rpc
    def stop(self) -> None:
        if self._subscription is not None:
            self._subscription.dispose()
            self._subscription = None
        super().stop()

    def _on_human_input(self, text: str) -> None:
        lower = text.lower()
        if not any(kw.lower() in lower for kw in SEAT_TRIGGER_KEYWORDS):
            return
        if not self._voice_busy.acquire(blocking=False):
            logger.info(f"SeatPlanner: voice trigger ignored, already running ({text!r})")
            return
        logger.info(f"SeatPlanner: voice trigger fired by {text!r}")
        threading.Thread(target=self._voice_worker, daemon=True, name="SeatPlannerVoice").start()

    def _voice_worker(self) -> None:
        try:
            result = self.find_empty_seat_now()
            logger.info(f"SeatPlanner: voice trigger result: {result}")
        finally:
            self._voice_busy.release()

    def _on_frame(self, image: Image) -> None:
        detections = self._detector.process_image(image)
        with self._lock:
            self._latest = detections
        self.detections_image.publish(self._annotate(image, detections.detections))

    @skill
    def navigate_to_point(self, x: float, y: float, yaw_deg: float = 0.0) -> str:
        """Publish a world-frame goal pose so the A* planner drives the robot there.

        Args:
            x: target X in the `map` frame [m]
            y: target Y in the `map` frame [m]
            yaw_deg: final heading in degrees (0 = +X), default 0
        """
        pose = PoseStamped(
            position=make_vector3(float(x), float(y), 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(float(yaw_deg)))),
            frame_id="map",
        )
        self.goal_request.publish(pose)
        msg = f"Published waypoint goal at ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}deg)."
        logger.info(f"SeatPlanner: {msg}")
        return msg

    @skill
    def find_empty_seat_now(self) -> str:
        """Pick an empty seat in the current camera view and publish a 3D goal.

        Uses the latest YOLO detections (chair/couch/bench minus person-overlap)
        and the world voxel cloud. Returns immediately; downstream A* plans the
        path. The robot only walks there if MovementManager is enabled.
        """
        with self._lock:
            detections = self._latest

        if detections is None:
            return "No detections yet — wait a second after the camera comes up."

        seats = [d for d in detections.detections if d.name in SEAT_CLASSES]
        persons = [d for d in detections.detections if d.name == "person"]
        empty = [s for s in seats if not self._is_occupied(s, persons)]

        seen = [d.name for d in detections.detections]
        logger.info(
            f"SeatPlanner: seen={seen} seats={len(seats)} persons={len(persons)} empty={len(empty)}"
        )

        if not empty:
            return f"No empty seat in view. Saw {len(seats)} seat(s), {len(persons)} person(s)."

        try:
            pointcloud = self.pointcloud.get_next(timeout=2.0)
        except Exception as e:
            return f"Pointcloud unavailable: {e}"

        transform = self.tf.get("camera_optical", pointcloud.frame_id, detections.image.ts, 2.0)
        if not transform:
            return "Camera transform unavailable — drive the robot a bit then retry."

        best = max(empty, key=lambda d: d.bbox_2d_volume())
        target3d = Detection3DPC.from_2d(
            best,
            world_pointcloud=pointcloud,
            camera_info=self.config.camera_info,
            world_to_optical_transform=transform,
        )
        if target3d is None:
            return "Found an empty seat but 3D projection failed (no cloud points in bbox)."

        pose = target3d.pose
        self.goal_request.publish(pose)
        msg = f"Published goal at ({pose.position.x:.2f}, {pose.position.y:.2f})."
        logger.info(f"SeatPlanner: {msg}")
        return msg

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


__all__ = ["SeatPlanner"]
