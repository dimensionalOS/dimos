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

"""AprilTag 3D detection module.

Consumes ``color_image`` + ``camera_info``, detects AprilTag/ArUco markers via
OpenCV's ArUco module, and estimates each tag's 6-DOF pose with ``solvePnP``.
Publishes TF transforms for every detected tag::

    camera_optical -> marker/{family}_{tag_id}

This allows downstream modules (navigation, manipulation, multi-robot
identification) to use detected tags as reference landmarks in world space.

Example blueprint wiring::

    from dimos.perception.detection.apriltag_detector import AprilTagDetectionModule
    from dimos.spec.perception import Camera

    tag_module = AprilTagDetectionModule(
        family="tag36h11",
        tag_size_m=0.165,
    )
    tag_module.color_image.connect(camera.color_image)
    tag_module.camera_info.connect(camera.camera_info)
"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.perception.detection.detectors.apriltag import (
    AprilTagDetection,
    AprilTagDetector,
)
from dimos.spec.perception import Camera
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


class Config(ModuleConfig):
    family: str = "tag36h11"
    tag_size_m: float = 0.165
    max_freq: float = 10.0
    publish_transforms: bool = True
    publish_annotated_image: bool = True
    publish_tags_json: bool = True


class AprilTagDetectionModule(Module):
    """DimOS module for AprilTag/ArUco fiducial marker detection.

    Subscribes to ``color_image`` and ``camera_info``, runs ArUco detection +
    solvePnP pose estimation, and publishes:

    * **tf** — ``Transform`` messages linking the camera to each detected tag.
    * **annotated_image** — input image with tag outlines and IDs overlaid.
    * **tags_json** — JSON-serialised list of detections (ID, pose, corners).

    Attributes:
        color_image: In[Image] — colour images from the camera.
        camera_info: In[CameraInfo] — calibration for the same camera.
        annotated_image: Out[Image] — debug / viz image with overlays.
        tags_json: Out[str] — JSON string of detections.
    """

    config: Config
    detector: AprilTagDetector

    color_image: In[Image]
    camera_info: In[CameraInfo]

    annotated_image: Out[Image]
    tags_json: Out[str]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.detector = AprilTagDetector(
            family=self.config.family,
            tag_size_m=self.config.tag_size_m,
        )
        self._latest_camera_info: CameraInfo | None = None

    def _process_frame(self, image: Image) -> None:
        """Run detection on one image frame and publish results."""
        camera_info = self._latest_camera_info
        if camera_info is None:
            logger.debug("AprilTag module: no camera_info available yet")
            return

        detections = self.detector.detect(image, camera_info)
        if not detections:
            return

        # Build TF transforms: camera_optical -> marker/{family}_{id}
        transforms: list[Transform] = []
        for det in detections:
            transforms.append(
                Transform(
                    frame_id=camera_info.frame_id or "camera_optical",
                    child_frame_id=det.tag_frame_id,
                    ts=image.ts,
                    translation=Vector3(
                        det.pose.position.x,
                        det.pose.position.y,
                        det.pose.position.z,
                    ),
                    rotation=det.pose.orientation,
                )
            )
            logger.debug(
                "AprilTag module: published TF %s -> %s  pos=(%.3f,%.3f,%.3f)",
                transforms[-1].frame_id,
                transforms[-1].child_frame_id,
                det.pose.position.x,
                det.pose.position.y,
                det.pose.position.z,
            )

        if self.config.publish_transforms and transforms:
            self.tf.publish(*transforms)

        if self.config.publish_annotated_image:
            annotated = self._annotate_image(image, detections)
            self.annotated_image.publish(annotated)

        if self.config.publish_tags_json:
            serialised = self._detections_to_json(detections)
            self.tags_json.publish(serialised)

    @staticmethod
    def _annotate_image(image: Image, detections: list[AprilTagDetection]) -> Image:
        """Draw tag outlines and IDs on the image."""
        cv_image = image.to_opencv()
        if cv_image is None:
            return image

        for det in detections:
            corners = det.corners_image.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(cv_image, [corners], True, (0, 255, 0), 2)

            # Draw tag ID text near first detected corner.
            text_x = int(corners[0][0][0])
            text_y = int(corners[0][0][1]) - 10
            label = f"{det.family}:{det.tag_id}"
            # Small black background for readability.
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(
                cv_image,
                (text_x, text_y - th - 4),
                (text_x + tw, text_y + 2),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                cv_image,
                label,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        annotated = Image.from_opencv(cv_image)
        annotated.ts = image.ts
        annotated.frame_id = image.frame_id
        return annotated

    @staticmethod
    def _detections_to_json(detections: list[AprilTagDetection]) -> str:
        """Serialise detections to a compact JSON string."""
        import json

        out = []
        for det in detections:
            out.append(
                {
                    "tag_id": det.tag_id,
                    "family": det.family,
                    "position": {
                        "x": det.pose.position.x,
                        "y": det.pose.position.y,
                        "z": det.pose.position.z,
                    },
                    "orientation": {
                        "x": det.pose.orientation.x,
                        "y": det.pose.orientation.y,
                        "z": det.pose.orientation.z,
                        "w": det.pose.orientation.w,
                    },
                    "frame_id": det.pose.frame_id,
                    "ts": det.pose.ts,
                    "corners": det.corners_image.tolist(),
                }
            )
        return json.dumps(out)

    @rpc
    def start(self) -> None:
        super().start()

        def _detection_stream():
            return backpressure(
                self.color_image.pure_observable().pipe(
                    sharpness_barrier(self.config.max_freq)
                )
            )

        _detection_stream().subscribe(lambda img: self._process_frame(img))

        # Cache latest camera_info so _process_frame never blocks.
        self.camera_info.subscribe(lambda info: setattr(self, "_latest_camera_info", info))

    @rpc
    def stop(self) -> None:
        self.detector.stop()
        super().stop()


def deploy(
    dimos: Any,
    camera: Camera,
    prefix: str = "/apriltag",
    **kwargs: Any,
) -> AprilTagDetectionModule:
    """Deploy an AprilTag detection module wired to a camera.

    Args:
        dimos: Module coordinator.
        camera: Camera spec providing ``color_image`` and ``camera_info``.
        prefix: Transport topic prefix.
        **kwargs: Passed to :class:`AprilTagDetectionModule` config.
    """
    detector = AprilTagDetectionModule(**kwargs)
    detector.color_image.connect(camera.color_image)
    detector.camera_info.connect(camera.camera_info)

    detector.annotated_image.transport = LCMTransport(f"{prefix}/annotated_image", Image)
    detector.tags_json.transport = LCMTransport(f"{prefix}/tags_json", str)

    detector.start()
    return detector
