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

from typing import TYPE_CHECKING, Any, Generic

import cv2
import numpy as np
from typing_extensions import TypeVar

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.person import Detection2DPerson
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.imageDetections import ImageDetections

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.vision_msgs import Detection2DArray

T2D = TypeVar("T2D", bound=Detection2D, default=Detection2DBBox)


class ImageDetections2D(ImageDetections[T2D], Generic[T2D]):
    @classmethod
    def from_ros_detection2d_array(  # type: ignore[no-untyped-def]
        cls,
        image: Image,
        ros_detections: Detection2DArray,
        **kwargs,
    ) -> ImageDetections2D[Detection2DBBox]:
        """Convert from ROS Detection2DArray message to ImageDetections2D object."""
        detections: list[Detection2DBBox] = []
        for ros_det in ros_detections.detections:
            detection = Detection2DBBox.from_ros_detection2d(ros_det, image=image, **kwargs)
            if detection.is_valid():
                detections.append(detection)

        return ImageDetections2D(image=image, detections=detections)

    @classmethod
    def from_ultralytics_result(
        cls,
        image: Image,
        results: list[Results],
    ) -> ImageDetections2D[Detection2DBBox]:
        """Create ImageDetections2D from ultralytics Results.

        Dispatches to appropriate Detection2D subclass based on result type:
        - If masks present: creates Detection2DSeg
        - If keypoints present: creates Detection2DPerson
        - Otherwise: creates Detection2DBBox

        Args:
            image: Source image
            results: List of ultralytics Results objects

        Returns:
            ImageDetections2D containing appropriate detection types
        """

        detections: list[Detection2DBBox] = []
        for result in results:
            if result.boxes is None:
                continue

            num_detections = len(result.boxes.xyxy)
            for i in range(num_detections):
                detection: Detection2D
                if result.masks is not None:
                    # Segmentation detection with mask
                    detection = Detection2DSeg.from_ultralytics_result(result, i, image)
                elif result.keypoints is not None:
                    # Pose detection with keypoints
                    detection = Detection2DPerson.from_ultralytics_result(result, i, image)
                else:
                    # Regular bbox detection
                    detection = Detection2DBBox.from_ultralytics_result(result, i, image)
                if detection.is_valid():
                    detections.append(detection)

        return ImageDetections2D(image=image, detections=detections)  # type: ignore[return-value]

    def overlay(self, alpha: float = 0.4) -> Image:
        """Overlay detection bboxes and masks onto the image.

        Args:
            alpha: Transparency for mask overlay (default: 0.4)

        Returns:
            Image with detection bboxes, masks, and labels drawn
        """
        img = self.image.to_opencv().copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        for i, det in enumerate(self.detections):
            track_id = det.track_id if hasattr(det, "track_id") else i
            name = det.name if hasattr(det, "name") else ""
            confidence = det.confidence if hasattr(det, "confidence") else 0.0

            # Generate consistent color from track_id
            np.random.seed(abs(track_id) if track_id >= 0 else i)
            color = tuple(np.random.randint(50, 255, 3).tolist())

            # Draw mask if available
            if hasattr(det, "mask") and det.mask is not None:
                mask = det.mask
                mask_indices = mask > 0
                if np.any(mask_indices):
                    # Create colored mask overlay
                    colored_mask = np.zeros_like(img)
                    colored_mask[mask_indices] = color
                    img[mask_indices] = cv2.addWeighted(
                        img[mask_indices], 1 - alpha, colored_mask[mask_indices], alpha, 0
                    )

                    # Draw contour
                    mask_uint8 = mask.astype(np.uint8) if mask.dtype == bool else mask
                    contours, _ = cv2.findContours(
                        mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                    )
                    cv2.drawContours(img, contours, -1, color, 2)

            # Draw bbox
            x1, y1, x2, y2 = map(int, det.bbox)  # type: ignore[attr-defined]
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # Draw label
            label = f"{name}" if name else f"ID:{track_id}"
            if track_id >= 0:
                label += f":{track_id}"
            if confidence > 0:
                label += f" ({confidence:.2f})"

            (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - text_h - baseline - 2), (x1 + text_w, y1), color, -1)
            cv2.putText(
                img,
                label,
                (x1, y1 - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        return Image.from_numpy(
            img, format=self.image.format, ts=self.image.ts, frame_id=self.image.frame_id
        )
