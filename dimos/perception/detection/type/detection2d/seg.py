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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import cv2
from dimos_lcm.foxglove_msgs.ImageAnnotations import PointsAnnotation
from dimos_lcm.foxglove_msgs.Point2 import Point2
import numpy as np
import torch

from dimos.msgs.foxglove_msgs.Color import Color
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.types.timestamped import to_ros_stamp

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs import Image


@dataclass
class Detection2DSeg(Detection2DBBox):
    """Represents a detection with a segmentation mask."""

    mask: np.ndarray[Any, np.dtype[np.uint8]]  # Binary mask [H, W], uint8 0 or 255

    @classmethod
    def from_sam2_result(
        cls,
        mask: np.ndarray[Any, Any] | torch.Tensor,
        obj_id: int,
        image: Image,
        class_id: int = 0,
        name: str = "object",
        confidence: float = 1.0,
    ) -> Detection2DSeg:
        """Create Detection2DSeg from SAM output (single object).

        Args:
            mask: Segmentation mask (logits or binary). Shape [H, W] or [1, H, W].
            obj_id: Tracking ID of the object.
            image: Source image.
            class_id: Class ID (default 0).
            name: Class name (default "object").
            confidence: Confidence score (default 1.0).

        Returns:
            Detection2DSeg instance.
        """
        # Convert mask to numpy if tensor
        if isinstance(mask, torch.Tensor):
            mask = mask.detach().cpu().numpy()

        # Handle dimensions (EdgeTAM might return [1, H, W] or [H, W])
        if mask.ndim == 3:
            mask = mask.squeeze()

        # Binarize if it's logits (usually < 0 is background, > 0 is foreground)
        # or if it's boolean
        if mask.dtype == bool:
            mask = mask.astype(np.uint8) * 255
        elif np.issubdtype(mask.dtype, np.floating):
            mask = (mask > 0.0).astype(np.uint8) * 255

        # Calculate bbox
        y_indices, x_indices = np.where(mask > 0)
        if len(x_indices) > 0:
            x1_val, y1_val = float(np.min(x_indices)), float(np.min(y_indices))
            x2_val, y2_val = float(np.max(x_indices)), float(np.max(y_indices))
        else:
            x1_val = y1_val = x2_val = y2_val = 0.0

        bbox = (x1_val, y1_val, x2_val, y2_val)

        return cls(
            bbox=bbox,
            track_id=obj_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
            mask=mask.astype(np.uint8),  # type: ignore[arg-type]
        )

    @classmethod
    def from_ultralytics_result(cls, result: Results, idx: int, image: Image) -> Detection2DSeg:
        """Create Detection2DSeg from ultralytics Results object with segmentation mask.

        Args:
            result: Ultralytics Results object containing detection and mask data
            idx: Index of the detection in the results
            image: Source image

        Returns:
            Detection2DSeg instance
        """
        if result.boxes is None:
            raise ValueError("Result has no boxes")

        # Extract bounding box coordinates
        bbox_array = result.boxes.xyxy[idx].cpu().numpy()
        bbox: Bbox = (
            float(bbox_array[0]),
            float(bbox_array[1]),
            float(bbox_array[2]),
            float(bbox_array[3]),
        )

        # Extract confidence
        confidence = float(result.boxes.conf[idx].cpu())

        # Extract class ID and name
        class_id = int(result.boxes.cls[idx].cpu())
        if hasattr(result, "names") and result.names is not None:
            if isinstance(result.names, dict):
                name = result.names.get(class_id, f"class_{class_id}")
            elif isinstance(result.names, list) and class_id < len(result.names):
                name = result.names[class_id]
            else:
                name = f"class_{class_id}"
        else:
            name = f"class_{class_id}"

        # Extract track ID if available
        track_id = -1
        if hasattr(result.boxes, "id") and result.boxes.id is not None:
            track_id = int(result.boxes.id[idx].cpu())

        # Extract mask
        mask = np.zeros((image.height, image.width), dtype=np.uint8)
        if result.masks is not None and idx < len(result.masks.data):
            mask_tensor = result.masks.data[idx]
            mask_np = mask_tensor.cpu().numpy()

            # Resize mask to image size if needed
            if mask_np.shape != (image.height, image.width):
                mask_np = cv2.resize(
                    mask_np.astype(np.float32),
                    (image.width, image.height),
                    interpolation=cv2.INTER_LINEAR,
                )

            # Binarize mask
            mask = (mask_np > 0.5).astype(np.uint8) * 255  # type: ignore[assignment]

        return cls(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=image.ts,
            image=image,
            mask=mask,
        )

    def to_points_annotation(self) -> list[PointsAnnotation]:
        """Override to include mask outline."""
        annotations = super().to_points_annotation()

        # Find contours
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Simplify contour to reduce points
            epsilon = 0.005 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            points = []
            for pt in approx:
                points.append(Point2(x=float(pt[0][0]), y=float(pt[0][1])))

            if len(points) < 3:
                continue

            annotations.append(
                PointsAnnotation(
                    timestamp=to_ros_stamp(self.ts),
                    outline_color=Color.from_string(str(self.class_id), alpha=1.0, brightness=1.25),
                    fill_color=Color.from_string(str(self.track_id), alpha=0.4),
                    thickness=1.0,
                    points_length=len(points),
                    points=points,
                    type=PointsAnnotation.LINE_LOOP,
                )
            )

        return annotations
