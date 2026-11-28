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

import numpy as np

from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox

if TYPE_CHECKING:
    import torch

    from dimos.msgs.sensor_msgs.Image import Image


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
        import torch

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
            mask=mask.astype(np.uint8),
        )
