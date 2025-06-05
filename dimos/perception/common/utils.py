# Copyright 2025 Dimensional Inc.
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

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.common.utils")


def colorize_depth(depth_img: np.ndarray, max_depth: float = 5.0) -> Optional[np.ndarray]:
    """
    Normalize and colorize depth image using COLORMAP_JET.

    Args:
        depth_img: Depth image (H, W) in meters
        max_depth: Maximum depth value for normalization

    Returns:
        Colorized depth image (H, W, 3) in RGB format, or None if input is None
    """
    if depth_img is None:
        return None

    valid_mask = np.isfinite(depth_img) & (depth_img > 0)
    depth_norm = np.zeros_like(depth_img)
    depth_norm[valid_mask] = np.clip(depth_img[valid_mask] / max_depth, 0, 1)
    depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)

    # Make the depth image less bright by scaling down the values
    depth_rgb = (depth_rgb * 0.6).astype(np.uint8)

    return depth_rgb


def draw_bounding_box(
    image: np.ndarray,
    bbox: List[float],
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
    label: Optional[str] = None,
    confidence: Optional[float] = None,
    object_id: Optional[int] = None,
    font_scale: float = 0.6,
) -> np.ndarray:
    """
    Draw a bounding box with optional label on an image.

    Args:
        image: Image to draw on (H, W, 3)
        bbox: Bounding box [x1, y1, x2, y2]
        color: RGB color tuple for the box
        thickness: Line thickness for the box
        label: Optional class label
        confidence: Optional confidence score
        object_id: Optional object ID
        font_scale: Font scale for text

    Returns:
        Image with bounding box drawn
    """
    x1, y1, x2, y2 = map(int, bbox)

    # Draw bounding box
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

    # Create label text
    text_parts = []
    if label is not None:
        text_parts.append(str(label))
    if object_id is not None:
        text_parts.append(f"ID: {object_id}")
    if confidence is not None:
        text_parts.append(f"({confidence:.2f})")

    if text_parts:
        text = ", ".join(text_parts)

        # Draw text background
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)[0]
        cv2.rectangle(
            image,
            (x1, y1 - text_size[1] - 5),
            (x1 + text_size[0], y1),
            (0, 0, 0),
            -1,
        )

        # Draw text
        cv2.putText(
            image,
            text,
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            1,
        )

    return image


def draw_segmentation_mask(
    image: np.ndarray,
    mask: np.ndarray,
    color: Tuple[int, int, int] = (0, 200, 200),
    alpha: float = 0.5,
    draw_contours: bool = True,
    contour_thickness: int = 2,
) -> np.ndarray:
    """
    Draw segmentation mask overlay on an image.

    Args:
        image: Image to draw on (H, W, 3)
        mask: Segmentation mask (H, W) - boolean or uint8
        color: RGB color for the mask
        alpha: Transparency factor (0.0 = transparent, 1.0 = opaque)
        draw_contours: Whether to draw mask contours
        contour_thickness: Thickness of contour lines

    Returns:
        Image with mask overlay drawn
    """
    if mask is None:
        return image

    try:
        # Ensure mask is uint8
        mask = mask.astype(np.uint8)

        # Create colored mask overlay
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = color

        # Apply the mask with transparency
        mask_area = mask > 0
        image[mask_area] = cv2.addWeighted(
            image[mask_area], 1 - alpha, colored_mask[mask_area], alpha, 0
        )

        # Draw mask contours if requested
        if draw_contours:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, contour_thickness)

    except Exception as e:
        logger.warning(f"Error drawing segmentation mask: {e}")

    return image


def draw_object_detection_visualization(
    image: np.ndarray,
    objects: List[ObjectData],
    draw_masks: bool = False,
    bbox_color: Tuple[int, int, int] = (0, 255, 0),
    mask_color: Tuple[int, int, int] = (0, 200, 200),
    font_scale: float = 0.6,
) -> np.ndarray:
    """
    Create object detection visualization with bounding boxes and optional masks.

    Args:
        image: Base image to draw on (H, W, 3)
        objects: List of ObjectData with detection information
        draw_masks: Whether to draw segmentation masks
        bbox_color: Default color for bounding boxes
        mask_color: Default color for segmentation masks
        font_scale: Font scale for text labels

    Returns:
        Image with detection visualization
    """
    viz_image = image.copy()

    for obj in objects:
        try:
            # Draw segmentation mask first (if enabled and available)
            if draw_masks and "segmentation_mask" in obj and obj["segmentation_mask"] is not None:
                viz_image = draw_segmentation_mask(
                    viz_image, obj["segmentation_mask"], color=mask_color, alpha=0.5
                )

            # Draw bounding box
            if "bbox" in obj and obj["bbox"] is not None:
                # Use object's color if available, otherwise default
                color = bbox_color
                if "color" in obj and obj["color"] is not None:
                    obj_color = obj["color"]
                    if isinstance(obj_color, np.ndarray):
                        color = tuple(int(c) for c in obj_color)
                    elif isinstance(obj_color, (list, tuple)):
                        color = tuple(int(c) for c in obj_color[:3])

                viz_image = draw_bounding_box(
                    viz_image,
                    obj["bbox"],
                    color=color,
                    label=obj.get("label"),
                    confidence=obj.get("confidence"),
                    object_id=obj.get("object_id"),
                    font_scale=font_scale,
                )

        except Exception as e:
            logger.warning(f"Error drawing object visualization: {e}")

    return viz_image
