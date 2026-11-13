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

"""Pure validation and per-instance depth quality at the RGB-D boundary."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg


def validate_rgbd(
    color: Image, depth: Image, info: CameraInfo, now: float, max_age: float, max_skew: float
) -> None:
    """Reject stale, unsynchronized or geometrically incompatible input."""
    if not all(math.isfinite(t) and 0 <= now - t <= max_age for t in (color.ts, depth.ts)):
        raise ValueError("RGB-D capture is stale or has an invalid timestamp")
    if abs(color.ts - depth.ts) > max_skew:
        raise ValueError("RGB-D capture timestamps do not match")
    if not color.frame_id or color.frame_id != depth.frame_id or info.frame_id != color.frame_id:
        raise ValueError("RGB-D and intrinsics must use one rectified optical frame")
    if color.data.shape[:2] != depth.data.shape or depth.data.shape != (info.height, info.width):
        raise ValueError("RGB-D dimensions do not match intrinsics")
    k = np.asarray(info.K, dtype=np.float64)
    if k.size != 9 or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0:
        raise ValueError("Camera intrinsics must have finite positive focal lengths")


@dataclass(frozen=True)
class DepthQuality:
    name: str
    mask_pixels: int
    valid_depth_pixels: int
    valid_depth_fraction: float
    accepted: bool
    reason: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def filter_depth_detections(
    detections: ImageDetections2D[Any],
    depth: NDArray[np.float32],
    min_fraction: float,
    min_pixels: int,
    require_mask: bool,
) -> tuple[ImageDetections2D[Any], list[DepthQuality], NDArray[np.uint8]]:
    """Measure eroded foreground support, preserving 2D-only failures in the overlay."""
    overlay = detections.image.to_rgb().data.copy()
    kept: list[Detection2DBBox] = []
    reports: list[DepthQuality] = []
    h, w = depth.shape
    for detection in detections.detections:
        mask: NDArray[np.uint8] = np.zeros((h, w), np.uint8)
        reason = ""
        if isinstance(detection, Detection2DSeg):
            if detection.mask.shape != depth.shape:
                reason = "mask dimensions do not match depth"
            else:
                mask = np.asarray(detection.mask > 0, dtype=np.uint8)
        elif require_mask:
            reason = "segmentation mask required"
        else:
            x1, y1, x2, y2 = map(int, detection.bbox)
            mask[
                min(h, max(0, y1)) : min(h, max(0, y2)), min(w, max(0, x1)) : min(w, max(0, x2))
            ] = 1
        mask = np.asarray(
            cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))), dtype=np.uint8
        )
        count = int(np.count_nonzero(mask))
        valid = int(np.count_nonzero((mask > 0) & np.isfinite(depth) & (depth > 0)))
        fraction = valid / count if count else 0.0
        if not reason and (not count or valid < min_pixels or fraction < min_fraction):
            reason = "insufficient valid foreground depth"
        accepted = not reason
        reports.append(DepthQuality(detection.name, count, valid, fraction, accepted, reason))
        if accepted:
            kept.append(detection)
        tint = np.array([40, 220, 90] if accepted else [255, 70, 50], np.uint8)
        foreground = mask > 0
        overlay[foreground] = (overlay[foreground].astype(np.float32) * 0.6 + tint * 0.4).astype(
            np.uint8
        )
        x1, y1, x2, y2 = map(int, detection.bbox)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), tuple(map(int, tint)), 1)
        cv2.putText(
            overlay,
            f"{detection.name}: {fraction:.0%} depth" + ("" if accepted else " REJECTED"),
            (max(0, x1), max(16, y1)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            tuple(map(int, tint)),
            1,
        )
    return ImageDetections2D(detections.image, kept), reports, overlay
