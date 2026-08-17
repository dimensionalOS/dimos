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

"""Synchronized RGB-D observations over ordinary Memory2 streams."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, TypeVar

import numpy as np

from dimos.memory2.tf import StreamTF
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.detection.type.detection3d.imageDetections3DPC import (
    ImageDetections3DPC,
)
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.perception.detection.type.detection3d.pointcloud_filters import (
        PointCloudFilter,
    )

T = TypeVar("T")


@dataclass(frozen=True)
class RGBDObservation:
    """One timestamp-aligned calibrated RGB-D camera observation."""

    color: Image
    depth: Image
    camera_info: CameraInfo
    world_to_optical: Transform

    @property
    def timestamp(self) -> float:
        return self.color.ts


def latest_rgbd(
    store: Store,
    *,
    color_stream: str,
    depth_stream: str,
    camera_info_stream: str,
    optical_frame: str,
    world_frame: str = "world",
    tolerance_s: float = 0.05,
) -> RGBDObservation:
    """Read the latest color frame and its nearest depth, calibration, and TF."""
    if tolerance_s <= 0:
        raise ValueError("tolerance_s must be positive")
    color_obs = store.stream(color_stream, Image).last()
    depth_obs = _nearest(store.stream(depth_stream, Image), color_obs.ts, tolerance_s)
    info_obs = _nearest(store.stream(camera_info_stream, CameraInfo), color_obs.ts, tolerance_s)
    color = color_obs.data
    depth = depth_obs.data
    info = info_obs.data
    if color.frame_id != optical_frame:
        raise ValueError(
            f"color frame {color.frame_id!r} does not match optical frame {optical_frame!r}"
        )
    if depth.frame_id != optical_frame or info.frame_id != optical_frame:
        raise ValueError("RGB-D calibration frame IDs are not aligned")
    if color.width != depth.width or color.height != depth.height:
        raise ValueError("color and depth image dimensions do not match")
    if info.width != color.width or info.height != color.height:
        raise ValueError("camera calibration dimensions do not match the images")
    if depth.format is not ImageFormat.DEPTH:
        raise ValueError("depth image must contain metric floating-point depth")
    tf = StreamTF.from_store(store)
    if tf is None:
        raise LookupError("Memory2 store has no recorded TF stream")
    world_to_optical = tf.get(
        optical_frame,
        world_frame,
        time_point=color_obs.ts,
        time_tolerance=tolerance_s,
    )
    if world_to_optical is None:
        raise LookupError(f"No {world_frame!r} to {optical_frame!r} transform near {color_obs.ts}")
    return RGBDObservation(color, depth, info, world_to_optical)


def project_depth(
    detections: ImageDetections2D,
    observation: RGBDObservation,
    filters: list[PointCloudFilter] | None = None,
) -> ImageDetections3DPC:
    """Lift masks or boxes from one RGB-D observation into world-frame point clouds."""
    if detections.image.ts != observation.color.ts:
        raise ValueError("detections and RGB-D observation have different timestamps")
    if detections.image.frame_id != observation.color.frame_id:
        raise ValueError("detections and RGB-D observation have different camera frames")
    return ImageDetections3DPC.from_depth(
        detections,
        observation.depth,
        observation.camera_info,
        observation.world_to_optical,
        filters,
    )


def crop_masks(
    detections: ImageDetections2D[Detection2DSeg],
    *,
    x_range: tuple[float, float] = (0.0, 1.0),
    y_range: tuple[float, float] = (0.0, 1.0),
) -> ImageDetections2D[Detection2DSeg]:
    """Keep a normalized image-space region of every segmentation mask."""
    _validate_normalized_range("x_range", x_range)
    _validate_normalized_range("y_range", y_range)
    cropped: list[Detection2DSeg] = []
    for detection in detections:
        rows, columns = np.nonzero(detection.mask)
        if not len(rows):
            continue
        x_min, x_max = int(columns.min()), int(columns.max()) + 1
        y_min, y_max = int(rows.min()), int(rows.max()) + 1
        left = x_min + math.floor((x_max - x_min) * x_range[0])
        right = x_min + math.ceil((x_max - x_min) * x_range[1])
        top = y_min + math.floor((y_max - y_min) * y_range[0])
        bottom = y_min + math.ceil((y_max - y_min) * y_range[1])
        mask = np.zeros_like(detection.mask)
        mask[top:bottom, left:right] = detection.mask[top:bottom, left:right]
        if not mask.any():
            continue
        cropped.append(
            Detection2DSeg.from_sam2_result(
                mask,
                detection.track_id,
                detections.image,
                class_id=detection.class_id,
                name=detection.name,
                confidence=detection.confidence,
            )
        )
    return ImageDetections2D(detections.image, cropped)


def _validate_normalized_range(name: str, value: tuple[float, float]) -> None:
    low, high = value
    if not 0.0 <= low < high <= 1.0:
        raise ValueError(f"{name} must satisfy 0 <= low < high <= 1")


def _nearest(stream: Stream[T], timestamp: float, tolerance_s: float):  # type: ignore[no-untyped-def]
    candidates = list(stream.at(timestamp, tolerance_s))
    if not candidates:
        raise LookupError(f"No {stream.name!r} observation near {timestamp}")
    return min(candidates, key=lambda observation: abs(observation.ts - timestamp))
