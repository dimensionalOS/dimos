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

from __future__ import annotations

from collections.abc import Iterable
import math
from typing import Any

import numpy as np

from dimos.memory.type.observation import EmbeddedObservation, Observation
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

# Rerun voxel render size at each replay tool's default voxel size.
DEFAULT_RENDER_VOXEL = 0.05

PairObs = Observation[tuple[Observation[PointCloud2], Observation[Odometry]]]


def default_render_voxel(voxel_size: float, default_voxel_size: float) -> float:
    """Rerun voxel render size, scaled with the configured voxel size."""
    return DEFAULT_RENDER_VOXEL * (voxel_size / default_voxel_size)


def attach_pose_from_odom(pair_obs: PairObs) -> Observation[PointCloud2]:
    """Tag an aligned lidar observation with its paired odometry pose."""
    lidar_obs, odom_obs = pair_obs.data
    odom = odom_obs.data
    pose_tuple = (
        float(odom.position.x),
        float(odom.position.y),
        float(odom.position.z),
        float(odom.orientation.x),
        float(odom.orientation.y),
        float(odom.orientation.z),
        float(odom.orientation.w),
    )
    return lidar_obs.with_pose(pose_tuple)


def mosaic(
    frames: Iterable[Image | Detection2D | Observation[Any]],
    cols: int = 3,
    cell_height: int = 160,
) -> Observation[Image]:
    """Tile images into a grid mosaic.

    Accepts Image instances, Observation/EmbeddedObservation with Image data,
    or any iterable of these (including Stream).  Returns a poseless
    Observation[Image] tagged ``{"mosaic": True}`` — the rerun renderer
    displays poseless image observations as flat 2D panels.
    """
    import cv2

    images: list[Image] = []
    for f in frames:
        if isinstance(f, Image):
            images.append(f)
        elif isinstance(f, ImageDetections2D):
            images.append(f.annotated_image(scale=4))
        elif isinstance(f, Observation) and isinstance(f.data, Image):
            images.append(f.data)
        elif isinstance(f, EmbeddedObservation) and isinstance(f.data, Image):
            images.append(f.data)
        elif isinstance(f, Observation) and isinstance(f.data, ImageDetections2D):
            images.append(f.data.annotated_image(scale=4))
        elif isinstance(f, EmbeddedObservation) and isinstance(f.data, ImageDetections2D):
            images.append(f.data.annotated_image(scale=4))
        else:
            raise TypeError(f"Cannot extract Image from {type(f).__name__}: {f!r}")
    if not images:
        raise ValueError("No images to mosaic")

    aspect = images[0].width / max(images[0].height, 1)
    cell_w = int(cell_height * aspect)
    rows = math.ceil(len(images) / cols)

    canvas = np.zeros((rows * cell_height, cols * cell_w, 3), dtype=np.uint8)
    for i, img in enumerate(images):
        r, c = divmod(i, cols)
        tile = cv2.resize(img.to_bgr().data, (cell_w, cell_height))
        canvas[r * cell_height : (r + 1) * cell_height, c * cell_w : (c + 1) * cell_w] = tile

    result = Image(data=canvas, format=ImageFormat.BGR)
    return Observation(id=0, ts=0.0, data_type=Image, _data=result, tags={"mosaic": True})
