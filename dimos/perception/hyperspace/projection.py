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

"""Project patch-embedding grids into 3D via a depth image.

Depth pixels are unprojected with the depth camera intrinsics, reprojected
into the color camera to pick each point's patch cell, and transformed to the
world frame. The depth and color cameras may be distinct (unaligned depth) —
pass the color-from-depth extrinsic; for depth already aligned to color pass
identity.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.models.embedding.base import PatchEmbeddings
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image


def depth_to_meters(depth: Image) -> NDArray[np.float32]:
    """Depth image data as float32 meters (uint16 inputs are millimeters)."""
    data = np.asarray(depth.data)
    meters = data.astype(np.float32)
    if data.dtype == np.uint16:
        meters *= 0.001
    return meters


def project_patches_to_world(
    patches: PatchEmbeddings,
    depth: Image,
    depth_camera_info: CameraInfo,
    color_camera_info: CameraInfo,
    color_from_depth: NDArray[np.floating],
    world_from_depth: NDArray[np.floating],
    stride: int = 4,
    min_depth_m: float = 0.3,
    max_depth_m: float = 6.0,
) -> tuple[NDArray[np.float32], NDArray[np.int64]]:
    """Turn one (patch grid, depth frame) pair into world points + patch rows.

    Returns (points_world (N, 3) float32, patch_indices (N,) into
    ``patches.flat()``). N is the count of valid depth samples whose
    reprojection lands inside the color image.
    """
    depth_m = depth_to_meters(depth)
    h, w = depth_m.shape[:2]

    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    us = us.reshape(-1)
    vs = vs.reshape(-1)
    z = depth_m[vs, us]

    valid = (z > min_depth_m) & (z < max_depth_m) & np.isfinite(z)
    us, vs, z = us[valid], vs[valid], z[valid]
    if z.size == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.int64)

    k = depth_camera_info.K
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    points_depth = np.stack(
        [(us - cx) / fx * z, (vs - cy) / fy * z, z],
        axis=1,
    ).astype(np.float32)

    # Reproject into the color camera to find each point's patch cell.
    color_from_depth = np.asarray(color_from_depth, dtype=np.float64)
    points_color = points_depth @ color_from_depth[:3, :3].T + color_from_depth[:3, 3]
    in_front = points_color[:, 2] > 1e-6
    points_depth, points_color = points_depth[in_front], points_color[in_front]
    if points_depth.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.int64)

    kc = color_camera_info.K
    u_color = kc[0] * points_color[:, 0] / points_color[:, 2] + kc[2]
    v_color = kc[4] * points_color[:, 1] / points_color[:, 2] + kc[5]
    in_image = (
        (u_color >= 0)
        & (u_color < color_camera_info.width)
        & (v_color >= 0)
        & (v_color < color_camera_info.height)
    )
    points_depth = points_depth[in_image]
    if points_depth.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.int64)

    patch_indices = patches.patch_index_for_pixel(u_color[in_image], v_color[in_image])

    world_from_depth = np.asarray(world_from_depth, dtype=np.float64)
    points_world = points_depth @ world_from_depth[:3, :3].T + world_from_depth[:3, 3]
    return points_world.astype(np.float32), patch_indices
