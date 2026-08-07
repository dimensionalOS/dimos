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

"""Static calibrated point-cloud projection for single-frame VQA."""

from __future__ import annotations

import numpy as np

from dimos.benchmark.vqa.models import CalibratedFrame, ProjectedPoints, ProjectionConfig


def project_visible_points(
    frame: CalibratedFrame, config: ProjectionConfig = ProjectionConfig()
) -> ProjectedPoints:
    """Project the nearest point at each image pixel using static calibration.

    The supplied transform maps point-cloud coordinates to the camera optical
    frame. This function intentionally has no time or TF dependency.
    """
    if not frame.image_is_rectified:
        raise ValueError("VQA projection requires a rectified pinhole image")
    if config.min_depth_m <= 0:
        raise ValueError("min_depth_m must be positive")

    camera_info = frame.camera_info
    if camera_info.width != frame.image.width or camera_info.height != frame.image.height:
        raise ValueError("camera intrinsics dimensions must match the image")
    if len(camera_info.K) != 9:
        raise ValueError("camera intrinsics must contain a 3x3 matrix")

    points, _ = frame.pointcloud.as_numpy()
    if len(points) == 0:
        return ProjectedPoints([], [], [])

    homogeneous = np.column_stack((points, np.ones(len(points), dtype=points.dtype)))
    camera_points = (frame.pointcloud_to_camera.to_matrix() @ homogeneous.T).T[:, :3]
    source_indices = np.arange(len(points))

    depth_mask = camera_points[:, 2] >= config.min_depth_m
    camera_points = camera_points[depth_mask]
    source_indices = source_indices[depth_mask]
    if len(camera_points) == 0:
        return ProjectedPoints([], [], [])

    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    if fx <= 0 or fy <= 0:
        raise ValueError("camera focal lengths must be positive")

    u = np.floor(fx * camera_points[:, 0] / camera_points[:, 2] + cx).astype(np.int64)
    v = np.floor(fy * camera_points[:, 1] / camera_points[:, 2] + cy).astype(np.int64)
    in_image = (u >= 0) & (u < frame.image.width) & (v >= 0) & (v < frame.image.height)
    camera_points = camera_points[in_image]
    source_indices = source_indices[in_image]
    u = u[in_image]
    v = v[in_image]
    if len(camera_points) == 0:
        return ProjectedPoints([], [], [])

    pixel_ids = v * frame.image.width + u
    nearest_first = np.lexsort((camera_points[:, 2], pixel_ids))
    first_per_pixel = np.concatenate(
        ([True], pixel_ids[nearest_first][1:] != pixel_ids[nearest_first][:-1])
    )
    visible = nearest_first[first_per_pixel]

    return ProjectedPoints(
        camera_points=[
            (float(point[0]), float(point[1]), float(point[2])) for point in camera_points[visible]
        ],
        pixels=[(int(x), int(y)) for x, y in zip(u[visible], v[visible], strict=True)],
        source_indices=[int(index) for index in source_indices[visible]],
    )
