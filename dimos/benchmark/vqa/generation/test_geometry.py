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

import numpy as np
import pytest

from dimos.benchmark.vqa.generation.geometry import project_visible_points
from dimos.benchmark.vqa.models import CalibratedFrame
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _frame(points: np.ndarray, *, rectified: bool = True) -> CalibratedFrame:
    return CalibratedFrame(
        id="frame-1",
        image=Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8)),
        pointcloud=PointCloud2.from_numpy(points),
        camera_info=CameraInfo.from_intrinsics(2.0, 2.0, 2.0, 2.0, 4, 4),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=rectified,
    )


def test_project_visible_points_keeps_nearest_point_per_pixel() -> None:
    frame = _frame(
        np.array(
            [
                [0.0, 0.0, 2.0],
                [0.0, 0.0, 1.0],
                [0.5, 0.0, 1.0],
                [0.0, 0.0, -1.0],
            ],
            dtype=np.float32,
        )
    )

    projected = project_visible_points(frame)

    assert projected.pixels == [(2, 2), (3, 2)]
    assert projected.source_indices == [1, 2]
    assert projected.camera_points == [(0.0, 0.0, 1.0), (0.5, 0.0, 1.0)]


def test_project_visible_points_rejects_unrectified_images() -> None:
    with pytest.raises(ValueError, match="rectified"):
        project_visible_points(_frame(np.zeros((1, 3), dtype=np.float32), rectified=False))
