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

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.pointcloud import pointcloud_from_xyz_rgb, pointcloud_rgb, pointcloud_xyz
from dimos.msgs.time import time_from_nanoseconds, to_nanoseconds
from dimos.perception.memory.inventory import _absorb_into, _pixel_bbox, _split_oversized
from dimos.perception.memory.types import InventoryPolicy, SupportObservation


def _observation(offset: float, timestamp_ns: int) -> SupportObservation:
    points = np.array([[offset, 0, 1], [offset + 0.1, 0, 1]])
    cloud = pointcloud_from_xyz_rgb(
        points,
        np.array([[1, 2, 3], [4, 5, 6]], dtype=np.uint8),
        header=Header(frame_id="world", stamp=time_from_nanoseconds(timestamp_ns)),
    )
    return SupportObservation(
        ts=timestamp_ns / 1e9,
        cloud=cloud,
        centroid=points.mean(axis=0),
        aabb_min=points.min(axis=0),
        aabb_max=points.max(axis=0),
        n_points=2,
        mask_area_px=2,
        camera_position=np.zeros(3),
    )


def test_generated_support_merge_preserves_colors_and_source_time():
    first = _observation(0, 1700000000123456789)
    second = _observation(0.2, 1700000000123456790)
    _absorb_into(first, second)
    assert first.n_points == 4
    assert to_nanoseconds(first.cloud.header.stamp) == 1700000000123456790
    np.testing.assert_array_equal(
        pointcloud_rgb(first.cloud), [[1, 2, 3], [4, 5, 6], [1, 2, 3], [4, 5, 6]]
    )
    np.testing.assert_allclose(first.centroid, pointcloud_xyz(first.cloud).mean(axis=0))
    assert second.n_points == 2 and second.cloud.width == 2


def test_generated_support_projection_reads_nested_transform_and_intrinsics():
    calibration = camera_info_from_intrinsics(10, 20, 5, 6, 100, 100, header=Header())
    transform = TransformStamped(
        header=Header(frame_id="optical"),
        child_frame_id="world",
        transform=Transform(translation=Vector3(x=-1), rotation=Quaternion(w=1)),
    )
    assert _pixel_bbox(np.array([[1, 0, 2], [2, 1, 2]]), calibration, transform) == (5, 6, 10, 16)


def test_unsplit_support_returns_source_point_mask():
    points = np.array([[0, 0, 0], [0.01, 0.01, 0.01]])
    masks = _split_oversized(points, None, InventoryPolicy())
    assert len(masks) == 1
    assert masks[0].dtype == np.bool_
    np.testing.assert_array_equal(points[masks[0]], points)
