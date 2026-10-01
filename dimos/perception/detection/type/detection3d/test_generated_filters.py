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

from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped
from dimos_generated.sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_view
from dimos.perception.detection.type.detection3d.pointcloud_filters import (
    height_filter,
    radius_outlier,
    raycast,
    statistical,
)


@pytest.mark.parametrize("make_filter", [height_filter, statistical, radius_outlier, raycast])
def test_filters_preserve_custom_point_records(make_filter):
    rng = np.random.default_rng(42)
    values = np.zeros(101, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<u4")])
    xyz = rng.normal(scale=0.1, size=(101, 3)) + np.array([0, 0, 2])
    xyz[-1] = [10, 10, -1]
    for i, name in enumerate(("x", "y", "z")):
        values[name] = xyz[:, i]
    values["id"] = np.arange(101)
    cloud = PointCloud2(
        header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
        height=1,
        width=101,
        point_step=16,
        row_step=1616,
        fields=[
            PointField(name=name, offset=i * 4, datatype=7 if i < 3 else 6, count=1)
            for i, name in enumerate(("x", "y", "z", "id"))
        ],
        data=values.tobytes(),
    )
    transform = TransformStamped(
        header=Header(frame_id="camera"),
        child_frame_id="world",
        transform=Transform(rotation=Quaternion(w=1)),
    )
    result = make_filter()(
        SimpleNamespace(), PointCloud2.decode(cloud.encode()), CameraInfo(), transform
    )
    assert result is not None
    result = PointCloud2.decode(result.encode())
    assert result.header == cloud.header
    assert result.fields == cloud.fields
    records = pointcloud_view(result).reshape(-1)
    assert 0 < len(records) < len(values)
    np.testing.assert_array_equal(records, values[records["id"]])
