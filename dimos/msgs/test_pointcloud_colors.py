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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_rgb


@pytest.mark.parametrize("bigendian", [False, True])
@pytest.mark.parametrize("name", ["rgb", "rgba"])
@pytest.mark.parametrize("datatype", [PointField.FLOAT32, PointField.UINT32])
def test_packed_colors_preserve_bits_and_row_padding(bigendian, name, datatype):
    order = ">" if bigendian else "<"
    data = bytearray(48)
    values = np.ndarray((2, 2), dtype=order + "u4", buffer=data, offset=4, strides=(24, 8))
    values[:] = [[0xFF123456, 0x00ABCDEF], [0x00FF0000, 0xFF00FF00]]
    cloud = PointCloud2(
        height=2,
        width=2,
        point_step=8,
        row_step=24,
        is_bigendian=bigendian,
        data=bytes(data),
        fields=[PointField(name=name, offset=4, count=1, datatype=datatype)],
    )
    decoded = PointCloud2.decode(cloud.encode())
    np.testing.assert_array_equal(
        pointcloud_rgb(decoded), [[0x12, 0x34, 0x56], [0xAB, 0xCD, 0xEF], [255, 0, 0], [0, 255, 0]]
    )


def test_missing_colors():
    assert pointcloud_rgb(PointCloud2()) is None


@pytest.mark.parametrize(
    "datatype,count", [(PointField.FLOAT64, 1), (PointField.INT32, 1), (PointField.UINT32, 2)]
)
def test_invalid_color_layout(datatype, count):
    cloud = PointCloud2(
        height=1,
        width=1,
        point_step=8,
        row_step=8,
        data=bytes(8),
        fields=[PointField(name="rgb", offset=0, count=count, datatype=datatype)],
    )
    with pytest.raises(ValueError, match="packed point cloud color"):
        pointcloud_rgb(cloud)
