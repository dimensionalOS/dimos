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

import gc
import struct

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.pointcloud import (
    pointcloud_from_xyz,
    pointcloud_view,
    pointcloud_xyz,
    select_points,
)


@pytest.fixture(params=[False, True], ids=["little-endian", "big-endian"])
def padded_cloud(request):
    # Two organized rows, two points per row; fields are deliberately reordered.
    endian = ">" if request.param else "<"
    payload = bytearray(128)
    for row in range(2):
        for column in range(2):
            value = row * 2 + column + 1
            struct.pack_into(
                endian + "df2Hf",
                payload,
                row * 64 + column * 24,
                value * 3.0,
                value * 1.0,
                value,
                value + 10,
                value * 2.0,
            )
    return PointCloud2(
        height=2,
        width=2,
        point_step=24,
        row_step=64,
        is_bigendian=request.param,
        data=bytes(payload),
        fields=[
            PointField(name="z", offset=0, datatype=8, count=1),
            PointField(name="x", offset=8, datatype=7, count=1),
            PointField(name="tags", offset=12, datatype=4, count=2),
            PointField(name="y", offset=16, datatype=7, count=1),
        ],
    )


def test_field_counts_padding_and_endianness(padded_cloud):
    view = pointcloud_view(padded_cloud)
    assert view["tags"].tolist() == [[[1, 11], [2, 12]], [[3, 13], [4, 14]]]
    assert pointcloud_xyz(padded_cloud).tolist() == [[1, 2, 3], [2, 4, 6], [3, 6, 9], [4, 8, 12]]
    assert not view.flags.writeable
    with pytest.raises(ValueError, match="WRITEABLE"):
        view.setflags(write=True)
    copied = view.copy()
    copied["x"][0, 0] = 99
    assert view["x"][0, 0] == 1


def test_view_retains_storage_after_message_is_deleted():
    message = PointCloud2(
        height=1,
        width=1,
        point_step=4,
        row_step=4,
        fields=[PointField(name="x", datatype=7, count=1)],
        data=struct.pack("<f", 1.5),
    )
    view = pointcloud_view(message)
    del message
    gc.collect()
    assert view["x"].tolist() == [[1.5]]


@pytest.mark.parametrize(
    "change",
    [
        {"row_step": 47},
        {"point_step": 0},
        {"point_step": 16},
        {"data": bytes(127)},
        {"fields": [PointField(name="x", datatype=9, count=1)]},
        {"fields": [PointField(name="x", datatype=7, count=0)]},
        {"fields": [PointField(name="x", datatype=7, count=1)] * 2},
    ],
)
def test_invalid_layout_rejected(padded_cloud, change):
    for name, value in change.items():
        setattr(padded_cloud, name, value)
    with pytest.raises(ValueError):
        pointcloud_view(padded_cloud)


def test_xyz_copy_is_independent(padded_cloud):
    xyz = pointcloud_xyz(padded_cloud)
    xyz[0, 0] = 99
    assert pointcloud_view(padded_cloud)["x"][0, 0] == 1
    assert xyz.dtype == np.float64


def test_missing_scalar_coordinate_rejected(padded_cloud):
    padded_cloud.fields = [field for field in padded_cloud.fields if field.name != "y"]
    with pytest.raises(ValueError, match="scalar 'y'"):
        pointcloud_xyz(padded_cloud)


@pytest.mark.parametrize("organized", [False, True])
def test_xyz_factory_copies_noncontiguous_coordinates_and_preserves_header(organized):
    values = np.arange(36, dtype=">f8").reshape(2, 6, 3)[:, ::2]
    points = values if organized else values.reshape(-1, 3)
    header = Header(frame_id="lidar", stamp=Time(sec=1700000000, nanosec=123456789))
    cloud = pointcloud_from_xyz(points, header=header)
    decoded = PointCloud2.decode(cloud.encode())
    np.testing.assert_array_equal(pointcloud_xyz(decoded), points.reshape(-1, 3))
    assert decoded.header == header
    assert decoded.is_dense and not decoded.is_bigendian
    assert (decoded.height, decoded.width) == ((2, 3) if organized else (1, 6))
    assert decoded.point_step == 12
    assert decoded.row_step == decoded.width * 12
    points[...] = -1
    assert pointcloud_xyz(cloud)[0, 0] == 0
    header.frame_id = "changed"
    assert cloud.header.frame_id == "lidar"


def test_xyz_factory_marks_missing_coordinates_and_supports_empty_cloud():
    values = np.array([[1, np.nan, 3], [np.inf, 2, 3]], dtype=np.float32)
    cloud = pointcloud_from_xyz(values, header=Header())
    assert not cloud.is_dense
    np.testing.assert_array_equal(pointcloud_xyz(cloud), values)
    empty = pointcloud_from_xyz(np.empty((0, 3)), header=Header())
    assert (empty.height, empty.width, len(empty.data)) == (1, 0, 0)


@pytest.mark.parametrize(
    "points", [np.zeros(3), np.zeros((2, 4)), np.array([["a", "b", "c"]]), np.full((1, 3), 1e100)]
)
def test_xyz_factory_rejects_invalid_shape_type_or_range(points):
    with pytest.raises(ValueError):
        pointcloud_from_xyz(points, header=Header())


def test_selection_preserves_custom_fields_padding_and_header(padded_cloud):
    padded_cloud.header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    # Nonzero point padding must survive too; row padding is discarded.
    raw = bytearray(padded_cloud.data)
    raw[20:24] = b"abcd"
    raw[108:112] = b"wxyz"
    padded_cloud.data = bytes(raw)
    result = PointCloud2.decode(
        select_points(padded_cloud, np.array([True, False, False, True])).encode()
    )
    assert bytes(result.data) == raw[:24] + raw[88:112]
    assert result.header == padded_cloud.header
    assert result.fields == padded_cloud.fields
    assert result.is_bigendian == padded_cloud.is_bigendian
    assert result.is_dense == padded_cloud.is_dense
    assert (result.height, result.width, result.row_step) == (1, 2, 48)
    np.testing.assert_array_equal(pointcloud_view(result)["tags"], [[[1, 11], [4, 14]]])
    padded_cloud.data = bytes(len(raw))
    assert bytes(result.data)[20:24] == b"abcd"


@pytest.mark.parametrize(
    "mask", [np.array([True]), np.ones((2, 2), dtype=bool), np.ones(4, dtype=int)]
)
def test_selection_rejects_mismatched_mask(padded_cloud, mask):
    with pytest.raises(ValueError, match="flat boolean mask"):
        select_points(padded_cloud, mask)


def test_selection_supports_no_retained_points(padded_cloud):
    result = select_points(padded_cloud, np.zeros(4, dtype=bool))
    assert (result.height, result.width, result.row_step, len(result.data)) == (1, 0, 0, 0)
    assert result.fields == padded_cloud.fields
