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

from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_view, pointcloud_xyz


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
