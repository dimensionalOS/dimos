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

"""Inspect padded, organized CDR point clouds without dropping non-XYZ fields."""

import struct

from dimos_generated.sensor_msgs.msg import PointCloud2, PointField

from dimos.msgs.pointcloud import pointcloud_view, pointcloud_xyz


def main() -> None:
    for big_endian in (False, True):
        order = ">" if big_endian else "<"
        payload = bytearray(64)
        for row in range(2):
            struct.pack_into(order + "3f2H", payload, row * 32, row + 1, 2, 3, 7, 9)
        message = PointCloud2(
            height=2,
            width=1,
            point_step=20,
            row_step=32,
            is_bigendian=big_endian,
            data=bytes(payload),
            fields=[
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="tags", offset=12, datatype=4, count=2),
            ],
        )
        decoded = PointCloud2.decode(message.encode(little_endian=not big_endian))
        view = pointcloud_view(decoded)
        assert pointcloud_xyz(decoded).tolist() == [[1, 2, 3], [2, 2, 3]]
        assert view["tags"].tolist() == [[[7, 9]], [[7, 9]]]
        assert not view.flags.writeable
        copied = view.copy()
        copied["x"][0, 0] = 99
        assert view["x"][0, 0] == 1
        print(f"{'big' if big_endian else 'little'}-endian: XYZ={pointcloud_xyz(decoded).tolist()}")
        print(
            f"  organized shape={view.shape}, point/row strides={view.strides}, tags={view['tags'].tolist()}"
        )
        print("  borrowed view is read-only; changing the explicit copy leaves source x=1")


if __name__ == "__main__":
    main()
