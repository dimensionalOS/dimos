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

import struct
from types import SimpleNamespace
from unittest.mock import patch

from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
import rerun as rr

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import cloud_archetype

pytestmark = pytest.mark.filterwarnings("error::rerun.error_utils.RerunWarning")


def test_padded_big_endian_cloud_keeps_color_alignment_and_frame() -> None:
    fields = [
        PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z", "rgb"))
    ]
    data = (
        struct.pack(">3fI", 1, 2, 3, 0x123456)
        + b"pad!"
        + struct.pack(">3fI", float("nan"), 5, 6, 0xFFFFFF)
        + b"pad!"
    )
    cloud = PointCloud2(
        header=Header(frame_id="lidar"),
        height=2,
        width=1,
        point_step=16,
        row_step=20,
        is_bigendian=True,
        fields=fields,
        data=data,
    )
    decoded = PointCloud2.decode(cloud.encode())
    before = decoded.encode()
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    try:
        with patch("rerun.log") as log:
            bridge._on_message(decoded, SimpleNamespace(name="/lidar"))
            rendered = log.call_args_list[0].args[1]
            assert isinstance(rendered, rr.Points3D)
            assert rendered.positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
            assert rendered.colors.as_arrow_array().to_pylist() == [0x123456FF]
            assert log.call_args_list[1].args[1].parent_frame.as_arrow_array().to_pylist() == [
                "tf#/lidar"
            ]
    finally:
        bridge.stop()
    assert decoded.encode() == before


def test_height_colors_and_empty_cloud() -> None:
    cloud = pointcloud_from_xyz(np.array([[0, 0, 0], [0, 0, 2]], dtype=np.float32), header=Header())
    rendered = cloud_archetype(cloud)
    assert len(set(rendered.colors.as_arrow_array().to_pylist())) == 2
    empty = pointcloud_from_xyz(np.empty((0, 3)), header=Header())
    assert cloud_archetype(empty).positions.as_arrow_array().to_pylist() == []
