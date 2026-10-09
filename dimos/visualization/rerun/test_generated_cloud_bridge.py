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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
import rerun as rr

from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_from_xyz_rgb
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
        header=Header(frame_id="lidar", stamp=Time(sec=0, nanosec=0)),
        height=2,
        width=1,
        point_step=16,
        row_step=20,
        is_bigendian=True,
        fields=fields,
        data=np.frombuffer(data, dtype=np.uint8),
        is_dense=False,
    )
    decoded = cdr_decode(cdr_encode(cloud), PointCloud2)
    before = cdr_encode(decoded)
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
    assert cdr_encode(decoded) == before


def test_height_colors_and_empty_cloud() -> None:
    cloud = pointcloud_from_xyz(
        np.array([[0, 0, 0], [0, 0, 2]], dtype=np.float32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    rendered = cloud_archetype(cloud)
    assert len(set(rendered.colors.as_arrow_array().to_pylist())) == 2
    empty = pointcloud_from_xyz(
        np.empty((0, 3)), header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    )
    assert cloud_archetype(empty).positions.as_arrow_array().to_pylist() == []


@pytest.mark.parametrize("mode", ["points", "spheres", "boxes"])
def test_cloud_modes_filter_height_without_losing_rgb_alignment(mode):
    cloud = pointcloud_from_xyz_rgb(
        np.array([[0, 0, -1], [1, 2, 3]], dtype=np.float32),
        np.array([[255, 0, 0], [0x12, 0x34, 0x56]], dtype=np.uint8),
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
    )
    before = cdr_encode(cloud)
    rendered = cloud_archetype(cloud, mode=mode, bottom_cutoff=0, voxel_size=0.2)
    positions = rendered.centers if mode == "boxes" else rendered.positions
    assert positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert rendered.colors.as_arrow_array().to_pylist() == [0x123456FF]
    if mode == "points":
        assert rendered.radii.as_arrow_array().to_pylist() == [-2.0]
    assert cdr_encode(cloud) == before


@pytest.mark.parametrize("mode", ["points", "boxes"])
def test_explicit_cloud_colors_follow_finite_and_height_filters(mode):
    cloud = pointcloud_from_xyz(
        np.array([[0, 0, -1], [np.nan, 0, 1], [1, 2, 3]], dtype=np.float32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    rendered = cloud_archetype(
        cloud,
        mode=mode,
        bottom_cutoff=0,
        colors=np.array([[255, 0, 0, 255], [0, 255, 0, 255], [18, 52, 86, 120]], dtype=np.uint8),
    )
    assert rendered.colors.as_arrow_array().to_pylist() == [0x12345678]
    uniform = cloud_archetype(cloud, bottom_cutoff=0, colors=[18, 52, 86])
    assert uniform.colors.as_arrow_array().to_pylist() == [0x123456FF]


@pytest.mark.parametrize("colors", [[256, 0, 0], [-1, 0, 0], [0.5, 0, 0], [[1, 2]]])
def test_explicit_cloud_colors_reject_invalid_channels(colors):
    cloud = pointcloud_from_xyz(
        np.array([[0, 0, 1]], dtype=np.float32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    with pytest.raises(ValueError, match="color"):
        cloud_archetype(cloud, colors=colors)
