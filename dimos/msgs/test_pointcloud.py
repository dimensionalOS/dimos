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
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import PointCloud2, PointField
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import (
    concatenate_clouds,
    pointcloud_from_rgbd,
    pointcloud_from_xyz,
    pointcloud_from_xyz_rgb,
    pointcloud_rgb,
    pointcloud_stamps,
    pointcloud_view,
    pointcloud_xyz,
    select_points,
    transform_cloud,
    voxel_downsample_cloud,
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
        data=np.frombuffer(bytes(payload), dtype=np.uint8),
        fields=[
            PointField(name="z", offset=0, datatype=8, count=1),
            PointField(name="x", offset=8, datatype=7, count=1),
            PointField(name="tags", offset=12, datatype=4, count=2),
            PointField(name="y", offset=16, datatype=7, count=1),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_dense=False,
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
        fields=[PointField(name="x", datatype=7, count=1, offset=0)],
        data=np.frombuffer(struct.pack("<f", 1.5), dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        is_bigendian=False,
        is_dense=False,
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
        {"fields": [PointField(name="x", datatype=9, count=1, offset=0)]},
        {"fields": [PointField(name="x", datatype=7, count=0, offset=0)]},
        {"fields": [PointField(name="x", datatype=7, count=1, offset=0)] * 2},
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
    decoded = cdr_decode(cdr_encode(cloud), PointCloud2)
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
    cloud = pointcloud_from_xyz(values, header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""))
    assert not cloud.is_dense
    np.testing.assert_array_equal(pointcloud_xyz(cloud), values)
    empty = pointcloud_from_xyz(
        np.empty((0, 3)), header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    )
    assert (empty.height, empty.width, len(empty.data)) == (1, 0, 0)


@pytest.mark.parametrize(
    "points", [np.zeros(3), np.zeros((2, 4)), np.array([["a", "b", "c"]]), np.full((1, 3), 1e100)]
)
def test_xyz_factory_rejects_invalid_shape_type_or_range(points):
    with pytest.raises(ValueError):
        pointcloud_from_xyz(points, header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""))


def test_selection_preserves_custom_fields_padding_and_header(padded_cloud):
    padded_cloud.header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    # Nonzero point padding must survive too; row padding is discarded.
    raw = bytearray(padded_cloud.data)
    raw[20:24] = b"abcd"
    raw[108:112] = b"wxyz"
    padded_cloud.data = bytes(raw)
    result = cdr_decode(
        cdr_encode(select_points(padded_cloud, np.array([True, False, False, True]))),
        PointCloud2,
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


def test_cloud_concatenation_preserves_all_records_and_newest_nanosecond(padded_cloud):
    first = cdr_decode(cdr_encode(padded_cloud), PointCloud2)
    first.header = Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789))
    second = cdr_decode(cdr_encode(first), PointCloud2)
    second.header.stamp.nanosec += 1
    second.is_dense = False
    result = cdr_decode(cdr_encode(concatenate_clouds(first, second)), PointCloud2)
    assert (result.height, result.width, result.row_step) == (1, 8, 8 * 24)
    assert result.fields == first.fields
    assert result.is_bigendian == first.is_bigendian
    assert result.header.frame_id == "world"
    assert result.header.stamp.nanosec == 123456790
    assert not result.is_dense
    packed = select_points(first, np.ones(4, dtype=bool))
    assert bytes(result.data) == bytes(packed.data) * 2
    np.testing.assert_array_equal(
        pointcloud_view(result)["tags"].reshape(-1, 2),
        np.tile([[1, 11], [2, 12], [3, 13], [4, 14]], (2, 1)),
    )
    result.data = bytes(len(result.data))
    assert bytes(first.data) == bytes(padded_cloud.data)


def test_cloud_concatenation_rejects_unrelated_frames_and_layouts(padded_cloud):
    second = cdr_decode(cdr_encode(padded_cloud), PointCloud2)
    second.header.frame_id = "unrelated"
    with pytest.raises(ValueError, match="different frames"):
        concatenate_clouds(padded_cloud, second)
    second.header.frame_id = padded_cloud.header.frame_id
    second.is_bigendian = not second.is_bigendian
    with pytest.raises(ValueError, match="different record layouts"):
        concatenate_clouds(padded_cloud, second)


def test_empty_cloud_is_a_copy_identity(padded_cloud):
    result = concatenate_clouds(
        PointCloud2(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            height=0,
            width=0,
            fields=[],
            is_bigendian=False,
            point_step=0,
            row_step=0,
            data=np.array([], dtype=np.uint8),
            is_dense=False,
        ),
        padded_cloud,
    )
    assert cdr_encode(result) == cdr_encode(padded_cloud)
    result.header.frame_id = "changed"
    assert padded_cloud.header.frame_id == ""


def test_transform_retains_extra_fields_padding_endian_and_source_stamp(padded_cloud):
    padded_cloud.header = Header(
        frame_id="sensor", stamp=Time(sec=1_700_000_000, nanosec=123_456_789)
    )
    original = bytes(padded_cloud.data)
    transform = TransformStamped(
        header=Header(frame_id="world", stamp=Time(sec=99, nanosec=0)),
        child_frame_id="sensor",
        transform=Transform(
            translation=Vector3(x=2.0, y=-1.0, z=0.0),
            rotation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
    )
    result = transform_cloud(padded_cloud, transform)
    np.testing.assert_allclose(
        pointcloud_xyz(result), pointcloud_xyz(padded_cloud) + np.array([2.0, -1.0, 0.0])
    )
    np.testing.assert_array_equal(
        pointcloud_view(result)["tags"], pointcloud_view(padded_cloud)["tags"]
    )
    assert result.header.frame_id == "world"
    assert result.header.stamp == padded_cloud.header.stamp
    assert (
        result.width,
        result.height,
        result.row_step,
        result.point_step,
        result.is_bigendian,
    ) == (
        padded_cloud.width,
        padded_cloud.height,
        padded_cloud.row_step,
        padded_cloud.point_step,
        padded_cloud.is_bigendian,
    )
    for row in range(2):
        assert (
            bytes(result.data)[row * 64 + 48 : row * 64 + 64]
            == original[row * 64 + 48 : row * 64 + 64]
        )
        for column in range(2):
            offset = row * 64 + column * 24
            assert (
                bytes(result.data)[offset + 20 : offset + 24] == original[offset + 20 : offset + 24]
            )
    assert bytes(padded_cloud.data) == original
    result.header.stamp.nanosec = 0
    assert padded_cloud.header.stamp.nanosec == 123_456_789


def test_transform_rejects_frame_mismatch(padded_cloud):
    with pytest.raises(ValueError, match="child frame"):
        transform_cloud(
            padded_cloud,
            TransformStamped(
                child_frame_id="other",
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                transform=Transform(
                    translation=Vector3(x=0.0, y=0.0, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
        )


@pytest.mark.parametrize("organized", [False, True])
def test_rgb_factory_roundtrip(organized):
    points = np.arange(18, dtype=np.float64).reshape(2, 3, 3)
    colors = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    if not organized:
        points, colors = points.reshape(-1, 3), colors.reshape(-1, 3)
    cloud = cdr_decode(
        cdr_encode(
            pointcloud_from_xyz_rgb(
                points, colors, header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")
            )
        ),
        PointCloud2,
    )
    np.testing.assert_array_equal(pointcloud_xyz(cloud), points.reshape(-1, 3))
    np.testing.assert_array_equal(pointcloud_rgb(cloud), colors.reshape(-1, 3))
    assert cloud.point_step == 16
    assert cloud.row_step == cloud.width * 16


@pytest.mark.parametrize("floating", [False, True])
def test_rgbd_projection_scaled_intrinsics_and_invalid_depth(floating):
    header = Header(frame_id="optical", stamp=Time(sec=1700000000, nanosec=123456789))
    rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    color = image_from_array(rgb, encoding="rgb8", header=header)
    values = np.array([[1, 0, 2], [3, 4, 5]], dtype=np.float32)
    if floating:
        values[0, 1] = np.nan
        values[1, 2] = np.inf
        depth = image_from_array(values, encoding="32FC1", header=header)
    else:
        depth = image_from_array((values * 1000).astype(">u2"), encoding="16UC1", header=header)
        # Add nonzero row padding to big-endian depth.
        payload = bytes(depth.data)
        depth.data = payload[:6] + b"xx" + payload[6:] + b"yy"
        depth.step = 8
    calibration = camera_info_from_intrinsics(4, 8, 2, 2, 6, 4, header=header)
    cloud = cdr_decode(
        cdr_encode(
            pointcloud_from_rgbd(color, depth, calibration, depth_scale=0.001, depth_trunc=4)
        ),
        PointCloud2,
    )
    np.testing.assert_allclose(
        pointcloud_xyz(cloud), [[-0.5, -0.25, 1], [1, -0.5, 2], [-1.5, 0, 3]]
    )
    np.testing.assert_array_equal(pointcloud_rgb(cloud), rgb.reshape(-1, 3)[[0, 2, 3]])
    assert cloud.header == header


@pytest.mark.parametrize(
    "scale,trunc", [(0, 1), (-1, 1), (float("nan"), 1), (1, 0), (1, float("inf"))]
)
def test_rgbd_rejects_invalid_units(scale, trunc):
    header = Header(stamp=Time(sec=0, nanosec=0), frame_id="")
    color = image_from_array(np.zeros((1, 1, 3), dtype=np.uint8), encoding="rgb8", header=header)
    depth = image_from_array(np.ones((1, 1), dtype=np.uint16), encoding="16UC1", header=header)
    calibration = camera_info_from_intrinsics(1, 1, 0, 0, 1, 1, header=header)
    with pytest.raises(ValueError, match="finite and positive"):
        pointcloud_from_rgbd(color, depth, calibration, depth_scale=scale, depth_trunc=trunc)


def test_transform_rejects_coordinate_overflow():
    cloud = pointcloud_from_xyz(
        np.array([[3e38, 0, 0]]), header=Header(frame_id="child", stamp=Time(sec=0, nanosec=0))
    )
    transform = TransformStamped(
        header=Header(frame_id="parent", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="child",
        transform=Transform(
            translation=Vector3(x=3e38, y=0.0, z=0.0), rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
    )
    with pytest.raises(ValueError, match="exceed field range"):
        transform_cloud(cloud, transform)
    assert np.isfinite(pointcloud_xyz(cloud)).all()


def test_voxel_downsample_retains_existing_small_cloud_fast_path():
    message = pointcloud_from_xyz(
        np.zeros((19, 3)), header=Header(frame_id="lidar", stamp=Time(sec=0, nanosec=0))
    )
    assert voxel_downsample_cloud(message, 0.5) is message
    assert voxel_downsample_cloud(message, 0.0) is message
    with pytest.raises(ValueError, match="finite"):
        voxel_downsample_cloud(message, float("nan"))


def test_voxel_downsample_native_centroids_colors_and_exact_header():
    pytest.importorskip("open3d", reason="native tensor downsampling requires Open3D")
    points = np.repeat([[0.1, 0.1, 0.1], [0.3, 0.3, 0.3]], 20, axis=0)
    colors = np.repeat([[0, 0, 0], [255, 255, 255]], 20, axis=0).astype(np.uint8)
    header = Header(frame_id="optical", stamp=Time(sec=5, nanosec=123456789))
    source = pointcloud_from_xyz_rgb(points, colors, header=header)
    result = voxel_downsample_cloud(source, 1.0)
    assert result.header == header
    assert result.width == 1
    np.testing.assert_allclose(pointcloud_xyz(result), [[0.2, 0.2, 0.2]], atol=1e-6)
    np.testing.assert_array_equal(pointcloud_rgb(result), [[127, 127, 127]])
    assert source.width == 40


@pytest.mark.parametrize("shape", [(3, 3), (2, 3, 3)])
def test_per_point_stamps_survive_cdr_and_point_selection(shape):
    points = np.arange(np.prod(shape), dtype=np.float32).reshape(shape)
    stamps = np.arange(np.prod(shape[:-1]), dtype=np.float64).reshape(shape[:-1]) + 17.25
    header = Header(stamp=Time(sec=19, nanosec=123), frame_id="map")
    cloud = pointcloud_from_xyz(points, header=header, stamps=stamps)
    restored = cdr_decode(cdr_encode(cloud), PointCloud2)
    np.testing.assert_array_equal(pointcloud_xyz(restored), points.reshape(-1, 3))
    np.testing.assert_array_equal(pointcloud_stamps(restored), stamps.reshape(-1))
    keep = np.arange(stamps.size) % 2 == 0
    selected = select_points(restored, keep)
    np.testing.assert_array_equal(pointcloud_stamps(selected), stamps.reshape(-1)[keep])
    assert selected.header == header
    assert pointcloud_stamps(pointcloud_from_xyz(points, header=header)) is None


@pytest.mark.parametrize("stamps", [np.array([1.0]), np.array([1.0, np.nan])])
def test_per_point_stamps_reject_wrong_count_or_nonfinite_values(stamps):
    with pytest.raises(ValueError, match="Per-point stamps"):
        pointcloud_from_xyz(
            np.zeros((2, 3)),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            stamps=stamps,
        )
