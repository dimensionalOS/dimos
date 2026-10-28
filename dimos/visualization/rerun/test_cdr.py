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

"""Generated values stay plain while the viewer owns rendering and subscriptions."""

import json
from unittest.mock import Mock

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest
import rerun as rr

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import ZenohTransport
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos.messages.geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.sensor_msgs.msg import (
    CameraInfo,
    Image,
    PointCloud2,
    PointField,
    RegionOfInterest,
)
from dimos.messages.std_msgs.msg import Header
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule, _resolve_pubsubs
from dimos.visualization.rerun.cdr import (
    cloud_archetype,
    generated_archetypes,
    image_archetype,
)
from dimos.visualization.vis_module import vis_module
from dimos.web.relay_bridge.tests.cdr_fixtures import build_messages


def image_from_array(pixels, *, encoding, header=None):
    return Image(
        header=header or Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        width=pixels.shape[1],
        height=pixels.shape[0],
        encoding=encoding,
        is_bigendian=int(pixels.dtype.byteorder == ">"),
        step=pixels.strides[0],
        data=np.frombuffer(pixels.tobytes(), dtype=np.uint8),
    )


def camera_info_from_intrinsics(fx, fy, cx, cy, width, height, *, header):
    return CameraInfo(
        distortion_model="plumb_bob",
        d=np.zeros(5, dtype=np.float64),
        r=np.eye(3, dtype=np.float64).ravel(),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
        header=header,
        width=width,
        height=height,
        k=np.asarray([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=np.float64),
    )


def pointcloud_from_xyz_rgb(points, colors, *, header):
    data = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    for i, name in enumerate(("x", "y", "z")):
        data[name] = points[:, i]
    rgb = colors.astype(np.uint32)
    data["rgb"] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        point_step=16,
        row_step=len(points) * 16,
        is_bigendian=False,
        is_dense=False,
        fields=[
            PointField(name=name, offset=i * 4, datatype=7 if i < 3 else 6, count=1)
            for i, name in enumerate(("x", "y", "z", "rgb"))
        ],
        data=np.frombuffer(data.tobytes(), dtype=np.uint8),
    )


@pytest.fixture
def bridge(mocker):
    mocker.patch.object(ZenohRPC, "start")
    mocker.patch.object(ZenohRPC, "serve_module_rpc")
    instance = RerunBridgeModule(pubsubs=[])
    yield instance
    instance.stop()


def test_pose_axes_fields_and_frame_are_kept_separate_from_value_class() -> None:
    message = build_messages()[0][1]
    entries = generated_archetypes(message, "world/pose")
    assert entries is not None
    assert [path for path, _ in entries] == ["world/pose", "world/pose", "world/pose/message"]
    transform, axes, text = [archetype for _, archetype in entries]
    np.testing.assert_allclose(
        transform.translation.as_arrow_array().to_pylist(), [[1.5, -2.5, 0.25]]
    )
    np.testing.assert_allclose(
        transform.quaternion.as_arrow_array().to_pylist(), [[0, 0, 0.6, 0.8]]
    )
    assert transform.parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
    assert isinstance(axes, rr.TransformAxes3D)
    fields = json.loads(text.text.as_arrow_array().to_pylist()[0])
    assert fields["header"] == {
        "stamp": {"sec": 1757400000, "nanosec": 500000123},
        "frame_id": "map",
    }
    assert "__msgtype__" not in fields
    assert not hasattr(message, "to_rerun")


def test_point_frame_and_unknown_values() -> None:
    pose = build_messages()[0][1]
    msg = PointStamped(header=pose.header, point=pose.pose.position)
    entries = generated_archetypes(msg, "world/point")
    assert entries is not None
    assert isinstance(entries[0][1], rr.Points3D)
    assert entries[1][1].parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]
    assert generated_archetypes({"x": 1}, "world/no") is None


@pytest.mark.parametrize("nanoseconds", [0, 1757400000500000123])
def test_source_time_is_scoped_and_reset_after_render_failure(bridge, mocker, nanoseconds) -> None:
    msg = build_messages()[0][1]
    msg.header.stamp = Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)
    render = mocker.patch.object(bridge, "_on_message", side_effect=ValueError("bad renderer"))
    set_time = mocker.patch.object(rr, "set_time")
    clear_time = mocker.patch.object(rr, "disable_timeline")
    with pytest.raises(ValueError, match="bad renderer"):
        bridge._on_cdr_message(msg, topic="/pose")
    render.assert_called_once_with(msg, "/pose")
    set_time.assert_called_once_with("source", timestamp=np.datetime64(nanoseconds, "ns"))
    clear_time.assert_called_once_with("source")


def test_subscription_cleanup_order_and_failed_start(bridge) -> None:
    operations: list[str] = []
    transport = Mock(spec=ZenohTransport)
    transport.subscribe.return_value = lambda: operations.append("unsubscribe")
    transport.stop.side_effect = lambda: operations.append("stop")
    bridge._subscribe_cdr("pose", transport)
    bridge.stop()
    assert operations == ["unsubscribe", "stop"]
    failed = Mock(spec=ZenohTransport)
    failed.subscribe.side_effect = RuntimeError("native start failed")
    with pytest.raises(RuntimeError, match="native start failed"):
        bridge._subscribe_cdr("pose", failed)
    failed.stop.assert_called_once_with()


def test_bridge_default_renderer_handles_generated_pose(bridge, mocker) -> None:
    log = mocker.patch.object(rr, "log")
    bridge._on_message(build_messages()[0][1], "/pose")
    assert [call.args[0] for call in log.call_args_list] == [
        "world/pose",
        "world/pose",
        "world/pose/message",
    ]
    assert isinstance(log.call_args_list[0].args[1], rr.Transform3D)


@pytest.mark.parametrize("shape", [(1, 2), (2, 1)])
def test_single_row_or_column_rgb_keeps_dimensions_and_color_bytes(shape) -> None:
    pixels = np.array([[[255, 0, 0], [0, 255, 0]]], dtype=np.uint8).reshape(*shape, 3)
    image = decode(encode(image_from_array(pixels, encoding="rgb8")), Image)
    archetype = image_archetype(image)
    assert archetype.format.as_arrow_array().to_pylist()[0]["height"] == shape[0]
    assert archetype.format.as_arrow_array().to_pylist()[0]["width"] == shape[1]
    assert archetype.buffer.as_arrow_array().to_pylist() == [[255, 0, 0, 0, 255, 0]]


def test_big_endian_depth_renderer_preserves_values_and_units() -> None:
    pixels = np.array([[1, 256, 65535]], dtype=">u2")
    archetype = image_archetype(image_from_array(pixels, encoding="16UC1"))
    assert isinstance(archetype, rr.DepthImage)
    assert archetype.meter.as_arrow_array().to_pylist() == [1000.0]
    raw = bytes(archetype.buffer.as_arrow_array().to_pylist()[0])
    np.testing.assert_array_equal(np.frombuffer(raw, dtype=np.uint16), pixels.ravel())


@pytest.mark.parametrize("camera_first", [False, True])
def test_calibration_attaches_in_both_arrival_orders(bridge, mocker, camera_first) -> None:
    header = build_messages()[0][1].header
    image = image_from_array(np.zeros((1, 2, 3), dtype=np.uint8), encoding="rgb8", header=header)
    calibration = camera_info_from_intrinsics(10, 20, 1, 0.5, 2, 1, header=header)
    log = mocker.patch.object(rr, "log")
    samples = [(calibration, "/calibration"), (image, "/image")]
    for message, topic in samples if camera_first else reversed(samples):
        bridge._on_message(message, topic)
    pinholes = [call for call in log.call_args_list if isinstance(call.args[1], rr.Pinhole)]
    assert len(pinholes) == 1
    assert pinholes[0].args[0] == "world/image"
    assert pinholes[0].args[1].parent_frame.as_arrow_array().to_pylist() == ["tf#/map"]


def test_cloud_renderer_filters_nonfinite_positions_with_matching_colors() -> None:
    header = build_messages()[0][1].header
    cloud = pointcloud_from_xyz_rgb(
        np.array([[1, 2, 3], [float("nan"), 0, 0], [4, 5, 6]]),
        np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8),
        header=header,
    )
    archetype = cloud_archetype(cloud)
    assert archetype.positions.as_arrow_array().to_pylist() == [[1, 2, 3], [4, 5, 6]]
    assert archetype.colors.as_arrow_array().to_pylist() == [0xFF0000FF, 0x0000FFFF]


@pytest.mark.parametrize("backend", ["zenoh", "lcm"])
def test_blueprint_does_not_construct_pubsubs_and_resolution_keeps_backend(mocker, backend) -> None:
    lcm = mocker.patch("dimos.visualization.rerun.bridge.LCM")
    zenoh = mocker.patch("dimos.visualization.rerun.bridge.Zenoh")
    config = Config(g=GlobalConfig(transport=backend))
    assert config.pubsubs is None
    vis_module("rerun")
    lcm.assert_not_called()
    zenoh.assert_not_called()
    resolved = _resolve_pubsubs(config)
    expected = zenoh if backend == "zenoh" else lcm
    expected.assert_called_once_with()
    assert resolved == [expected.return_value]


def test_generated_entity_marker_adapter_keeps_labels_colors_frame_and_exact_fields():
    header = Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789))
    message = EntityMarkers(
        header=header,
        markers=[
            EntityMarker(
                entity_id="E1",
                label="person",
                entity_type="person",
                position=Point(x=1, y=2, z=0.3),
            )
        ],
    )
    message = decode(encode(message), EntityMarkers)
    entries = generated_archetypes(message, "memory/entities")
    assert entries is not None
    points, frame, fields = [value for _, value in entries]
    np.testing.assert_allclose(points.positions.as_arrow_array().to_pylist(), [[1, 2, 0.3]])
    assert points.labels.as_arrow_array().to_pylist() == ["E1: person"]
    assert points.colors.as_arrow_array().to_pylist() == [0xFF6464FF]
    assert frame.parent_frame.as_arrow_array().to_pylist() == ["tf#/world"]
    assert (
        json.loads(fields.text.as_arrow_array().to_pylist()[0])["header"]["stamp"]["nanosec"]
        == 123456789
    )


@pytest.mark.parametrize("mode", ["points", "boxes", "spheres"])
def test_fixed_display_color_overrides_sensor_rgb_after_invalid_points_filter(mode):
    cloud = pointcloud_from_xyz_rgb(
        np.array([[1, 2, 3], [np.nan, 0, 0]], dtype=np.float32),
        np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id="world"),
    )
    before = encode(cloud)
    value = cloud_archetype(cloud, mode=mode, color=(76, 220, 41))
    positions = value.centers if mode == "boxes" else value.positions
    assert positions.as_arrow_array().to_pylist() == [[1, 2, 3]]
    assert value.colors.as_arrow_array().to_pylist() == [0x4CDC29FF]
    assert encode(cloud) == before


def test_generated_odometry_exposes_pose_axes_and_full_covariance_fields():
    pose = build_messages()[0][1]
    value = Odometry(
        pose.header,
        "base",
        PoseWithCovariance(pose.pose, np.zeros(36)),
        TwistWithCovariance(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), np.zeros(36)),
    )
    value.pose.covariance[0] = 0.25
    entries = generated_archetypes(value, "world/odometry")
    assert entries is not None
    assert isinstance(entries[0][1], rr.Transform3D)
    assert isinstance(entries[1][1], rr.TransformAxes3D)
    assert entries[0][1].translation.as_arrow_array().to_pylist() == [[1.5, -2.5, 0.25]]
    fields = json.loads(entries[-1][1].text.as_arrow_array().to_pylist()[0])
    assert fields["pose"]["covariance"][0] == 0.25
    assert fields["child_frame_id"] == "base"
    assert entries[0][1].child_frame.as_arrow_array().to_pylist() == ["tf#/base"]
    assert fields["header"]["stamp"]["nanosec"] == 500000123
    assert value.pose.covariance[0] == 0.25


def test_generated_tf_preserves_frame_tree_layout_and_transform_values(bridge, mocker):
    bridge.config.tf_axes = 0.35
    bridge._tf_tree = bridge._new_tf_tree()
    log = mocker.patch.object(rr, "log")
    value = TFMessage(
        [
            TransformStamped(
                Header(Time(0, 1), "world"),
                "camera",
                Transform(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            )
        ]
    )
    bridge._on_message(value, "tf")
    assert bridge._tf_tree.placements() == {
        "world": "world/tf/world",
        "camera": "world/tf/world/camera",
    }
    transform = next(call.args[1] for call in log.call_args_list if call.args[0] == "tf#/camera")
    assert transform.translation.as_arrow_array().to_pylist() == [[1.0, 2.0, 3.0]]
    assert transform.child_frame.as_arrow_array().to_pylist() == ["tf#/camera"]
    assert value.transforms[0].header.stamp == Time(0, 1)
