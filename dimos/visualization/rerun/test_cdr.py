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

from dimos.core.cdr_transport import CdrTransport
from dimos.core.global_config import GlobalConfig
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos.messages.geometry_msgs.msg import Point, PointStamped
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.camera_info import camera_info_from_intrinsics
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz_rgb
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.visualization.rerun.bridge import Config, RerunBridgeModule, _resolve_pubsubs
from dimos.visualization.rerun.cdr import (
    cloud_archetype,
    generated_archetypes,
    image_archetype,
    register_colormap_annotation,
)
from dimos.visualization.vis_module import vis_module
from dimos.web.relay_bridge.gen_cdr_fixtures import build_messages


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
    transport = Mock(spec=CdrTransport)
    transport.subscribe.return_value = lambda: operations.append("unsubscribe")
    transport.stop.side_effect = lambda: operations.append("stop")
    bridge._subscribe_cdr("pose", transport)
    bridge.stop()
    assert operations == ["unsubscribe", "stop"]
    failed = Mock(spec=CdrTransport)
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


def test_height_palette_uses_all_class_ids_at_the_recording_root(mocker):
    log = mocker.patch.object(rr, "log")
    register_colormap_annotation("turbo")
    log.assert_called_once()
    path, annotation = log.call_args.args
    assert path == "/" and log.call_args.kwargs == {"static": True}
    [classes] = annotation.context.as_arrow_array().to_pylist()
    assert [item["class_id"] for item in classes] == list(range(256))
    assert classes[0]["class_description"]["info"]["color"] == 0x30123BFF
