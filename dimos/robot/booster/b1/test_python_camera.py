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

from collections.abc import Iterator
from typing import Any
from unittest.mock import call

import booster_robotics_sdk_python as booster
import cv2
import numpy as np
import pytest

from dimos.msgs.metrics_msgs.MetricsArray import MetricsArray
from dimos.msgs.sensor_msgs.Image import ImageFormat
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.booster.b1 import python_camera as python_camera_module
from dimos.robot.booster.b1.blueprints.basic.booster_b1_camera_python import (
    booster_b1_camera_python,
)
from dimos.robot.booster.b1.python_camera import (
    BoosterCameraPython,
    _FrameAgeMetrics,
    _FrameAgeSnapshot,
    booster_camera_info_to_dimos,
    booster_compressed_image_to_dimos,
    booster_depth_image_to_dimos,
    booster_image_to_dimos,
)


class _FixtureRPC(RPCSpec):
    def __init__(self, **kwargs: Any) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def serve_module_rpc(self, module: Any) -> None:
        pass


def set_header(message: Any, *, frame_id: str = "camera", ts: float = 12.25) -> None:
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(ts)
    message.header.stamp.nanosec = round((ts - int(ts)) * 1_000_000_000)


def test_frame_age_metrics_aggregate_source_age_and_reset(mocker) -> None:
    mocker.patch.object(
        python_camera_module.time,
        "perf_counter",
        side_effect=[10.0, 10.05, 10.10, 10.15, 11.0, 12.0],
    )
    mocker.patch.object(
        python_camera_module.time,
        "time",
        side_effect=[100.10, 100.20, 100.30],
    )
    metrics = _FrameAgeMetrics()

    metrics.record(source_timestamp=100.0, processing_started=10.01, payload_bytes=10)
    metrics.record(source_timestamp=100.25, processing_started=10.06, payload_bytes=20)
    metrics.record(source_timestamp=0.0, processing_started=10.11, payload_bytes=30)
    snapshot = metrics.snapshot_and_reset()

    assert snapshot.window_seconds == pytest.approx(1.0)
    assert snapshot.frames == 3
    assert snapshot.payload_bytes == 60
    assert snapshot.invalid_timestamps == 1
    assert snapshot.negative_ages == 1
    assert snapshot.frame_age_ms == pytest.approx(
        {"p50": 25.0, "p95": 92.5, "p99": 98.5, "max": 100.0}
    )
    assert snapshot.bridge_ms == pytest.approx({"p50": 40.0, "p95": 40.0, "p99": 40.0, "max": 40.0})

    empty_snapshot = metrics.snapshot_and_reset()
    assert empty_snapshot.frames == 0
    assert empty_snapshot.frame_age_ms is None
    assert empty_snapshot.bridge_ms is None


def test_raw_color_conversion_handles_padded_rows() -> None:
    message = booster.Image()
    set_header(message, frame_id="rgb_optical")
    message.height = 2
    message.width = 2
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = 8
    message.data = [1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88]

    result = booster_image_to_dimos(message)

    assert result.format is ImageFormat.RGB
    assert result.frame_id == "rgb_optical"
    assert result.ts == 12.25
    np.testing.assert_array_equal(
        result.data,
        np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8),
    )


def test_compressed_color_conversion_decodes_image() -> None:
    source = np.array(
        [
            [[0, 0, 255], [0, 255, 0]],
            [[255, 0, 0], [255, 255, 255]],
        ],
        dtype=np.uint8,
    )
    encoded_ok, encoded = cv2.imencode(".png", source)
    assert encoded_ok
    message = booster.CompressedImage()
    set_header(message, frame_id="compressed_rgb", ts=20.5)
    message.format = "png"
    message.data = encoded.tolist()

    result = booster_compressed_image_to_dimos(message)

    assert result.format is ImageFormat.BGR
    assert result.frame_id == "compressed_rgb"
    assert result.ts == 20.5
    np.testing.assert_array_equal(result.data, source)


def test_uint16_depth_conversion_handles_big_endian_padding_and_scale() -> None:
    values = np.array([[1000, 250], [65535, 0]], dtype=">u2")
    message = booster.Image()
    set_header(message, frame_id="depth_optical")
    message.height = 2
    message.width = 2
    message.encoding = "16UC1"
    message.is_bigendian = 1
    message.step = 6
    message.data = list(values[0].tobytes() + b"\xaa\xbb" + values[1].tobytes() + b"\xcc\xdd")

    result = booster_depth_image_to_dimos(message, depth_scale=0.001)

    assert result.format is ImageFormat.DEPTH
    assert result.data.dtype == np.float32
    np.testing.assert_allclose(
        result.data,
        np.array([[1.0, 0.25], [65.535, 0.0]], dtype=np.float32),
    )


def test_float32_depth_conversion_preserves_meter_values() -> None:
    values = np.array([[1.25, np.inf], [0.0, np.nan]], dtype=">f4")
    message = booster.Image()
    set_header(message)
    message.height = 2
    message.width = 2
    message.encoding = "32FC1"
    message.is_bigendian = 1
    message.step = 8
    message.data = list(values.tobytes())

    result = booster_depth_image_to_dimos(message, depth_scale=0.001)

    np.testing.assert_array_equal(result.data, values.astype(np.float32))


def test_camera_info_conversion_preserves_calibration_and_roi() -> None:
    message = booster.CameraInfo()
    set_header(message, frame_id="camera_calibration", ts=30.75)
    message.height = 480
    message.width = 640
    message.distortion_model = "plumb_bob"
    message.d = [0.1, 0.2, 0.3, 0.4, 0.5]
    message.k = [float(value) for value in range(9)]
    message.r = [float(value + 10) for value in range(9)]
    message.p = [float(value + 20) for value in range(12)]
    message.binning_x = 2
    message.binning_y = 3
    message.roi.x_offset = 4
    message.roi.y_offset = 5
    message.roi.height = 200
    message.roi.width = 300
    message.roi.do_rectify = True

    result = booster_camera_info_to_dimos(message)

    assert result.height == 480
    assert result.width == 640
    assert result.distortion_model == "plumb_bob"
    assert result.D == [0.1, 0.2, 0.3, 0.4, 0.5]
    assert result.K == [float(value) for value in range(9)]
    assert result.R == [float(value + 10) for value in range(9)]
    assert result.P == [float(value + 20) for value in range(12)]
    assert (result.binning_x, result.binning_y) == (2, 3)
    assert (
        result.roi_x_offset,
        result.roi_y_offset,
        result.roi_height,
        result.roi_width,
        result.roi_do_rectify,
    ) == (4, 5, 200, 300, True)
    assert result.frame_id == "camera_calibration"
    assert result.ts == 30.75


@pytest.fixture
def started_camera(mocker) -> Iterator[tuple[BoosterCameraPython, list[Any], Any]]:
    mocker.patch("dimos.core.module.get_loop", return_value=(mocker.Mock(), None))
    factory = mocker.Mock()
    channel_factory = mocker.patch.object(python_camera_module.booster, "ChannelFactory")
    channel_factory.Instance.return_value = factory

    subscribers = [
        mocker.Mock(spec=["InitChannel", "CloseChannel"]),
        mocker.Mock(spec=["InitChannel", "CloseChannel"]),
        mocker.Mock(spec=["InitChannel", "CloseChannel"]),
        mocker.Mock(spec=["InitChannel", "CloseChannel"]),
    ]
    compressed_subscriber = mocker.patch.object(
        python_camera_module.booster,
        "CompressedImageSubscriber",
        return_value=subscribers[0],
    )
    mocker.patch.object(
        python_camera_module.booster,
        "ImageSubscriber",
        return_value=subscribers[1],
    )
    mocker.patch.object(
        python_camera_module.booster,
        "CameraInfoSubscriber",
        side_effect=subscribers[2:],
    )

    camera = BoosterCameraPython(network_interface="eth-test", rpc_transport=_FixtureRPC)
    try:
        camera.start()
        yield camera, subscribers, (factory, compressed_subscriber)
    finally:
        camera.stop()


def test_camera_starts_and_closes_all_sdk_subscribers(started_camera) -> None:
    camera, subscribers, (factory, compressed_subscriber) = started_camera

    factory.Init.assert_called_once_with(0, "eth-test")
    assert compressed_subscriber.call_args.args[1] == "rt/booster_video_stream"
    for subscriber in subscribers:
        subscriber.InitChannel.assert_called_once_with()

    camera.stop()

    for subscriber in subscribers:
        subscriber.CloseChannel.assert_called_once_with()


def test_color_callback_records_metrics_after_publish(started_camera, mocker) -> None:
    camera, _, _ = started_camera
    message = booster.Image()
    set_header(message, ts=100.25)
    message.height = 1
    message.width = 1
    message.encoding = "rgb8"
    message.is_bigendian = 0
    message.step = 3
    message.data = [1, 2, 3]
    events: list[str] = []
    publish = mocker.patch.object(
        camera.color_image,
        "publish",
        side_effect=lambda image: events.append("publish"),
    )
    record = mocker.patch.object(
        camera._color_metrics,
        "record",
        side_effect=lambda *args: events.append("record"),
    )
    mocker.patch.object(camera, "_maybe_log_metrics")
    mocker.patch.object(python_camera_module.time, "perf_counter", return_value=42.0)

    camera._handle_color(message)

    image = publish.call_args.args[0]
    assert events == ["publish", "record"]
    record.assert_called_once_with(image.ts, 42.0, image.data.nbytes)


@pytest.mark.parametrize(
    ("handler_name", "output_name", "metrics_name"),
    [
        ("_handle_color_camera_info", "camera_info", "_color_camera_info_metrics"),
        (
            "_handle_depth_camera_info",
            "depth_camera_info",
            "_depth_camera_info_metrics",
        ),
    ],
)
def test_camera_info_callback_records_metrics_after_publish(
    started_camera,
    mocker,
    handler_name,
    output_name,
    metrics_name,
) -> None:
    camera, _, _ = started_camera
    message = booster.CameraInfo()
    set_header(message, frame_id="camera_info", ts=100.25)
    message.height = 480
    message.width = 640
    message.distortion_model = "plumb_bob"
    message.d = [0.1, 0.2, 0.3, 0.4, 0.5]
    message.k = [float(value) for value in range(9)]
    message.r = [float(value + 10) for value in range(9)]
    message.p = [float(value + 20) for value in range(12)]
    events: list[str] = []
    publish = mocker.patch.object(
        getattr(camera, output_name),
        "publish",
        side_effect=lambda camera_info: events.append("publish"),
    )
    record = mocker.patch.object(
        getattr(camera, metrics_name),
        "record",
        side_effect=lambda *args: events.append("record"),
    )
    maybe_log_metrics = mocker.patch.object(camera, "_maybe_log_metrics")
    mocker.patch.object(python_camera_module.time, "perf_counter", return_value=42.0)

    getattr(camera, handler_name)(message)

    camera_info = publish.call_args.args[0]
    assert events == ["publish", "record"]
    record.assert_called_once_with(
        camera_info.ts,
        42.0,
        len(camera_info.lcm_encode()),
    )
    maybe_log_metrics.assert_called_once_with()


def test_metrics_logging_snapshots_all_camera_streams_once_per_interval(
    started_camera, mocker
) -> None:
    camera, _, _ = started_camera
    camera._last_metrics_log = 10.0
    color_snapshot = _FrameAgeSnapshot(1.0, 1, 100, 0, 0, None, None)
    depth_snapshot = _FrameAgeSnapshot(1.0, 1, 200, 0, 0, None, None)
    color_info_snapshot = _FrameAgeSnapshot(1.0, 1, 300, 0, 0, None, None)
    depth_info_snapshot = _FrameAgeSnapshot(1.0, 1, 400, 0, 0, None, None)
    mocker.patch.object(python_camera_module.time, "perf_counter", return_value=11.0)
    mocker.patch.object(
        camera._color_metrics,
        "snapshot_and_reset",
        return_value=color_snapshot,
    )
    mocker.patch.object(
        camera._depth_metrics,
        "snapshot_and_reset",
        return_value=depth_snapshot,
    )
    mocker.patch.object(
        camera._color_camera_info_metrics,
        "snapshot_and_reset",
        return_value=color_info_snapshot,
    )
    mocker.patch.object(
        camera._depth_camera_info_metrics,
        "snapshot_and_reset",
        return_value=depth_info_snapshot,
    )
    log_metrics = mocker.patch.object(camera, "_log_metrics")

    camera._maybe_log_metrics()
    camera._maybe_log_metrics()

    assert log_metrics.call_args_list == [
        call("color", color_snapshot),
        call("depth", depth_snapshot),
        call("color_camera_info", color_info_snapshot),
        call("depth_camera_info", depth_info_snapshot),
    ]


def test_metrics_publish_structured_diagnostic_stream(started_camera, mocker) -> None:
    camera, _, _ = started_camera
    publish = mocker.patch.object(camera.camera_metrics, "publish")
    snapshot = _FrameAgeSnapshot(
        window_seconds=2.0,
        frames=4,
        payload_bytes=1_000_000,
        invalid_timestamps=1,
        negative_ages=2,
        frame_age_ms={"p50": 10.0, "p95": 20.0, "p99": 25.0, "max": 30.0},
        bridge_ms={"p50": 1.0, "p95": 2.0, "p99": 2.5, "max": 3.0},
    )

    camera._log_metrics("color", snapshot)

    message = publish.call_args.args[0]
    assert isinstance(message, MetricsArray)
    assert message.status_length == 1
    [status] = message.status
    assert status.name == "metrics/camera/color"
    assert status.hardware_id == "booster_b1"
    assert {item.key: float(item.value) for item in status.values} == {
        "fps": 2.0,
        "payload_mbps": 4.0,
        "frame_age/p50_ms": 10.0,
        "frame_age/p95_ms": 20.0,
        "frame_age/max_ms": 30.0,
        "bridge/p50_ms": 1.0,
        "bridge/p95_ms": 2.0,
        "bridge/max_ms": 3.0,
        "invalid_timestamps": 1.0,
        "negative_ages": 2.0,
    }


def test_python_camera_blueprint_uses_dedicated_python_module() -> None:
    [atom] = booster_b1_camera_python.blueprints

    assert atom.module is BoosterCameraPython
    assert BoosterCameraPython.dedicated_worker is True
    assert any(
        stream.name == "camera_metrics"
        and stream.type is MetricsArray
        and stream.direction == "out"
        for stream in atom.streams
    )
    assert all_blueprints["booster-b1-camera-python"].endswith(":booster_b1_camera_python")
