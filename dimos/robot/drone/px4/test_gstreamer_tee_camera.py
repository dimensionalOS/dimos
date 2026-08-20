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
import importlib.util
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.msgs.sensor_msgs.Image import ImageFormat

if importlib.util.find_spec("gi") is None:
    gi = MagicMock()
    repository = MagicMock()
    with patch.dict(
        sys.modules,
        {"gi": gi, "gi.repository": repository},
    ):
        from dimos.robot.drone.px4 import gstreamer_tee_camera as camera
else:
    from dimos.robot.drone.px4 import gstreamer_tee_camera as camera


@pytest.fixture(scope="module")
def camera_module() -> Iterator[camera.GsTeeCamera]:
    instance = camera.GsTeeCamera()
    yield instance
    instance.stop()


@pytest.fixture
def module(
    camera_module: camera.GsTeeCamera,
    mocker: MockerFixture,
) -> Iterator[camera.GsTeeCamera]:
    mocker.patch.object(camera, "Thread")
    camera_module.config.input_pipeline = camera.DEFAULT_INPUT_PIPELINE
    camera_module.config.input_format = camera.GstInputFormat.RAW
    camera_module.config.encoder = camera.GstEncoder.NVV4L2
    camera_module.config.bitrate = 4_000_000
    camera_module.config.gop = 30
    camera_module._release_pipeline()
    yield camera_module
    camera_module._release_pipeline()


@pytest.fixture
def gst_pipeline(mocker: MockerFixture) -> SimpleNamespace:
    raw_sink = mocker.Mock()
    h264_sink = mocker.Mock()
    bus = mocker.Mock()
    pipeline = mocker.Mock()
    pipeline.get_by_name.side_effect = {
        "raw_sink": raw_sink,
        "h264_sink": h264_sink,
    }.get
    pipeline.get_bus.return_value = bus
    pipeline.set_state.return_value = camera.Gst.StateChangeReturn.SUCCESS
    parse_launch = mocker.patch.object(camera.Gst, "parse_launch", return_value=pipeline)
    main_loop = mocker.patch.object(camera.GLib, "MainLoop")
    return SimpleNamespace(
        pipeline=pipeline,
        raw_sink=raw_sink,
        h264_sink=h264_sink,
        bus=bus,
        parse_launch=parse_launch,
        main_loop=main_loop,
    )


def test_x264_pipeline_uses_configured_bitrate_and_gop(
    module: camera.GsTeeCamera,
    gst_pipeline: SimpleNamespace,
) -> None:
    module.config.encoder = camera.GstEncoder.X264
    module.config.bitrate = 2_000_000
    module.config.gop = 15

    module.start()

    pipeline_description = gst_pipeline.parse_launch.call_args.args[0]
    assert "x264enc bitrate=2000 key-int-max=15" in pipeline_description


def test_h264_input_decodes_only_the_raw_branch(
    module: camera.GsTeeCamera, gst_pipeline: SimpleNamespace
) -> None:
    module.config.input_format = camera.GstInputFormat.H264

    module.start()

    pipeline_description = gst_pipeline.parse_launch.call_args.args[0]
    assert "avdec_h264" in pipeline_description
    assert "x264enc" not in pipeline_description
    assert "nvv4l2h264enc" not in pipeline_description
    gst_pipeline.raw_sink.connect.assert_called_once_with("new-sample", module._on_raw_sample)
    gst_pipeline.h264_sink.connect.assert_called_once_with("new-sample", module._on_h264_sample)


def test_raw_sample_publishes_bgr_image(module: camera.GsTeeCamera, mocker: MockerFixture) -> None:
    pixels = bytes(range(12))
    mapping = SimpleNamespace(data=pixels)
    buffer = mocker.Mock(pts=2_500_000_000)
    buffer.map.return_value = (True, mapping)
    caps = mocker.Mock()
    caps.get_value.side_effect = {"width": 2, "height": 2}.get
    sample = mocker.Mock()
    sample.get_buffer.return_value = buffer
    sample.get_caps.return_value.get_structure.return_value = caps
    sink = mocker.Mock()
    sink.emit.return_value = sample
    publish = mocker.patch.object(module.color_image, "publish")

    result = module._on_raw_sample(sink)

    assert result == camera.Gst.FlowReturn.OK
    image = publish.call_args.args[0]
    assert image.data.tolist() == [
        [[0, 1, 2], [3, 4, 5]],
        [[6, 7, 8], [9, 10, 11]],
    ]
    assert image.format is ImageFormat.BGR
    assert image.frame_id == "camera_optical"
    assert image.ts == 2.5
    buffer.unmap.assert_called_once_with(mapping)


def test_h264_sample_publishes_annex_b_packet(
    module: camera.GsTeeCamera, mocker: MockerFixture
) -> None:
    mapping = SimpleNamespace(data=b"\x00\x00\x00\x01\x65")
    buffer = mocker.Mock(pts=camera.Gst.CLOCK_TIME_NONE)
    buffer.map.return_value = (True, mapping)
    sample = mocker.Mock()
    sample.get_buffer.return_value = buffer
    sink = mocker.Mock()
    sink.emit.return_value = sample
    publish = mocker.patch.object(module.video_h264, "publish")

    result = module._on_h264_sample(sink)

    assert result == camera.Gst.FlowReturn.OK
    packet = publish.call_args.args[0]
    assert np.array_equal(packet.data, np.array([0, 0, 0, 1, 0x65], dtype=np.uint8))
    assert packet.format == "h264"
    assert packet.frame_id == "camera_optical"
    assert packet.ts == 0.0
    buffer.unmap.assert_called_once_with(mapping)


@pytest.mark.parametrize("callback", ("_on_raw_sample", "_on_h264_sample"))
def test_sample_callback_reports_pull_failure(
    module: camera.GsTeeCamera, mocker: MockerFixture, callback: str
) -> None:
    sink = mocker.Mock()
    sink.emit.return_value = None

    result = getattr(module, callback)(sink)

    assert result == camera.Gst.FlowReturn.ERROR


def test_start_failure_releases_pipeline(
    module: camera.GsTeeCamera, gst_pipeline: SimpleNamespace
) -> None:
    gst_pipeline.pipeline.set_state.return_value = camera.Gst.StateChangeReturn.FAILURE

    with pytest.raises(RuntimeError, match="failed to enter PLAYING"):
        module.start()

    assert module._pipeline is None
    gst_pipeline.bus.remove_signal_watch.assert_called_once_with()
    gst_pipeline.pipeline.set_state.assert_called_with(camera.Gst.State.NULL)
    gst_pipeline.main_loop.return_value.quit.assert_called_once_with()


@pytest.mark.parametrize("message_type", (camera.Gst.MessageType.ERROR, camera.Gst.MessageType.EOS))
def test_terminal_bus_message_releases_pipeline(
    module: camera.GsTeeCamera,
    gst_pipeline: SimpleNamespace,
    mocker: MockerFixture,
    message_type: object,
) -> None:
    module.start()
    message = mocker.Mock(type=message_type)
    message.parse_error.return_value = ("failure", "debug")

    module._on_bus_message(gst_pipeline.bus, message)

    assert module._pipeline is None
    gst_pipeline.bus.remove_signal_watch.assert_called_once_with()
    gst_pipeline.pipeline.set_state.assert_called_with(camera.Gst.State.NULL)


def test_h264_input_rejects_unused_encoder_options() -> None:
    with pytest.raises(ValueError, match="bitrate cannot be set"):
        camera.GsTeeCameraConfig(
            input_format=camera.GstInputFormat.H264,
            bitrate=2_000_000,
        )
