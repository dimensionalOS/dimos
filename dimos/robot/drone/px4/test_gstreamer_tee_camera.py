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

import pytest
from pytest_mock import MockerFixture

pytest.importorskip("gi", reason="GStreamer camera tests require system bindings")

from dimos.robot.drone.px4 import gstreamer_tee_camera as camera


def test_x264_pipeline_uses_configured_bitrate_and_gop(mocker: MockerFixture) -> None:
    pipeline = mocker.Mock()
    pipeline.get_by_name.return_value = mocker.Mock()
    pipeline.get_bus.return_value = mocker.Mock()
    pipeline.set_state.return_value = camera.Gst.StateChangeReturn.SUCCESS
    parse_launch = mocker.patch.object(camera.Gst, "parse_launch", return_value=pipeline)
    main_loop = mocker.patch.object(camera.GLib, "MainLoop")

    module = camera.GsTeeCamera(
        encoder=camera.GstEncoder.X264,
        bitrate=2_000_000,
        gop=15,
    )
    try:
        module.start()

        pipeline_description = parse_launch.call_args.args[0]
        assert "x264enc bitrate=2000 key-int-max=15" in pipeline_description
    finally:
        module.stop()
        main_loop.return_value.quit.assert_called_once_with()


def test_h264_input_rejects_unused_encoder_options() -> None:
    with pytest.raises(ValueError, match="bitrate cannot be set"):
        camera.GsTeeCameraConfig(
            input_format=camera.GstInputFormat.H264,
            bitrate=2_000_000,
        )
