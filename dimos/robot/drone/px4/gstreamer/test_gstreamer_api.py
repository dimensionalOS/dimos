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

from collections.abc import Callable

import pytest

from dimos.robot.drone.px4.gstreamer.fake_gstreamer import FakeGstApi
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstEncoder,
    GstPipelineError,
    GstSource,
    GstSourceConfig,
    GstTeePipelineSpec,
    source_config_from_environment,
)
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import (
    CameraCalibration,
    Px4GstTeeCamera,
)


def test_tee_pipeline_has_one_live_test_source_and_a_bounded_queue_per_branch(
    make_camera: Callable[
        [GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]
    ],
) -> None:
    # Given: a test-source camera with no attached hardware.
    gst, camera = make_camera(GstSource.VIDEOTEST, None)

    # When: the camera starts its owned source pipeline.
    camera.start()

    # Then: raw remains latest-wins while H.264 applies bounded backpressure without loss.
    description = gst.pipeline_description
    raw_branch, h264_branch = description.split(" ! tee name=t ", maxsplit=1)[1].split(" t. ! ")
    assert description.count("videotestsrc") == 1
    assert "videotestsrc is-live=true" in description
    assert "tee name=t" in description
    assert "queue max-size-buffers=2 leaky=downstream" in raw_branch
    assert "drop=true" in raw_branch
    assert "queue max-size-buffers=2" in h264_branch
    assert "leaky=" not in h264_branch
    assert "drop=false" in h264_branch
    assert "drop=true" not in h264_branch
    assert "video/x-raw,format=BGR" in description
    assert "x264enc bframes=0" in description
    assert "h264parse config-interval=-1" in description
    assert "video/x-h264,stream-format=byte-stream,alignment=au" in description
    assert "sync=false max-buffers=1 drop=true" in raw_branch
    assert "sync=false max-buffers=1 drop=false" in h264_branch


def test_default_environment_config_uses_v4l2_and_jetson_nvenc() -> None:
    # Given: no explicit smoke-test overrides.
    config = source_config_from_environment({})

    # When: the production pipeline specification renders.
    description = GstTeePipelineSpec(source=config).render()

    # Then: V4L2 uses the low-latency Jetson encoder, never the software encoder.
    assert config.source is GstSource.V4L2
    assert config.encoder is GstEncoder.NVV4L2
    assert (
        "nvv4l2h264enc bitrate=4000000 iframeinterval=30 idrinterval=30 insert-sps-pps=true"
        in description
    )
    assert "x264enc" not in description


def test_environment_config_selects_x264_only_when_explicitly_requested(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # Given: a smoke-test source with an explicit software encoder selection.
    monkeypatch.setenv("DIMOS_PX4_CAMERA_SOURCE", "videotestsrc")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_ENCODER", "x264enc")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_WIDTH", "320")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_HEIGHT", "240")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_FPS", "15")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_BITRATE", "2000000")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_GOP", "15")

    # When: settings are parsed and rendered.
    config = source_config_from_environment()
    description = GstTeePipelineSpec(source=config).render()

    # Then: only the selected low-latency x264 branch is rendered without H.264 loss settings.
    h264_branch = description.split(" t. ! ")[-1]
    assert config.source is GstSource.VIDEOTEST
    assert config.encoder is GstEncoder.X264
    assert (config.width, config.height, config.fps, config.bitrate, config.gop) == (
        320,
        240,
        15,
        2_000_000,
        15,
    )
    assert "x264enc bframes=0 tune=zerolatency" in description
    assert "nvv4l2h264enc" not in description
    assert "leaky=" not in h264_branch
    assert "drop=false" in h264_branch


def test_environment_config_renders_gazebo_udp_rtp_h264_source() -> None:
    # Given: the PX4 Gazebo Harmonic camera's default RTP/H.264 endpoint.
    config = source_config_from_environment(
        {
            "DIMOS_PX4_CAMERA_SOURCE": "udp-rtp-h264",
            "DIMOS_PX4_CAMERA_UDP_PORT": "5600",
            "DIMOS_PX4_CAMERA_ENCODER": "x264enc",
        }
    )

    # When: the camera tee pipeline renders the source.
    description = GstTeePipelineSpec(source=config).render()

    # Then: it depayloads and decodes the fixed RTP H.264 stream before teeing raw frames.
    assert config.source is GstSource.UDP_RTP_H264
    assert config.udp_port == 5600
    assert (
        "udpsrc port=5600 caps=application/x-rtp,media=video,encoding-name=H264,payload=96"
        in description
    )
    assert "rtph264depay ! h264parse config-interval=-1 ! tee name=t" in description
    assert "avdec_h264 ! videoconvert" in description
    assert "nvv4l2h264enc" not in description
    assert "x264enc" not in description


def test_udp_rtp_h264_config_rejects_out_of_range_port() -> None:
    # Given: a port outside the UDP range.
    # When: it crosses the typed source boundary.
    with pytest.raises(GstPipelineError, match="DIMOS_PX4_CAMERA_UDP_PORT"):
        _ = GstSourceConfig(source=GstSource.UDP_RTP_H264, udp_port=0)

    # Then: the pipeline is never rendered.


def test_environment_config_rejects_unknown_encoder(monkeypatch: pytest.MonkeyPatch) -> None:
    # Given: an unsupported encoder value.
    monkeypatch.setenv("DIMOS_PX4_CAMERA_ENCODER", "auto")

    # When: configuration crosses the environment boundary.
    with pytest.raises(GstPipelineError, match="DIMOS_PX4_CAMERA_ENCODER"):
        source_config_from_environment()

    # Then: no implicit fallback is chosen.


def test_v4l2_config_accepts_safe_device_paths() -> None:
    # Given: standard, by-id, and by-path V4L2 device paths.
    video_device = GstSourceConfig(device="/dev/video0")
    by_id_device = GstSourceConfig(device="/dev/v4l/by-id/camera-1")
    by_path_device = GstSourceConfig(
        device="/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0"
    )

    # When: source configuration is constructed before launch rendering.
    video_description = GstTeePipelineSpec(source=video_device).render()
    by_id_description = GstTeePipelineSpec(source=by_id_device).render()
    by_path_description = GstTeePipelineSpec(source=by_path_device).render()

    # Then: both valid paths become V4L2 source properties.
    assert "v4l2src device=/dev/video0" in video_description
    assert "v4l2src device=/dev/v4l/by-id/camera-1" in by_id_description
    assert (
        "v4l2src device=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0"
        in by_path_description
    )


def test_v4l2_config_rejects_gstreamer_device_injection() -> None:
    # Given: device strings that would alter a Gst launch description.
    unsafe_devices = (
        "/dev/video0 ! fakesink",
        "/dev/../etc/passwd",
        "/dev/video0;fakesink",
        "/dev/video0|fakesink",
        "/dev/video 0",
        "/dev/'video0'",
        "/dev/video0\n",
    )

    # When: each string crosses the source-configuration boundary.
    for device in unsafe_devices:
        with pytest.raises(GstPipelineError, match="DIMOS_PX4_CAMERA_DEVICE"):
            GstSourceConfig(device=device)

    # Then: rendering and parsing are never reached.
