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

from pathlib import Path
from typing import Protocol, runtime_checkable
from unittest.mock import patch

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import Blueprint
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.drone.px4 import camera
from dimos.robot.drone.px4.gstreamer.gstreamer_api import GstPipelineError, GstSource
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import Px4GstTeeCamera
from dimos.robot.drone.px4.rerun_pshm_pubsub import Px4FixedTopicPshm
from dimos.visualization.rerun.bridge import RerunBridgeModule


@runtime_checkable
class OriginView(Protocol):
    @property
    def origin(self) -> str: ...


def test_h264_mode_records_compressed_video_at_drone_video(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "h264")

    overrides = camera.camera_visual_override({"world/region_bounds": None})
    packet = CompressedVideo(data=b"h264")

    assert camera.camera_rerun_mode() == "h264"
    assert overrides["world/region_bounds"] is None
    assert overrides["world/color_image"] is None
    assert callable(overrides["world/camera_info"])
    h264_converter = overrides["world/video_h264"]
    assert callable(h264_converter)
    with patch.object(packet, "to_rerun", return_value="video-stream"):
        assert h264_converter(packet) == [("drone/video", "video-stream")]


def test_color_image_mode_compresses_raw_image_without_image_to_rerun(
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "color_image")

    overrides = camera.camera_visual_override()
    image = Image.from_numpy(
        np.zeros((2, 3, 3), dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera_optical",
    )

    assert camera.camera_rerun_mode() == "color_image"
    assert callable(overrides["world/color_image"])
    assert callable(overrides["world/camera_info"])
    assert overrides["world/video_h264"] is None
    color_image_converter = overrides["world/color_image"]
    assert callable(color_image_converter)
    rgb_image = mocker.Mock()
    rgb_image.data = mocker.sentinel.rgb_data
    to_rgb = mocker.patch.object(image, "to_rgb", return_value=rgb_image)
    to_rerun = mocker.patch.object(image, "to_rerun")
    rerun_image = mocker.patch("rerun.Image")
    rerun_image.return_value.compress.return_value = mocker.sentinel.compressed_image

    image_result = color_image_converter(image)

    assert image_result == [("world/color_image", mocker.sentinel.compressed_image)]
    to_rgb.assert_called_once_with()
    to_rerun.assert_not_called()
    rerun_image.assert_called_once_with(mocker.sentinel.rgb_data)
    rerun_image.return_value.compress.assert_called_once_with(jpeg_quality=75)


def test_camera_info_visual_override_targets_active_media_entity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "color_image")
    overrides = camera.camera_visual_override()
    camera_info = CameraInfo.from_intrinsics(
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
        frame_id="camera_optical",
    )
    camera_info_converter = overrides["world/camera_info"]
    assert callable(camera_info_converter)
    pinhole_result = camera_info_converter(camera_info)

    assert isinstance(pinhole_result, list)
    assert pinhole_result[0][0] == "world/color_image"
    pinhole = pinhole_result[0][1]
    assert type(pinhole).__name__ == "Pinhole"
    assert pinhole.parent_frame.as_arrow_array().to_pylist() == ["tf#/camera_optical"]
    assert type(pinhole).__name__ != "Transform3D"


def test_camera_info_visual_override_suppresses_zero_focal_lengths(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "h264")
    zero_focal_info = CameraInfo.from_intrinsics(
        fx=0.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        width=640,
        height=480,
        frame_id="camera_optical",
    )
    converter = camera.camera_visual_override()["world/camera_info"]

    assert callable(converter)
    assert converter(zero_focal_info) is None


@pytest.mark.parametrize(
    ("mode", "expected_origin"),
    (("h264", "drone/video"), ("color_image", "world/color_image")),
)
def test_camera_layout_targets_active_media_entity(
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    expected_origin: str,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", mode)

    layout = camera.px4_camera_layout()
    camera_view = next(iter(layout.root_container.contents))

    assert isinstance(camera_view, OriginView)
    assert camera_view.origin == expected_origin


def test_camera_from_environment_creates_one_tee_blueprint_with_both_outputs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_PX4_CAMERA_SOURCE", "videotestsrc")
    monkeypatch.setenv("DIMOS_PX4_CAMERA_ENCODER", "x264enc")
    monkeypatch.delenv("DIMOS_PX4_CAMERA_CALIBRATION_YAML", raising=False)

    blueprint = camera.px4_camera_from_environment()

    assert isinstance(blueprint, Blueprint)
    assert len(blueprint.active_blueprints) == 1
    atom = blueprint.active_blueprints[0]
    assert atom.module is Px4GstTeeCamera
    assert atom.kwargs["source_config"].source is GstSource.VIDEOTEST
    assert atom.kwargs["calibration"] is None
    assert {stream.name for stream in atom.streams} >= {"color_image", "video_h264"}
    assert "h264_sink" not in atom.kwargs


def test_gazebo_camera_source_defaults_to_udp_rtp_h264_and_allows_port_override(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("DIMOS_PX4_CAMERA_SOURCE", raising=False)
    monkeypatch.setenv("DIMOS_PX4_CAMERA_UDP_PORT", "5601")

    source_config = camera.gazebo_camera_source_from_environment()

    assert source_config.source is GstSource.UDP_RTP_H264
    assert source_config.udp_port == 5601


def test_camera_from_environment_uses_only_valid_yaml_calibration(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    calibration_file = tmp_path / "camera.yaml"
    _ = calibration_file.write_text(
        """image_width: 640
image_height: 480
camera_matrix:
  data: [100.0, 0.0, 320.0, 0.0, 120.0, 240.0, 0.0, 0.0, 1.0]
"""
    )
    monkeypatch.setenv("DIMOS_PX4_CAMERA_CALIBRATION_YAML", str(calibration_file))

    blueprint = camera.px4_camera_from_environment()
    calibration = blueprint.active_blueprints[0].kwargs["calibration"]

    assert calibration is not None
    assert calibration.camera_info.K[0] == 100.0
    assert calibration.camera_info.K[4] == 120.0
    assert calibration.camera_link == Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        frame_id="base_link",
        child_frame_id="camera_link",
    )


def test_camera_from_environment_suppresses_zero_focal_yaml_calibration(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    calibration_file = tmp_path / "zero-focal-camera.yaml"
    _ = calibration_file.write_text(
        """image_width: 640
image_height: 480
camera_matrix:
  data: [0.0, 0.0, 320.0, 0.0, 120.0, 240.0, 0.0, 0.0, 1.0]
"""
    )
    monkeypatch.setenv("DIMOS_PX4_CAMERA_CALIBRATION_YAML", str(calibration_file))

    blueprint = camera.px4_camera_from_environment()

    assert blueprint.active_blueprints[0].kwargs["calibration"] is None


def test_color_image_viewer_adds_fixed_pshm_pubsub_only_for_raw_mode(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "color_image")

    raw_bridge = next(
        atom
        for atom in camera.camera_viewer().active_blueprints
        if atom.module is RerunBridgeModule
    )

    assert isinstance(raw_bridge.kwargs["pubsubs"][1], Px4FixedTopicPshm)

    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "h264")
    h264_bridge = next(
        atom
        for atom in camera.camera_viewer().active_blueprints
        if atom.module is RerunBridgeModule
    )

    assert len(h264_bridge.kwargs["pubsubs"]) == 1


def test_camera_mode_rejects_unknown_value(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DIMOS_CAMERA_RERUN_MODE", "unknown")

    with pytest.raises(GstPipelineError):
        _ = camera.camera_rerun_mode()
