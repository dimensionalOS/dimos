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

"""PX4-local tee camera configuration and Rerun conversion."""

from collections.abc import Callable
import os
from pathlib import Path
from typing import Any, Final, Literal

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstPipelineError,
    GstSourceConfig,
    source_config_from_environment,
)
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import (
    CameraCalibration,
    Px4GstTeeCamera,
)
from dimos.robot.drone.px4.rerun_pshm_pubsub import Px4FixedTopicPshm
from dimos.visualization.vis_module import vis_module

CameraRerunMode = Literal["h264", "color_image"]
VisualOverride = Callable[[Any], Any] | None
RERUN_CAMERA_IMAGE_ENTITY: Final = "world/color_image"
RERUN_CAMERA_IMAGE_MAX_HZ: Final = 15.0
RERUN_H264_VIDEO_ENTITY: Final = "drone/video"


def camera_rerun_mode() -> CameraRerunMode:
    """Return the configured Rerun representation without changing camera outputs."""
    match os.environ.get("DIMOS_CAMERA_RERUN_MODE", "h264"):  # noqa: MATCH_OK
        case "h264":
            return "h264"
        case "color_image":
            return "color_image"
        case unexpected:
            raise GstPipelineError(
                detail=f"DIMOS_CAMERA_RERUN_MODE must be h264 or color_image, got {unexpected!r}"
            )


def px4_camera_from_environment() -> Blueprint:
    """Build the one PX4 tee camera with both raw and encoded output branches."""
    return Px4GstTeeCamera.blueprint(
        source_config=source_config_from_environment(),
        calibration=_calibration_from_environment(),
    )


def gazebo_camera_source_from_environment() -> GstSourceConfig:
    values = {
        "DIMOS_PX4_CAMERA_SOURCE": "udp-rtp-h264",
        "DIMOS_PX4_CAMERA_UDP_PORT": "5600",
        **os.environ,
    }
    return source_config_from_environment(values)


def _calibration_from_environment() -> CameraCalibration | None:
    calibration_path = os.environ.get("DIMOS_PX4_CAMERA_CALIBRATION_YAML")
    if calibration_path is None or not Path(calibration_path).is_file():
        return None
    camera_info = CameraInfo.from_yaml(calibration_path)
    if not _has_valid_focal_lengths(camera_info):
        return None
    return CameraCalibration(
        camera_info=camera_info,
        camera_link=Transform(
            translation=Vector3(0.0, 0.0, 0.0), frame_id="base_link", child_frame_id="camera_link"
        ),
    )


def _has_valid_focal_lengths(camera_info: CameraInfo) -> bool:
    return len(camera_info.K) >= 5 and camera_info.K[0] > 0.0 and camera_info.K[4] > 0.0


def camera_media_entity() -> str:
    """Return the Rerun entity carrying the selected camera representation."""
    if camera_rerun_mode() == "h264":
        return RERUN_H264_VIDEO_ENTITY
    return RERUN_CAMERA_IMAGE_ENTITY


def _camera_info_to_rerun(camera_info: CameraInfo) -> Any:
    if not _has_valid_focal_lengths(camera_info):
        return None
    return camera_info.to_rerun(
        image_topic=camera_media_entity(), optical_frame=camera_info.frame_id
    )


def _color_image_to_rerun(image: Image) -> Any:
    import rerun as rr

    return [(RERUN_CAMERA_IMAGE_ENTITY, rr.Image(image.to_rgb().data).compress(jpeg_quality=75))]


def _video_h264_to_rerun(video: CompressedVideo) -> Any:
    return [(RERUN_H264_VIDEO_ENTITY, video.to_rerun())]


def camera_visual_override(
    existing: dict[str, VisualOverride] | None = None,
) -> dict[str, VisualOverride]:
    """Convert exactly one tee branch for Rerun while suppressing the other."""
    overrides = {} if existing is None else existing.copy()
    overrides["world/camera_info"] = _camera_info_to_rerun
    if camera_rerun_mode() == "h264":
        overrides["world/color_image"] = None
        overrides["world/video_h264"] = _video_h264_to_rerun
    else:
        overrides["world/color_image"] = _color_image_to_rerun
        overrides["world/video_h264"] = None
    return overrides


def px4_camera_layout() -> Any:
    """Build the PX4 camera layout for the selected media representation."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin=camera_media_entity(), name="Camera"),
            rrb.Spatial3DView(origin="world", name="World"),
            column_shares=[1, 2],
        ),
    )


def camera_rerun_config(
    *,
    max_hz: dict[str, float] | None = None,
    visual_override: dict[str, VisualOverride] | None = None,
) -> dict[str, Any]:
    """Build bridge configuration for a local tee camera and selected display branch."""
    config: dict[str, Any] = {
        "blueprint": px4_camera_layout,
        "visual_override": camera_visual_override(visual_override),
    }
    if max_hz is not None:
        config["max_hz"] = max_hz
    if camera_rerun_mode() == "color_image":
        config["pubsubs"] = [LCM(), Px4FixedTopicPshm()]
    return config


def camera_viewer() -> Blueprint:
    """Build the remote Rerun viewer for the PX4 camera stream."""
    return vis_module(global_config.viewer, rerun_config=camera_rerun_config())
