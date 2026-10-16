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

"""``px4`` plus the SIYI A8: the RTSP camera and the gimbal module.

Usage:
    dimos run px4-a8 --siyia8gimbal.mount-xyz '[0.1, 0, -0.08]'
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.gimbal.siyi.gimbal import SiyiA8Gimbal
from dimos.hardware.sensors.camera.rtsp.camera import RtspCamera
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.robot.px4.blueprints.basic.px4 import px4, zenoh_transport

# The A8 mini's main stream on its factory address.
A8_RTSP_URL = "rtsp://192.168.144.25:8554/main.264"


def a8_payload(
    *, camera: dict[str, Any] | None = None, gimbal: dict[str, Any] | None = None
) -> Blueprint:
    """RtspCamera on the A8's stream and SiyiA8Gimbal, with their streams.

    ``camera`` and ``gimbal`` override ``RtspCameraConfig`` and ``SiyiA8GimbalConfig`` fields.
    The gimbal module owns the gimbal tf chain. ``color_image`` keeps the default transport
    (zenoh, latest-wins): 2.8 MB a frame, not for subscribers off the aircraft.
    """
    camera = {
        "url": A8_RTSP_URL,
        "frame_id": "a8_optical",
        "capture_latency_s": 0.08,
        **(camera or {}),
    }
    return autoconnect(
        RtspCamera.blueprint(**camera),
        SiyiA8Gimbal.blueprint(**(gimbal or {})),
    ).transports(
        {
            # Reliable: an H.265 access unit depends on the ones before it, so a dropped one
            # stalls decode until the next keyframe.
            ("video", CompressedVideo): zenoh_transport("video", CompressedVideo),
            ("color_jpeg", CompressedImage): zenoh_transport(
                "color_jpeg", CompressedImage, latest_wins=True
            ),
            ("camera_info", CameraInfo): zenoh_transport("camera_info", CameraInfo),
        }
    )


px4_a8 = autoconnect(px4, a8_payload())
