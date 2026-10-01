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

"""Compressed RTSP camera relay for the Deep Robotics M20."""

import threading
import time
from typing import Any

import av
from dimos_generated.foxglove_msgs.msg import CompressedVideo
from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.camera_info import camera_info_from_intrinsics, camera_info_with_stamp
from dimos.msgs.time import time_from_seconds
from dimos.robot.deeprobotics.m20.constants import (
    FRONT_CAMERA_RTSP_URL,
    REAR_CAMERA_RTSP_URL,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Deep Robotics documents the camera centers relative to the body frame. The
# intrinsics scale the provisional M20/Whale calibration to the 800x600 images
# actually emitted by this robot; the vendor documentation does not publish a
# factory calibration.
_FRONT_CAMERA_XYZ = (0.37646, 0.0, 0.03738)
_REAR_CAMERA_XYZ = (-0.37646, 0.0, 0.03738)
_OPTICAL_ROT = Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5)
_CAMERA_WIDTH = 800
_CAMERA_HEIGHT = 600
_CAMERA_FOCAL_LENGTH = 607.0 * _CAMERA_WIDTH / 1280.0
_FRONT_CAMERA_INFO = camera_info_from_intrinsics(
    fx=_CAMERA_FOCAL_LENGTH,
    fy=_CAMERA_FOCAL_LENGTH,
    cx=_CAMERA_WIDTH * 0.5,
    cy=_CAMERA_HEIGHT * 0.5,
    width=_CAMERA_WIDTH,
    height=_CAMERA_HEIGHT,
    header=Header(frame_id="front_camera_optical"),
)
_REAR_CAMERA_INFO = camera_info_from_intrinsics(
    fx=_CAMERA_FOCAL_LENGTH,
    fy=_CAMERA_FOCAL_LENGTH,
    cx=_CAMERA_WIDTH * 0.5,
    cy=_CAMERA_HEIGHT * 0.5,
    width=_CAMERA_WIDTH,
    height=_CAMERA_HEIGHT,
    header=Header(frame_id="rear_camera_optical"),
)


class M20CameraRelay(Module):
    """Relay both vendor RTSP cameras as compressed H.265 streams."""

    front_camera: Out[CompressedVideo]
    rear_camera: Out[CompressedVideo]
    front_camera_info: Out[CameraInfo]
    rear_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        streams = (
            (FRONT_CAMERA_RTSP_URL, self.front_camera, "m20_front_camera"),
            (REAR_CAMERA_RTSP_URL, self.rear_camera, "m20_rear_camera"),
        )
        relay_threads = [
            threading.Thread(
                target=self._relay,
                args=stream,
                name=f"{stream[2]}-rtsp",
                daemon=True,
            )
            for stream in streams
        ]
        self._threads = [
            threading.Thread(
                target=self._publish_camera_metadata,
                name="m20-camera-metadata",
                daemon=True,
            ),
            *relay_threads,
        ]
        for thread in self._threads:
            thread.start()

    def _publish_camera_metadata(self) -> None:
        while not self._stop_event.is_set():
            now = time.time()
            self.tf.publish(
                TFMessage(
                    transforms=[
                        TransformStamped(
                            header=Header(frame_id="base_link", stamp=time_from_seconds(now)),
                            child_frame_id="front_camera_link",
                            transform=Transform(
                                translation=Vector3(
                                    x=_FRONT_CAMERA_XYZ[0],
                                    y=_FRONT_CAMERA_XYZ[1],
                                    z=_FRONT_CAMERA_XYZ[2],
                                ),
                                rotation=Quaternion(w=1.0),
                            ),
                        ),
                        TransformStamped(
                            header=Header(
                                frame_id="front_camera_link", stamp=time_from_seconds(now)
                            ),
                            child_frame_id="front_camera_optical",
                            transform=Transform(rotation=_OPTICAL_ROT),
                        ),
                        TransformStamped(
                            header=Header(frame_id="base_link", stamp=time_from_seconds(now)),
                            child_frame_id="rear_camera_link",
                            transform=Transform(
                                translation=Vector3(
                                    x=_REAR_CAMERA_XYZ[0],
                                    y=_REAR_CAMERA_XYZ[1],
                                    z=_REAR_CAMERA_XYZ[2],
                                ),
                                rotation=Quaternion(z=1.0),
                            ),
                        ),
                        TransformStamped(
                            header=Header(
                                frame_id="rear_camera_link", stamp=time_from_seconds(now)
                            ),
                            child_frame_id="rear_camera_optical",
                            transform=Transform(rotation=_OPTICAL_ROT),
                        ),
                    ]
                )
            )
            self.front_camera_info.publish(
                camera_info_with_stamp(_FRONT_CAMERA_INFO, time_from_seconds(now))
            )
            self.rear_camera_info.publish(
                camera_info_with_stamp(_REAR_CAMERA_INFO, time_from_seconds(now))
            )
            self._stop_event.wait(1.0)

    def _relay(self, url: str, output: Out[CompressedVideo], camera_name: str) -> None:
        while not self._stop_event.is_set():
            try:
                with av.open(
                    url,
                    options={"rtsp_transport": "tcp", "fflags": "nobuffer"},
                    timeout=(1.0, 1.0),
                ) as container:
                    video = container.streams.video[0]
                    if video.codec_context.name != "hevc":
                        raise ValueError(f"expected H.265, got {video.codec_context.name}")
                    logger.info("M20 camera stream connected", camera=camera_name)
                    for packet in container.demux(video):
                        if self._stop_event.is_set():
                            return
                        # The vendor RTSP stream already contains Annex-B access
                        # units and emits an IDR about every other frame. Publishing
                        # only IDRs makes every Zenoh sample independently decodable.
                        if not packet.size or packet.is_corrupt or not packet.is_keyframe:
                            continue
                        output.publish(
                            CompressedVideo(
                                data=bytes(packet),
                                format="h265",
                                frame_id="",
                                timestamp=time_from_seconds(time.time()),
                            )
                        )
            except (av.FFmpegError, IndexError, ValueError) as exc:
                logger.warning(
                    "M20 camera stream unavailable",
                    camera=camera_name,
                    error=str(exc),
                )
                self._stop_event.wait(2.0)

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        super().stop()
