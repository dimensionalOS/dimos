# Copyright 2025-2026 Dimensional Inc.
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

import time

import pytest

from dimos import core
from dimos.hardware.sensors.camera import zed
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image


@pytest.fixture
def dimos():
    dimos_instance = core.start(1)
    yield dimos_instance
    dimos_instance.stop()


@pytest.mark.tool
def test_streaming_single(dimos) -> None:
    camera = dimos.deploy(
        CameraModule,
        transform=Transform(
            translation=Vector3(0.05, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="sensor",
            child_frame_id="camera_link",
        ),
        hardware=lambda: Webcam(
            camera_index=0,
            frequency=0.0,  # full speed but set something to test sharpness barrier
            camera_info=zed.CameraInfo.SingleWebcam,
        ),
    )

    camera.color_image.transport = core.LCMTransport("/color_image", Image)
    camera.camera_info.transport = core.LCMTransport("/camera_info", CameraInfo)
    camera.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        camera.stop()
        dimos.stop()
