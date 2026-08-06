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

from collections.abc import Callable, Iterator

import pytest

from dimos.robot.drone.px4.gstreamer.fake_gstreamer import FakeGstApi
from dimos.robot.drone.px4.gstreamer.gstreamer_api import (
    GstEncoder,
    GstSource,
    GstSourceConfig,
)
from dimos.robot.drone.px4.gstreamer.gstreamer_tee_camera import (
    CameraCalibration,
    Px4GstTeeCamera,
)


def make_camera() -> Iterator[
    Callable[[GstSource, CameraCalibration | None], tuple[FakeGstApi, Px4GstTeeCamera]]
]:
    cameras: list[Px4GstTeeCamera] = []

    def create(
        source: GstSource, calibration: CameraCalibration | None
    ) -> tuple[FakeGstApi, Px4GstTeeCamera]:
        gst = FakeGstApi()
        camera = Px4GstTeeCamera(
            gst_api=gst,
            source_config=GstSourceConfig(
                source=source,
                device="/dev/video0",
                width=640,
                height=480,
                fps=30,
                encoder=GstEncoder.X264,
                bitrate=4_000_000,
                gop=30,
            ),
            calibration=calibration,
            clock=lambda: 42.0,
        )
        cameras.append(camera)
        return gst, camera

    yield create

    for camera in cameras:
        camera.stop()


make_camera = pytest.fixture(make_camera)
