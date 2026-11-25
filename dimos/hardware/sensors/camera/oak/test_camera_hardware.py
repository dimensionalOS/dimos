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

"""A real OAK on this machine's USB bus: ``pytest -m self_hosted`` on this file.

Skipped when no Luxonis device is on the bus. It needs the ``oak`` extra.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import threading
import time
from typing import Any

import pytest

from dimos.hardware.sensors.camera.oak.camera import (
    USB_SETTLE_TIMEOUT_S,
    OakCamera,
    oak_usb_products,
)
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(not oak_usb_products(), reason="no Luxonis OAK on the USB bus"),
]

MEASURE_S = 10.0
# Measured rates may fall this far below the configured ones.
RATE_TOLERANCE = 0.2


class Counted:
    def __init__(self, camera: OakCamera) -> None:
        self.stamps: dict[str, list[float]] = {"color_image": [], "depth_image": []}
        self.infos: list[CameraInfo] = []
        self.first_color = threading.Event()
        camera.color_image.subscribe(self._color)
        camera.depth_image.subscribe(lambda image: self.stamps["depth_image"].append(image.ts))
        camera.camera_info.subscribe(self.infos.append)

    def _color(self, image: Any) -> None:
        self.stamps["color_image"].append(image.ts)
        self.first_color.set()

    def rate(self, name: str, since: float) -> float:
        return sum(1 for ts in self.stamps[name] if ts >= since) / MEASURE_S


@pytest.fixture
def start_camera() -> Iterator[Callable[[], tuple[OakCamera, Counted]]]:
    OakCamera.check_dependencies()
    cameras: list[OakCamera] = []

    def start() -> tuple[OakCamera, Counted]:
        camera = OakCamera()
        cameras.append(camera)
        counted = Counted(camera)
        camera.start()
        return camera, counted

    yield start
    for camera in cameras:
        camera.stop()


def test_streams_at_the_configured_rates_and_again_right_after_a_stop(
    start_camera: Callable[[], tuple[OakCamera, Counted]],
) -> None:
    for _ in ("first launch", "relaunch right after the stop"):
        camera, counted = start_camera()
        bring_up = camera.config.open_timeout + USB_SETTLE_TIMEOUT_S
        assert counted.first_color.wait(bring_up), f"no picture within {bring_up:g} s"
        since = time.time()
        time.sleep(MEASURE_S)
        camera.stop()

        config = camera.config
        assert counted.rate("color_image", since) >= config.fps * (1 - RATE_TOLERANCE)
        assert counted.rate("depth_image", since) >= config.depth_fps * (1 - RATE_TOLERANCE)
        info = counted.infos[-1]
        assert (info.width, info.height) == (config.width, config.height)
        assert info.K[0] > 0 and info.K[4] > 0
