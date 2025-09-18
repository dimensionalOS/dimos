# Copyright 2025 Dimensional Inc.
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

from dimos.core import LCMTransport, start
from dimos.hardware.webcam import ColorCameraModule, Webcam
from dimos.msgs.sensor_msgs import Image


@pytest.mark.tool
def test_basic():
    webcam = Webcam()
    subscription = webcam.color_stream().subscribe(
        on_next=lambda img: print(f"Got image: {img.width}x{img.height}"),
        on_error=lambda e: print(f"Error: {e}"),
        on_completed=lambda: print("Stream completed"),
    )

    # Keep the subscription alive for a few seconds
    try:
        time.sleep(3)
    finally:
        # Clean disposal
        subscription.dispose()
        print("Test completed")


@pytest.mark.tool
def test_module():
    dimos = start(1)
    # Deploy ColorCameraModule, not Webcam directly
    camera_module = dimos.deploy(
        ColorCameraModule,
        hardware=lambda: Webcam(camera_index=4, frequency=30, stereo_slice="left"),
    )
    camera_module.image.transport = LCMTransport("/image", Image)
    camera_module.start()

    test_transport = LCMTransport("/image", Image)
    test_transport.subscribe(print)

    time.sleep(60)

    print("shutting down")
    camera_module.stop()
    time.sleep(1.0)
    dimos.stop()


if __name__ == "__main__":
    test_module()
