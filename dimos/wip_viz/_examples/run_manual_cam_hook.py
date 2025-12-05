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

"""Manually deploy webcam module and log received images (no autoconnect)."""

import time

from reactivex.disposable import Disposable

from dimos import core
from dimos.core import In, Module, rpc
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam, WebcamConfig
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.robot.foxglove_bridge import FoxgloveBridge


class CameraListener(Module):
    """Simple sink that prints when it receives images."""

    image: In[Image] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        def _on_frame(img: Image) -> None:
            self._count += 1
            print(
                f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                f"shape={img.height}x{img.width}"
            )

        unsub = self.image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


def main() -> None:
    # Start dimos cluster with minimal workers.
    dimos_client = core.start(n=6)

    # Deploy camera and listener manually.
    cam = dimos_client.deploy(CameraModule, frequency=30.0, hardware=lambda: Webcam(frequency=30.0))  # type: ignore[attr-defined]
    listener = dimos_client.deploy(CameraListener)  # type: ignore[attr-defined]
    foxglove = dimos_client.deploy(FoxgloveBridge)

    # Manually wire the transport: share the camera's Out[Image] to the listener's In[Image].
    # Use shared-memory transport to avoid LCM setup.
    cam.color_image.transport = core.LCMTransport("/color_image", Image)
    cam.camera_info.transport = core.LCMTransport("/camera_info", CameraInfo)
    listener.image.connect(cam.color_image)

    # Start modules.
    foxglove.start()
    cam.start()
    listener.start()

    print("Manual webcam hook running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        cam.stop()
        dimos_client.close_all()  # type: ignore[attr-defined]


if __name__ == "__main__":
    main()
