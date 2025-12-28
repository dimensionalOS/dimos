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

"""Minimal blueprint runner that reads from a webcam and logs frames."""

import numpy as np
from reactivex.disposable import Disposable
import rerun as rr

from dimos.core import In, Module, Out, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard, RerunConnection
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat


class CameraListener(Module):
    color_image: In[Image] = None  # type: ignore[assignment]
    color_image_1: Out[Image] = None
    color_image_2: Out[Image] = None
    color_image_3: Out[Image] = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        self.rc = RerunConnection()  # one connection per process

        def _on_frame(img: Image) -> None:
            # Expect HxWx3 uint8
            frame = img.to_rgb().to_opencv()
            try:
                r = frame.copy()
                r[..., 1] = 0
                r[..., 2] = 0

                g = frame.copy()
                g[..., 0] = 0
                g[..., 2] = 0

                b = frame.copy()
                b[..., 0] = 0
                b[..., 1] = 0

                out1 = Image(data=r, format=ImageFormat.RGB)
                out2 = Image(data=g, format=ImageFormat.RGB)
                out3 = Image(data=b, format=ImageFormat.RGB)

                self.color_image_1.publish(out1)
                self.color_image_2.publish(out2)
                self.color_image_3.publish(out3)

                print("logging color images")
                self.rc.log(f"/{self.__class__.__name__}/color_image_1", out1.to_rerun())
                self.rc.log(f"/{self.__class__.__name__}/color_image_2", out2.to_rerun())
                self.rc.log(f"/{self.__class__.__name__}/color_image_3", out3.to_rerun())
            except Exception as error:
                print(f"""error = {error}""")

        print("camera subscribing")
        unsub = self.color_image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))

    @rpc
    def stop(self) -> None:
        super().stop()


cam_listener = CameraListener.blueprint()

blueprint = (
    autoconnect(
        CameraModule.blueprint(),
        cam_listener,
        Dashboard.blueprint(
            open_rerun=True,
        ),
    )
    .transports({("color_image", Image): pSHMTransport("/cam/image")})
    .global_config(n_dask_workers=3)
)


def main() -> None:
    # Use the default webcam-based CameraModule, then tap its images with CameraListener.
    # Force the image transport to shared memory to avoid LCM env issues.
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()


if __name__ == "__main__":
    main()
