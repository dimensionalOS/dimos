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

from typing import Any

import numpy as np
from reactivex.disposable import Disposable
import rerun as rr

from dimos.core import In, Module, Out, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard, RerunConfig
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat


class CameraListener(Module):
    color_image: In[Image] = None
    color_image_1: Out[Image] = None
    color_image_2: Out[Image] = None
    color_image_3: Out[Image] = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

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
            except Exception as error:
                print(f"""error = {error}""")

        self._disposables.add(Disposable(self.color_image.subscribe(_on_frame)))

    @rpc
    def stop(self) -> None:
        super().stop()


if __name__ == "__main__":
    cam_generator = CameraModule.blueprint()
    blueprint = autoconnect(
        cam_generator,
        Dashboard.blueprint(
            open_rerun=True,
        ),
    ).global_config(n_dask_workers=3)
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()
