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

from reactivex.disposable import Disposable

from dimos.core import In, Module, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard, RerunConnection
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.sensor_msgs import Image


class CameraListener(Module):
    color_image: In[Image] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.rc = RerunConnection()  # one connection per process

        def _on_frame(img: Image) -> None:
            self._count += 1
            self.rc.log(f"/{self.__class__.__name__}/color_image", img.to_rerun())
            if self._count % 20 == 0:
                print(
                    f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                    f"shape={img.height}x{img.width}"
                )

        print("camera subscribing")
        unsub = self.color_image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


if __name__ == "__main__":
    blueprint = (
        autoconnect(
            CameraModule.blueprint(
                # hardware=lambda: Webcam(
                #     camera_index=0,
                #     frequency=15,
                #     stereo_slice="left",
                #     camera_info=zed.CameraInfo.SingleWebcam,
                # ),
            ),
            CameraListener.blueprint(),
            Dashboard.blueprint(
                auto_open=True,
                terminal_commands={
                    "lcm-spy": "dimos lcmspy",
                    "skill-spy": "dimos skillspy",
                },
            ),
        )
        .transports({("color_image", Image): pSHMTransport("/cam/image")})
        .global_config(n_dask_workers=1)
    )
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()
