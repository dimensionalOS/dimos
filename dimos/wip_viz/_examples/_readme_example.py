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

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam, WebcamConfig
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.wip_viz.dashboard.dimos_dashboard_module import Dashboard
from dimos.wip_viz.rerun.layouts import RerunAllTabsLayout
from dimos.wip_viz.rerun.types import RerunRender

# # FIXME: get a way to list what entity-targets are available for the selected layout
# blueprint = (
#     autoconnect(
#         camera_module(),  # default hardware=Webcam(camera_index=0)
#         ManipulationModule.blueprint(),
#         Dashboard(), # FIXME: ask/test if we need to do .blueprint() here
#         RerunAllTabsLayout.blueprint(), # rerun is one part of the Dashboard
#     )
#     .global_config(n_dask_workers=1)
# )


class CameraListener(Module):
    image: In[Image] = None  # type: ignore[assignment]
    render_image: Out[RerunRender[rr.Image, None]] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        def _on_frame(img: Image) -> None:
            self._count += 1
            if self._count % 20 == 0:
                print(
                    f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                    f"shape={img.height}x{img.width}"
                )
                print("[camera-listener] publishing to /spatial2d")
                # RUNS (should trigger ->)
                # rr.log("/spatial2d", img.to_rerun()) # this is just running whats in the hook to bypass this testing issue
                self.render_image.publish(RerunRender([img.to_rerun(), "/spatial2d"]))
                self.render_image.publish(img)

        unsub = self.image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


def main() -> None:
    dimos_client = core.start(n=6)

    # Deploy camera and listener manually.
    cam = dimos_client.deploy(CameraModule, frequency=30, hardware=lambda: Webcam(frequency=30))
    camera_listener = dimos_client.deploy(CameraListener)
    rerun_layout = dimos_client.deploy(RerunAllTabsLayout)
    dashboard = dimos_client.deploy(Dashboard)
    foxglove = dimos_client.deploy(FoxgloveBridge)

    foxglove.start()
    # Manually wire the transport: share the camera's Out[Image] to the camera_listener's In[Image].
    # Use shared-memory transport to avoid LCM setup.
    #
    cam.color_image.transport = core.LCMTransport("/cam/image", Image)
    cam.camera_info.transport = core.LCMTransport("/cam/camera_info", CameraInfo)
    camera_listener.image.connect(cam.color_image)

    # connect camera_listener to rerun_layout
    camera_listener.render_image.transport = core.pLCMTransport("/cam/render_image")
    rerun_layout.render_image.connect(camera_listener.render_image)
    # rerun_layout to dashboard
    rerun_layout.rerun_blueprint.transport = core.pLCMTransport("/rerun_layout/rerun_blueprint")
    dashboard.blueprint_record.connect(rerun_layout.rerun_blueprint)

    # Start modules.
    cam.start()
    camera_listener.start()
    rerun_layout.start()
    dashboard.start()

    print("Manual webcam hook running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera_listener.stop()
        cam.stop()
        rerun_layout.stop()
        dashboard.stop()
        dimos_client.close_all()  # type: ignore[attr-defined]


if __name__ == "__main__":
    main()
