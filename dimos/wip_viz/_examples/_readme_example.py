import time

from reactivex.disposable import Disposable

from dimos import core
from dimos.core import In, Out, Module
from dimos.core.blueprints import autoconnect
from dimos.msgs.sensor_msgs import Image
from dimos.hardware.camera.module import CameraModule
from dimos.wip_viz.dashboard.dimos_dashboard_module import Dashboard
from dimos.wip_viz.rerun.layouts import RerunAllTabsLayout
from dimos.wip_viz.rerun.types import RerunRender
import rerun as rr  # pip install rerun-sdk

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

    def start(self) -> None:
        def _on_frame(img: Image) -> None:
            self._count += 1
            if self._count % 100 == 0:
                print(
                    f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                    f"shape={img.height}x{img.width}"
                )
                print(f"[camera-listener] publishing to /spatial2d")
                # RUNS (should trigger ->)
                self.render_image.publish(RerunRender(img.to_rerun(), "/spatial2d"))
                self.render_image.publish(img)
        unsub = self.image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


def main() -> None:
    # Start dimos cluster with minimal workers.
    dimos_client = core.start(n=1)

    # Deploy camera and listener manually.
    cam = dimos_client.deploy(CameraModule)
    camera_listener = dimos_client.deploy(CameraListener)
    rerun_layout = dimos_client.deploy(RerunAllTabsLayout)
    dashboard = dimos_client.deploy(Dashboard)

    # Manually wire the transport: share the camera's Out[Image] to the camera_listener's In[Image].
    # Use shared-memory transport to avoid LCM setup.
    cam.image.transport = core.pSHMTransport("/cam/image")
    camera_listener.image.transport = cam.image.transport
    
    # connect camera_listener to rerun_layout
    camera_listener.render_image.transport = core.pSHMTransport("/cam/render_image")
    rerun_layout.render_image.transport = camera_listener.render_image.transport
    # rerun_layout to dashboard
    rerun_layout.rerun_blueprint.transport = core.pSHMTransport("/rerun_layout/rerun_blueprint")
    dashboard.blueprint_record.transport = rerun_layout.rerun_blueprint.transport

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
