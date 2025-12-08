"""Minimal blueprint runner that reads from a webcam and logs frames."""

from reactivex.disposable import Disposable

from dimos.core import In, Module, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.hardware.camera import zed
from dimos.hardware.camera.module import CameraModule
from dimos.hardware.camera.webcam import Webcam
from dimos.msgs.sensor_msgs import Image


from dimos.dashboard.module import Dashboard
from dimos.msgs.sensor_msgs import Image


class CameraListener(Module):
    color_image: In[Image] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        super().start()

        def _on_frame(img: Image) -> None:
            self._count += 1
            if self._count % 20 == 0:
                rr.log(f"/color_image", img.to_rerun())
                print(
                    f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                    f"shape={img.height}x{img.width}"
                )

        print("camera subscribing")
        unsub = self.color_image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


blueprint = (
    autoconnect(
        CameraModule.blueprint(
            hardware=lambda: Webcam(
                camera_index=0,
                frequency=15,
                stereo_slice="left",
                camera_info=zed.CameraInfo.SingleWebcam,
            ),
        ),
        Dashboard().blueprint(
            auto_open=True,
            terminal_commands={
                "agent-spy": "htop",
                "lcm-spy": "dimos lcmspy",
                # "skill-spy": "dimos skillspy",
            },
        ),
    )
    .transports({("color_image", Image): pSHMTransport("/cam/image")})
    .global_config(n_dask_workers=1)
)


def main() -> None:
    # Use the default webcam-based CameraModule, then tap its images with CameraListener.
    # Force the image transport to shared memory to avoid LCM env issues.
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()


if __name__ == "__main__":
    main() 