"""Minimal blueprint runner that reads from a webcam and logs frames."""

from __future__ import annotations

from reactivex.disposable import Disposable

from dimos.core import In, Module, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import camera_module
from dimos.msgs.sensor_msgs import Image


class CameraListener(Module):
    image: In[Image] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0
        print(f'''self._count = {self._count}''')

    def start(self) -> None:
        def _on_frame(img: Image) -> None:
            self._count += 1
            print(
                f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                f"shape={img.height}x{img.width}"
            )
        
        print("subscribing")
        unsub = self.image.subscribe(_on_frame)
        self._disposables.add(Disposable(unsub))


def main() -> None:
    # Use the default webcam-based CameraModule, then tap its images with CameraListener.
    # Force the image transport to shared memory to avoid LCM env issues.
    blueprint = (
        autoconnect(
            camera_module(),  # default hardware=Webcam(camera_index=0)
            CameraListener.blueprint(),
        )
        .transports({("image", Image): pSHMTransport("/cam/image")})
        .global_config(n_dask_workers=1)
    )
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()


if __name__ == "__main__":
    main()
