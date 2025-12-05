"""Manually deploy webcam module and log received images (no autoconnect)."""

from __future__ import annotations

import time

from reactivex.disposable import Disposable

from dimos import core
from dimos.core import In, Module
from dimos.hardware.camera.module import CameraModule
from dimos.msgs.sensor_msgs import Image


class CameraListener(Module):
    """Simple sink that prints when it receives images."""

    image: In[Image] = None  # type: ignore[assignment]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._count = 0

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
    dimos_client = core.start(n=1)

    # Deploy camera and listener manually.
    cam = dimos_client.deploy(CameraModule)  # type: ignore[attr-defined]
    listener = dimos_client.deploy(CameraListener)  # type: ignore[attr-defined]

    # Manually wire the transport: share the camera's Out[Image] to the listener's In[Image].
    # Use shared-memory transport to avoid LCM setup.
    cam.image.transport = core.pSHMTransport("/cam/image")
    listener.image.transport = cam.image.transport

    # Start modules.
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
