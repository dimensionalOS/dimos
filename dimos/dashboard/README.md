# How do I use the Dashboard?

If you have something like this (executing a blueprint with `autoconnect`):

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import CameraModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule

blueprint = (
    autoconnect(
        CameraModule.blueprint(),
        ManipulationModule.blueprint(),
    )
    .global_config(n_dask_workers=1)
)

if __name__ == "__main__":
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()
```



```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import CameraModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.dashboard.module import Dashboard, RerunConnection
from dimos.msgs.sensor_msgs import Image

class CameraListener(Module):
    color_image: In[Image] = None

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._count = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.rc = RerunConnection() # one connection per process

        def _on_frame(img: Image) -> None:
            self._count += 1
            if self._count % 20 == 0:
                self.rc.log(f"/{self.__class__.__name__}/color_image", img.to_rerun())
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
        CameraListener.blueprint(),
        Dashboard.blueprint(
            open_rerun=True,
        ),
    )
    .transports({("color_image", Image): pSHMTransport("/cam/image")})
    .global_config(n_dask_workers=1)
)
if __name__ == "__main__":
    coordinator = blueprint.build()
    print("Webcam pipeline running. Press Ctrl+C to stop.")
    coordinator.loop()
```
