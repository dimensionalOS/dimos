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
coordinator = blueprint.build()
print("Webcam pipeline running. Press Ctrl+C to stop.")
coordinator.loop()
```

Pick a layout,  `Dashboard`, to the autoconnected modules:

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import CameraModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.dashboard.module import Dashboard
from dimos.msgs.sensor_msgs import Image

class CameraListener(Module):
    color_image: In[Image] = None  # type: ignore[assignment]
    
    @rpc
    def start(self) -> None:
        @self.color_image.subscribe
        def _on_frame(img: Image) -> None:
            self._count += 1
            if self._count % 20 == 0:
                rr.log(f"/color_image", img.to_rerun())
                print(
                    f"[camera-listener] frame={self._count} ts={img.ts:.3f} "
                    f"shape={img.height}x{img.width}"
                )


blueprint = (
    autoconnect(
        CameraModule.blueprint(),
        Dashboard().blueprint(
            layout=layout,
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

coordinator = blueprint.build()
print("Webcam pipeline running. Press Ctrl+C to stop.")
coordinator.loop()
```
