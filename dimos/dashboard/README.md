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
from dimos.dashboard.rerun import layouts, RerunHook
from dimos.msgs.sensor_msgs import Image

layout = layouts.AllTabs(collapse_panels=False)

blueprint = (
    autoconnect(
        CameraModule.blueprint(),
        ManipulationModule.blueprint(),
        Dashboard(
            layout=layout,
        ).blueprint(),
        RerunHook(
            Image,
            target_entity=layout.entities.spatial2d
        ).blueprint(),
    )
    .global_config(n_dask_workers=1)
)
coordinator = blueprint.build()
print("Webcam pipeline running. Press Ctrl+C to stop.")
coordinator.loop()
```
