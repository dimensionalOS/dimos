# How do I use Rerun?

## 1. Basic: Get the Dashboard up and running

If you have something like this (executing a blueprint with `autoconnect`):

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import camera_module
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule

blueprint = (
    autoconnect(
        camera_module(),  # default hardware=Webcam(camera_index=0)
        ManipulationModule.blueprint(),
    )
    .global_config(n_dask_workers=1)
)
coordinator = blueprint.build()
print("Webcam pipeline running. Press Ctrl+C to stop.")
coordinator.loop()
```

just add `Dashboard` to the autoconnected modules:

```py
from dimos.core.blueprints import autoconnect
from dimos.hardware.camera.module import camera_module
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.wip_viz.dashboard.dimos_dashboard import Dashboard
from dimos.wip_viz.rerun.layouts import RerunAllTabsLayout

# FIXME: get a way to list what entity-targets are available for the selected layout
blueprint = (
    autoconnect(
        camera_module(),  # default hardware=Webcam(camera_index=0)
        ManipulationModule.blueprint(),
        Dashboard(), # FIXME: ask/test if we need to do .blueprint() here
        RerunAllTabsLayout.blueprint(), # rerun is one part of the Dashboard
    )
    .global_config(n_dask_workers=1)
)
```

## 2. Customizing the output of your module

```py
from dimos.core.stream import In, Out
from dimos.wip_viz.rerun.types import RerunRender

class YourCameraListener(Module):
    image: In[Image] = None
    render_image: Out[RerunRender[Image]] = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)

    def start(self) -> None:
        @self.image.subscribe
        def _on_frame(img: Image) -> None:
            self.render_image.publish(RerunRender(img, "/camera1")) 
            # NOTE: if another module happens to publish to /camera1, 
            # there will be "fighting" on that entity name
            # different parts of the layout will have different names
            # you can target it, or just pick a random name
            # NOTE 2: if you pick a random name, every frame is going to 
            # accumulate in the viewer, use the same name to update the image
```