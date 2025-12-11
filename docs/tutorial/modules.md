


# Dimos Modules

Module is a subsystem on a robot that operates autonomously and communicates to other subsystems.
Some examples of are:

- Webcam (outputs image)
- Navigation (inputs a map and a target, outputs a path)
- Detection (takes an image and a vision model like yolo, outputs a stream of detections)

etc

## Example Module

```pythonx session=camera_module_demo
from dimos.hardware.camera.module import CameraModule
CameraModule,io()
```

<!--Result:-->
```
┌┴─────────────┐
│ CameraModule │
└┬─────────────┘
 ├─ Out color_image[Image]
 ├─ Out camera_info[CameraInfo]
 │
 ├─ RPC start -> None
 ├─ RPC stop -> None
 |
 ├─ TOOL video_stream
```

We can see that camera module outputs two streams:

color_image with [sensor_msgs.Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) type
camera_info with [sensor_msgs.CameraInfo](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) type

As well as offers two RPC calls, start and stop, and a tool for an agent called video_stream (about this later)

We can easily start this module and explore it's output

```python
from dimos.hardware.camera.module import CameraModule

camera = CameraModule()
camera.start()

# now this module runs in our main loop in a thread. we can observe it's outputs

unsubscribe = camera.color_image.subscribe(print)
time.sleep(1)
unsubscribe()
```

<!--Result:-->
```
<Image>
<Image>
<Image>
<Image>
<Image>
```











## Visualizing Blueprints

Basic unitree go2 blueprint looks like

```pythonx output=go2_basic.svg
from dimos.core.blueprint_vis import to_svg, to_dot
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic

to_svg(basic, "go2_basic.svg")
```

<!--Result:-->
![output](go2_basic.svg)





```pythonx output=go2_standard.svg
from dimos.core.blueprint_vis import to_svg, to_dot
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic

to_svg(standard, "go2_standard.svg")
```

<!--Result:-->
![output](go2_standard.svg)
