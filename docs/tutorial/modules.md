


# Dimos Modules

Module is a subsystem on a robot that operates autonomously and communicates to other subsystems.
Some examples of are:

- Webcam (outputs image)
- Navigation (inputs a map and a target, outputs a path)
- Detection (takes an image and a vision model like yolo, outputs a stream of detections)

etc

## Example Module

```python session=camera_module_demo ansi=false
from dimos.hardware.camera.module import CameraModule
print(CameraModule.io())
```

<!--Result:-->
```
┌┴─────────────┐
│ CameraModule │
└┬─────────────┘
 ├─ color_image: Image
 ├─ camera_info: CameraInfo
 │
 ├─ RPC start() -> str
 ├─ RPC stop() -> None
 │
 ├─ Skill video_stream (stream=passive, reducer=latest_reducer, output=image)
```



We can see that camera module outputs two streams:

color_image with [sensor_msgs.Image](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html) type
camera_info with [sensor_msgs.CameraInfo](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html) type

As well as offers two RPC calls, start and stop, and a tool for an agent called video_stream (about this later)

We can easily start this module and explore it's output

```pythonx session=camera_module_demo

camera = CameraModule()
camera.start()

# now this module runs in our main loop in a thread. we can observe it's outputs

unsubscribe = camera.color_image.subscribe(print)
time.sleep(1)
unsubscribe()
```

<!--Error:-->
```
Traceback (most recent call last):
  File "/home/lesh/coding/md-babel-python/md_babel_py/session_server.py", line 56, in main
    result = eval(compile(code, "<block>", "eval"), namespace)
                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "<block>", line 2
    camera = CameraModule()
           ^
SyntaxError: invalid syntax

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/lesh/coding/md-babel-python/md_babel_py/session_server.py", line 61, in main
    exec(compile(code, "<block>", "exec"), namespace)
  File "<block>", line 7, in <module>
AttributeError: 'Out' object has no attribute 'subscribe'

```












## Visualizing Blueprints

Basic unitree go2 blueprint looks like

```pythonx output=go2_basic.svg
from dimos.core.introspection import to_svg, to_dot
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic

to_svg(basic, "go2_basic.svg")
```

<!--Result:-->
![output](go2_basic.svg)





```pythonx output=go2_standard.svg
from dimos.core.introspection import to_svg, to_dot
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic

to_svg(standard, "go2_standard.svg")
```

<!--Result:-->
![output](go2_standard.svg)
