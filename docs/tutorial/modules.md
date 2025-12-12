


# Dimos Modules

Module is a subsystem on a robot that operates autonomously and communicates to other subsystems.
Some examples of are:

- Webcam (outputs image)
- Navigation (inputs a map and a target, outputs a path)
- Detection (takes an image and a vision model like yolo, outputs a stream of detections)

etc

## Example Module

```pythonx session=camera_module_demo ansi=false
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
# camera.io() returns the same result as above
camera.start()
# now this module runs in our main loop in a thread. we can observe it's outputs

camera.color_image.subscribe(print)
time.sleep(1)
camera.stop()
```

<!--Result:-->
```
<image>
<image>
<image>
```

## Blueprints

Blueprint is a structure of interconnected modules. basic unitree go2 blueprint looks like this,
```pythonx
from dimos.core.introspection import to_svg, to_dot
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic

print(to_dot(basic))
```

<!--Result:-->
```
digraph modules {
    bgcolor=transparent;
    rankdir=LR;
    splines=true;
    remincross=true;
    nodesep=1.5;
    ranksep=1.5;
    node [shape=box, style=filled, fillcolor="#0b0f0f", fontcolor="#b5e4f4", color="#5c9ff0", fontname=fixed, fontsize=12, width=2, height=0.8, margin="0.2,0.1"];
    edge [fontname=fixed, fontsize=10];

    subgraph cluster_navigation {
        label="navigation";
         labeljust=r;
         fontname=fixed;
         fontsize=14;
        fontcolor="#b5e4f4";
         style="filled,dashed";
        color="#AA96DA";
         penwidth=1;
        fillcolor="#AA96DA10";
        AstarPlanner;
        BehaviorTreeNavigator;
        HolonomicLocalPlanner;
        WavefrontFrontierExplorer;
    }

    subgraph cluster_robot {
        label="robot";
         labeljust=r;
         fontname=fixed;
         fontsize=14;
        fontcolor="#b5e4f4";
         style="filled,dashed";
        color="#64B5F6";
         penwidth=1;
        fillcolor="#64B5F610";
        GO2Connection;
        Map;
    }

    AstarPlanner -> HolonomicLocalPlanner [xlabel="path:Path", color="#F06292", fontcolor="#F06292", forcelabels=false, sametail="path_Path", samehead="path_Path"];
    BehaviorTreeNavigator -> AstarPlanner [xlabel="target:PoseStamped", color="#FF6B6B", fontcolor="#FF6B6B", forcelabels=false, sametail="target_PoseStamped", samehead="target_PoseStamped"];
    BehaviorTreeNavigator -> WavefrontFrontierExplorer [xlabel="goal_reached:Bool", color="#DCE775", fontcolor="#DCE775", forcelabels=false, sametail="goal_reached_Bool", samehead="goal_reached_Bool"];
    GO2Connection -> Map [xlabel="lidar:LidarMessage", color="#7986CB", fontcolor="#7986CB", forcelabels=false, sametail="lidar_LidarMessage", samehead="lidar_LidarMessage"];
    HolonomicLocalPlanner -> GO2Connection [xlabel="cmd_vel:Twist", color="#4DB6AC", fontcolor="#4DB6AC", forcelabels=false, sametail="cmd_vel_Twist", samehead="cmd_vel_Twist"];
    Map -> AstarPlanner [xlabel="global_costmap:OccupancyGrid", color="#FF8A65", fontcolor="#FF8A65", forcelabels=false, sametail="global_costmap_OccupancyGrid", samehead="global_costmap_OccupancyGrid"];
    Map -> BehaviorTreeNavigator [xlabel="global_costmap:OccupancyGrid", color="#FF8A65", fontcolor="#FF8A65", forcelabels=false, sametail="global_costmap_OccupancyGrid", samehead="global_costmap_OccupancyGrid"];
    Map -> HolonomicLocalPlanner [xlabel="local_costmap:OccupancyGrid", color="#F06292", fontcolor="#F06292", forcelabels=false, sametail="local_costmap_OccupancyGrid", samehead="local_costmap_OccupancyGrid"];
    Map -> WavefrontFrontierExplorer [xlabel="global_costmap:OccupancyGrid", color="#FF8A65", fontcolor="#FF8A65", forcelabels=false, sametail="global_costmap_OccupancyGrid", samehead="global_costmap_OccupancyGrid"];
    WavefrontFrontierExplorer -> BehaviorTreeNavigator [xlabel="goal_request:PoseStamped", color="#FFF59D", fontcolor="#FFF59D", forcelabels=false, sametail="goal_request_PoseStamped", samehead="goal_request_PoseStamped"];
}
```


```python output=go2_basic.svg
from dimos.core.introspection.blueprint import dot2
from dimos.robot.unitree_webrtc.unitree_go2_blueprints import basic, standard, agentic



dot2.render_svg(standard, "go2_basic.svg")
```

<!--Result:-->
![output](go2_basic.svg)
